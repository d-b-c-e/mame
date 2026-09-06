// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Driver for Midway V-Unit games

**************************************************************************/

#include "emu.h"
#include <chrono>
#include "midvunit.h"
#include "midvunit_hud_ocr.h"
#include "cruisn/hud_speed_filter.h"
#include "cruisn/hud_numeric_speed.h"
#include "cruisn/motor_signal.h"
#include "cruisn/tjunctions.h"
#include "cruisn/retained_texture.h"

#include "williamssound.h"

#include "cpu/adsp2100/adsp2100.h"
#include "cpu/tms34010/tms34010.h"

// ── POC instrumentation (env-gated, zero cost when unset) ────────────────
//
// MIDV_QUADLOG=<file>          append every DMA quad as a binary record:
//                              u32 frame, u16 page_control, u16 dma_data[16]
// MIDV_STATEDUMP_FRAME=<n>     at screen_update of frame >= n, dump
// MIDV_STATEDUMP_DIR=<dir>     videoram/textureram/paletteram + meta there
//
// Purpose: prove process_dma_queue() is the complete render surface by
// re-rasterizing the captured stream offline and diffing against videoram.
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <algorithm>
#include <thread>
#include <set>
#include <vector>
#include <string>
#ifdef _WIN32
#include <winsock2.h>
#include <windows.h>
// Shared conditioning chain, vendored from dbce-wheel-mod-toolkit.
// Cruis'n uses the SHAPER ONLY - the arcade board hands over a finished
// motor byte, so there is no force model to run. See dbce/README.md.
#include "dbce/force_model.h"
#include "dbce/force_profile.h"
#include "dbce/impact_mixer.h"
#include "dbce/signal_sample.h"
#include <mutex>
#include <condition_variable>
#include <cmath>
// POC built-in force feedback: SDL2 haptics, types only - SDL2.dll is loaded
// at run time (see mvffb below), vunit.exe carries no import of it
#include <SDL2/SDL_haptic.h>
#include <SDL2/SDL_joystick.h>
#include "midvunit_gl_shaders.h"
#include "midvunit_menu_assets.h"
#endif

namespace {

FILE *quadlog_open()
{
	const char *path = std::getenv("MIDV_QUADLOG");
	if (!path)
		return nullptr;
	FILE *f = std::fopen(path, "wb");
	if (f)
	{
		setvbuf(f, nullptr, _IOFBF, 1 << 20);
		std::fwrite("MVQ1", 1, 4, f);
	}
	return f;
}

// ── MIDV_LIVE: stream the render state to an external GPU renderer ──────
//
// MIDV_LIVE=1 opens a named shared-memory ring ("Local\\MIDV_LIVE") and
// streams, in strict emulation order: quads, page flips, coalesced CPU
// videoram writes, and texture/palette snapshots when dirty. A viewer
// process (cruisn-collection/gpu/live_viewer.py) renders and presents them live.
// Single producer (the scheduler thread), single consumer. Zero cost when
// the env var is unset.
//
// Ring layout: 64-byte header {magic 'MVL1', u32 size, u64 wpos, u64 rpos,
// u64 dropped, u32 flags} then the data region. Messages are 8-byte
// aligned: {u32 type, u32 payload_len, payload}.
//   1 QUAD    u32 frame, u16 pc, u16 pad, u16 dma[16]
//   2 FLIP    u32 frame, u16 old_pc, u16 new_pc
//   3 PALETTE u32 frame, u8 bytes[]
//   4 TEXTURE u32 frame, u8 bytes[]
//   5 VRAM    u32 frame, u32 offset, u32 count, u16 data[count]
//   6 FRAME   u32 frame (end of visible screen update)
#ifdef _WIN32

// Interlocked operations preserve the cross-process ring layout and publish payloads
// before the consumer sees a position. Volatile alone is not a memory barrier.
static uint64_t ring_load(volatile uint64_t *p) {
	return uint64_t(InterlockedCompareExchange64((volatile LONG64 *)p, 0, 0));
}
static void ring_store(volatile uint64_t *p, uint64_t value) {
	InterlockedExchange64((volatile LONG64 *)p, LONG64(value));
}
struct midv_live
{
	volatile float speed_pct = 0.0f;   // MAME speed %, for the overlay stats
	static constexpr uint32_t RING_SIZE = 128u << 20;
	static constexpr uint32_t HDR_SIZE = 64;

	volatile uint8_t *base = nullptr;
	volatile uint64_t *wpos = nullptr;
	volatile uint64_t *rpos = nullptr;
	volatile uint64_t *dropped = nullptr;
	uint8_t *data = nullptr;
	uint32_t capacity = RING_SIZE;
	bool enabled = false;
	bool tex_dirty = true;    // force initial snapshots
	bool pal_dirty = true;
	uint32_t last_frame = 0;
	cruisn::retained_texture ui_assets;
	std::vector<uint8_t> texture_upload;
	bool retain_ui_assets = false;

	// coalescing buffer for CPU videoram writes
	uint32_t span_start = 0xffffffff;
	uint32_t span_count = 0;
	uint16_t span_data[2048];

	midv_live()
	{
		if (!std::getenv("MIDV_LIVE") && !std::getenv("MIDV_GL"))
			return;
		std::string tag = "Local\\MIDV_LIVE";
		if (std::getenv("MIDV_GL")) tag += "_" + std::to_string(GetCurrentProcessId());
		HANDLE h = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr,
			PAGE_READWRITE, 0, HDR_SIZE + RING_SIZE, tag.c_str());
		if (!h)
			return;
		base = (volatile uint8_t *)MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		CloseHandle(h);
		if (!base)
			return;
		if (const char *e = std::getenv("MIDV_GL_QUEUE_MB"))
			capacity = uint32_t(std::clamp(atoi(e), 16, 128)) << 20;
		memset((void *)base, 0, HDR_SIZE);
		memcpy((void *)base, "MVL1", 4);
		*(volatile uint32_t *)(base + 4) = RING_SIZE;
		wpos = (volatile uint64_t *)(base + 8);
		rpos = (volatile uint64_t *)(base + 16);
		dropped = (volatile uint64_t *)(base + 24);
		data = (uint8_t *)(base + HDR_SIZE);
		enabled = true;
		const char *scale = std::getenv("MIDV_GL_SCALE");
		const char *ui = std::getenv("MIDV_GL_UI_ASSETS");
		retain_ui_assets = std::getenv("MIDV_GL") && (!scale || atoi(scale) > 1)
			&& (!ui || atoi(ui) != 0);
	}

	bool write_msg(uint32_t type, const void *p1, uint32_t l1,
	               const void *p2 = nullptr, uint32_t l2 = 0)
	{
		if (!enabled)
			return false;
		uint32_t const need = (8 + l1 + l2 + 7) & ~7u;
		uint64_t const w = ring_load(wpos);
		uint64_t r = ring_load(rpos);
		// Every message mutates persistent state, including quads and CPU writes.
		// Backpressure is lossless; a dead consumer fails the entire stream.
		for (int wait = 0; need <= capacity && w - r + need > capacity && wait < 500; ++wait)
		{
			Sleep(1);
			r = ring_load(rpos);
		}
		if (need > capacity || w - r + need > capacity)
		{
			InterlockedIncrement64((volatile LONG64 *)dropped);
			InterlockedExchange((volatile LONG *)(base + 32), 1);
			enabled = false;
			osd_printf_error("MIDV render stream failed: consumer timeout; native presentation fallback\n");
			return false;
		}
		auto put = [&](uint64_t at, const void *src, uint32_t len)
		{
			uint32_t off = uint32_t(at % RING_SIZE);
			uint32_t first = std::min(len, RING_SIZE - off);
			memcpy(data + off, src, first);
			if (len > first)
				memcpy(data, (const uint8_t *)src + first, len - first);
		};
		uint32_t hdr[2] = { type, l1 + l2 };
		put(w, hdr, 8);
		put(w + 8, p1, l1);
		if (l2)
			put(w + 8 + l1, p2, l2);
		ring_store(wpos, w + need);
		return true;
	}

	void flush_span()
	{
		if (span_count == 0)
			return;
		uint32_t hdr[3] = { last_frame, span_start, span_count };
		write_msg(5, hdr, 12, span_data, span_count * 2);
		span_start = 0xffffffff;
		span_count = 0;
	}

	void vram_write(uint32_t frame, uint32_t offset, uint16_t value)
	{
		last_frame = frame;
		if (span_count > 0 &&
			(offset != span_start + span_count || span_count >= 2048))
			flush_span();
		if (span_count == 0)
			span_start = offset;
		span_data[span_count++] = value;
	}

	// call at any scene-ordered point to push pending big state
	void sync_state(uint32_t frame, const void *pal, uint32_t pal_len,
	                const void *tex, uint32_t tex_len)
	{
		flush_span();
		if (pal_dirty && write_msg(3, &frame, 4, pal, pal_len))
			pal_dirty = false;
		if (tex_dirty)
		{
			const void *upload = tex;
			if (ui_assets.active())
			{
				const uint8_t *source = static_cast<const uint8_t *>(tex);
				texture_upload.assign(source, source + tex_len);
				ui_assets.apply(texture_upload.data(), texture_upload.size());
				upload = texture_upload.data();
			}
			if (write_msg(4, &frame, 4, upload, tex_len))
				tex_dirty = false;   // stays dirty on drop; retried next scene
		}
	}
};

midv_live &live();

// ── MIDV_GL: in-process GL renderer (Phase 1 step 2) ─────────────────────
//
// MIDV_GL=1 spawns a render thread that consumes the same ring the external
// viewer uses, renders scenes with the verified shaders, and presents into a
// DISABLED overlay child window covering MAME's own window - so input,
// audio, FFB proxying and window focus all stay exactly as stock MAME.
// MIDV_GL_SCALE (default 3) sets internal scale; MIDV_GL_SNAP=<dir> writes a
// backbuffer BMP every ~150 presents for unattended verification.
// ---- UDP telemetry (Phase A): mirror MAME outputs as JSON datagrams -------
// MIDV_TELEM_UDP=host:port (or just port; default 127.0.0.1:20777).
// Consumers: SimHub custom UDP / Buttkicker pipelines. The "wheel" output
// carries the FFB force value each frame.
static SOCKET s_telem_sock = INVALID_SOCKET;
static sockaddr_in s_telem_addr;
static char s_telem_game[16];
static bool s_telem_json = false;   // JSON stream requested (MIDV_TELEM_UDP)

// ---- Forza-compatible telemetry (Phase C) ---------------------------------
// MIDV_TELEM_FORZA=host:port (default 127.0.0.1:5300): emit the Forza
// Horizon 4/5 "Data Out" 324-byte binary packet each frame, with our hunted
// speed (m/s) and RPM filled in. SimHub / dash apps / bass-shaker profiles
// then treat the game as Forza Horizon with zero custom configuration.
// comma-separated targets ("127.0.0.1:8000,127.0.0.1:8001") so SimHub and
// the forza_probe diagnostic can watch the same stream simultaneously
static sockaddr_in s_forza_addr[4];
static int s_forza_n = 0;
static uint32_t s_forza_ms = 0;

// "host:port" / "port" / null -> sockaddr (defaults preserved on null)
static void telem_parse_addr(const char *spec, sockaddr_in &out,
		const char *def_host, int def_port)
{
	char host[64];
	strncpy(host, def_host, sizeof(host) - 1);
	host[sizeof(host) - 1] = 0;
	int port = def_port;
	if (spec && *spec)
	{
		const char *c = strchr(spec, ':');
		if (c)
		{
			size_t n = std::min(size_t(c - spec), sizeof(host) - 1);
			memcpy(host, spec, n);
			host[n] = 0;
			port = atoi(c + 1);
		}
		else if (atoi(spec) > 0)
			port = atoi(spec);
	}
	memset(&out, 0, sizeof(out));
	out.sin_family = AF_INET;
	out.sin_port = htons(uint16_t(port));
	out.sin_addr.s_addr = inet_addr(host);
}

// FFB diagnostics: MIDV_FFB_TRACE=<file> appends every output change the
// game makes (wheel force, lamps) with a millisecond timestamp - the
// discriminator for "force feedback comes and goes": a continuous stream
// of changing wheel values while the wheel is quiet points at the plugin/
// driver side (FFBlog.txt, Logging=1); gaps in the stream point at the
// emulation/output side. Env-gated; inert unset.
static FILE *s_ffb_trace = nullptr;
static std::chrono::steady_clock::time_point s_ffb_trace_t0;

static void telem_notify(const char *outname, s32 value, void *)
{
	if (s_ffb_trace)
	{
		auto const ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::steady_clock::now() - s_ffb_trace_t0).count();
		fprintf(s_ffb_trace, "%lld,%s,%d\n", (long long)ms,
				outname ? outname : "", int(value));
		fflush(s_ffb_trace);
	}
	if (s_telem_sock == INVALID_SOCKET || !s_telem_json)
		return;
	char buf[160];
	int n = snprintf(buf, sizeof(buf),
			"{\"game\":\"%s\",\"out\":\"%s\",\"value\":%d}\n",
			s_telem_game, outname ? outname : "", int(value));
	sendto(s_telem_sock, buf, n, 0,
			(const sockaddr *)&s_telem_addr, sizeof(s_telem_addr));
}

// ---- HUD OCR speed (Phase B final form) -----------------------------------
// USA numeric HUD text is now traced; other games still use OCR (see RESULTS.md).
// the HUD digits are CPU-blitted into videoram. So read them back: decode
// the MPH box glyphs from the visible page each frame against the baked
// templates (validated offline: 822/823 agreement with the calibration
// OCR). Correct by construction - it reads exactly what the player reads.
// Per-game box coords (absolute videoram rows/cols); 0-width = not
// calibrated yet (speed stays 0 for that game).
struct HudBox { const char *game; int x0, x1, y0, y1; };
static const HudBox s_hud_box[] = {
	// crusnusa: x0 was 30 and CLIPPED the hundreds digit. The reader s own
	// diagnostics settled it - leftmost lit column sat exactly on 30 in
	// 2758 samples (content cut off at the edge), only 112 reads ever
	// segmented 3 cells, and the speed ceiling was 98 mph in every capture.
	// Swept x0 with MIDV_HUD_X0: 30 -> max 88; 18 -> max 141 with 15x the
	// three-digit reads and no extra failures; 12 pulls in the road drawn
	// behind the HUD and breaks every read. 18 it is.
	{ "crusnusa", 18, 72, 347, 370 },
	// crusnwld: calibrated offline from the 2026-08-25 drive captures -
	// same digit font as crusnusa (USA templates read the World digits
	// as-is). Re-checked 2026-09-05 after USA turned out to be clipping
	// its hundreds digit: this box is NOT clipped. It reads to 142 mph
	// with 1198 three-digit reads in two minutes, and moving x0 left of
	// 14 breaks every read. The old "0->97" note was that drive's top
	// speed, not a ceiling.
	{ "crusnwld", 14, 76, 342, 368 },
	// offroadc: STILL PROVISIONAL AND STILL WRONG (checked 2026-09-05).
	// A 120 s run reads a single lit cell nearly every frame and every
	// value is rejected, so this game has never reported a speed. Moving
	// x0 does not help - the box is not on the digits at all, and a hunt
	// across a dumped frame found no 2-3 digit cluster near these coords.
	// Needs the same calibration USA and World got: a screenshot of the
	// speedo while driving, then coords read off it.
	// offroadc: PROVISIONAL - the MPH box sits at the TOP of the screen
	// (outside the old dump window), coords derived from the user's
	// 4x driving screenshot geometry; the OCR rejects unreadable frames
	// so a miss emits 0, never garbage. Validate from the next drive
	// capture (the dumper below now grabs rows 0-99 for this game).
	{ "offroadc", 228, 270, 22, 52 },
	{ nullptr, 0, 0, 0, 0 },
};
static const HudBox *s_hud = nullptr;   // resolved at telem_init
static cruisn::HudSpeedFilter s_hud_speed;
static int s_numeric_mph[2] = { -1, -1 };
static uint64_t s_numeric_frame[2] = { 0, 0 };
static double s_numeric_seconds[2] = { 0, 0 };
static bool s_numeric_seen[2] = { false, false };

static int hud_ocr_digit(const float *cell, int ch, int cw)
{
	// bilinear resample to 12x18, L2-match the baked templates
	float g[18][12];
	for (int oy = 0; oy < 18; oy++)
	{
		float fy = (oy + 0.5f) * ch / 18.0f - 0.5f;
		int y0 = int(std::floor(fy));
		float ty = fy - y0;
		int y0c = std::min(std::max(y0, 0), ch - 1), y1c = std::min(y0c + 1, ch - 1);
		for (int ox = 0; ox < 12; ox++)
		{
			float fx = (ox + 0.5f) * cw / 12.0f - 0.5f;
			int x0 = int(std::floor(fx));
			float tx = fx - x0;
			int x0c = std::min(std::max(x0, 0), cw - 1), x1c = std::min(x0c + 1, cw - 1);
			g[oy][ox] = (1 - ty) * ((1 - tx) * cell[y0c * cw + x0c] + tx * cell[y0c * cw + x1c])
			          + ty * ((1 - tx) * cell[y1c * cw + x0c] + tx * cell[y1c * cw + x1c]);
		}
	}
	int bestk = -1;
	float bestd = 1e9f;
	for (int k = 0; k < 10; k++)
	{
		float acc = 0;
		for (int r = 0; r < 18; r++)
			for (int c = 0; c < 12; c++)
			{
				float dd = g[r][c] / 255.0f - s_hud_digit_tmpl[k][r][c] / 255.0f;
				acc += dd * dd;
			}
		acc /= 18 * 12;
		if (acc < bestd) { bestd = acc; bestk = k; }
	}
	return (bestd <= 0.035f) ? bestk : -1;
}

// returns displayed MPH, or -1 when the box is absent/unreadable
// Diagnostics for the reader itself: how many digit cells the last call
// segmented, and how far left the leftmost lit column reached. A speed
// that reads two digits when the game is showing three is the difference
// between "the reader is fine" and "the window is too narrow", and no
// amount of staring at the output can tell them apart.
// Experiment knobs for the OCR window, so a sweep does not need a rebuild.
// MIDV_HUD_X0 moves the left edge, MIDV_HUD_THR the "lit" threshold.
static float hud_lit_threshold()
{
	static float v = -1.0f;
	if (v < 0.0f)
	{
		const char *e = std::getenv("MIDV_HUD_THR");
		v = e ? float(atof(e)) : 110.0f;
	}
	return v;
}

static int s_hud_cells = 0;
static int s_hud_leftcol = -1;

static int hud_ocr_mph(const uint16_t *videoram, uint16_t page_control,
		const HudBox *box)
{
	uint32_t const base = (page_control & 1) ? 0x40000 : 0x00000;
	HudBox tuned = *box;
	if (const char *e = std::getenv("MIDV_HUD_X0"))
	{
		int const x0 = atoi(e);
		if (x0 >= 0 && x0 < tuned.x1 - 8) tuned.x0 = x0;
	}
	box = &tuned;
	int const W = box->x1 - box->x0, H = box->y1 - box->y0;
	if (W <= 0 || W > 64 || H <= 0 || H > 32)
		return -1;
	float win[32][64];
	bool coloncol[64];
	for (int x = 0; x < W; x++) coloncol[x] = false;
	for (int y = 0; y < H; y++)
		for (int x = 0; x < W; x++)
		{
			float v = float(videoram[base + (box->y0 + y) * 512 + box->x0 + x] & 0xff);
			win[y][x] = v;
					if (v > hud_lit_threshold()) coloncol[x] = true;
		}
	s_hud_leftcol = -1;
	for (int x = 0; x < W; x++)
		if (coloncol[x]) { s_hud_leftcol = box->x0 + x; break; }
	// segment cells: lit column runs, gaps >=2 split, min width 2
	int cells[4][2];
	int ncell = 0, s0 = -1, gap = 0;
	for (int x = 0; x < W && ncell < 4; x++)
	{
		if (coloncol[x]) { if (s0 < 0) s0 = x; gap = 0; }
		else if (s0 >= 0 && ++gap >= 2)
		{
			if (x - gap - s0 + 1 >= 2) { cells[ncell][0] = s0; cells[ncell][1] = x - gap + 1; ncell++; }
			s0 = -1; gap = 0;
		}
	}
	if (s0 >= 0 && W - s0 >= 2 && ncell < 4) { cells[ncell][0] = s0; cells[ncell][1] = W; ncell++; }
	if (ncell < 1 || ncell > 3)
		return -1;
	int value = 0;
	s_hud_cells = ncell;
	for (int ci = 0; ci < ncell; ci++)
	{
		int a = cells[ci][0], b = cells[ci][1];
		// row bounds of lit pixels
		int r0 = -1, r1 = -1;
		for (int y = 0; y < H; y++)
			for (int x = a; x < b; x++)
				if (win[y][x] > hud_lit_threshold()) { if (r0 < 0) r0 = y; r1 = y; }
		if (r0 < 0 || r1 - r0 + 1 < 12)
			return -1;
		int const ch = r1 - r0 + 1, cw = b - a;
		float cell[32 * 64];
		for (int y = 0; y < ch; y++)
			for (int x = 0; x < cw; x++)
				cell[y * cw + x] = win[r0 + y][a + x];
		int d = hud_ocr_digit(cell, ch, cw);
		if (d < 0)
			return -1;
		value = value * 10 + d;
	}
	return value;
}

// Telemetry Phase B: per-game DSP-RAM word address of the car speed (MPH,
// TMS320C3x float), found by differential RAM-hunt over a demo-race
// acceleration (results/ramhunt; see RESULTS.md). 0 = not yet hunted.
// crusnusa 0x0F22D CONFIRMED (0->~295 monotone accel, resets per demo lap).
struct SpeedAddr { const char *game; uint32_t addr; };
// SPEED IS CURRENTLY OFF FOR ALL GAMES (all zero): the HUD-OCR ground-truth
// investigation (2026-08-23, RESULTS.md) proved the earlier attract-hunted
// addresses (crusnusa 0x0F22D, crusnwld 0x0DDDC) track DRONE cars, not the
// player - attract demos are drone-driven, so shape-hunting them lies. The
// player's physics speed was not identified in the earlier searches (the dumped
// memory: both external banks + C31 internal RAM, all decodings, r<0.55).
// The replacement is the HUD-quad DMA tap (reads the MPH digits at the
// source); until it lands, emitting 0 beats emitting another car.
static const SpeedAddr s_speed_addr[] = {
	{ "crusnusa", 0 },
	{ "crusnwld", 0 },
	{ "offroadc", 0 },
	{ nullptr, 0 },
};
static uint32_t s_speed_word = 0;   // resolved at telem_init

// No validated engine-RPM producer exists yet. USA RAM word E632 is packed
// decimal speed text, as established by its formatter and HUD submissions.
// The retired low-16-bit correlation was not an RPM measurement.

// TMS320C3x 32-bit float -> host float: [exp 8b two's-comp][sign][frac 23b]
static float c3x_to_float(uint32_t w)
{
	int e = (w >> 24) & 0xff;
	int s = (w >> 23) & 1;
	float frac = float(w & 0x7fffff) / float(1 << 23);
	if (e == 0 && s == 0 && frac == 0.0f)
		return 0.0f;
	int exp = (e < 0x80) ? e : e - 256;
	return ((s ? -2.0f : 1.0f) + frac) * std::ldexp(1.0f, exp);
}

static void telem_init(const char *spec, const char *game)
{
	// clone-tolerant match: crusnwld24 (the shifter rev) etc. share the
	// parent's screen-space HUD box; RAM-address rows are all 0 for World
	// so nothing rev-specific can misapply.
	auto game_match = [game](const char *entry) {
		return strncmp(entry, game, strlen(entry)) == 0;
	};
	s_speed_word = 0;
	for (const SpeedAddr *p = s_speed_addr; p->game; ++p)
		if (game_match(p->game)) { s_speed_word = p->addr; break; }
	s_hud = nullptr;
	s_hud_speed.reset();
	for (const HudBox *p = s_hud_box; p->game; ++p)
		if (game_match(p->game)) { s_hud = p; break; }

	strncpy(s_telem_game, game, sizeof(s_telem_game) - 1);
	// Source selection also serves file-only diagnostics. Opening a trace must
	// not require a UDP destination, nor silently leave OCR uninitialized.
	if (!spec && !std::getenv("MIDV_TELEM_FORZA"))
		return;
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		return;
	SOCKET s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s == INVALID_SOCKET)
		return;
	s_telem_json = (spec != nullptr);
	telem_parse_addr(spec, s_telem_addr, "127.0.0.1", 20777);
	if (const char *fz = std::getenv("MIDV_TELEM_FORZA"))
	{
		char buf[256];
		strncpy(buf, fz, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = 0;
		char *save = nullptr;
		for (char *tok = strtok_s(buf, ",", &save);
				tok && s_forza_n < 4;
				tok = strtok_s(nullptr, ",", &save))
			telem_parse_addr(tok, s_forza_addr[s_forza_n++], "127.0.0.1", 5300);
	}
	strncpy(s_telem_game, game, sizeof(s_telem_game) - 1);
	s_telem_sock = s;
}

namespace mvgl {

// teardown handshake: machine exit flags the GL thread down and waits for
// its acknowledgement (see midvunit_base_state::mvgl_exit)
static std::atomic<bool> s_stop{false};
static std::atomic<bool> s_done{false};
// Esc-menu pause request: set by the GL thread, acted on by the emu
// thread in screen_update (pause/resume are not thread-safe from here)
static std::atomic<int> s_menu_pause{0};

// ---- minimal dynamic GL loader (no link-time deps beyond user32/gdi32) ----
#define MVGL_E(n, v) constexpr unsigned n = v;
MVGL_E(FRAGMENT_SHADER, 0x8B30) MVGL_E(VERTEX_SHADER, 0x8B31)
MVGL_E(COMPILE_STATUS, 0x8B81) MVGL_E(LINK_STATUS, 0x8B82)
MVGL_E(ARRAY_BUFFER, 0x8892) MVGL_E(STREAM_DRAW, 0x88E0)
MVGL_E(FRAMEBUFFER, 0x8D40) MVGL_E(COLOR_ATTACHMENT0, 0x8CE0)
MVGL_E(FRAMEBUFFER_COMPLETE, 0x8CD5) MVGL_E(TEXTURE0, 0x84C0)
MVGL_E(R16UI, 0x8234) MVGL_E(R8UI, 0x8232) MVGL_E(R32UI, 0x8236)
MVGL_E(RED_INTEGER, 0x8D94)
#undef MVGL_E

typedef unsigned uint;
typedef ptrdiff_t glsizeiptr;
struct GL
{
	HMODULE dll = nullptr;
	// wgl
	HGLRC (WINAPI *CreateContext)(HDC);
	BOOL (WINAPI *DeleteContext)(HGLRC);
	BOOL (WINAPI *MakeCurrent)(HDC, HGLRC);
	PROC (WINAPI *GetProc)(LPCSTR);
	HGLRC (WINAPI *CreateContextAttribs)(HDC, HGLRC, const int *);
	BOOL (WINAPI *SwapIntervalEXT)(int);
	// gl 1.1 (from opengl32.dll directly)
	void (WINAPI *Viewport)(int, int, int, int);
	void (WINAPI *ClearColor)(float, float, float, float);
	void (WINAPI *Clear)(unsigned);
	void (WINAPI *GenTextures)(int, uint *);
	void (WINAPI *BindTexture)(unsigned, uint);
	void (WINAPI *TexParameteri)(unsigned, unsigned, int);
	void (WINAPI *TexImage2D)(unsigned, int, int, int, int, int, unsigned, unsigned, const void *);
	void (WINAPI *TexSubImage2D)(unsigned, int, int, int, int, int, unsigned, unsigned, const void *);
	void (WINAPI *PixelStorei)(unsigned, int);
	void (WINAPI *DrawArrays)(unsigned, int, int);
	void (WINAPI *Enable)(unsigned);
	void (WINAPI *Disable)(unsigned);
	void (WINAPI *Scissor)(int, int, int, int);
	void (WINAPI *ReadPixels)(int, int, int, int, unsigned, unsigned, void *);
	unsigned (WINAPI *GetError)();
	// modern (via wglGetProcAddress)
	uint (WINAPI *CreateShader)(unsigned);
	void (WINAPI *ShaderSource)(uint, int, const char *const *, const int *);
	void (WINAPI *CompileShader)(uint);
	void (WINAPI *GetShaderiv)(uint, unsigned, int *);
	void (WINAPI *GetShaderInfoLog)(uint, int, int *, char *);
	uint (WINAPI *CreateProgram)();
	void (WINAPI *AttachShader)(uint, uint);
	void (WINAPI *LinkProgram)(uint);
	void (WINAPI *GetProgramiv)(uint, unsigned, int *);
	void (WINAPI *GetProgramInfoLog)(uint, int, int *, char *);
	void (WINAPI *UseProgram)(uint);
	int (WINAPI *GetUniformLocation)(uint, const char *);
	void (WINAPI *Uniform1i)(int, int);
	void (WINAPI *Uniform1f)(int, float);
	void (WINAPI *Uniform2f)(int, float, float);
	void (WINAPI *Uniform4f)(int, float, float, float, float);
	void (WINAPI *BlendFunc)(unsigned, unsigned);
	int (WINAPI *GetAttribLocation)(uint, const char *);
	void (WINAPI *GenBuffers)(int, uint *);
	void (WINAPI *BindBuffer)(unsigned, uint);
	void (WINAPI *BufferData)(unsigned, glsizeiptr, const void *, unsigned);
	void (WINAPI *BufferSubData)(unsigned, glsizeiptr, glsizeiptr, const void *);
	void (WINAPI *GenVertexArrays)(int, uint *);
	void (WINAPI *BindVertexArray)(uint);
	void (WINAPI *EnableVertexAttribArray)(uint);
	void (WINAPI *VertexAttribPointer)(uint, int, unsigned, unsigned char, int, const void *);
	void (WINAPI *VertexAttribIPointer)(uint, int, unsigned, int, const void *);
	void (WINAPI *GenFramebuffers)(int, uint *);
	void (WINAPI *BindFramebuffer)(unsigned, uint);
	void (WINAPI *FramebufferTexture2D)(unsigned, unsigned, unsigned, uint, int);
	unsigned (WINAPI *CheckFramebufferStatus)(unsigned);
	void (WINAPI *ActiveTexture)(unsigned);
	void (WINAPI *DrawBuffers)(int, const unsigned *);
	void (WINAPI *ClearBufferuiv)(unsigned, int, const uint *);

	template <typename T> void load1(T &fn, const char *name)
	{
		fn = (T)(void *)GetProcAddress(dll, name);
		if (!fn)
			fn = (T)(void *)GetProc(name);
	}
	bool load_base()
	{
		dll = LoadLibraryA("opengl32.dll");
		if (!dll) return false;
		CreateContext = (decltype(CreateContext))(void *)GetProcAddress(dll, "wglCreateContext");
		DeleteContext = (decltype(DeleteContext))(void *)GetProcAddress(dll, "wglDeleteContext");
		MakeCurrent = (decltype(MakeCurrent))(void *)GetProcAddress(dll, "wglMakeCurrent");
		GetProc = (decltype(GetProc))(void *)GetProcAddress(dll, "wglGetProcAddress");
		return CreateContext && MakeCurrent && GetProc;
	}
	bool load_rest()
	{
#define L(f, n) load1(f, n); if (!(f)) return false;
		L(Viewport, "glViewport") L(ClearColor, "glClearColor") L(Clear, "glClear")
		L(GenTextures, "glGenTextures") L(BindTexture, "glBindTexture")
		L(TexParameteri, "glTexParameteri") L(TexImage2D, "glTexImage2D")
		L(TexSubImage2D, "glTexSubImage2D") L(PixelStorei, "glPixelStorei")
		L(DrawArrays, "glDrawArrays") L(Enable, "glEnable")
		L(Disable, "glDisable") L(Scissor, "glScissor") L(ReadPixels, "glReadPixels") L(GetError, "glGetError")
		L(CreateShader, "glCreateShader") L(ShaderSource, "glShaderSource")
		L(CompileShader, "glCompileShader") L(GetShaderiv, "glGetShaderiv")
		L(GetShaderInfoLog, "glGetShaderInfoLog") L(CreateProgram, "glCreateProgram")
		L(AttachShader, "glAttachShader") L(LinkProgram, "glLinkProgram")
		L(GetProgramiv, "glGetProgramiv") L(GetProgramInfoLog, "glGetProgramInfoLog")
		L(UseProgram, "glUseProgram") L(GetUniformLocation, "glGetUniformLocation")
		L(Uniform1i, "glUniform1i") L(Uniform1f, "glUniform1f")
		L(Uniform2f, "glUniform2f") L(Uniform4f, "glUniform4f")
		L(BlendFunc, "glBlendFunc")
		L(GetAttribLocation, "glGetAttribLocation") L(GenBuffers, "glGenBuffers")
		L(BindBuffer, "glBindBuffer") L(BufferData, "glBufferData")
		L(BufferSubData, "glBufferSubData") L(GenVertexArrays, "glGenVertexArrays")
		L(BindVertexArray, "glBindVertexArray")
		L(EnableVertexAttribArray, "glEnableVertexAttribArray")
		L(VertexAttribPointer, "glVertexAttribPointer")
		L(VertexAttribIPointer, "glVertexAttribIPointer")
		L(GenFramebuffers, "glGenFramebuffers") L(BindFramebuffer, "glBindFramebuffer")
		L(FramebufferTexture2D, "glFramebufferTexture2D")
		L(CheckFramebufferStatus, "glCheckFramebufferStatus")
		L(ActiveTexture, "glActiveTexture")
		L(DrawBuffers, "glDrawBuffers") L(ClearBufferuiv, "glClearBufferuiv")
#undef L
		load1(CreateContextAttribs, "wglCreateContextAttribsARB");
		load1(SwapIntervalEXT, "wglSwapIntervalEXT");
		return true;
	}
};

// ---- CPU-side quad -> vertex building (scalar port of the verified path) ----
struct QuadMsg { uint32_t frame; uint16_t pc, pad; uint16_t dma[16]; };

static void make_inclusive(float *vx, float *vy)
{
	int rmask = 0, bmask = 0, eqmask = 0;
	for (int v = 0; v < 4; v++)
	{
		int n = (v + 1) & 3;
		if (vy[n] == vy[v] && vx[n] == vx[v]) eqmask |= 1 << v;
		if (vy[n] > vy[v] || (vy[n] == vy[v] && vx[n] < vx[v])) rmask |= 1 << v;
		if (vx[n] < vx[v] || (vx[n] == vx[v] && vy[n] < vy[v])) bmask |= 1 << v;
	}
	if (eqmask == 0x0f) return;
	for (int v = 0; v < 4; v++)
	{
		int eff = v;
		while (eqmask & (1 << eff)) eff = (eff + 1) & 3;
		if (rmask & (1 << eff)) vx[v] += 0.001f;
		if (bmask & (1 << eff)) vy[v] += 0.001f;
	}
}

// Half-pixel outward dilation for strict axis-aligned rectangles (port of
// renderer.py _dilate_rect - keep in sync). Quality mode's continuous
// coverage ends at the vertex CENTERS, so adjacent 2D tiles each
// half-cover their shared boundary columns and the layer underneath
// grooves through the seam (crusnusa continue-map vertical line;
// offroadc track-select lines, G4). The hardware DDA fills endpoint
// pixels inclusively; expanding each side to the pixel's OUTER edge
// (0.5 + 0.001 tie guard) reproduces that span exactly - adjacent tiles
// then partition the fine pixels with no gap and no overlap.
// us/vs: the texture params are extrapolated along each axis by the
// same amount so du/dx, dv/dy - and thus which texel every fine pixel
// samples - stay the hardware's (moving vertices alone squeezed each
// tile's texture inward by up to half a texel: review 2026-08-30).
static void dilate_rect(float *vx, float *vy, const int16_t *ix, const int16_t *iy,
	float *us, float *vs)
{
	constexpr float E = 0.501f;
	int sx[2][2], sy[2][2], px[2][2], py[2][2];   // sides, same-row pairs
	if (ix[0] == ix[1] && ix[2] == ix[3] && iy[1] == iy[2] && iy[3] == iy[0])
	{
		sx[0][0] = 0; sx[0][1] = 1; sx[1][0] = 2; sx[1][1] = 3;
		sy[0][0] = 1; sy[0][1] = 2; sy[1][0] = 3; sy[1][1] = 0;
		px[0][0] = 0; px[0][1] = 3; px[1][0] = 1; px[1][1] = 2;
		py[0][0] = 1; py[0][1] = 0; py[1][0] = 2; py[1][1] = 3;
	}
	else if (iy[0] == iy[1] && iy[2] == iy[3] && ix[1] == ix[2] && ix[3] == ix[0])
	{
		sx[0][0] = 3; sx[0][1] = 0; sx[1][0] = 1; sx[1][1] = 2;
		sy[0][0] = 0; sy[0][1] = 1; sy[1][0] = 2; sy[1][1] = 3;
		px[0][0] = 0; px[0][1] = 1; px[1][0] = 3; px[1][1] = 2;
		py[0][0] = 0; py[0][1] = 3; py[1][0] = 1; py[1][1] = 2;
	}
	else
		return;
	auto extrap = [&](float *v, int (*pairs)[2], float e)
	{
		for (int k = 0; k < 2; k++)
		{
			int const p = pairs[k][0], q = pairs[k][1];   // p moves -e, q +e
			float const span = v[q] - v[p];
			if (span == 0.0f) continue;
			float const gu = (us[q] - us[p]) / span, gv = (vs[q] - vs[p]) / span;
			us[p] -= e * gu; us[q] += e * gu;
			vs[p] -= e * gv; vs[q] += e * gv;
		}
	};
	float ex = (ix[sx[0][0]] <= ix[sx[1][0]]) ? E : -E;
	extrap(vx, px, ex);
	vx[sx[0][0]] -= ex; vx[sx[0][1]] -= ex;
	vx[sx[1][0]] += ex; vx[sx[1][1]] += ex;
	float ey = (iy[sy[0][0]] <= iy[sy[1][0]]) ? E : -E;
	extrap(vy, py, ey);
	vy[sy[0][0]] -= ey; vy[sy[0][1]] -= ey;
	vy[sy[1][0]] += ey; vy[sy[1][1]] += ey;
}

static size_t build_vertices(const std::vector<QuadMsg> &quads, float xoff,
	std::vector<float> &fdata, std::vector<uint32_t> &udata, bool align_joins)
{
	cruisn::JoinResult joined;
	if (align_joins) joined = cruisn::align_tjunctions(quads.size(), [&](size_t q) { return quads[q].dma; });
	fdata.resize(quads.size() * 6 * 22);
	udata.resize(quads.size() * 6 * 4);
	for (size_t q = 0; q < quads.size(); q++)
	{
		const uint16_t *dma = quads[q].dma;
		float vx[4], vy[4], us[4] = {}, vs[4] = {};
		for (int i = 0; i < 4; i++)
		{
			vx[i] = float(int16_t(dma[2 + i * 2])) + 0.5f + xoff;
			vy[i] = float(int16_t(dma[3 + i * 2])) + 0.5f;
		}
		if (align_joins) for (int i=0;i<4;++i)
		{
			vx[i]=joined.positions[q][i*2]+0.5f+xoff;
			vy[i]=joined.positions[q][i*2+1]+0.5f;
		}
		uint32_t pixdata = dma[1];
		bool const textured = (dma[0] & 0x300) == 0x100;
		// dither bit0; bit1 flags backdrop (sky/horizon band) so the shader
		// can suppress it in the 16:9 margins. Per-game texbase low byte
		// (offroadc 0x7f, crusnusa 0x56, crusnwld 0xc5) AND full-width -
		// excludes incidental small quads; verified no terrain false hits.
		int16_t const bx0 = int16_t(dma[2]), bx1 = int16_t(dma[4]),
			bx2 = int16_t(dma[6]), bx3 = int16_t(dma[8]);
		int const bxmin = std::min(std::min(bx0, bx1), std::min(bx2, bx3));
		int const bxmax = std::max(std::max(bx0, bx1), std::max(bx2, bx3));
		uint32_t const blo = dma[14] & 0xff;
		bool const backdrop = (blo == 0x7f || blo == 0x56 || blo == 0xc5)
			&& (bxmax - bxmin) > 200;
		// bit2: parked screen-space UI - untextured panels the game slides
		// in from past the 4:3 edge (crusnwld radio) PARK fully off-screen
		// where the hardware raster crop hid them; the shader never draws
		// the parked position (wide builds only: xoff > 0). Sliding or
		// deployed quads straddle x=511 and render normally.
		bool parked = false;
		if (xoff > 0.0f && !textured && bxmin >= 512)
		{
			int16_t const by0 = int16_t(dma[3]), by1 = int16_t(dma[5]),
				by2 = int16_t(dma[7]), by3 = int16_t(dma[9]);
			int const bymin = std::min(std::min(by0, by1), std::min(by2, by3));
			int const bymax = std::max(std::max(by0, by1), std::max(by2, by3));
			// dithered panel or tiny indicator dot only - moving untextured
			// margin objects (USA traffic shadows/LOD, ~20px) must stay
			bool const small = (bxmax - bxmin) <= 8 && (bymax - bymin) <= 8;
			parked = bymin >= 60 && bymax <= 260
				&& (((dma[0] & 0x2000) != 0) || small);
		}
		uint32_t const dither = ((dma[0] & 0x2000) ? 1u : 0u)
			| (backdrop ? 2u : 0u) | (parked ? 4u : 0u);
		uint32_t mode = 0;
		if (!textured)
			pixdata = (pixdata + (dma[0] & 0xff)) & 0xffff;
		else
		{
			for (int i = 0; i < 4; i++)
			{
				us[i] = float(dma[10 + i] & 0xff) * 65536.0f + 32768.0f;
				vs[i] = float(dma[10 + i] >> 8) * 65536.0f + 32768.0f;
			}
			switch (dma[0] & 0xc00)
			{
			case 0x000: mode = 1; break;
			case 0x800: mode = 2; break;
			case 0xc00: mode = 3; pixdata = (pixdata + (dma[0] & 0xff)) & 0xffff; break;
			default: mode = 0; pixdata = (pixdata + (dma[0] & 0xff)) & 0xffff; break;
			}
		}
		float const bounds[4] = {
			*std::min_element(us, us + 4), *std::max_element(us, us + 4),
			*std::min_element(vs, vs + 4), *std::max_element(vs, vs + 4) };
		make_inclusive(vx, vy);
		{
			// the live overlay is always quality mode; exact/DDA replay
			// (renderer.py) applies this only outside exact mode
			int16_t const ix[4] = { int16_t(dma[2]), int16_t(dma[4]),
				int16_t(dma[6]), int16_t(dma[8]) };
			int16_t const iy[4] = { int16_t(dma[3]), int16_t(dma[5]),
				int16_t(dma[7]), int16_t(dma[9]) };
			dilate_rect(vx, vy, ix, iy, us, vs);
		}
		float x0 = vx[0], x1 = vx[0], y0 = vy[0], y1 = vy[0];
		for (int i = 1; i < 4; i++)
		{
			x0 = std::min(x0, vx[i]); x1 = std::max(x1, vx[i]);
			y0 = std::min(y0, vy[i]); y1 = std::max(y1, vy[i]);
		}
		x0 -= 1.0f; x1 += 1.0f; y0 -= 1.0f; y1 += 1.0f;
		const float cx[6] = { x0, x1, x1, x0, x1, x0 };
		const float cy[6] = { y0, y0, y1, y0, y1, y1 };
		for (int k = 0; k < 6; k++)
		{
			float *f = &fdata[(q * 6 + k) * 22];
			f[0] = cx[k]; f[1] = cy[k];
			for (int i = 0; i < 4; i++) { f[2 + i * 2] = vx[i]; f[3 + i * 2] = vy[i]; }
			f[10] = us[0]; f[11] = vs[0]; f[12] = us[1]; f[13] = vs[1];
			f[14] = us[2]; f[15] = vs[2]; f[16] = us[3]; f[17] = vs[3];
			for (int i = 0; i < 4; i++) f[18 + i] = bounds[i];
			uint32_t *u = &udata[(q * 6 + k) * 4];
			u[0] = pixdata; u[1] = mode; u[2] = dither; u[3] = uint32_t(dma[14]) * 256;
		}
	}
	return joined.aligned;
}

// ---- the render thread ----
// HEIGHT is the default coarse height; offroadc runs a 512x401 mode, so
// the launcher passes MIDV_GL_HEIGHT=401 and statics size for MAXH
constexpr int MARGIN_DEFAULT = 86, HEIGHT = 400, MAXH = 401;
constexpr float PAR = 1.0417f;

static FILE *s_log;
static void logf(const char *fmt, ...)
{
	if (!s_log) return;
	va_list ap; va_start(ap, fmt);
	vfprintf(s_log, fmt, ap);
	fprintf(s_log, "\n");
	fflush(s_log);
	va_end(ap);
}

static HWND find_mame_window()
{
	struct Ctx { DWORD pid; HWND found; } ctx{ GetCurrentProcessId(), nullptr };
	EnumWindows([](HWND h, LPARAM lp) -> BOOL
	{
		Ctx &c = *(Ctx *)lp;
		DWORD pid = 0;
		GetWindowThreadProcessId(h, &pid);
		if (pid != c.pid || !IsWindowVisible(h)) return TRUE;
		char cls[64] = {};
		GetClassNameA(h, cls, 63);
		if (strcmp(cls, "MAME") == 0) { c.found = h; return FALSE; }
		return TRUE;
	}, (LPARAM)&ctx);
	return ctx.found;
}

static uint compile(GL &gl, unsigned type, const char *src)
{
	uint sh = gl.CreateShader(type);
	gl.ShaderSource(sh, 1, &src, nullptr);
	gl.CompileShader(sh);
	int ok = 0;
	gl.GetShaderiv(sh, COMPILE_STATUS, &ok);
	if (!ok)
	{
		char buf[4096]; int n = 0;
		gl.GetShaderInfoLog(sh, 4095, &n, buf);
		logf("shader compile failed:\n%.*s", n, buf);
		return 0;
	}
	return sh;
}

static uint link(GL &gl, const char *vs, const char *fs)
{
	uint p = gl.CreateProgram();
	uint v = compile(gl, VERTEX_SHADER, vs);
	uint f = compile(gl, FRAGMENT_SHADER, fs);
	if (!v || !f) return 0;
	gl.AttachShader(p, v);
	gl.AttachShader(p, f);
	gl.LinkProgram(p);
	int ok = 0;
	gl.GetProgramiv(p, LINK_STATUS, &ok);
	if (!ok)
	{
		char buf[4096]; int n = 0;
		gl.GetProgramInfoLog(p, 4095, &n, buf);
		logf("link failed:\n%.*s", n, buf);
		return 0;
	}
	return p;
}

void thread_main()
{
	// every exit path must acknowledge shutdown or mvgl_exit stalls 1 s
	struct DoneGuard { ~DoneGuard() { s_done.store(true); } } done_guard;
	midv_live &lv = live();
	if (std::getenv("MIDV_GL_LOG"))
		s_log = fopen("midv_gl.log", "w");
	int const S = std::getenv("MIDV_GL_SCALE") ? atoi(std::getenv("MIDV_GL_SCALE")) : 3;
	const char *snapdir = std::getenv("MIDV_GL_SNAP");
	int const snap_every = std::getenv("MIDV_GL_SNAP_EVERY")
		? std::max(1, atoi(std::getenv("MIDV_GL_SNAP_EVERY"))) : 150;
	int const snap_first = std::getenv("MIDV_GL_SNAP_FIRST")
		? std::max(0, atoi(std::getenv("MIDV_GL_SNAP_FIRST"))) : 0;
	int const snap_last = std::getenv("MIDV_GL_SNAP_LAST")
		? atoi(std::getenv("MIDV_GL_SNAP_LAST")) : -1;
	int const snap_max = std::getenv("MIDV_GL_SNAP_MAX")
		? std::max(0, atoi(std::getenv("MIDV_GL_SNAP_MAX"))) : 0;
	int H = std::getenv("MIDV_GL_HEIGHT") ? atoi(std::getenv("MIDV_GL_HEIGHT")) : HEIGHT;
	if (H < HEIGHT || H > MAXH)
		H = HEIGHT;
	// 16:9 margin width per side, runtime (MIDV_GL_MARGIN). 86 = full
	// widescreen; a smaller value trims the outer margin where a game's
	// backdrop/water plane is legitimately drawn but reads as an artifact
	// (offroadc's canyon-edge water in the bottom corners). 0 = pure 4:3.
	int const MARGIN = std::getenv("MIDV_GL_MARGIN")
		? std::max(0, std::min(86, atoi(std::getenv("MIDV_GL_MARGIN"))))
		: MARGIN_DEFAULT;
	int const WIDE = 512 + 2 * MARGIN;
	int const fw = WIDE * S, fh = H * S;

	// wait for MAME's window
	HWND parent = nullptr;
	for (int i = 0; i < 100 && !parent; i++) { Sleep(100); parent = find_mame_window(); }
	if (!parent) { logf("no MAME window found"); return; }

	// Owned top-level popup, NOT a child: MAME's gdi renderer caches its
	// window DC, so child-clipping (WS_CLIPCHILDREN, even with
	// SWP_FRAMECHANGED) never reaches it and its blit punches through the
	// overlay - seen at the rig as alternating stretched/letterboxed frames.
	// A separate owned window is composited by DWM and occludes the owner
	// absolutely. WS_EX_NOACTIVATE keeps keyboard focus and DirectInput
	// foreground on MAME's window; the wndproc hides the cursor over the
	// game and turns clicks into refocus-the-owner (the earlier
	// DISABLED+TRANSPARENT combination made every click a Windows error
	// beep - EX_TRANSPARENT does not pass hit-tests without EX_LAYERED).
	WNDCLASSA wc = {};
	wc.lpfnWndProc = [](HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) -> LRESULT
	{
		switch (msg)
		{
		case WM_SETCURSOR:
			SetCursor(nullptr);   // no arrow over the game
			return TRUE;
		case WM_LBUTTONDOWN:
		case WM_RBUTTONDOWN:
		case WM_MBUTTONDOWN:
			// we never activate; hand the click to MAME's window instead
			SetForegroundWindow(GetWindow(hwnd, GW_OWNER));
			return 0;
		}
		return DefWindowProcA(hwnd, msg, wp, lp);
	};
	wc.hInstance = GetModuleHandleA(nullptr);
	wc.lpszClassName = "MidvGLOverlay";
	RegisterClassA(&wc);
	RECT rc; GetClientRect(parent, &rc);
	POINT tl = { 0, 0 };
	ClientToScreen(parent, &tl);
	HWND child = CreateWindowExA(
		WS_EX_NOACTIVATE | WS_EX_TOOLWINDOW,
		"MidvGLOverlay", "", WS_POPUP | WS_VISIBLE,
		tl.x, tl.y, rc.right, rc.bottom, parent, nullptr, wc.hInstance, nullptr);
	if (!child) { logf("overlay window failed"); return; }

	HDC dc = GetDC(child);
	PIXELFORMATDESCRIPTOR pfd = {};
	pfd.nSize = sizeof(pfd); pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32;
	SetPixelFormat(dc, ChoosePixelFormat(dc, &pfd), &pfd);

	GL gl;
	if (!gl.load_base()) { logf("opengl32 load failed"); return; }
	HGLRC legacy = gl.CreateContext(dc);
	HGLRC context = legacy;
	gl.MakeCurrent(dc, legacy);
	if (!gl.load_rest()) { logf("GL function resolution failed"); return; }
	if (gl.CreateContextAttribs)
	{
		const int attrs[] = { 0x2091, 4, 0x2092, 3, 0x9126, 1, 0 };  // 4.3 core
		HGLRC core = gl.CreateContextAttribs(dc, nullptr, attrs);
		if (core)
		{
			gl.MakeCurrent(dc, core);
			gl.DeleteContext(legacy);
			context = core;
			gl.load_rest();   // re-resolve in the core context
		}
	}
	if (gl.SwapIntervalEXT) gl.SwapIntervalEXT(1);

	uint prog = link(gl, MVGL_VS, MVGL_FS);
	uint pal = link(gl, MVGL_PAL_VS, MVGL_PAL_FS);
	if (!prog || !pal) return;

	gl.PixelStorei(0x0CF5 /*GL_UNPACK_ALIGNMENT*/, 1);
	auto make_tex = [&](int w, int h, unsigned ifmt) -> uint
	{
		uint t; gl.GenTextures(1, &t);
		gl.BindTexture(0x0DE1, t);
		gl.TexParameteri(0x0DE1, 0x2801, 0x2600);  // MIN_FILTER NEAREST
		gl.TexParameteri(0x0DE1, 0x2800, 0x2600);  // MAG_FILTER NEAREST
		gl.TexParameteri(0x0DE1, 0x2802, 0x812F);  // WRAP_S CLAMP_TO_EDGE
		gl.TexParameteri(0x0DE1, 0x2803, 0x812F);
		unsigned fmt = RED_INTEGER;
		unsigned type = (ifmt == R8UI) ? 0x1401 : (ifmt == R16UI) ? 0x1403 : 0x1405;
		gl.TexImage2D(0x0DE1, 0, int(ifmt), w, h, 0, fmt, type, nullptr);
		return t;
	};
	uint texram = make_tex(4096, 2048, R8UI);
	uint paltex = make_tex(256, 128, R32UI);
	uint pageTex[2] = { make_tex(fw, fh, R16UI), make_tex(fw, fh, R16UI) };
	// crack-fill mask: attachment 1 flags pixels the CURRENT scene wrote;
	// the palette pass fills unwritten slivers (hardware quad cracks that
	// would show the stale page) from axis-bounded neighbours
	uint maskTex[2] = { make_tex(fw, fh, R8UI), make_tex(fw, fh, R8UI) };
	uint fbo[2];
	gl.GenFramebuffers(2, fbo);
	for (int i = 0; i < 2; i++)
	{
		gl.BindFramebuffer(FRAMEBUFFER, fbo[i]);
		gl.FramebufferTexture2D(FRAMEBUFFER, COLOR_ATTACHMENT0, 0x0DE1, pageTex[i], 0);
		gl.FramebufferTexture2D(FRAMEBUFFER, COLOR_ATTACHMENT0 + 1, 0x0DE1, maskTex[i], 0);
		unsigned const bufs[2] = { COLOR_ATTACHMENT0, COLOR_ATTACHMENT0 + 1 };
		gl.DrawBuffers(2, bufs);
		if (gl.CheckFramebufferStatus(FRAMEBUFFER) != FRAMEBUFFER_COMPLETE)
			logf("fbo %d incomplete", i);
		gl.Clear(0x4000);
	}
	gl.BindFramebuffer(FRAMEBUFFER, 0);

	// geometry VAO: two streamed VBOs, attributes by name
	uint vao, vbo_f, vbo_u, vao_empty;
	gl.GenVertexArrays(1, &vao);
	gl.GenVertexArrays(1, &vao_empty);
	gl.GenBuffers(1, &vbo_f);
	gl.GenBuffers(1, &vbo_u);
	gl.BindVertexArray(vao);
	gl.BindBuffer(ARRAY_BUFFER, vbo_f);
	const char *fattr[] = { "in_corner", "in_v0", "in_v1", "in_v2", "in_v3", "in_uv01", "in_uv23", "in_uvBounds" };
	int const fsize[] = { 2, 2, 2, 2, 2, 4, 4, 4 };
	int off = 0;
	for (int i = 0; i < 8; i++)
	{
		int loc = gl.GetAttribLocation(prog, fattr[i]);
		if (loc >= 0)
		{
			gl.EnableVertexAttribArray(loc);
			gl.VertexAttribPointer(loc, fsize[i], 0x1406 /*FLOAT*/, 0, 22 * 4,
				(const void *)(uintptr_t)(off * 4));
		}
		off += fsize[i];
	}
	gl.BindBuffer(ARRAY_BUFFER, vbo_u);
	{
		int loc = gl.GetAttribLocation(prog, "in_meta");
		gl.EnableVertexAttribArray(loc);
		gl.VertexAttribIPointer(loc, 4, 0x1405 /*UNSIGNED_INT*/, 16, nullptr);
	}

	gl.UseProgram(prog);
	gl.Uniform2f(gl.GetUniformLocation(prog, "uCanvas"), float(WIDE), float(H));
	gl.Uniform1i(gl.GetUniformLocation(prog, "uScale"), S);
	gl.Uniform1i(gl.GetUniformLocation(prog, "uClipRight"), WIDE - 1);
	gl.Uniform1i(gl.GetUniformLocation(prog, "texram"), 0);
	gl.Uniform1i(gl.GetUniformLocation(prog, "texMask"), (8 << 20) - 1);
	gl.Uniform1i(gl.GetUniformLocation(prog, "uDbgQuadId"), 0);
	gl.Uniform1i(gl.GetUniformLocation(prog, "uClipW"), WIDE);
	// Legacy margin suppression/column stretching destroyed valid skies in
	// recorded World/Off Road races. Retain it only as an explicit experiment.
	bool const align_joins = S > 1 && std::getenv("MIDV_GL_TJUNCTIONS")
		&& atoi(std::getenv("MIDV_GL_TJUNCTIONS")) == 1;
	uint64_t n_aligned = 0;
	bool const margin_on = std::getenv("MIDV_GL_MARGINFILL")
		&& atoi(std::getenv("MIDV_GL_MARGINFILL")) == 1;
	bool const bg_gate = margin_on &&
		!(std::getenv("MIDV_GL_CRACKFILL") && atoi(std::getenv("MIDV_GL_CRACKFILL")) == 0);
	gl.Uniform1i(gl.GetUniformLocation(prog, "uBgMargin"), bg_gate ? MARGIN : 0);
	gl.UseProgram(pal);
	gl.Uniform1i(gl.GetUniformLocation(pal, "idxTex"), 1);
	gl.Uniform1i(gl.GetUniformLocation(pal, "palTex"), 2);
	int const uCrop = gl.GetUniformLocation(pal, "uCrop");
	// CRT pass: MIDV_GL_CRT=1 enables at boot, F9 toggles live. The raw
	// (uCrt=0) shader path is byte-identical to the pre-CRT palette pass.
	int const uCrt = gl.GetUniformLocation(pal, "uCrt");
	int const uSrcH = gl.GetUniformLocation(pal, "uSrcH");
	int const uFillR = gl.GetUniformLocation(pal, "uFillR");
	bool crt = std::getenv("MIDV_GL_CRT") && atoi(std::getenv("MIDV_GL_CRT")) != 0;
	// Local crack fill remains a cosmetic option; missing geometry and
	// intentional gaps cannot be distinguished by coverage alone.
	bool const fill_on = !(std::getenv("MIDV_GL_CRACKFILL")
			&& atoi(std::getenv("MIDV_GL_CRACKFILL")) == 0);
	// Explicit legacy margin experiment shares the local fill prerequisite.
	gl.Uniform1i(uCrt, crt ? 1 : 0);
	gl.Uniform1f(uSrcH, float(H));
	gl.Uniform1i(gl.GetUniformLocation(pal, "maskTex"), 3);
	gl.Uniform1i(gl.GetUniformLocation(pal, "uMargin"),
		(fill_on && margin_on) ? MARGIN * S : 0);
	bool f9_prev = false;

	// ---- in-game Esc options menu (drawn by the overlay itself) ----
	// Esc is detached from MAME's quit in the generated ctrlr (F12 remains
	// the emergency quit); the GL thread polls physical keys via
	// GetAsyncKeyState, which rawinput cannot intercept away from us.
	uint menuprog = link(gl, MVGL_MENU_VS, MVGL_MENU_FS);
	struct Label { uint tex; int w, h; };
	auto make_label = [&](const unsigned char *px, int w, int h) -> Label
	{
		uint t;
		gl.GenTextures(1, &t);
		gl.BindTexture(0x0DE1, t);
		gl.TexParameteri(0x0DE1, 0x2801, 0x2601);   // MIN LINEAR
		gl.TexParameteri(0x0DE1, 0x2800, 0x2601);   // MAG LINEAR
		gl.TexParameteri(0x0DE1, 0x2802, 0x812F);
		gl.TexParameteri(0x0DE1, 0x2803, 0x812F);
		gl.TexImage2D(0x0DE1, 0, 0x8229 /*R8*/, w, h, 0,
				0x1903 /*RED*/, 0x1401 /*UNSIGNED_BYTE*/, px);
		return Label{ t, w, h };
	};
	Label lb_title = make_label(MVMENU_TITLE, MVMENU_TITLE_W, MVMENU_TITLE_H);
	Label lb_resume = make_label(MVMENU_RESUME, MVMENU_RESUME_W, MVMENU_RESUME_H);
	Label lb_crt_on = make_label(MVMENU_CRT_ON, MVMENU_CRT_ON_W, MVMENU_CRT_ON_H);
	Label lb_crt_off = make_label(MVMENU_CRT_OFF, MVMENU_CRT_OFF_W, MVMENU_CRT_OFF_H);
	Label lb_exit = make_label(MVMENU_EXIT, MVMENU_EXIT_W, MVMENU_EXIT_H);
	Label lb_hint = make_label(MVMENU_HINT, MVMENU_HINT_W, MVMENU_HINT_H);
	int const mRect = gl.GetUniformLocation(menuprog, "uRect");
	int const mScreen = gl.GetUniformLocation(menuprog, "uScreen");
	int const mColor = gl.GetUniformLocation(menuprog, "uColor");
	int const mSolid = gl.GetUniformLocation(menuprog, "uSolid");
	gl.UseProgram(menuprog);
	gl.Uniform1i(gl.GetUniformLocation(menuprog, "uTex"), 0);
	bool menu_open = false;
	int menu_sel = 0;
	bool esc_prev = false, up_prev = false, down_prev = false, ret_prev = false;

	logf("GL up: scale %d canvas %dx%d crt=%d locs crop=%d crt=%d srch=%d err=%u snapdir=%s",
		S, fw, fh, int(crt), uCrop, uCrt, uSrcH, gl.GetError(),
		snapdir ? snapdir : "(null)");

	// ---- stream state ----
	std::vector<QuadMsg> run;
	uint16_t run_pc = 0xffff, active_pc = 0xffff;
	int scene_axis[2] = {}, cpu_written[2] = {};
	static uint16_t shadow[2][MAXH * 512];
	bool quad_fresh[2] = {};
	bool crop2d[2] = {};
	int quad_count[2] = {};
	int visible = 0;
	std::vector<uint8_t> staging(8 << 20);
	static uint32_t pal_copy[32768];
	std::vector<float> fdata;
	std::vector<uint32_t> udata;
	uint64_t presents = 0, n_quads = 0, n_scenes = 0, n_pal = 0, n_tex = 0, n_vram = 0;
	int snap_n = 0;
	uint32_t last_received_frame = 0;
	int const stall_frame = std::getenv("MIDV_GL_STALL_FRAME") ? atoi(std::getenv("MIDV_GL_STALL_FRAME")) : -1;
	int const stall_ms = std::getenv("MIDV_GL_STALL_MS") ? std::clamp(atoi(std::getenv("MIDV_GL_STALL_MS")), 0, 5000) : 0;
	bool stalled = false;
	// Explicit, output-free menu regression: exercise the same key edges as the
	// player without injecting global Windows input. Game captures stay separate.
	int const menu_test_frame = (std::getenv("MIDV_FFB") &&
		strcmp(std::getenv("MIDV_FFB"), "0") == 0 && std::getenv("MIDV_GL_MENU_TEST_FRAME"))
		? atoi(std::getenv("MIDV_GL_MENU_TEST_FRAME")) : -1;
	int menu_test_step = -1, menu_test_saved = -1;
	ULONGLONG menu_test_next = 0;
	uint32_t completed_frame = 0;

	auto ring_read = [&](uint64_t pos, void *dst, uint32_t len)
	{
		uint32_t o = uint32_t(pos % midv_live::RING_SIZE);
		uint32_t first = std::min(len, midv_live::RING_SIZE - o);
		memcpy(dst, lv.data + o, first);
		if (len > first) memcpy((uint8_t *)dst + first, lv.data, len - first);
	};
	uint64_t n_flips = 0;
	auto complete_run = [&]()
	{
		if (run.empty()) return;
		int const pg = (run_pc & 4) ? 1 : 0;
		bool const new_scene = active_pc != run_pc;
		if (new_scene) { quad_count[pg] = 0; scene_axis[pg] = 0; active_pc = run_pc; }
		n_aligned += build_vertices(run, float(MARGIN), fdata, udata, align_joins);
		quad_count[pg] += int(run.size());
		cpu_written[pg] = 0;
		// 2D screens (menus, high scores) are drawn almost entirely from
		// axis-aligned rectangles; 3D scenes almost never are. Quad-count
		// thresholds proved unreliable (2D ~160 vs 3D dipping to ~260).
		int axis = 0;
		for (auto const &q : run)
		{
			int16_t const x0 = int16_t(q.dma[2]), y0 = int16_t(q.dma[3]);
			int16_t const x1 = int16_t(q.dma[4]), y1 = int16_t(q.dma[5]);
			int16_t const x2 = int16_t(q.dma[6]), y2 = int16_t(q.dma[7]);
			int16_t const x3 = int16_t(q.dma[8]), y3 = int16_t(q.dma[9]);
			if ((y0 == y1 && y2 == y3 && x1 == x2 && x3 == x0) ||
				(x0 == x1 && x2 == x3 && y1 == y2 && y3 == y0))
				++axis;
		}
		scene_axis[pg] += axis;
		crop2d[pg] = (scene_axis[pg] * 10 >= quad_count[pg] * 7);
		gl.UseProgram(prog);
		gl.BindVertexArray(vao);
		gl.BindBuffer(ARRAY_BUFFER, vbo_f);
		gl.BufferData(ARRAY_BUFFER, fdata.size() * 4, fdata.data(), STREAM_DRAW);
		gl.BindBuffer(ARRAY_BUFFER, vbo_u);
		gl.BufferData(ARRAY_BUFFER, udata.size() * 4, udata.data(), STREAM_DRAW);
		gl.BindFramebuffer(FRAMEBUFFER, fbo[pg]);
		gl.Viewport(0, 0, fw, fh);
		if (new_scene)
		{
			// reset the crack-fill mask to "unwritten" for the whole frame -
			// glClearBufferuiv touches ONLY attachment 1, so the page itself
			// still persists between scenes (hardware behavior)
			{
				uint const zero[4] = { 0, 0, 0, 0 };
				gl.ClearBufferuiv(0x1800 /*GL_COLOR*/, 1, zero);
			}
			// The game guarantees repainting only the 512-wide hardware
			// region; the 16:9 margins are ours. Scenes that draw nothing
			// there (2D screens, showcase scenes) would otherwise show the
			// previous scene's stale margins - seen at the rig as scenery
			// strips beside a 4:3 screen and a thin border at crop edges.
			gl.Enable(0x0C11 /*SCISSOR_TEST*/);
			gl.ClearColor(0, 0, 0, 0);
			// widen by a 2-pixel overscan inset: the outermost rows/columns
			// of the hardware region carry edge pixels a real CRT never
			// showed (seen as a thin bright border at the rig)
			int const os = 2 * S;
			gl.Scissor(0, 0, MARGIN * S + os, fh);
			gl.Clear(0x4000);
			gl.Scissor(fw - MARGIN * S - os, 0, MARGIN * S + os, fh);
			gl.Clear(0x4000);
			gl.Scissor(0, 0, fw, os);
			gl.Clear(0x4000);
			gl.Scissor(0, fh - os, fw, os);
			gl.Clear(0x4000);
			gl.Disable(0x0C11);
		}
		gl.ActiveTexture(TEXTURE0);
		gl.BindTexture(0x0DE1, texram);
		gl.DrawArrays(0x0004 /*TRIANGLES*/, 0, int(run.size() * 6));
		gl.BindFramebuffer(FRAMEBUFFER, 0);
		quad_fresh[pg] = true;
		if (new_scene) ++n_scenes;
		run.clear();
	};

	while (IsWindow(parent) && !s_stop.load())
	{
		if (InterlockedCompareExchange((volatile LONG *)(lv.base + 32), 0, 0))
		{
			logf("render stream failed: persistent state incomplete; closing overlay");
			break;
		}
		MSG msg;
		while (PeekMessageA(&msg, child, 0, 0, PM_REMOVE)) DispatchMessageA(&msg);
		bool ui_changed = false;
		int menu_test_key = 0;
		if (menu_test_frame >= 0 && completed_frame >= uint32_t(menu_test_frame) &&
			menu_test_step < 8 && GetTickCount64() >= menu_test_next)
		{
			static int const keys[] = { VK_ESCAPE, VK_DOWN, VK_RETURN, VK_UP,
				VK_RETURN, VK_ESCAPE, VK_DOWN, VK_DOWN, VK_RETURN };
			menu_test_key = keys[++menu_test_step];
			menu_test_next = GetTickCount64() + 350;
			logf("menu test step=%d key=%d completed_frame=%u", menu_test_step, menu_test_key, completed_frame);
		}
		// F9: live CRT toggle (edge-triggered; global key, only polled here)
		bool const f9 = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
		if (f9 && !f9_prev)
		{
			ui_changed = true;
			crt = !crt;
			gl.UseProgram(pal);
			gl.Uniform1i(uCrt, crt ? 1 : 0);
			logf("F9 -> crt=%d", int(crt));
		}
		f9_prev = f9;

		// Esc options menu: physical-key polls, only while MAME is foreground
		{
			bool const fg = (GetForegroundWindow() == parent);
			auto edge = [&](int vk, bool &prev) -> bool
			{
				bool const down = fg && (GetAsyncKeyState(vk) & 0x8000) != 0;
				bool const e = down && !prev;
				prev = down;
				return e || menu_test_key == vk;
			};
			if (edge(VK_ESCAPE, esc_prev) && menuprog)
			{
				ui_changed = true;
				menu_open = !menu_open;
				if (menu_open)
					midv_ffb_cancel();   // POC: drop the wheel force the moment the menu opens
			}
			bool const up = edge(VK_UP, up_prev);
			bool const dn = edge(VK_DOWN, down_prev);
			bool const ok = edge(VK_RETURN, ret_prev);
			if (menu_open)
			{
				ui_changed = ui_changed || up || dn || ok;
				if (up) menu_sel = (menu_sel + 2) % 3;
				if (dn) menu_sel = (menu_sel + 1) % 3;
				if (ok)
				{
					if (menu_sel == 0)
						menu_open = false;
					else if (menu_sel == 1)
					{
						crt = !crt;
						gl.UseProgram(pal);
						gl.Uniform1i(uCrt, crt ? 1 : 0);
					}
					else
					{
						PostMessageA(parent, WM_CLOSE, 0, 0);
						menu_open = false;
					}
				}
			}
			s_menu_pause.store(menu_open ? 1 : 0);
		}
		GetClientRect(parent, &rc);
		POINT ntl = { 0, 0 };
		ClientToScreen(parent, &ntl);
		RECT crc; GetWindowRect(child, &crc);
		if (IsIconic(parent))
			ShowWindow(child, SW_HIDE);
		else
		{
			if (!IsWindowVisible(child))
				ShowWindow(child, SW_SHOWNA);
			if (crc.left != ntl.x || crc.top != ntl.y ||
				crc.right - crc.left != rc.right || crc.bottom - crc.top != rc.bottom)
				SetWindowPos(child, nullptr, ntl.x, ntl.y, rc.right, rc.bottom,
					SWP_NOACTIVATE | SWP_NOZORDER);
		}

		if (!stalled && stall_frame >= 0 && int(last_received_frame) >= stall_frame)
		{
			stalled = true;
			logf("diagnostic consumer stall frame=%u ms=%d", last_received_frame, stall_ms);
			Sleep(stall_ms);
		}
		// ---- drain ----
		bool frame_complete = false;
		uint64_t w = ring_load(lv.wpos), r = ring_load(lv.rpos);
		while (r < w)
		{
			uint32_t hdr[2];
			ring_read(r, hdr, 8);
			uint32_t const type = hdr[0], len = hdr[1];
			if (len > staging.size()) staging.resize(len);
			ring_read(r + 8, staging.data(), len);
			if (type >= 1 && type <= 6 && len >= 4)
				memcpy(&last_received_frame, staging.data(), 4);
			r += (8 + len + 7) & ~7u;
			ring_store(lv.rpos, r);
			switch (type)
			{
			case 1:
			{
				QuadMsg q;
				memcpy(&q, staging.data(), sizeof(q));
				if (run_pc != 0xffff && q.pc != run_pc) complete_run();
				run_pc = q.pc;
				run.push_back(q);
				++n_quads;
				break;
			}
			case 2:
			{
				++n_flips;
				complete_run();
				uint16_t newpc; memcpy(&newpc, staging.data() + 6, 2);
				visible = (newpc & 1) ? 1 : 0;
				break;
			}
			case 3:
				complete_run();
				++n_pal;
				memcpy(pal_copy, staging.data() + 4, sizeof(pal_copy));
				gl.ActiveTexture(TEXTURE0 + 2);
				gl.BindTexture(0x0DE1, paltex);
				gl.TexSubImage2D(0x0DE1, 0, 0, 0, 256, 128, RED_INTEGER, 0x1405, staging.data() + 4);
				break;
			case 4:
				complete_run();
				++n_tex;
				gl.ActiveTexture(TEXTURE0);
				gl.BindTexture(0x0DE1, texram);
				gl.TexSubImage2D(0x0DE1, 0, 0, 0, 4096, 2048, RED_INTEGER, 0x1401, staging.data() + 4);
				break;
			case 5:
			{
				complete_run();
				uint32_t o, n;
				memcpy(&o, staging.data() + 4, 4);
				memcpy(&n, staging.data() + 8, 4);
				int pg = (o & 0x40000) ? 1 : 0;
				uint32_t rel = o & 0x3ffff;
				++n_vram;
				if (rel < uint32_t(H) * 512)
				{
					uint32_t end = std::min(rel + n, uint32_t(H) * 512);
					memcpy(&shadow[pg][rel], staging.data() + 12, (end - rel) * 2);
					// CPU pixels and quads mutate the same persistent indexed page.
					// Upload only this span, with nearest expansion and the same Y flip.
					gl.ActiveTexture(TEXTURE0 + 1);
					std::vector<uint16_t> pixels;
					std::vector<uint8_t> mask;
					for (uint32_t at = rel; at < end; )
					{
						int const x = at % 512, y = at / 512;
						int const count = std::min(end - at, uint32_t(512 - x));
						pixels.resize(count * S * S); mask.assign(count * S * S, 1);
						for (int yy = 0; yy < S; ++yy)
							for (int xx = 0; xx < count * S; ++xx)
								pixels[yy * count * S + xx] = shadow[pg][at + xx / S];
						gl.BindTexture(0x0DE1, pageTex[pg]);
						gl.TexSubImage2D(0x0DE1, 0, (MARGIN + x) * S, (H - 1 - y) * S,
							count * S, S, RED_INTEGER, 0x1403, pixels.data());
						gl.BindTexture(0x0DE1, maskTex[pg]);
						gl.TexSubImage2D(0x0DE1, 0, (MARGIN + x) * S, (H - 1 - y) * S,
							count * S, S, RED_INTEGER, 0x1401, mask.data());
						at += count;
					}
					cpu_written[pg] += end - rel;
					if (cpu_written[pg] >= H * 512 * 7 / 10)
						quad_fresh[pg] = false, active_pc = 0xffff;
				}
				break;
			}
			case 6:
				complete_run();
				frame_complete = true;
				completed_frame = last_received_frame;
				break;
			}
			if (frame_complete) break;
		}
		// Opening Esc pauses the producer before its next frame fence. The UI
		// must still present and process selection/CRT/resume without that fence.
		if (!frame_complete && !menu_open && !ui_changed) { Sleep(1); continue; }

		// ---- present ----
		int const cw = rc.right, ch = rc.bottom;
		gl.Viewport(0, 0, cw, ch);
		gl.ClearColor(0, 0, 0, 1);
		gl.Clear(0x4000);
		bool const wide3d = quad_fresh[visible] && !crop2d[visible];
		float const content_w = wide3d ? float(WIDE) : 512.0f;
		float const aspect = (content_w * PAR) / float(H);
		int vw = cw, vh = int(cw / aspect + 0.5f);
		if (vh > ch) { vh = ch; vw = int(ch * aspect + 0.5f); }
		gl.UseProgram(pal);
		gl.Uniform1i(uCrop, !wide3d ? MARGIN * S : 0);
		// fill radius: live 3D scenes get the full crack fill; 2D screens
		// (menus, track select) get a tight 1-px pass only - their bitmap
		// tiles leave hairline unwritten seams (offroadc track select's
		// vertical lines, rig bug G4) that read as black scratches at 4x,
		// while anything wider on a 2D screen may be intentional
		gl.Uniform1i(uFillR,
			(fill_on && quad_fresh[visible])
				? (crop2d[visible] ? 1 * S : 4 * S) : 0);
		gl.ActiveTexture(TEXTURE0 + 3);
		gl.BindTexture(0x0DE1, maskTex[visible]);
		gl.ActiveTexture(TEXTURE0 + 1);
		gl.BindTexture(0x0DE1, pageTex[visible]);
		gl.ActiveTexture(TEXTURE0 + 2);
		gl.BindTexture(0x0DE1, paltex);
		gl.Viewport((cw - vw) / 2, (ch - vh) / 2, vw, vh);
		gl.BindVertexArray(vao_empty);
		gl.DrawArrays(0x0004, 0, 3);

		// ---- Esc options menu overlay ----
		if (menu_open)
		{
			gl.Viewport(0, 0, cw, ch);
			gl.Enable(0x0BE2 /*BLEND*/);
			gl.BlendFunc(0x0302 /*SRC_ALPHA*/, 0x0303 /*ONE_MINUS_SRC_ALPHA*/);
			gl.UseProgram(menuprog);
			gl.Uniform2f(mScreen, float(cw), float(ch));
			auto mrect = [&](Label const *L, float x, float y, float w, float h,
					float r, float g2, float b, float a)
			{
				gl.Uniform4f(mRect, x, y, w, h);
				gl.Uniform4f(mColor, r, g2, b, a);
				gl.Uniform1i(mSolid, L ? 0 : 1);
				if (L)
				{
					gl.ActiveTexture(TEXTURE0);
					gl.BindTexture(0x0DE1, L->tex);
				}
				gl.DrawArrays(0x0005 /*TRIANGLE_STRIP*/, 0, 4);
			};
			auto mlabel = [&](Label const &L, float cy, float px,
					float r, float g2, float b)
			{
				float const h = px, w = L.w * px / L.h;
				mrect(&L, (cw - w) / 2.0f, cy, w, h, r, g2, b, 1.0f);
			};
			float const sc = ch / 1080.0f;
			mrect(nullptr, 0, 0, float(cw), float(ch), 0, 0, 0, 0.55f);
			mlabel(lb_title, ch * 0.24f, 72 * sc, 1.0f, 0.72f, 0.20f);
			Label const *items[3] = { &lb_resume, crt ? &lb_crt_on : &lb_crt_off, &lb_exit };
			for (int i = 0; i < 3; i++)
			{
				bool const s = (i == menu_sel);
				mlabel(*items[i], ch * (0.42f + 0.10f * i), 44 * sc,
					s ? 1.0f : 0.85f, s ? 0.72f : 0.85f, s ? 0.20f : 0.90f);
			}
			mlabel(lb_hint, ch * 0.86f, 22 * sc, 0.75f, 0.75f, 0.80f);
			gl.Disable(0x0BE2);
			gl.ActiveTexture(TEXTURE0);
			gl.BindTexture(0x0DE1, texram);   // restore for the quad pass
		}

		++presents;
		if (s_log && (presents % 300) == 0)
			logf("t=%llu quads=%llu scenes=%llu pal=%llu tex=%llu vram=%llu "
				"flips=%llu backlog=%llu vis=%d fresh=%d cnt=%d err=%u speed=%.0f%%",
				(unsigned long long)presents, (unsigned long long)n_quads,
				(unsigned long long)n_scenes, (unsigned long long)n_pal,
				(unsigned long long)n_tex, (unsigned long long)n_vram,
				(unsigned long long)n_flips,
				(unsigned long long)(ring_load(lv.wpos) - ring_load(lv.rpos)), visible,
				int(quad_fresh[visible]), quad_count[visible], gl.GetError(), double(lv.speed_pct));
		bool const menu_test_capture = menu_test_step >= 0 && menu_test_step < 8 && menu_test_saved != menu_test_step;
		if (snapdir && (menu_test_capture || (frame_complete && !menu_open &&
			(last_received_frame % snap_every) == 0 &&
			int(last_received_frame) >= snap_first &&
			(snap_last < 0 || int(last_received_frame) <= snap_last) &&
			(snap_max == 0 || snap_n < snap_max))))
		{
			std::vector<uint8_t> px(size_t(cw) * ch * 3);
			// tight rows: the default GL_PACK_ALIGNMENT of 4 pads each row
			// whenever cw*3 isn't a multiple of 4, overrunning px (heap
			// corruption -> crash at the first snap on any window width
			// like 2350; every earlier run happened to be 2352 wide).
			// The BMP writer below does its own 4-byte row padding.
			gl.PixelStorei(0x0D05 /*GL_PACK_ALIGNMENT*/, 1);
			gl.ReadPixels(0, 0, cw, ch, 0x80E0 /*BGR*/, 0x1401, px.data());
			char path[512];
			if (menu_test_capture)
				snprintf(path, sizeof(path), "%s\\menu_%02d.bmp", snapdir, menu_test_step);
			else
				snprintf(path, sizeof(path), "%s\\mvgl_%03d.bmp", snapdir, snap_n++);
			FILE *f = fopen(path, "wb");
			logf("snap %s -> %s", path, f ? "ok" : "FOPEN FAILED");
			if (f)
			{
				int const rowsz = (cw * 3 + 3) & ~3;
				uint32_t const img = rowsz * ch;
				uint8_t bh[54] = { 'B', 'M' };
				*(uint32_t *)(bh + 2) = 54 + img;
				*(uint32_t *)(bh + 10) = 54;
				*(uint32_t *)(bh + 14) = 40;
				*(int32_t *)(bh + 18) = cw;
				*(int32_t *)(bh + 22) = ch;
				*(uint16_t *)(bh + 26) = 1;
				*(uint16_t *)(bh + 28) = 24;
				*(uint32_t *)(bh + 34) = img;
				fwrite(bh, 1, 54, f);
				std::vector<uint8_t> row(rowsz, 0);
				for (int y = 0; y < ch; y++)
				{
					memcpy(row.data(), &px[size_t(y) * cw * 3], cw * 3);
					fwrite(row.data(), 1, rowsz, f);
				}
				fclose(f);
				if (menu_test_capture)
				{
					menu_test_saved = menu_test_step;
					logf("menu snapshot step=%d open=%d selected=%d crt=%d completed_frame=%u new_frame=%d",
						menu_test_step, int(menu_open), menu_sel, int(crt), completed_frame, int(frame_complete));
				}
				// This is the most recently CONSUMED stream frame, not a promise
				// that the async backbuffer equals a native frame at that instant.
				// Preserve that distinction and the queue/drop state for analysis.
				char index_path[512];
				snprintf(index_path, sizeof(index_path), "%s\\captures.csv", snapdir);
				if (FILE *index = menu_test_capture ? nullptr : fopen(index_path, snap_n == 1 ? "w" : "a"))
				{
					if (snap_n == 1) fprintf(index, "file,present,last_received_frame,width,height,visible_page,queued_bytes,dropped_messages,completed_frame\n");
					fprintf(index, "mvgl_%03d.bmp,%llu,%u,%d,%d,%d,%llu,%llu,%u\n", snap_n - 1,
						(unsigned long long)presents, last_received_frame, cw, ch, visible,
						(unsigned long long)(ring_load(lv.wpos) - ring_load(lv.rpos)), (unsigned long long)ring_load(lv.dropped), last_received_frame);
					fclose(index);
				}
			}
		}
		SwapBuffers(dc);
	}
	// persist live-toggle state (F9 / Esc-menu CRT) for the launcher: the
	// collection shell reads this back so its SETTINGS row and the next
	// launch match what the player last saw on screen
	if (char const *sf = std::getenv("MIDV_GL_STATEFILE"))
	{
		FILE *f = fopen(sf, "w");
		if (f)
		{
			fprintf(f, "crt=%d\n", crt ? 1 : 0);
			fclose(f);
		}
	}
	logf("T-junction alignment enabled=%d vertices=%llu", int(align_joins), (unsigned long long)n_aligned);
	logf("%s after %llu presents, %d snaps",
		s_stop.load() ? "machine exit" : "parent gone or failed stream",
		(unsigned long long)presents, snap_n);
	gl.MakeCurrent(nullptr, nullptr);
	gl.DeleteContext(context);
	ReleaseDC(child, dc);
	DestroyWindow(child);
}

} // namespace mvgl

midv_live &live()
{
	static midv_live s;
	static bool gl_spawned = [&]() -> bool
	{
		if (s.enabled && std::getenv("MIDV_GL"))
			std::thread(mvgl::thread_main).detach();
		return true;
	}();
	(void)gl_spawned;
	return s;
}

#else
struct midv_live
{
	volatile float speed_pct = 0.0f;   // MAME speed %, for the overlay stats
	bool enabled = false;
	bool tex_dirty = false, pal_dirty = false;
	uint32_t last_frame = 0;
	bool write_msg(uint32_t, const void *, uint32_t,
	               const void * = nullptr, uint32_t = 0) { return false; }
	void flush_span() {}
	void vram_write(uint32_t, uint32_t, uint16_t) {}
	void sync_state(uint32_t, const void *, uint32_t,
	                const void *, uint32_t) {}
};
midv_live &live() { static midv_live s; return s; }
#endif

} // anonymous namespace

#define LOG_DMA             (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


#define WATCH_RENDER        (0)

#define DMA_CLOCK           40000000

// for when we implement DMA timing
#define DMA_QUEUE_SIZE      273
#define TIME_PER_PIXEL      41e-9


midvunit_renderer::midvunit_renderer(midvunit_base_state &state)
	: poly_manager<float, midvunit_object_data, 2>(state.machine())
	, m_state(state)
{
}


/*************************************
 *
 *  Video system start
 *
 *************************************/

TIMER_CALLBACK_MEMBER(midvunit_base_state::scanline_timer_cb)
{
	m_maincpu->set_input_line(0, ASSERT_LINE);
	m_scanline_timer->adjust(m_screen->time_until_pos(param + 1), param);
	m_eoi_timer->adjust(attotime::from_hz(25000000), -1);
}

TIMER_CALLBACK_MEMBER(midvunit_base_state::eoi_timer_cb)
{
	m_maincpu->set_input_line(0, CLEAR_LINE);
}


// POC: UDP telemetry (Phase A) + FFB trace - mirror every output change
// (wheel force, lamps) to a UDP consumer and/or a CSV. Env-gated, inert
// unset. MIDV_TELEM_FORZA alone also works (Forza packets, no JSON
// stream). Shared by the V-Unit games (video_start) and Exotica
// (midzeus machine_start) - both emit the "wheel" output now.
// POC: the other half of the force loop - the steering INPUT the game reads,
// logged into the same trace ("wheelpos" rows) whenever it changes. Called
// once per frame from screen_update (V-Unit ":WHEEL", Exotica ":ANALOG3").
void midv_trace_wheelpos(running_machine &machine, const char *tag)
{
	if (!s_ffb_trace)
		return;
	static int s_last = -1;
	ioport_port *port = machine.root_device().ioport(tag);
	if (!port)
		return;
	int const v = int(port->read() & 0xff);
	if (v == s_last)
		return;
	s_last = v;
	auto const ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - s_ffb_trace_t0).count();
	fprintf(s_ffb_trace, "%lld,wheelpos,%d\n", (long long)ms, v);
}


// ---- POC: built-in force feedback (2026-09-03, replaces the FFB Arcade Plugin)
// The games' wheel-motor byte (V-Unit WHLCTLZ, Exotica LED-board offset 0 -
// after the driver's gain/slew/clamp) becomes ONE signed constant-force level
// on the wheel's steering axis through SDL2 haptics, the way Cannonball DX
// (Endprodukt) and Flycast drive wheels: SDL_HAPTIC_STEERING_AXIS, infinite
// length, level updated in place, run after every update. SDL2.dll is loaded
// at run time, so without it FFB is simply off and everything else runs.
// The byte's interpretation is the FFB Arcade Plugin's Cruis'n handler
// (RacingFullValueActive2): 0 = no force, 1..127 = one way at v/126,
// 0x81..0xFF = the other way at (256-v)/126, 0x80 = no force, clamped to 1.
// Env (all inert unset):
//   MIDV_FFB=1            on
//   MIDV_FFB_STRENGTH=N   0..100 % of the wheel's full constant force (100)
//   MIDV_FFB_DEVICE=S     pick the wheel: name substring (case-insensitive)
//                         or vid:pid hex; else the first wheel-type haptic
//                         device, else the first haptic device that can do
//                         a constant force
//   MIDV_FFB_INVERT=1     flip the direction (bases differ in axis sign; the
//                         default is right for a Moza R12 - see midv_ffb_write)
//   MIDV_FFB_SMOOTH=N     first-order low-pass on the level, time constant N ms
//                         (0 = off). The arcade motor + wheel had inertia a
//                         direct-drive base lacks: without it the V-Unit damper
//                         kicks, arriving a couple of frames late, drive a strong
//                         wheel into a left-right limit cycle even hands-off
//   MIDV_FFB_DAMPER=N     stand-in for the arcade wheel mechanism: a DirectInput
//                         damper (resistance proportional to wheel speed) at N %
//                         for the whole session (0 = off). MIDV_FFB_FRICTION=N adds
//                         constant drag the same way. Both are condition effects
//                         the base renders itself, independent of the game forces.
//   MIDV_FFB_RUMBLE=N     a 100 ms vibration burst on every force update, at
//                         |force| x N % (the FFB Arcade Plugin did exactly this;
//                         it is what made crashes shake - Exotica's crash and
//                         jump effects are single 17 ms full-force spikes that
//                         a direct-drive base alone renders as a tick). 0 = off.
//   (smoothing lets a jump of >= 60 % of full through unfiltered, so those
//   spikes still reach the constant-force channel at full size)
//   MIDV_FFB_HOLD_MS=N    release the force when the game has not written the
//                         motor for N ms (500; 0 = hold forever like the
//                         arcade board - a paused direct-drive base holding
//                         torque is a hazard, so it is on by default)
//   MIDV_FFB_TEST=L       apply level L (-100..100 %) for 1.5 s at start,
//                         then 0 - sign calibration with a wheel-position probe
//   MIDV_FFB_LOG=2        also log every motor write (1 = init/device lines
//                         only, the default when MIDV_FFB is on)
// Log: midv_ffb.log in the working directory (the support bundle ships it).
namespace mvffb {

#define MVFFB_SDL_FUNCS(X) \
	X(int, SDL_Init, (Uint32)) \
	X(void, SDL_Quit, (void)) \
	X(SDL_bool, SDL_SetHint, (const char *, const char *)) \
	X(const char *, SDL_GetError, (void)) \
	X(int, SDL_NumJoysticks, (void)) \
	X(void, SDL_JoystickUpdate, (void)) \
	X(SDL_Joystick *, SDL_JoystickOpen, (int)) \
	X(void, SDL_JoystickClose, (SDL_Joystick *)) \
	X(const char *, SDL_JoystickName, (SDL_Joystick *)) \
	X(SDL_JoystickType, SDL_JoystickGetType, (SDL_Joystick *)) \
	X(Uint16, SDL_JoystickGetVendor, (SDL_Joystick *)) \
	X(Uint16, SDL_JoystickGetProduct, (SDL_Joystick *)) \
	X(SDL_Haptic *, SDL_HapticOpenFromJoystick, (SDL_Joystick *)) \
	X(void, SDL_HapticClose, (SDL_Haptic *)) \
	X(unsigned int, SDL_HapticQuery, (SDL_Haptic *)) \
	X(int, SDL_HapticSetAutocenter, (SDL_Haptic *, int)) \
	X(int, SDL_HapticSetGain, (SDL_Haptic *, int)) \
	X(int, SDL_HapticNewEffect, (SDL_Haptic *, SDL_HapticEffect *)) \
	X(int, SDL_HapticUpdateEffect, (SDL_Haptic *, int, SDL_HapticEffect *)) \
	X(int, SDL_HapticRunEffect, (SDL_Haptic *, int, Uint32)) \
	X(int, SDL_HapticStopEffect, (SDL_Haptic *, int)) \
	X(int, SDL_HapticDestroyEffect, (SDL_Haptic *, int)) \
	X(int, SDL_HapticRumbleSupported, (SDL_Haptic *)) \
	X(int, SDL_HapticRumbleInit, (SDL_Haptic *)) \
	X(int, SDL_HapticRumblePlay, (SDL_Haptic *, float, Uint32)) \
	X(int, SDL_HapticRumbleStop, (SDL_Haptic *))

#define MVFFB_DECL(ret, name, args) static ret (*p_##name) args = nullptr;
MVFFB_SDL_FUNCS(MVFFB_DECL)
#undef MVFFB_DECL

static std::atomic<bool> s_running{false};     // worker up and a device in hand
static std::atomic<bool> s_stop{false};
static std::atomic<int> s_level{0};            // requested signed level (-32767..32767)
static std::atomic<long long> s_last_write{0}; // ms clock of the last motor write
static std::mutex s_mtx;
static std::condition_variable s_cv;
static bool s_dirty = false;
static std::thread s_thread;
static int s_strength = 100;
static std::atomic<bool> s_cancel_impact{false};
static std::atomic<int> s_raw_level{0};
static bool s_impact_axis = false; // explicit opt-in; physical acceptance pending
static bool s_invert = false;
static int s_hold_ms = 500;
static int s_smooth_ms = 0;   // MIDV_FFB_SMOOTH: first-order low-pass time constant
static bool s_smooth_set = false;  // ...and whether it was given at all
static std::string s_profile_id = "cruisn-vunit@1";  // MIDV_FFB_PROFILE
static int s_damper = 0;      // MIDV_FFB_DAMPER: velocity-proportional resistance, % of full
static int s_friction = 0;    // MIDV_FFB_FRICTION: constant drag, % of full
static int s_spring = 0;      // MIDV_FFB_SPRING: centring toward straight-ahead, % of full
static int s_rumble = 0;      // MIDV_FFB_RUMBLE: vibration burst per force update, % scale
static int s_loglevel = 1;
static FILE *s_log = nullptr;
static std::mutex s_logmtx;
static std::chrono::steady_clock::time_point s_t0;

// Directory holding vunit.exe. force-profiles.ini is deployed beside it by the
// launcher, and the working directory is not reliably that folder.
static std::string exe_dir()
{
	char buf[MAX_PATH] = { 0 };
	DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
	std::string p(buf, buf + (n ? n : 0));
	size_t cut = p.find_last_of("\\/");
	return cut == std::string::npos ? std::string(".") : p.substr(0, cut);
}

static long long now_ms()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - s_t0).count();
}

static void flog(const char *fmt, ...)
{
	std::lock_guard<std::mutex> lk(s_logmtx);
	if (!s_log)
		return;
	fprintf(s_log, "%8lld  ", now_ms());
	va_list ap;
	va_start(ap, fmt);
	vfprintf(s_log, fmt, ap);
	va_end(ap);
	fputc('\n', s_log);
	fflush(s_log);
}

static bool load_sdl()
{
	HMODULE h = LoadLibraryA("SDL2.dll");
	if (!h)
	{
		flog("SDL2.dll not found beside the emulator - force feedback off");
		return false;
	}
	bool ok = true;
#define MVFFB_LOAD(ret, name, args) \
	p_##name = reinterpret_cast<ret (*) args>(GetProcAddress(h, #name)); \
	if (!p_##name) { flog("SDL2.dll lacks %s", #name); ok = false; }
	MVFFB_SDL_FUNCS(MVFFB_LOAD)
#undef MVFFB_LOAD
	return ok;
}

static bool icontains(const char *hay, const char *needle)
{
	if (!hay || !needle || !*needle)
		return false;
	std::string h(hay), n(needle);
	for (auto &c : h) c = char(tolower((unsigned char)c));
	for (auto &c : n) c = char(tolower((unsigned char)c));
	return h.find(n) != std::string::npos;
}

struct Device
{
	SDL_Joystick *js = nullptr;
	SDL_Haptic *hp = nullptr;
	unsigned caps = 0;
	bool is_wheel = false;
	int effect = -1;
	std::string name;
};

// Cannonball DX's selection order, trimmed to what we know: the device the
// launcher names (wizard steering device) is authoritative; else a wheel-type
// device with a constant-force actuator; else any constant-force device.
static bool select_device(Device &d)
{
	const char *want = std::getenv("MIDV_FFB_DEVICE");
	unsigned wvid = 0, wpid = 0;
	bool const want_vidpid = want && sscanf(want, "%x:%x", &wvid, &wpid) == 2;
	bool const want_name = want && *want && !want_vidpid;
	int const n = p_SDL_NumJoysticks();
	flog("%d joystick(s) via SDL %s", n, want ? want : "(no MIDV_FFB_DEVICE: first wheel)");
	for (int pass = 0; pass < 3; pass++)
	{
		if (pass == 0 && !want_vidpid && !want_name)
			continue;
		for (int i = 0; i < n; i++)
		{
			SDL_Joystick *js = p_SDL_JoystickOpen(i);
			if (!js)
				continue;
			const char *name = p_SDL_JoystickName(js);
			unsigned const vid = p_SDL_JoystickGetVendor(js), pid = p_SDL_JoystickGetProduct(js);
			bool const wheel = (p_SDL_JoystickGetType(js) == SDL_JOYSTICK_TYPE_WHEEL);
			if (pass == 0)
				flog("  [%d] \"%s\" vid %04x pid %04x%s", i, name ? name : "?", vid, pid, wheel ? " (wheel)" : "");
			bool match = true;
			if (pass == 0)
				match = want_vidpid ? (vid == wvid && pid == wpid) : icontains(name, want);
			else if (pass == 1)
				match = wheel;
			if (!match)
			{
				p_SDL_JoystickClose(js);
				continue;
			}
			SDL_Haptic *hp = p_SDL_HapticOpenFromJoystick(js);
			if (!hp)
			{
				if (pass == 0)
					flog("  [%d] matches but has no haptics: %s", i, p_SDL_GetError());
				p_SDL_JoystickClose(js);
				if (pass == 0)
					return false;   // the named device cannot do it: never push another wheel
				continue;
			}
			unsigned const caps = p_SDL_HapticQuery(hp);
			if (!(caps & SDL_HAPTIC_CONSTANT))
			{
				flog("  [%d] haptic but no constant force (caps 0x%x)", i, caps);
				p_SDL_HapticClose(hp);
				p_SDL_JoystickClose(js);
				if (pass == 0)
					return false;
				continue;
			}
			d.js = js; d.hp = hp; d.caps = caps; d.is_wheel = wheel;
			d.name = name ? name : "?";
			flog("using [%d] \"%s\" (pass %d, caps 0x%x%s)", i, d.name.c_str(), pass, caps,
					wheel ? ", steering axis" : ", cartesian");
			return true;
		}
		if (pass == 0)
		{
			flog("MIDV_FFB_DEVICE \"%s\" matched nothing - force feedback off", want);
			return false;
		}
	}
	flog("no constant-force device found - force feedback off");
	return false;
}

static void apply(Device &d, int level, bool &running, int &applied)
{
	SDL_HapticEffect e;
	memset(&e, 0, sizeof(e));
	e.type = SDL_HAPTIC_CONSTANT;
	e.constant.length = SDL_HAPTIC_INFINITY;
	e.constant.delay = 0;
	if (d.is_wheel)
	{
		e.constant.direction.type = SDL_HAPTIC_STEERING_AXIS;
		e.constant.direction.dir[0] = 1;
		e.constant.level = Sint16(level);
	}
	else
	{
		e.constant.direction.type = SDL_HAPTIC_CARTESIAN;
		e.constant.direction.dir[0] = (level < 0) ? -1 : 1;
		e.constant.level = Sint16(level < 0 ? -level : level);
	}
	if (level == 0)
	{
		if (running)
			p_SDL_HapticStopEffect(d.hp, d.effect);
		running = false;
		applied = 0;
		return;
	}
	if (p_SDL_HapticUpdateEffect(d.hp, d.effect, &e) < 0)
	{
		static bool s_said = false;
		if (!s_said) { flog("update failed: %s", p_SDL_GetError()); s_said = true; }
		return;
	}
	if (p_SDL_HapticRunEffect(d.hp, d.effect, 1) < 0)
	{
		static bool s_said = false;
		if (!s_said) { flog("run failed: %s", p_SDL_GetError()); s_said = true; }
		return;
	}
	running = true;
	applied = level;
}

static void worker()
{
	Device d;
	p_SDL_SetHint("SDL_JOYSTICK_RAWINPUT", "0");   // DirectInput enumeration (the plugin did the same)
	if (p_SDL_Init(0x00000200u /*JOYSTICK*/ | 0x00001000u /*HAPTIC*/) < 0)   // SDL.h flags, header not included
	{
		flog("SDL_Init failed: %s", p_SDL_GetError());
		s_running.store(false);
		return;
	}
	p_SDL_JoystickUpdate();
	if (!select_device(d))
	{
		p_SDL_Quit();
		s_running.store(false);
		return;
	}
	if (d.caps & SDL_HAPTIC_AUTOCENTER)
		p_SDL_HapticSetAutocenter(d.hp, 0);
	if (d.caps & SDL_HAPTIC_GAIN)
		p_SDL_HapticSetGain(d.hp, 100);
	{
		SDL_HapticEffect e;
		memset(&e, 0, sizeof(e));
		e.type = SDL_HAPTIC_CONSTANT;
		e.constant.direction.type = d.is_wheel ? SDL_HAPTIC_STEERING_AXIS : SDL_HAPTIC_CARTESIAN;
		e.constant.direction.dir[0] = 1;
		e.constant.length = SDL_HAPTIC_INFINITY;
		e.constant.level = 0;
		d.effect = p_SDL_HapticNewEffect(d.hp, &e);
		if (d.effect < 0)
		{
			flog("constant-force effect unavailable: %s - force feedback off", p_SDL_GetError());
			p_SDL_HapticClose(d.hp);
			p_SDL_JoystickClose(d.js);
			p_SDL_Quit();
			s_running.store(false);
			return;
		}
	}
	flog("ready: strength %d%%, invert %d, hold %d ms, smooth %d ms, damper %d%%, friction %d%%, rumble %d%%",
			s_strength, int(s_invert), s_hold_ms, s_smooth_ms, s_damper, s_friction, s_rumble);
	flog("spring: %d%% (the plugin runs one for these games; 0 = off)", s_spring);
	bool rumble_ok = false;
	if (s_rumble > 0)
	{
		if (p_SDL_HapticRumbleSupported(d.hp) == 1 && p_SDL_HapticRumbleInit(d.hp) == 0)
		{
			rumble_ok = true;
			flog("rumble: %d%% - 100 ms burst per force update", s_rumble);
		}
		else
			flog("rumble: not available on this device (%s)", p_SDL_GetError());
	}
	// static condition effects (the arcade wheel mechanism the base lacks)
	// The spring is the one the FFB Arcade Plugin runs for these games
	// (EnableForceSpringEffectCrusnUSA=1 in its stock FFBPlugin.ini) and this
	// code did not have. It is a POSITION-based centring torque, so its
	// authority is greatest exactly where the game's own motor force is
	// smallest - around straight-ahead. Without it the centre goes light, which
	// is what "the self-centring near the centre feels looser than normal"
	// describes. Off by default; the arcade cabinet's own mechanism is what it
	// stands in for, so how much you want is a matter of taste and of base.
	int cond_ids[3] = { -1, -1, -1 };
	struct { int pct; unsigned cap; Uint16 type; const char *name; } const conds[3] = {
		{ s_damper, SDL_HAPTIC_DAMPER, SDL_HAPTIC_DAMPER, "damper" },
		{ s_friction, SDL_HAPTIC_FRICTION, SDL_HAPTIC_FRICTION, "friction" },
		{ s_spring, SDL_HAPTIC_SPRING, SDL_HAPTIC_SPRING, "spring" } };
	for (int i = 0; i < 3; i++)
	{
		if (conds[i].pct <= 0)
			continue;
		if (!(d.caps & conds[i].cap))
		{
			flog("%s: not supported by this device (caps 0x%x)", conds[i].name, d.caps);
			continue;
		}
		SDL_HapticEffect e;
		memset(&e, 0, sizeof(e));
		e.type = conds[i].type;
		e.condition.direction.type = SDL_HAPTIC_STEERING_AXIS;
		e.condition.direction.dir[0] = 1;
		e.condition.length = SDL_HAPTIC_INFINITY;
		// Scale by FFB STRENGTH, and cap the saturation at the same ceiling.
		// These used to be absolute: a 72% spring sat at 72% of the wheel s
		// full force with saturation wide open, while the game s own forces
		// were scaled to FFB STRENGTH (50%). The spring was therefore
		// STRONGER than the feedback it accompanies and reached maximum at a
		// modest angle, burying everything - "the spring masks any other
		// feedback whatsoever" (rig, 2026-09-05). Scaling keeps the balance
		// fixed as STRENGTH moves, which is the only way one dial can mean
		// anything.
		int const eff = conds[i].pct * s_strength / 100;
		if (eff <= 0)
			continue;
		Sint16 const coeff = Sint16(0x7fff * eff / 100);
		Uint16 const sat = Uint16(0xffff * eff / 100);
		e.condition.right_sat[0] = e.condition.left_sat[0] = sat;
		e.condition.right_coeff[0] = e.condition.left_coeff[0] = coeff;
		cond_ids[i] = p_SDL_HapticNewEffect(d.hp, &e);
		if (cond_ids[i] < 0 || p_SDL_HapticRunEffect(d.hp, cond_ids[i], 1) < 0)
			flog("%s: could not start: %s", conds[i].name, p_SDL_GetError());
		else
			flog("%s: %d%% of full (%d%% x strength %d%%)", conds[i].name,
				eff, conds[i].pct, s_strength);
	}

	bool running = false;
	int applied = 0;
	bool hold_said = false;
	if (const char *t = std::getenv("MIDV_FFB_TEST"))
	{
		int const pct = std::clamp(atoi(t), -100, 100);
		int const lvl = pct * 32767 / 100;
		flog("TEST: level %d (%d%%) for 1500 ms", lvl, pct);
		apply(d, lvl, running, applied);
		for (int i = 0; i < 150 && !s_stop.load(); i++)
			Sleep(10);
		apply(d, 0, running, applied);
		flog("TEST: released");
	}
	// ---- conditioning: the toolkit's shaper, driven by a named profile ----
	// Everything this used to do by hand (impulse bypass, dt-aware low-pass,
	// settle-to-zero, re-level threshold) now lives in dbce::force::Shaper and is
	// described by cruisn-vunit@1. A conformance test in the toolkit holds that
	// profile to this code's previous output over 62 s of real captured driving.
	dbce::force::Profile prof;
	{
		std::string why, dir = exe_dir();
		if (!dbce::force::load_profile_dir(dir, s_profile_id, prof, &why))
		{
			flog("profile '%s' not loaded from %s (%s) - using built-in values",
					s_profile_id.c_str(), dir.c_str(), why.c_str());
			// Built-in fallback, identical to cruisn-vunit@1. Force feedback must
			// not depend on a data file being present.
			prof.shaper = dbce::force::ShaperSettings();
			prof.shaper.deadzone = 0.f;
			prof.shaper.attack_smoothing = 0.f;
			prof.shaper.decay_smoothing = 0.f;
			prof.shaper.output_deadband = 0.f;
			prof.shaper.fade_start_kmh = 0.f;
			prof.shaper.fade_full_kmh = 0.f;
			prof.shaper.ramp_seconds = 0.f;
			prof.shaper.impulse_bypass = 0.6f;
			prof.shaper.settle_below = 160.f / 32767.f;
			prof.shaper.releveling = 33.f / 32767.f;
		}
		else
			flog("profile '%s' loaded from %s", s_profile_id.c_str(), dir.c_str());
		// The PROFILE owns smoothing. This used to overwrite it unconditionally
		// with MIDV_FFB_SMOOTH, whose default is 0 - so an unset env silently
		// turned every tune into an unfiltered one, and a set env made the
		// shipped tunes identical to each other. They differ ONLY in this
		// value, so that made the whole family inert. Now the env is a
		// developer override that applies only when actually present.
		if (s_smooth_set)
			prof.shaper.smoothing_ms = float(s_smooth_ms);
		// Their FFB STRENGTH is a plain percent; shaper.strength is 50-is-unity.
		prof.shaper.strength = s_impact_axis ? 50 : std::clamp(s_strength / 2, 0, 100);
		prof.shaper.invert = false;   // sign is applied in midv_ffb_write
		flog("shaper: strength %d (from %d%%), smoothing %.0f ms, impulse %.2f, "
				"settle %.5f, releveling %.5f",
				prof.shaper.strength, s_strength, prof.shaper.smoothing_ms,
				prof.shaper.impulse_bypass, prof.shaper.settle_below, prof.shaper.releveling);
	}
	dbce::force::Shaper shaper(prof.shaper);

	auto last_tick = std::chrono::steady_clock::now();
	dbce::force::RiseDetector impact_detector;
	dbce::force::ImpactMixer impact_mixer;
	while (!s_stop.load())
	{
		{
			std::unique_lock<std::mutex> lk(s_mtx);
			s_cv.wait_for(lk, std::chrono::milliseconds((s_impact_axis || prof.shaper.smoothing_ms > 0) ? 4 : 20), [] { return s_stop.load() || s_dirty; });
			s_dirty = false;
		}
		if (s_stop.load())
			break;
		int want = s_level.load();
		if (s_cancel_impact.exchange(false)) { impact_mixer.reset(); impact_detector.reset(); shaper.reset(); }
		if (s_hold_ms > 0 && want != 0 && now_ms() - s_last_write.load() > s_hold_ms)
		{
			want = 0;
			s_level.store(0);
			impact_detector.reset();
			impact_mixer.reset();
			s_raw_level.store(0);
			if (rumble_ok) p_SDL_HapticRumbleStop(d.hp);
			if (!hold_said) { flog("no motor write for %d ms - force released", s_hold_ms); hold_said = true; }
		}
		else if (want != 0)
			hold_said = false;
		// The detector consumes source units, before strength. Feed every tick,
		// including idle: changed-nonzero-only history missed an isolated hit and
		// dividing this ungained level by a gained maximum changed classification
		// whenever the player moved the strength slider.
		int const candidate_level = s_impact_axis ? s_raw_level.load() : want;
		bool const impact_candidate = impact_detector.observe(float(candidate_level) / 32767.f,
			double(now_ms()) / 1000.0);
		if (impact_candidate)
		{
			// This is a waveform candidate, not a decoded game collision flag.
			// SDL's generic rumble backend still needs wheel-specific evaluation.
			// Normal zero writes do not cut a 120 ms cue short; watchdog/exit do.
			int result = -1;
			if (s_impact_axis)
				impact_mixer.trigger(impact_detector.last_arrival, float(candidate_level), double(now_ms()) / 1000.0);
			if (rumble_ok && !s_impact_axis)
			{
				float const amplitude = impact_detector.last_arrival * float(s_rumble) / 100.f
					* float(s_strength) / 100.f;
				result = p_SDL_HapticRumblePlay(d.hp, amplitude, 120);
				if (result < 0) { flog("rumble play failed: %s", p_SDL_GetError()); rumble_ok = false; }
			}
			flog("impact candidate: arrived %.3f rise %.3f rumble_result %d",
				impact_detector.last_arrival, impact_detector.last_rise, result);
			if (s_ffb_trace)
			{
				auto const tms = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now() - s_ffb_trace_t0).count();
				fprintf(s_ffb_trace, "%lld,impact_candidate,1\n%lld,rumble_result,%d\n",
					(long long)tms, (long long)tms, result);
			}
		}

		// One tick of the shared chain. The impulse bypass, the low-pass, the
		// settle and the re-level threshold are all inside it now.
		auto const now = std::chrono::steady_clock::now();
		double const dt_ms = std::chrono::duration<double, std::milli>(now - last_tick).count();
		last_tick = now;
		float const shaped = shaper.shape(float(want) / 32767.f, 0.f, float(dt_ms / 1000.0), false);
		float const mixed = s_impact_axis ? impact_mixer.mix(shaped, double(now_ms()) / 1000.0,
			float(s_strength) / 100.f) : shaped;
		int const out = int(std::lround(double(mixed) * 32767.0));
		if (out != applied)
		{
			// Trace our SHAPED output beside the game's motor byte, so a run of
			// this and a run of the stock FFB Arcade Plugin can be compared as
			// algorithms rather than as impressions: same input signal, two
			// outputs, one file each. (Diagnostic only; the FILE* is shared with
			// telem_notify on the game thread and CRT stream locking covers it.)
			if (s_ffb_trace)
			{
				auto const tms = std::chrono::duration_cast<std::chrono::milliseconds>(
						std::chrono::steady_clock::now() - s_ffb_trace_t0).count();
				fprintf(s_ffb_trace, "%lld,shaped,%d\n", (long long)tms, out);
				fflush(s_ffb_trace);
			}
			apply(d, out, running, applied);
			if (s_ffb_trace) {
				auto const ms = std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now() - s_ffb_trace_t0).count();
				fprintf(s_ffb_trace, "%lld,constant_api_accepted,%d\n", (long long)ms, applied == out);
			}
		}
	}
	apply(d, 0, running, applied);
	if (rumble_ok)
		p_SDL_HapticRumbleStop(d.hp);
	for (int id : cond_ids)
		if (id >= 0) { p_SDL_HapticStopEffect(d.hp, id); p_SDL_HapticDestroyEffect(d.hp, id); }
	p_SDL_HapticDestroyEffect(d.hp, d.effect);
	p_SDL_HapticClose(d.hp);
	p_SDL_JoystickClose(d.js);
	p_SDL_Quit();
	flog("closed");
	s_running.store(false);
}

static void shutdown()
{
	if (!s_thread.joinable())
		return;
	s_stop.store(true);
	{ std::lock_guard<std::mutex> lk(s_mtx); s_dirty = true; }
	s_cv.notify_all();
	s_thread.join();
	if (s_log) { fclose(s_log); s_log = nullptr; }
}

static void start(running_machine &machine)
{
	const char *on = std::getenv("MIDV_FFB");
	if (!on || atoi(on) == 0)
		return;
	s_t0 = std::chrono::steady_clock::now();
	s_impact_axis = std::getenv("MIDV_FFB_IMPACT") && atoi(std::getenv("MIDV_FFB_IMPACT")) == 1;
	if (const char *l = std::getenv("MIDV_FFB_LOG"))
		s_loglevel = atoi(l);
	s_log = fopen("midv_ffb.log", "w");
	if (const char *s = std::getenv("MIDV_FFB_STRENGTH"))
		s_strength = std::clamp(atoi(s), 0, 100);
	if (const char *s = std::getenv("MIDV_FFB_INVERT"))
		s_invert = atoi(s) != 0;
	if (const char *s = std::getenv("MIDV_FFB_HOLD_MS"))
		s_hold_ms = std::clamp(atoi(s), 0, 60000);
	if (const char *s = std::getenv("MIDV_FFB_SMOOTH"))
	{
		s_smooth_ms = std::clamp(atoi(s), 0, 2000);
		s_smooth_set = true;
	}
	if (const char *s = std::getenv("MIDV_FFB_PROFILE"))
		if (*s)
			s_profile_id = s;
	if (const char *s = std::getenv("MIDV_FFB_DAMPER"))
		s_damper = std::clamp(atoi(s), 0, 100);
	if (const char *s = std::getenv("MIDV_FFB_SPRING"))
		s_spring = std::clamp(atoi(s), 0, 100);
	if (const char *s = std::getenv("MIDV_FFB_FRICTION"))
		s_friction = std::clamp(atoi(s), 0, 100);
	if (const char *s = std::getenv("MIDV_FFB_RUMBLE"))
		s_rumble = std::clamp(atoi(s), 0, 100);
	flog("built-in force feedback for %s; steering impact enhancement=%d", machine.system().name, int(s_impact_axis));
	if (s_strength == 0)
	{
		flog("strength 0 - force feedback off");
		return;
	}
	if (!load_sdl())
		return;
	s_running.store(true);
	s_thread = std::thread(worker);
	machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate([] () { shutdown(); }));
}

} // namespace mvffb

static FILE *s_force_source = nullptr;
static FILE *s_signal_trace = nullptr;
void midv_ffb_source(int raw, int adapted, uint64_t frame, double seconds)
{
	mvffb::s_raw_level.store(cruisn::motor_level(raw, mvffb::s_invert));
	if (s_force_source)
		fprintf(s_force_source, "%.9f,%llu,%d,%d\n", seconds,
			(unsigned long long)frame, raw, adapted);
}

// Motor byte from the drivers (signed, after gain/slew/clamp). Any thread.
void midv_ffb_cancel() { mvffb::s_raw_level.store(0); mvffb::s_cancel_impact.store(true); midv_ffb_write(0); }
void midv_ffb_write(int f)
{
	if (!mvffb::s_running.load())
		return;
	int const level = cruisn::motor_level(f, mvffb::s_invert);
	if (mvffb::s_loglevel >= 2)
		mvffb::flog("write %d -> level %d", f, level);
	mvffb::s_level.store(level);
	mvffb::s_last_write.store(mvffb::now_ms());
	{ std::lock_guard<std::mutex> lk(mvffb::s_mtx); mvffb::s_dirty = true; }
	mvffb::s_cv.notify_one();
}

void midv_telemetry_start(running_machine &machine)
{
	static bool s_started = false;
	if (s_started)
		return;
	s_started = true;
	if (const char *path = std::getenv("MIDV_FFB_SOURCE_TRACE")) {
		s_force_source = fopen(path, "w");
		if (s_force_source) {
			fprintf(s_force_source, "# schema=1 game=%s units=signed_motor_byte clock=emulated\nseconds,frame,raw,adapted\n", machine.system().name);
			machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate([] () {
				fclose(s_force_source); s_force_source = nullptr;
			}));
		}
	}
	if (const char *path = std::getenv("MIDV_SIGNAL_TRACE")) {
		s_signal_trace = fopen(path, "w");
		if (s_signal_trace) {
			fprintf(s_signal_trace, "# schema=1 clock=emulated signal=speed unit=metres_per_second\nseconds,frame,source,quality,sample_seconds,sample_frame,value\n");
			machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate([] () {
				fclose(s_signal_trace); s_signal_trace = nullptr;
			}));
		}
	}
	mvffb::start(machine);   // POC built-in force feedback (MIDV_FFB=1)
	const char *telem_spec = std::getenv("MIDV_TELEM_UDP");
	if (telem_spec || std::getenv("MIDV_TELEM_FORZA") || std::getenv("MIDV_FFB_TRACE") || std::getenv("MIDV_SIGNAL_TRACE"))
		telem_init(telem_spec, machine.system().name);
	if (const char *tr = std::getenv("MIDV_FFB_TRACE"))
	{
		s_ffb_trace = fopen(tr, "w");
		if (s_ffb_trace)
		{
			s_ffb_trace_t0 = std::chrono::steady_clock::now();
			fprintf(s_ffb_trace, "ms,output,value\n# game %s\n",
					machine.system().name);
			fflush(s_ffb_trace);
		}
	}
	// one global notifier serves both the JSON telemetry and the trace
	if (telem_spec || s_ffb_trace)
		machine.output().set_global_notifier(&telem_notify, nullptr);
}

void midvunit_base_state::mvgl_exit()
{
	// POC: a detached GL thread that outlives the machine races teardown
	// (msvcrt!memcpy AVs logged at roughly every second exit). Flag it down
	// and give it up to a second to acknowledge before destruction proceeds.
	mvgl::s_menu_pause.store(0);
	mvgl::s_stop.store(true);
	for (int i = 0; i < 100 && !mvgl::s_done.load(); i++)
		Sleep(10);
}

void midvunit_base_state::video_start()
{
	// POC: only when the GL overlay can actually spawn
	if (std::getenv("MIDV_GL"))
		machine().add_notifier(MACHINE_NOTIFY_EXIT,
			machine_notify_delegate(&midvunit_base_state::mvgl_exit, this));

	midv_telemetry_start(machine());

	m_scanline_timer = timer_alloc(FUNC(midvunit_base_state::scanline_timer_cb), this);
	m_eoi_timer = timer_alloc(FUNC(midvunit_base_state::eoi_timer_cb), this);

	m_poly = std::make_unique<midvunit_renderer>(*this);

	save_item(NAME(m_video_regs));
	save_item(NAME(m_dma_data));
	save_item(NAME(m_dma_data_index));
	save_item(NAME(m_page_control));
	save_item(NAME(m_video_changed));

	m_video_changed = true;
}

void midvunit_base_state::device_post_load()
{
	m_video_changed = true;
	reset_display_assets();
}

void midvunit_base_state::reset_display_assets()
{
#ifdef _WIN32
	if (std::getenv("MIDV_GL"))
	{
		live().ui_assets.release();
		live().tex_dirty = true;
	}
#endif
}

/*************************************
 *
 *  Generic flat quad renderer
 *
 *************************************/

void midvunit_renderer::render_flat(int32_t scanline, const extent_t &extent, const midvunit_object_data &objectdata, int threadid)
{
	uint16_t const pixdata = objectdata.pixdata;
	int const xstep = objectdata.dither + 1;
	uint16_t *dest = objectdata.destbase + scanline * 512;
	int startx = extent.startx;

	// if dithering, ensure that we start on an appropriate pixel
	startx += (scanline ^ startx) & objectdata.dither;

	// non-dithered 0 pixels can use a memset
	if (pixdata == 0 && xstep == 1)
		memset(&dest[startx], 0, 2 * (extent.stopx - startx + 1));

	// otherwise, we fill manually
	else
	{
		for (int x = startx; x < extent.stopx; x += xstep)
			dest[x] = pixdata;
	}
}



/*************************************
 *
 *  Generic textured quad renderers
 *
 *************************************/

void midvunit_renderer::render_tex(int32_t scanline, const extent_t &extent, const midvunit_object_data &objectdata, int threadid)
{
	uint16_t const pixdata = objectdata.pixdata;
	uint8_t const *const texbase = objectdata.texbase;
	int const xstep = objectdata.dither + 1;
	uint16_t *const dest = objectdata.destbase + scanline * 512;
	int startx = extent.startx;
	int const stopx = extent.stopx;
	int32_t u = extent.param[0].start;
	int32_t v = extent.param[1].start;
	int32_t dudx = extent.param[0].dpdx;
	int32_t dvdx = extent.param[1].dpdx;

	// if dithering, we advance by 2x; also ensure that we start on an appropriate pixel
	if (xstep == 2)
	{
		if ((scanline ^ startx) & 1)
		{
			startx++;
			u += dudx;
			v += dvdx;
		}
		dudx *= 2;
		dvdx *= 2;
	}

	// general case; render every pixel
	for (int x = startx; x < stopx; x += xstep)
	{
		dest[x] = pixdata + texbase[((v >> 8) & 0xff00) + (u >> 16)];
		u += dudx;
		v += dvdx;
	}
}


void midvunit_renderer::render_textrans(int32_t scanline, const extent_t &extent, const midvunit_object_data &objectdata, int threadid)
{
	uint16_t const pixdata = objectdata.pixdata;
	uint8_t const *const texbase = objectdata.texbase;
	int const xstep = objectdata.dither + 1;
	uint16_t *const dest = objectdata.destbase + scanline * 512;
	int startx = extent.startx;
	int const stopx = extent.stopx;
	int32_t u = extent.param[0].start;
	int32_t v = extent.param[1].start;
	int32_t dudx = extent.param[0].dpdx;
	int32_t dvdx = extent.param[1].dpdx;

	// if dithering, we advance by 2x; also ensure that we start on an appropriate pixel
	if (xstep == 2)
	{
		if ((scanline ^ startx) & 1)
		{
			startx++;
			u += dudx;
			v += dvdx;
		}
		dudx *= 2;
		dvdx *= 2;
	}

	// general case; render every non-zero texel
	for (int x = startx; x < stopx; x += xstep)
	{
		uint8_t const pix = texbase[((v >> 8) & 0xff00) + (u >> 16)];
		if (pix != 0)
			dest[x] = pixdata + pix;
		u += dudx;
		v += dvdx;
	}
}


void midvunit_renderer::render_textransmask(int32_t scanline, const extent_t &extent, const midvunit_object_data &objectdata, int threadid)
{
	uint16_t const pixdata = objectdata.pixdata;
	uint8_t const *const texbase = objectdata.texbase;
	int const xstep = objectdata.dither + 1;
	uint16_t *const dest = objectdata.destbase + scanline * 512;
	int startx = extent.startx;
	int const stopx = extent.stopx;
	int32_t u = extent.param[0].start;
	int32_t v = extent.param[1].start;
	int32_t dudx = extent.param[0].dpdx;
	int32_t dvdx = extent.param[1].dpdx;

	// if dithering, we advance by 2x; also ensure that we start on an appropriate pixel
	if (xstep == 2)
	{
		if ((scanline ^ startx) & 1)
		{
			startx++;
			u += dudx;
			v += dvdx;
		}
		dudx *= 2;
		dvdx *= 2;
	}

	// general case; every non-zero texel renders pixdata
	for (int x = startx; x < stopx; x += xstep)
	{
		uint8_t const pix = texbase[((v >> 8) & 0xff00) + (u >> 16)];
		if (pix != 0)
			dest[x] = pixdata;
		u += dudx;
		v += dvdx;
	}
}



/*************************************
 *
 *  DMA queue processor
 *
 *************************************/

void midvunit_renderer::make_vertices_inclusive(vertex_t *vert)
{
	// build up a mask of right and bottom points
	// note we assume clockwise orientation here
	uint8_t rmask = 0, bmask = 0, eqmask = 0;
	for (int vnum = 0; vnum < 4; vnum++)
	{
		vertex_t *currv = &vert[vnum];
		vertex_t *nextv = &vert[(vnum + 1) & 3];

		// if this vertex equals the next one, tag it
		if (nextv->y == currv->y && nextv->x == currv->x)
			eqmask |= 1 << vnum;

		// if the next vertex is down from us, we are a right coordinate
		if (nextv->y > currv->y || (nextv->y == currv->y && nextv->x < currv->x))
			rmask |= 1 << vnum;

		// if the next vertex is left from us, we are a bottom coordinate
		if (nextv->x < currv->x || (nextv->x == currv->x && nextv->y < currv->y))
			bmask |= 1 << vnum;
	}

	// bail on the edge case
	if (eqmask == 0x0f)
		return;

	// adjust the right/bottom points so that they get included
	for (int vnum = 0; vnum < 4; vnum++)
	{
		vertex_t *currv = &vert[vnum];
		int effvnum = vnum;

		// if we're equal to the next vertex, use that instead
		while (eqmask & (1 << effvnum))
			effvnum = (effvnum + 1) & 3;

		// adjust the points
		if (rmask & (1 << effvnum))
			currv->x += 0.001f;
		if (bmask & (1 << effvnum))
			currv->y += 0.001f;
	}
}


// per-frame DMA quad count (emu thread), for the MINQUADS statedump trigger
static uint32_t s_last_scene_quads = 0;
static uint32_t s_cur_frame_quads = 0, s_cur_frame_no = 0xffffffff;

void midvunit_base_state::observe_numeric_hud()
{
	// USA v4.5: numeric text buffer produced by A7C2 and consumed by 7A91.
	// Accept only the foreground speed digits actually submitted on this page.
	// Opcodes guard the ROM revision; the short lifetime rejects retained menu text.
	static bool const numeric_rom = strcmp(machine().system().name, "crusnusa") == 0;
	const auto &dq = m_dma_data;
	if (numeric_rom && dq[0] == 0x900 && dq[14] == 0x2f8b && dq[3] == 346 &&
		dq[7] == 368 && dq[2] >= 14 && dq[4] <= 72 &&
		m_ram_base[0xa7c2] == 0x1541c200 && m_ram_base[0x7a91] == 0x0848c200)
	{
		int const pg = (m_page_control & 4) ? 1 : 0;
		s_numeric_mph[pg] = cruisn::packed_hud_speed(m_ram_base[0xe632]);
		s_numeric_frame[pg] = m_screen->frame_number();
		s_numeric_seconds[pg] = machine().time().as_double();
		s_numeric_seen[pg] = true;
	}
}

void midvunit_renderer::process_dma_queue()
{
	m_state.observe_numeric_hud();
	{
		uint32_t const fr = uint32_t(m_state.m_screen->frame_number());
		if (fr != s_cur_frame_no)
		{
			s_last_scene_quads = s_cur_frame_quads;
			s_cur_frame_quads = 0;
			s_cur_frame_no = fr;
		}
		++s_cur_frame_quads;
	}

	// POC: log the raw quad before any processing, tagged with frame + page
	static FILE *s_quadlog = quadlog_open();
	if (s_quadlog)
	{
		uint32_t const frame = uint32_t(m_state.m_screen->frame_number());
		uint16_t const page = m_state.m_page_control;
		std::fwrite(&frame, 4, 1, s_quadlog);
		std::fwrite(&page, 2, 1, s_quadlog);
		std::fwrite(m_state.m_dma_data, 2, 16, s_quadlog);
	}

	// live bridge: pending big state first (order matters), then the quad
	if (live().enabled)
	{
		uint32_t const frame = uint32_t(m_state.m_screen->frame_number());
		live().last_frame = frame;
		live().sync_state(frame,
			m_state.m_paletteram.target(), uint32_t(m_state.m_paletteram.bytes()),
			m_state.m_textureram.target(), uint32_t(m_state.m_textureram.bytes()));
		struct { uint32_t frame; uint16_t pc, pad; } h =
			{ frame, m_state.m_page_control, 0 };
		live().write_msg(1, &h, 8, m_state.m_dma_data, 32);
	}

	// if we're rendering to the same page we're viewing, it has changed
	if ((((m_state.m_page_control >> 2) ^ m_state.m_page_control) & 1) == 0 || WATCH_RENDER)
		m_state.m_video_changed = true;

	// fill in the vertex data
	vertex_t vert[4];
	vert[0].x = (float)(int16_t)m_state.m_dma_data[2] + 0.5f;
	vert[0].y = (float)(int16_t)m_state.m_dma_data[3] + 0.5f;
	vert[1].x = (float)(int16_t)m_state.m_dma_data[4] + 0.5f;
	vert[1].y = (float)(int16_t)m_state.m_dma_data[5] + 0.5f;
	vert[2].x = (float)(int16_t)m_state.m_dma_data[6] + 0.5f;
	vert[2].y = (float)(int16_t)m_state.m_dma_data[7] + 0.5f;
	vert[3].x = (float)(int16_t)m_state.m_dma_data[8] + 0.5f;
	vert[3].y = (float)(int16_t)m_state.m_dma_data[9] + 0.5f;

	// make the vertices inclusive of right/bottom points
	make_vertices_inclusive(vert);

	// set the palette base
	uint16_t pixdata = m_state.m_dma_data[1];

	render_delegate callback;
	bool const textured = ((m_state.m_dma_data[0] & 0x300) == 0x100);

	// handle flat-shaded quads here
	if (!textured)
	{
		callback = render_delegate(&midvunit_renderer::render_flat, this);
		pixdata += (m_state.m_dma_data[0] & 0x00ff);
	}
	// handle textured quads here
	else
	{
		// add the texture info
		vert[0].p[0] = (float)(m_state.m_dma_data[10] & 0xff) * 65536.0f + 32768.0f;
		vert[0].p[1] = (float)(m_state.m_dma_data[10] >> 8) * 65536.0f + 32768.0f;
		vert[1].p[0] = (float)(m_state.m_dma_data[11] & 0xff) * 65536.0f + 32768.0f;
		vert[1].p[1] = (float)(m_state.m_dma_data[11] >> 8) * 65536.0f + 32768.0f;
		vert[2].p[0] = (float)(m_state.m_dma_data[12] & 0xff) * 65536.0f + 32768.0f;
		vert[2].p[1] = (float)(m_state.m_dma_data[12] >> 8) * 65536.0f + 32768.0f;
		vert[3].p[0] = (float)(m_state.m_dma_data[13] & 0xff) * 65536.0f + 32768.0f;
		vert[3].p[1] = (float)(m_state.m_dma_data[13] >> 8) * 65536.0f + 32768.0f;

		// handle non-masked, non-transparent quads
		if ((m_state.m_dma_data[0] & 0xc00) == 0x000)
		{
			callback = render_delegate(&midvunit_renderer::render_tex, this);
		}
		// handle non-masked, transparent quads
		else if ((m_state.m_dma_data[0] & 0xc00) == 0x800)
		{
			callback = render_delegate(&midvunit_renderer::render_textrans, this);
		}
		// handle masked, transparent quads
		else if ((m_state.m_dma_data[0] & 0xc00) == 0xc00)
		{
			callback = render_delegate(&midvunit_renderer::render_textransmask, this);
			pixdata += (m_state.m_dma_data[0] & 0x00ff);
		}
		// handle masked, non-transparent quads (invalid?)
		else
		{
			callback = render_delegate(&midvunit_renderer::render_flat, this);
			pixdata += (m_state.m_dma_data[0] & 0x00ff);
		}
	}

	// set up the object data for this triangle
	midvunit_object_data &objectdata = object_data().next();
	objectdata.destbase = &m_state.m_videoram[(m_state.m_page_control & 4) ? 0x40000 : 0x00000];
	objectdata.texbase = (uint8_t *)m_state.m_textureram.target() + (m_state.m_dma_data[14] * 256);
	objectdata.pixdata = pixdata;
	objectdata.dither = ((m_state.m_dma_data[0] & 0x2000) != 0);

	// render as a quad
	if (textured)
		render_polygon<4, 2>(m_state.m_screen->visible_area(), callback, vert);
	else
		render_polygon<4, 0>(m_state.m_screen->visible_area(), callback, vert);
}



/*************************************
 *
 *  DMA pipe control control
 *
 *************************************/

void midvunit_base_state::dma_queue_w(uint32_t data)
{
	if (machine().input().code_pressed(KEYCODE_L))
		LOGMASKED(LOG_DMA, "%06X:queue(%X) = %08X\n", m_maincpu->pc(), m_dma_data_index, data);
	if (m_dma_data_index < 16)
	{
		// POC (env-gated): log each unique DSP PC that writes the quad DMA
		// port - locates a game's poly-emit loops in one full-speed run
		// (the debugger-watchpoint route runs at ~0.03x and crawls).
		static const char *const s_pclog = std::getenv("MIDV_DMA_PCLOG");
		if (const char *pclog = s_pclog)
		{
			static std::set<uint32_t> s_seen;
			uint32_t const pc = m_maincpu->pc();
			if (s_seen.insert(pc).second)
				if (FILE *f = fopen(pclog, "a"))
				{
					fprintf(f, "%05X", pc);
					fputc(10, f);
					fclose(f);
				}
		}
		m_dma_data[m_dma_data_index++] = data;
	}
}


uint32_t midvunit_base_state::dma_queue_entries_r()
{
	// always return 0 entries
	return 0;
}


uint32_t midvunit_base_state::dma_trigger_r(offs_t offset)
{
	if (offset)
	{
		if (!machine().side_effects_disabled())
		{
			if (machine().input().code_pressed(KEYCODE_L))
				LOGMASKED(LOG_DMA, "%06X:trigger\n", m_maincpu->pc());
			m_poly->process_dma_queue();
			m_dma_data_index = 0;
		}
	}
	return 0;
}



/*************************************
 *
 *  Paging control
 *
 *************************************/

void midvunit_base_state::page_control_w(uint32_t data)
{
	// watch for the display page to change
	if ((m_page_control ^ data) & 1)
	{
		m_video_changed = true;
		if (machine().input().code_pressed(KEYCODE_L))
			LOGMASKED(LOG_DMA, "##########################################################\n");
		m_screen->update_partial(m_screen->vpos() - 1);
	}
	if (live().enabled)
	{
		uint32_t const frame = uint32_t(m_screen->frame_number());
		live().last_frame = frame;
		live().flush_span();
		struct { uint32_t frame; uint16_t oldpc, newpc; } h =
			{ frame, uint16_t(m_page_control), uint16_t(data) };
		live().write_msg(2, &h, 8);
	}
	m_page_control = data;
}


uint32_t midvunit_base_state::page_control_r()
{
	return m_page_control;
}



/*************************************
 *
 *  Video control
 *
 *************************************/

void midvunit_base_state::video_control_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	uint16_t const old = m_video_regs[offset];

	// update the data
	COMBINE_DATA(&m_video_regs[offset]);

	// update the scanline timer
	if (offset == 0)
		m_scanline_timer->adjust(m_screen->time_until_pos((data & 0x1ff) + 1, 0), data & 0x1ff);

	// if something changed, update our parameters
	if (old != m_video_regs[offset] && m_video_regs[6] != 0 && m_video_regs[11] != 0)
	{
		rectangle visarea;

		// derive visible area from blanking
		visarea.min_x = 0;
		visarea.max_x = (m_video_regs[6] + m_video_regs[2] - m_video_regs[5]) % m_video_regs[6];
		visarea.min_y = 0;
		visarea.max_y = (m_video_regs[11] + m_video_regs[7] - m_video_regs[10]) % m_video_regs[11];
		m_screen->configure(m_video_regs[6], m_video_regs[11], visarea, HZ_TO_ATTOSECONDS(MIDVUNIT_VIDEO_CLOCK / 2) * m_video_regs[6] * m_video_regs[11]);
	}
}


uint32_t midvunit_base_state::scanline_r()
{
	return m_screen->vpos();
}



/*************************************
 *
 *  Video RAM access
 *
 *************************************/

void midvunit_base_state::videoram_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	m_poly->wait("Video RAM write");
	if (!m_video_changed)
	{
		int const visbase = (m_page_control & 1) ? 0x40000 : 0x00000;
		if ((offset & 0x40000) == visbase)
			m_video_changed = true;
	}
	COMBINE_DATA(&m_videoram[offset]);
	if (live().enabled)   // post-COMBINE value, so masking is already applied
		live().vram_write(uint32_t(m_screen->frame_number()), offset,
			uint16_t(m_videoram[offset]));
}


uint32_t midvunit_base_state::videoram_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		m_poly->wait("Video RAM read");
	return m_videoram[offset];
}



/*************************************
 *
 *  Palette RAM access
 *
 *************************************/

void midvunit_base_state::paletteram_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	COMBINE_DATA(&m_paletteram[offset]);
	uint32_t const newword = m_paletteram[offset];
	m_palette->set_pen_color(offset, pal5bit(newword >> 10), pal5bit(newword >> 5), pal5bit(newword >> 0));
	live().pal_dirty = true;
}



/*************************************
 *
 *  Texture RAM access
 *
 *************************************/

void midvunit_base_state::textureram_w(offs_t offset, uint32_t data)
{
	uint8_t *const base = (uint8_t *)m_textureram.target();
	m_poly->wait("Texture RAM write");
#ifdef _WIN32
	// World 2.4 loads level textures over the still-linked transmission UI.
	// Retain the outgoing atlas for enhanced presentation ONLY. Guest RAM,
	// DMA, instruction timing and native snapshots remain untouched.
	if (offset * 2 >= 0x393000 && offset * 2 < 0x3c1000
		&& live().enabled && live().retain_ui_assets && !live().ui_assets.active()
		&& !strcmp(machine().system().name, "crusnwld24")
		&& cruisn::world24_transmission_visible(m_ram_base, m_ram_base.bytes() / 4))
	{
		live().ui_assets.capture(base, m_textureram.bytes(), 0x393000, 0x2e000);
		osd_printf_info("MIDV UI assets retained: frame=%llu World 2.4 transmission\n",
			(unsigned long long)m_screen->frame_number());
	}
#endif
	base[offset * 2] = data;
	base[offset * 2 + 1] = data >> 8;
	live().tex_dirty = true;
}


uint32_t midvunit_base_state::textureram_r(offs_t offset)
{
	uint8_t const *const base = (uint8_t *)m_textureram.target();
	return (base[offset * 2 + 1] << 8) | base[offset * 2];
}




/*************************************
 *
 *  Video system update
 *
 *************************************/

uint32_t midvunit_base_state::screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	uint32_t offset;

	m_poly->wait("Refresh Time");
#ifdef _WIN32
	if (live().ui_assets.active()
		&& !cruisn::world24_transmission_visible(m_ram_base, m_ram_base.bytes() / 4))
	{
		live().ui_assets.release();
		live().tex_dirty = true;
		osd_printf_info("MIDV UI assets released: frame=%llu\n",
			(unsigned long long)screen.frame_number());
	}
#endif

	// POC: the games' startup code re-copies the low program words from
	// ROM after machine_reset applied MIDV_PATCH - re-assert reverted
	// entries once per frame (no-op without MIDV_PATCH)
	midv_patches_tick(m_ram_base);
	if (live().enabled)
		live().speed_pct = float(machine().video().speed_percent() * 100.0);
	midv_trace_wheelpos(machine(), ":WHEEL");   // POC: FFB trace, input half (every frame)

	// Esc options menu pause: block the emu thread here while the menu is
	// open (freezes emulation + sound). The GL thread clears s_menu_pause
	// on Resume/Exit so this always exits. machine().pause() was unreliable
	// - its resume needed screen_update to run again while paused, which
	// MAME does not do, leaving the menu stuck paused. Pump messages so the
	// window stays responsive (Exit clears the flag before its WM_CLOSE).
	while (mvgl::s_menu_pause.load())
	{
		MSG msg;
		while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessageA(&msg);
		}
		Sleep(15);
	}

	// Telemetry Phase B: mirror the car speed (MPH) as a UDP datagram each
	// frame, alongside the Phase-A output mirror. Only when telemetry is on
	// and this game's speed word has been hunted.
	// Gating (found live on the rig): OUTSIDE races the DSP words hold
	// unrelated data - the "speed" word spiked to ~990 in menus/transitions
	// (pegging SimHub gauges). Unreadable speed uses bounded held/invalid state.
	// RPM is unavailable until a real producer is validated; never derive it
	// from the speed formatter's packed ASCII buffer.
	float telem_mph = 0.0f;
	if (s_speed_word)
	{
		float v = c3x_to_float(m_ram_base[s_speed_word]);
		if (v >= 0.0f && v < 320.0f)   // real max ~301; menu junk seen at 373/990
			telem_mph = v;
	}
	else if (s_hud)
	{
		int const mph = hud_ocr_mph(&m_videoram[0], m_page_control, s_hud);
		telem_mph = float(s_hud_speed.observe(mph));
	}
	int const numeric_page = (m_page_control & 1) ? 1 : 0;
	uint64_t const numeric_age = screen.frame_number() - s_numeric_frame[numeric_page];
	int const numeric_mph = s_numeric_seen[numeric_page] && numeric_age <= 3 ? s_numeric_mph[numeric_page] : -1;
	bool const numeric_enabled = !std::getenv("MIDV_SPEED_NUMERIC") || atoi(std::getenv("MIDV_SPEED_NUMERIC")) != 0;
	bool const numeric_used = numeric_enabled && numeric_mph >= 0;
	if (numeric_used) telem_mph = float(numeric_mph);
	// Versioned shared sample contract. Held OCR retains its actual acceptance
	// timestamp; an unavailable value is distinguishable from a measured zero.
	static dbce::telemetry::SignalSample speed_sample;
	if (numeric_used) {
		speed_sample.source = dbce::telemetry::Source::numeric_hud;
		speed_sample.quality = numeric_age ? dbce::telemetry::Quality::held : dbce::telemetry::Quality::fresh;
		speed_sample.seconds = s_numeric_seconds[numeric_page];
		speed_sample.frame = s_numeric_frame[numeric_page];
		speed_sample.value = double(telem_mph) * .44704;
	} else if (s_hud && s_hud_speed.has_value) {
		if (s_hud_speed.fresh || speed_sample.source != dbce::telemetry::Source::hud_ocr) {
			speed_sample.seconds = machine().time().as_double();
			speed_sample.frame = screen.frame_number();
		}
		speed_sample.source = dbce::telemetry::Source::hud_ocr;
		speed_sample.quality = s_hud_speed.fresh ? dbce::telemetry::Quality::fresh : dbce::telemetry::Quality::held;
		speed_sample.value = double(telem_mph) * .44704;
	} else {
		speed_sample = dbce::telemetry::SignalSample{};
	}
	speed_sample.unit = dbce::telemetry::Unit::metres_per_second;
	if (s_signal_trace)
		fprintf(s_signal_trace, "%.9f,%llu,%d,%d,%.9f,%llu,%.9f\n", machine().time().as_double(),
			(unsigned long long)screen.frame_number(), int(speed_sample.source), int(speed_sample.quality),
			speed_sample.seconds, (unsigned long long)speed_sample.frame, speed_sample.value);


	// Emit the speed whenever anything is listening OR the trace is open.
	// It used to be gated on s_speed_word, which is 0 for every game (the
	// RAM addresses were retired for the HUD OCR), so the speed the OCR
	// read never reached the trace - and a support bundle could not answer
	// "what did the reader actually see?" about a dropout report.
	if (s_telem_sock != INVALID_SOCKET || s_ffb_trace)
		telem_notify("speed", s32(telem_mph + 0.5f), nullptr);
	if (s_ffb_trace)
	{
		telem_notify("speed_numeric_hud", numeric_mph, nullptr);
		telem_notify("speed_source", numeric_used ? 2 : (s_hud && s_hud_speed.status() ? 1 : 0), nullptr);
		telem_notify("speed_cells", s_hud_cells, nullptr);
		telem_notify("speed_leftcol", s_hud_leftcol, nullptr);
		telem_notify("speed_ocr_reading", s_hud ? s_hud_speed.raw : -1, nullptr);
		telem_notify("speed_status", s_hud ? s_hud_speed.status() : 0, nullptr);
		telem_notify("speed_age_frames", s_hud ? s_hud_speed.age_frames : 0, nullptr);
	}
	if (s_telem_sock != INVALID_SOCKET || s_ffb_trace)
		telem_notify("rpm_status", 0, nullptr); // unavailable, not a measured zero

	// Telemetry Phase C: Forza Horizon 4/5 "Data Out" packet (324 bytes),
	// so SimHub / dash apps consume us as Forza with stock profiles.
	// Layout: FM7 sled (0-231) + 12-byte Horizon block + dash section.
	// RPM fields remain zero: this legacy packet has no per-signal validity
	// field. JSON/diagnostics publish rpm_status=0 and omit the RPM value.
	if (s_forza_n > 0 && s_telem_sock != INVALID_SOCKET)
	{
		float const mph = telem_mph;   // gated above
		uint8_t pkt[324] = { 0 };
		auto put32 = [&pkt](int off, const void *v) { memcpy(pkt + off, v, 4); };
		const int32_t one = 1;
		const float spd_ms = mph * 0.44704f;
		s_forza_ms += 17;
		put32(0, &one);              // IsRaceOn
		put32(4, &s_forza_ms);       // TimestampMS
		put32(40, &spd_ms);          // VelocityZ (forward, m/s)
		put32(256, &spd_ms);         // Speed (m/s, FH dash offset)
		// Gear: mirror the REAL latched shifter (driver-maintained state,
		// fed by both the 4-position shifter and the sequential buttons).
		// Neutral shows 1st - Forza has no neutral code and 0 means reverse.
		uint8_t gear = 1;
		if (auto *mvs = dynamic_cast<midvunit_state *>(this))
			switch (mvs->shifter_state())
			{
				case 0x2000: gear = 1; break;
				case 0x1000: gear = 2; break;
				case 0x0800: gear = 3; break;
				case 0x0400: gear = 4; break;
				default: break;
			}
		pkt[319] = gear;
		for (int fi = 0; fi < s_forza_n; fi++)
			sendto(s_telem_sock, (const char *)pkt, sizeof(pkt), 0,
					(const sockaddr *)&s_forza_addr[fi], sizeof(s_forza_addr[fi]));
	}

	// live bridge: palette/texture must flow even before any quad is drawn -
	// boot and test screens are CPU-drawn, and without this the palette never
	// reaches the renderer until the first 3D scene (boot showed black).
	if (live().enabled)
	{
		live().sync_state(uint32_t(screen.frame_number()),
			m_paletteram.target(), uint32_t(m_paletteram.bytes()),
			m_textureram.target(), uint32_t(m_textureram.bytes()));
		// update_partial can call this more than once in one emulated frame.
		// Only the final visible scanline seals the displayed transaction.
		if (cliprect.max_y >= screen.visible_area().max_y)
		{
			uint32_t const frame = uint32_t(screen.frame_number());
			live().write_msg(6, &frame, sizeof(frame));
		}
	}

	// POC telemetry Phase B RAM hunt: dump the DSP work RAM every N frames
	// (MIDV_RAMDUMP_DIR + MIDV_RAMDUMP_EVERY, default 30) so a demo-race
	// acceleration can be differential-searched offline for speed/RPM.
	{
		static const char *s_rdir = std::getenv("MIDV_RAMDUMP_DIR");
		static int s_every = std::getenv("MIDV_RAMDUMP_EVERY")
			? atoi(std::getenv("MIDV_RAMDUMP_EVERY")) : 30;
		if (s_rdir && (screen.frame_number() % std::max(1, s_every)) == 0)
		{
			char path[512];
			snprintf(path, sizeof(path), "%s/ram_%06u.bin", s_rdir,
				uint32_t(screen.frame_number()));
			if (FILE *f = std::fopen(path, "wb"))
			{
				std::fwrite(m_ram_base.target(), 1, 0x20000 * 4, f);
				std::fclose(f);
			}
			// second work-RAM bank (0x400000): the live player physics was
			// proven NOT to be in m_ram_base (no word matched a 5x
			// accelerate-to-full-stop fingerprint) - it must be here
			if (m_fastram.found())
			{
				snprintf(path, sizeof(path), "%s/ram2_%06u.bin", s_rdir,
					uint32_t(screen.frame_number()));
				if (FILE *f = std::fopen(path, "wb"))
				{
					std::fwrite(m_fastram.target(), 1, 0x20000 * 4, f);
					std::fclose(f);
				}
			}
			// C31 on-chip internal RAM (2K words at 0x809800): the last
			// unexplored memory - neither external bank correlates with the
			// on-screen MPH under any decoding, so the hot player state
			// must live here
			{
				address_space &sp = m_maincpu->space(AS_PROGRAM);
				static uint32_t buf[0x800];
				for (int i = 0; i < 0x800; i++)
					buf[i] = sp.read_dword(0x809800 + i);
				snprintf(path, sizeof(path), "%s/ram3_%06u.bin", s_rdir,
					uint32_t(screen.frame_number()));
				if (FILE *f = std::fopen(path, "wb"))
				{
					std::fwrite(buf, 4, 0x800, f);
					std::fclose(f);
				}
			}
			// HUD ground truth: bottom quarter of the VISIBLE page (rows
			// 300-399) - the on-screen MPH digits + gear digit, OCR'd
			// offline and correlated against every RAM word to locate the
			// true player state (shape-hunting kept finding drones)
			{
				uint32_t base = (m_page_control & 1) ? 0x40000 : 0x00000;
				// per-game window: offroadc draws its MPH box at the TOP
				// of the screen; everyone else at the bottom
				static int s_hud_row0 = -1;
				if (s_hud_row0 < 0)
					s_hud_row0 = strcmp(machine().system().name, "offroadc") == 0
						? 0 : 300;
				snprintf(path, sizeof(path), "%s/hud_%06u.bin", s_rdir,
					uint32_t(screen.frame_number()));
				if (FILE *f = std::fopen(path, "wb"))
				{
					std::fwrite(&m_videoram[base + s_hud_row0 * 512], 2, 100 * 512, f);
					std::fclose(f);
				}
			}
		}
	}

	// POC: one-shot memory dump so the quad stream can be re-rasterized and
	// verified offline. Runs after the poly wait, so all quads have landed.
	{
		static bool s_dumped = false;
		static const char *s_dir = std::getenv("MIDV_STATEDUMP_DIR");
		static const char *s_frame = std::getenv("MIDV_STATEDUMP_FRAME");
		// MIDV_STATEDUMP_MINQUADS: arm on the first frame past the floor
		// whose last scene had >= N quads (a dense 3D driving scene), so a
		// specific scene can be captured without frame-number guessing -
		// mirrors the Zeus MIDZ_CAPTURE_MINQUADS trigger.
		static const char *s_minq = std::getenv("MIDV_STATEDUMP_MINQUADS");
		bool trigger = s_frame
			&& screen.frame_number() >= strtoul(s_frame, nullptr, 10);
		if (s_minq)
			trigger = trigger && s_last_scene_quads >= atoi(s_minq);
		if (!s_dumped && s_dir && trigger)
		{
			s_dumped = true;
			auto dump = [&](const char *name, const void *data, size_t bytes)
			{
				std::string path = std::string(s_dir) + "/" + name;
				if (FILE *f = std::fopen(path.c_str(), "wb"))
				{
					std::fwrite(data, 1, bytes, f);
					std::fclose(f);
				}
			};
			dump("videoram.bin", m_videoram.target(), m_videoram.bytes());
			dump("textureram.bin", m_textureram.target(), m_textureram.bytes());
			dump("paletteram.bin", m_paletteram.target(), m_paletteram.bytes());
			std::string meta = std::string(s_dir) + "/meta.txt";
			if (FILE *f = std::fopen(meta.c_str(), "w"))
			{
				std::fprintf(f, "frame %u\npage_control %u\n"
					"visible_page_offset 0x%x\nvisarea %d %d\n",
					uint32_t(screen.frame_number()), m_page_control,
					(m_page_control & 1) ? 0x40000 : 0x00000,
					screen.visible_area().max_x, screen.visible_area().max_y);
				std::fclose(f);
			}
		}
	}

	// if the video didn't change, indicate as much
	if (!m_video_changed)
		return UPDATE_HAS_NOT_CHANGED;
	m_video_changed = false;

	// determine the base of the videoram
#if WATCH_RENDER
	offset = (m_page_control & 4) ? 0x40000 : 0x00000;
#else
	offset = (m_page_control & 1) ? 0x40000 : 0x00000;
#endif

	// determine how many pixels to copy
	int const xoffs = cliprect.min_x;
	int const width = cliprect.max_x - xoffs + 1;

	// adjust the offset
	offset += xoffs;
	offset += 512 * (cliprect.min_y - screen.visible_area().min_y);

	// loop over rows
	for (int y = cliprect.min_y; y <= cliprect.max_y; y++)
	{
		uint16_t *dest = &bitmap.pix(y, cliprect.min_x);
		for (int x = 0; x < width; x++)
			*dest++ = m_videoram[offset + x] & 0x7fff;
		offset += 512;
	}
	return 0;
}
