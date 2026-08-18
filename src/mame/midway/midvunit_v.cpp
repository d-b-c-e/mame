// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Driver for Midway V-Unit games

**************************************************************************/

#include "emu.h"
#include "midvunit.h"

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
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <algorithm>
#include <thread>
#include <vector>
#include <string>
#ifdef _WIN32
#include <windows.h>
#include "midvunit_gl_shaders.h"
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
// process (cruisn-poc/gpu/live_viewer.py) renders and presents them live.
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
#ifdef _WIN32

struct midv_live
{
	static constexpr uint32_t RING_SIZE = 128u << 20;
	static constexpr uint32_t HDR_SIZE = 64;

	volatile uint8_t *base = nullptr;
	volatile uint64_t *wpos = nullptr;
	volatile uint64_t *rpos = nullptr;
	volatile uint64_t *dropped = nullptr;
	uint8_t *data = nullptr;
	bool enabled = false;
	bool tex_dirty = true;    // force initial snapshots
	bool pal_dirty = true;
	uint32_t last_frame = 0;

	// coalescing buffer for CPU videoram writes
	uint32_t span_start = 0xffffffff;
	uint32_t span_count = 0;
	uint16_t span_data[2048];

	midv_live()
	{
		if (!std::getenv("MIDV_LIVE") && !std::getenv("MIDV_GL"))
			return;
		HANDLE h = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr,
			PAGE_READWRITE, 0, HDR_SIZE + RING_SIZE, "Local\\MIDV_LIVE");
		if (!h)
			return;
		base = (volatile uint8_t *)MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		if (!base)
			return;
		memset((void *)base, 0, HDR_SIZE);
		memcpy((void *)base, "MVL1", 4);
		*(volatile uint32_t *)(base + 4) = RING_SIZE;
		wpos = (volatile uint64_t *)(base + 8);
		rpos = (volatile uint64_t *)(base + 16);
		dropped = (volatile uint64_t *)(base + 24);
		data = (uint8_t *)(base + HDR_SIZE);
		enabled = true;
	}

	bool write_msg(uint32_t type, const void *p1, uint32_t l1,
	               const void *p2 = nullptr, uint32_t l2 = 0)
	{
		if (!enabled)
			return false;
		uint32_t const need = (8 + l1 + l2 + 7) & ~7u;
		uint64_t const w = *wpos, r = *rpos;
		if (RING_SIZE - uint32_t(w - r) < need)
		{
			++*dropped;
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
		*wpos = w + need;   // x86: plain store publishes after the memcpys
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
		if (tex_dirty && write_msg(4, &frame, 4, tex, tex_len))
			tex_dirty = false;   // stays dirty on drop; retried next scene
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
namespace mvgl {

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
	void (WINAPI *Uniform2f)(int, float, float);
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
		L(DrawArrays, "glDrawArrays") L(ReadPixels, "glReadPixels") L(GetError, "glGetError")
		L(CreateShader, "glCreateShader") L(ShaderSource, "glShaderSource")
		L(CompileShader, "glCompileShader") L(GetShaderiv, "glGetShaderiv")
		L(GetShaderInfoLog, "glGetShaderInfoLog") L(CreateProgram, "glCreateProgram")
		L(AttachShader, "glAttachShader") L(LinkProgram, "glLinkProgram")
		L(GetProgramiv, "glGetProgramiv") L(GetProgramInfoLog, "glGetProgramInfoLog")
		L(UseProgram, "glUseProgram") L(GetUniformLocation, "glGetUniformLocation")
		L(Uniform1i, "glUniform1i") L(Uniform2f, "glUniform2f")
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

static void build_vertices(const std::vector<QuadMsg> &quads, float xoff,
	std::vector<float> &fdata, std::vector<uint32_t> &udata)
{
	fdata.resize(quads.size() * 6 * 18);
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
		uint32_t pixdata = dma[1];
		bool const textured = (dma[0] & 0x300) == 0x100;
		uint32_t const dither = (dma[0] & 0x2000) ? 1 : 0;
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
		make_inclusive(vx, vy);
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
			float *f = &fdata[(q * 6 + k) * 18];
			f[0] = cx[k]; f[1] = cy[k];
			for (int i = 0; i < 4; i++) { f[2 + i * 2] = vx[i]; f[3 + i * 2] = vy[i]; }
			f[10] = us[0]; f[11] = vs[0]; f[12] = us[1]; f[13] = vs[1];
			f[14] = us[2]; f[15] = vs[2]; f[16] = us[3]; f[17] = vs[3];
			uint32_t *u = &udata[(q * 6 + k) * 4];
			u[0] = pixdata; u[1] = mode; u[2] = dither; u[3] = uint32_t(dma[14]) * 256;
		}
	}
}

// ---- the render thread ----
constexpr int MARGIN = 86, WIDE = 512 + 2 * MARGIN, HEIGHT = 400;
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
	midv_live &lv = live();
	if (std::getenv("MIDV_GL_LOG"))
		s_log = fopen("midv_gl.log", "w");
	int const S = std::getenv("MIDV_GL_SCALE") ? atoi(std::getenv("MIDV_GL_SCALE")) : 3;
	const char *snapdir = std::getenv("MIDV_GL_SNAP");
	int const fw = WIDE * S, fh = HEIGHT * S;

	// wait for MAME's window
	HWND parent = nullptr;
	for (int i = 0; i < 100 && !parent; i++) { Sleep(100); parent = find_mame_window(); }
	if (!parent) { logf("no MAME window found"); return; }

	// Owned top-level popup, NOT a child: MAME's gdi renderer caches its
	// window DC, so child-clipping (WS_CLIPCHILDREN, even with
	// SWP_FRAMECHANGED) never reaches it and its blit punches through the
	// overlay - seen at the rig as alternating stretched/letterboxed frames.
	// A separate owned window is composited by DWM and occludes the owner
	// absolutely. NOACTIVATE+TRANSPARENT+DISABLED keep every input event
	// (keyboard focus, mouse, DirectInput foreground) on MAME's window.
	WNDCLASSA wc = {};
	wc.lpfnWndProc = DefWindowProcA;
	wc.hInstance = GetModuleHandleA(nullptr);
	wc.lpszClassName = "MidvGLOverlay";
	RegisterClassA(&wc);
	RECT rc; GetClientRect(parent, &rc);
	POINT tl = { 0, 0 };
	ClientToScreen(parent, &tl);
	HWND child = CreateWindowExA(
		WS_EX_NOACTIVATE | WS_EX_TRANSPARENT | WS_EX_TOOLWINDOW,
		"MidvGLOverlay", "", WS_POPUP | WS_VISIBLE | WS_DISABLED,
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
	uint underTex[2] = { make_tex(512, HEIGHT, R16UI), make_tex(512, HEIGHT, R16UI) };
	uint fbo[2];
	gl.GenFramebuffers(2, fbo);
	for (int i = 0; i < 2; i++)
	{
		gl.BindFramebuffer(FRAMEBUFFER, fbo[i]);
		gl.FramebufferTexture2D(FRAMEBUFFER, COLOR_ATTACHMENT0, 0x0DE1, pageTex[i], 0);
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
	const char *fattr[] = { "in_corner", "in_v0", "in_v1", "in_v2", "in_v3", "in_uv01", "in_uv23" };
	int const fsize[] = { 2, 2, 2, 2, 2, 4, 4 };
	int off = 0;
	for (int i = 0; i < 7; i++)
	{
		int loc = gl.GetAttribLocation(prog, fattr[i]);
		if (loc >= 0)
		{
			gl.EnableVertexAttribArray(loc);
			gl.VertexAttribPointer(loc, fsize[i], 0x1406 /*FLOAT*/, 0, 18 * 4,
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
	gl.Uniform2f(gl.GetUniformLocation(prog, "uCanvas"), float(WIDE), float(HEIGHT));
	gl.Uniform1i(gl.GetUniformLocation(prog, "uScale"), S);
	gl.Uniform1i(gl.GetUniformLocation(prog, "uClipRight"), WIDE - 1);
	gl.Uniform1i(gl.GetUniformLocation(prog, "texram"), 0);
	gl.Uniform1i(gl.GetUniformLocation(prog, "texMask"), (8 << 20) - 1);
	gl.UseProgram(pal);
	gl.Uniform1i(gl.GetUniformLocation(pal, "idxTex"), 1);
	gl.Uniform1i(gl.GetUniformLocation(pal, "palTex"), 2);
	int const uCrop = gl.GetUniformLocation(pal, "uCrop");
	logf("GL up: scale %d canvas %dx%d snapdir=%s", S, fw, fh,
		snapdir ? snapdir : "(null)");

	// ---- stream state ----
	std::vector<QuadMsg> run, pend[2];
	uint16_t run_pc = 0xffff, pend_pc[2] = {};
	bool pend_valid[2] = {};
	static uint16_t shadow[2][HEIGHT * 512];
	bool quad_fresh[2] = {};
	int quad_count[2] = {};
	int visible = 0;
	std::vector<uint8_t> staging(8 << 20);
	static uint32_t pal_copy[32768];
	std::vector<float> fdata;
	std::vector<uint32_t> udata;
	uint64_t presents = 0, n_quads = 0, n_scenes = 0, n_pal = 0, n_tex = 0, n_vram = 0;
	int snap_n = 0;

	auto ring_read = [&](uint64_t pos, void *dst, uint32_t len)
	{
		uint32_t o = uint32_t(pos % midv_live::RING_SIZE);
		uint32_t first = std::min(len, midv_live::RING_SIZE - o);
		memcpy(dst, lv.data + o, first);
		if (len > first) memcpy((uint8_t *)dst + first, lv.data, len - first);
	};
	uint64_t n_flips = 0;
	int logged_runs = 0;
	auto complete_run = [&]()
	{
		if (run.empty()) return;
		if (logged_runs < 30)
			logf("run: pc=%u quads=%zu", run_pc, run.size()), ++logged_runs;
		int pg = (run_pc & 4) ? 1 : 0;
		pend[pg].swap(run);
		pend_pc[pg] = run_pc;
		pend_valid[pg] = true;
		run.clear();
	};

	while (IsWindow(parent))
	{
		MSG msg;
		while (PeekMessageA(&msg, child, 0, 0, PM_REMOVE)) DispatchMessageA(&msg);
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

		// ---- drain ----
		uint64_t w = *lv.wpos, r = *lv.rpos;
		while (r < w)
		{
			uint32_t hdr[2];
			ring_read(r, hdr, 8);
			uint32_t const type = hdr[0], len = hdr[1];
			if (len > staging.size()) staging.resize(len);
			ring_read(r + 8, staging.data(), len);
			r += (8 + len + 7) & ~7u;
			*lv.rpos = r;
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
				++n_pal;
				memcpy(pal_copy, staging.data() + 4, sizeof(pal_copy));
				gl.ActiveTexture(TEXTURE0 + 2);
				gl.BindTexture(0x0DE1, paltex);
				gl.TexSubImage2D(0x0DE1, 0, 0, 0, 256, 128, RED_INTEGER, 0x1405, staging.data() + 4);
				break;
			case 4:
				++n_tex;
				gl.ActiveTexture(TEXTURE0);
				gl.BindTexture(0x0DE1, texram);
				gl.TexSubImage2D(0x0DE1, 0, 0, 0, 4096, 2048, RED_INTEGER, 0x1401, staging.data() + 4);
				break;
			case 5:
			{
				uint32_t o, n;
				memcpy(&o, staging.data() + 4, 4);
				memcpy(&n, staging.data() + 8, 4);
				int pg = (o & 0x40000) ? 1 : 0;
				uint32_t rel = o & 0x3ffff;
				++n_vram;
				if (rel < HEIGHT * 512)
				{
					uint32_t end = std::min(rel + n, uint32_t(HEIGHT * 512));
					memcpy(&shadow[pg][rel], staging.data() + 12, (end - rel) * 2);
					quad_fresh[pg] = false;
				}
				break;
			}
			}
		}

		// ---- render pending scenes (skip-to-latest already applied) ----
		for (int pg = 0; pg < 2; pg++)
		{
			if (!pend_valid[pg]) continue;
			pend_valid[pg] = false;
			build_vertices(pend[pg], float(MARGIN), fdata, udata);
			quad_count[pg] = int(pend[pg].size());
			gl.UseProgram(prog);
			gl.BindVertexArray(vao);
			gl.BindBuffer(ARRAY_BUFFER, vbo_f);
			gl.BufferData(ARRAY_BUFFER, fdata.size() * 4, fdata.data(), STREAM_DRAW);
			gl.BindBuffer(ARRAY_BUFFER, vbo_u);
			gl.BufferData(ARRAY_BUFFER, udata.size() * 4, udata.data(), STREAM_DRAW);
			gl.BindFramebuffer(FRAMEBUFFER, fbo[pg]);
			gl.Viewport(0, 0, fw, fh);
			gl.ActiveTexture(TEXTURE0);
			gl.BindTexture(0x0DE1, texram);
			gl.DrawArrays(0x0004 /*TRIANGLES*/, 0, int(pend[pg].size() * 6));
			gl.BindFramebuffer(FRAMEBUFFER, 0);
			quad_fresh[pg] = true;
			++n_scenes;
		}

		// ---- present ----
		int const cw = rc.right, ch = rc.bottom;
		gl.Viewport(0, 0, cw, ch);
		gl.ClearColor(0, 0, 0, 1);
		gl.Clear(0x4000);
		bool const wide3d = quad_fresh[visible] && quad_count[visible] >= 120;
		float const content_w = wide3d ? float(WIDE) : 512.0f;
		float const aspect = (content_w * PAR) / float(HEIGHT);
		int vw = cw, vh = int(cw / aspect + 0.5f);
		if (vh > ch) { vh = ch; vw = int(ch * aspect + 0.5f); }
		gl.UseProgram(pal);
		gl.Uniform1i(uCrop, (quad_fresh[visible] && !wide3d) ? MARGIN * S : 0);
		gl.ActiveTexture(TEXTURE0 + 1);
		if (quad_fresh[visible])
			gl.BindTexture(0x0DE1, pageTex[visible]);
		else
		{
			// the pal pass samples rows bottom-up (the quad path flips Y in
			// its VS); the CPU shadow is top-down, so upload row-reversed or
			// boot/test screens display vertically flipped
			static uint16_t flipped[HEIGHT * 512];
			for (int y = 0; y < HEIGHT; y++)
				memcpy(&flipped[y * 512], &shadow[visible][(HEIGHT - 1 - y) * 512], 512 * 2);
			gl.BindTexture(0x0DE1, underTex[visible]);
			gl.TexSubImage2D(0x0DE1, 0, 0, 0, 512, HEIGHT, RED_INTEGER, 0x1403, flipped);
		}
		gl.ActiveTexture(TEXTURE0 + 2);
		gl.BindTexture(0x0DE1, paltex);
		gl.Viewport((cw - vw) / 2, (ch - vh) / 2, vw, vh);
		gl.BindVertexArray(vao_empty);
		gl.DrawArrays(0x0004, 0, 3);

		++presents;
		if (s_log && (presents % 300) == 0)
			logf("t=%llu quads=%llu scenes=%llu pal=%llu tex=%llu vram=%llu "
				"flips=%llu backlog=%llu vis=%d fresh=%d cnt=%d err=%u",
				(unsigned long long)presents, (unsigned long long)n_quads,
				(unsigned long long)n_scenes, (unsigned long long)n_pal,
				(unsigned long long)n_tex, (unsigned long long)n_vram,
				(unsigned long long)n_flips,
				(unsigned long long)(*lv.wpos - *lv.rpos), visible,
				int(quad_fresh[visible]), quad_count[visible], gl.GetError());
		if (snapdir && (presents % 150) == 0)
		{
			std::vector<uint8_t> px(size_t(cw) * ch * 3);
			gl.ReadPixels(0, 0, cw, ch, 0x80E0 /*BGR*/, 0x1401, px.data());
			char path[512];
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
			}
		}
		SwapBuffers(dc);
	}
	logf("parent gone after %llu presents, %d snaps", (unsigned long long)presents, snap_n);
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

void midvunit_base_state::video_start()
{
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


void midvunit_renderer::process_dma_queue()
{
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
		m_dma_data[m_dma_data_index++] = data;
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

	// live bridge: palette/texture must flow even before any quad is drawn -
	// boot and test screens are CPU-drawn, and without this the palette never
	// reaches the renderer until the first 3D scene (boot showed black).
	if (live().enabled)
		live().sync_state(uint32_t(screen.frame_number()),
			m_paletteram.target(), uint32_t(m_paletteram.bytes()),
			m_textureram.target(), uint32_t(m_textureram.bytes()));

	// POC: one-shot memory dump so the quad stream can be re-rasterized and
	// verified offline. Runs after the poly wait, so all quads have landed.
	{
		static bool s_dumped = false;
		static const char *s_dir = std::getenv("MIDV_STATEDUMP_DIR");
		static const char *s_frame = std::getenv("MIDV_STATEDUMP_FRAME");
		if (!s_dumped && s_dir && s_frame
			&& screen.frame_number() >= strtoul(s_frame, nullptr, 10))
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
