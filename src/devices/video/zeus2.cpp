// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Midway Zeus2 Video

**************************************************************************/
#include "emu.h"
#include "zeus2.h"
#include "../../mame/midway/cruisn/zeus_render_policy.h"
#include "../../mame/midway/cruisn/zeus_palette_lifetime.h"
#include "../../mame/midway/cruisn/zeus_margin_clear.h"
#include "../../mame/midway/cruisn/zeus_sky_repeat.h"

#include "screen.h"

#include <algorithm>
#include <map>
#include <cstdlib>
#include <vector>

// MIDZ_GL in-process renderer (Windows-only, env-gated; see mzgl below)
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <atomic>
#include <thread>
#include <vector>
#include "zeus2_gl_shaders.h"
#include "../../mame/midway/midvunit_menu_assets.h"
#include "../../mame/midway/cruisn/pause_cheats_win.h"
namespace mzgl { void start(); void stop(); }
void midv_trace_wheelpos(running_machine &machine, const char *tag);   // midvunit_v.cpp (POC)
#endif


#define LOG_REGS         1
// Setting ALWAYS_LOG_FIFO will always log the fifo versus having to hold 'L'
#define ALWAYS_LOG_FIFO  0

/*************************************
*  Constructor
*************************************/
zeus2_renderer::zeus2_renderer(zeus2_device *state)
	: poly_manager<float, zeus2_poly_extra_data, 4>(state->machine())
	, m_state(state)
{
}

DEFINE_DEVICE_TYPE(ZEUS2, zeus2_device, "zeus2", "Midway Zeus2")

uint8_t *zeus2_device::s_waveram_base = nullptr;

zeus2_device::zeus2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, ZEUS2, tag, owner, clock)
	, device_video_interface(mconfig, *this)
	, m_vblank(*this), m_irq(*this), m_atlantis(0)
{
}

/*************************************
*  Display interrupt generation
*************************************/

TIMER_CALLBACK_MEMBER(zeus2_device::display_irq_off)
{
	m_vblank(CLEAR_LINE);

	//attotime vblank_period = screen().time_until_pos(m_zeusbase[0x37] & 0xffff);

	///* if zero, adjust to next frame, otherwise we may get stuck in an infinite loop */
	//if (vblank_period == attotime::zero)
	//  vblank_period = screen().frame_period();
	//vblank_timer->adjust(vblank_period);
	vblank_timer->adjust(screen().time_until_vblank_start());
}

TIMER_CALLBACK_MEMBER(zeus2_device::display_irq)
{
	m_vblank(ASSERT_LINE);
	/* set a timer for the next off state */
	vblank_off_timer->adjust(screen().time_until_vblank_end());
}

TIMER_CALLBACK_MEMBER(zeus2_device::int_timer_callback)
{
	//m_maincpu->set_input_line(2, ASSERT_LINE);
	m_irq(ASSERT_LINE);
}

/*************************************
 *  Video startup
 *************************************/


void zeus2_device::device_start()
{
	const char *stall_frame_text = std::getenv("MIDZ_GL_STALL_FRAME");
	const char *stall_ms_text = std::getenv("MIDZ_GL_STALL_MS");
	if (stall_frame_text || stall_ms_text)
	{
		auto number = [](const char *text, unsigned limit) -> unsigned
		{
			if (!text || *text < '0' || *text > '9') return 0;
			char *end = nullptr;
			const unsigned long value = strtoul(text, &end, 10);
			return !*end && value >= 1 && value <= limit ? unsigned(value) : 0;
		};
		const unsigned frame = number(stall_frame_text, 1000000), ms = number(stall_ms_text, 5000);
		const char *force = std::getenv("MIDV_FFB"), *live = std::getenv("MIDZ_GL");
		if (!frame || !ms || !force || strcmp(force,"0") || !live || strcmp(live,"1") ||
			strcmp(machine().system().name,"crusnexo"))
			fatalerror("Zeus consumer stall requires Exotica/liveGL/FFB0, frame1..1000000 and milliseconds1..5000");
		fprintf(stderr,"MIDZ_GL_STALL frame=%u milliseconds=%u\n",frame,ms);
	}
	if (const char *mode = std::getenv("MIDZ_SKY_REPEAT"))
	{
		const char *force=std::getenv("MIDV_FFB"), *live=std::getenv("MIDZ_GL");
		const char *page=std::getenv("MIDZ_GL_MARGIN_PAGE_CLEAR"), *palette=std::getenv("MIDZ_PALETTE_GUARD");
		if ((mode[0]!='0' && mode[0]!='1') || mode[1] || strcmp(machine().system().name,"crusnexo") ||
			!force || strcmp(force,"0") || !live || strcmp(live,"1") ||
			(mode[0]=='1' && (!page || strcmp(page,"1") || !palette || strcmp(palette,"1"))))
			fatalerror("Zeus sky diagnostic requires Exotica/liveGL/FFB0; repeat needs page clearing and palette guard");
		fprintf(stderr,"MIDZ_SKY_REPEAT=%c\n",mode[0]);
	}
	if (const char *mode = std::getenv("MIDZ_GL_MARGIN_PAGE_CLEAR"))
	{
		const char *force = std::getenv("MIDV_FFB");
		const char *live = std::getenv("MIDZ_GL");
		if ((mode[0] != '0' && mode[0] != '1') || mode[1] ||
			strcmp(machine().system().name,"crusnexo") || !force || strcmp(force,"0") ||
			!live || strcmp(live,"1"))
			fatalerror("Zeus margin-clear diagnostic requires Exotica, mask0/1, live GL and MIDV_FFB=0");
		fprintf(stderr,"MIDZ_GL_MARGIN_PAGE_CLEAR=%c\n",mode[0]);
	}
	if (const char *mode = std::getenv("MIDZ_PALETTE_GUARD"))
	{
		const char *force = std::getenv("MIDV_FFB");
		if ((mode[0] != '0' && mode[0] != '1') || mode[1] ||
			strcmp(machine().system().name,"crusnexo") || !force || strcmp(force,"0"))
			fatalerror("Zeus palette diagnostic requires Exotica, mask0/1 and MIDV_FFB=0");
		fprintf(stderr,"MIDZ_PALETTE_GUARD=%c\n",mode[0]);
	}
	if (const char *mode = std::getenv("MIDZ_UPSTREAM_RENDER"))
	{
		if (mode[0] < '0' || mode[0] > '7' || mode[1] || strcmp(machine().system().name,"crusnexo"))
			fatalerror("MIDZ_UPSTREAM_RENDER requires Exotica 2.4 and a single mask0..7");
		m_upstream_render = mode[0] - '0';
		const char *force = std::getenv("MIDV_FFB");
		if (m_upstream_render && (!force || strcmp(force,"0")))
			fatalerror("Zeus upstream rendering diagnostic requires MIDV_FFB=0");
		fprintf(stderr,"MIDZ_UPSTREAM_RENDER=%u PR=16094\n",m_upstream_render);
	}
#ifdef _WIN32
	// MIDZ_GL=1: spawn the in-process GL renderer thread (see mzgl below)
	if (std::getenv("MIDZ_GL") && atoi(std::getenv("MIDZ_GL")) != 0)
	{
		midz_live = true;
		mzgl::start();
	}
#endif

	/* allocate memory for "wave" RAM */
	m_waveram = std::make_unique<uint32_t[]>(WAVERAM0_WIDTH * WAVERAM0_HEIGHT * 8/4);
	s_waveram_base = reinterpret_cast<uint8_t *>(m_waveram.get());
	m_frameColor = std::make_unique<uint32_t[]>(WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2);
	m_frameDepth = std::make_unique<int32_t[]>(WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2);

	/* initialize polygon engine */
	poly = std::make_unique<zeus2_renderer>(this);

	/* we need to cleanup on exit */
	//machine().add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&zeus2_device::exit_handler2, this));

	int_timer = timer_alloc(FUNC(zeus2_device::int_timer_callback), this);
	vblank_timer = timer_alloc(FUNC(zeus2_device::display_irq), this);
	vblank_off_timer = timer_alloc(FUNC(zeus2_device::display_irq_off), this);

	//printf("%s\n", machine().system().name);
	// Set system type
	if (strcmp(machine().system().name, "thegrid") == 0 || strcmp(machine().system().name, "thegrida") == 0) {
		m_system = THEGRID;
	}
	else if (strcmp(machine().system().name, "crusnexo") == 0) {
		m_system = CRUSNEXO;
	}
	else {
		m_system = MWSKINS;
	}

	/* save states */
	save_item(NAME(m_atlantis));
	save_item(NAME(m_zeusbase));
	save_item(NAME(m_renderRegs));
	// poly
	save_item(NAME(zeus_cliprect.min_x));
	save_item(NAME(zeus_cliprect.max_x));
	save_item(NAME(zeus_cliprect.min_y));
	save_item(NAME(zeus_cliprect.max_y));
	save_item(NAME(m_palSize));
	save_item(NAME(zeus_matrix));
	save_item(NAME(zeus_trans));
	save_item(NAME(zeus_light));
	save_item(NAME(zeus_texbase));
	save_item(NAME(zeus_quad_size));
	save_item(NAME(m_useZOffset));
	save_pointer(NAME(m_waveram), WAVERAM0_WIDTH * WAVERAM0_HEIGHT * 8 / 4);
	save_pointer(NAME(m_frameColor), WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2);
	save_pointer(NAME(m_frameDepth), WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2);
	save_item(NAME(m_pal_table));
	// m_ucode
	save_item(NAME(m_curUCodeSrc));
	save_item(NAME(m_curPalTableSrc));
	save_item(NAME(m_texmodeReg));
	// int_timer
	// vblank_timer
	// yoffs
	// texel_width
	// zbase
	save_item(NAME(m_system));
	save_item(NAME(zeus_fifo));
	save_item(NAME(zeus_fifo_words));
	save_item(NAME(m_fill_color));
	save_item(NAME(m_fill_depth));
	save_item(NAME(m_yScale));
}

void zeus2_device::device_reset()
{
	memset(m_zeusbase, 0, sizeof(m_zeusbase[0]) * 0x80);
	memset(m_renderRegs, 0, sizeof(m_renderRegs[0]) * 0x50);

	m_curUCodeSrc = 0;
	m_curPalTableSrc = 0;
	m_palSize = 0;
	zbase = 32.0f;
	m_yScale = 0;
	yoffs = 0x1dc000;
	//yoffs = 0x00040000;
	texel_width = 256;
	zeus_fifo_words = 0;
	m_fill_color = 0;
	m_fill_depth = 0xffffff;
	m_texmodeReg = 0;
	zeus_trans[3] = 0.0f;
	m_useZOffset = false;
}
#if DUMP_WAVE_RAM
#include <iostream>
#include <fstream>
#endif

// POC scoping (env-gated, inert unset): count quads per frame to size a GL
// renderer replacement - see cruisn-collection RESULTS.md, Zeus scoping
static int s_mz_stats = -1;
static unsigned s_mz_quads = 0, s_mz_frames = 0, s_mz_qmax = 0;

// ---- POC offline capture (MIDZ_CAPTURE=<dir>, MIDZ_CAPTURE_FRAME=<n>) ----
// records.bin stream: (u32 type, u32 bytes, payload). type 1 = quad
// (midz_quad_rec: clipped verts + full raster state), 2 = pal_table
// (256*u32, emitted when it changed), 3 = fast clear, 4 = frame_write
// register snapshot. pre/post color+depth dumps bracket the stream;
// waveram + regs dumped at close. Submission order == FB mutation order
// (poly->wait precedes the direct-write paths).
static int s_cap_state = -2;      // -2 env unread, -1 off, 0 armed, 1 recording, 2 done
static uint32_t s_cap_frame = 2400;
static uint32_t s_cap_minq = 0;   // MIDZ_CAPTURE_MINQUADS: with it set, arm on
                                  // the first >=minq-quad frame past the frame
                                  // floor and record until minq quads landed
                                  // (frame numbers drift between boots and 3D
                                  // scenes render every other frame)
static uint32_t s_cap_fq = 0;     // quads submitted since last screen_update
static uint32_t s_cap_recq = 0;   // quads recorded so far
static uint32_t s_cap_frames_rec = 0;
static FILE *s_cap_rec = nullptr;
// Optional bounded model-source journal. It copies pre-execution operands and
// associates them with the existing projected-quad stream, without touching it.
// Raw WaveRAM/program/material operands are private diagnostic artifacts.
static FILE *s_cap_models = nullptr;
static uint32_t s_cap_model_count = 0, s_cap_model_bytes = 0;
struct midz_model_rec
{
	uint32_t version, frame, id, baseaddr, count, quad_size, system, first_quad;
	uint32_t last_quad, ucode, palette, texture, yscale, zoffset, raw_words, reserved;
	double time;
	float matrix[9], translation[4], light[3];
	uint32_t regs[0x80], render[0x50];
};
static_assert(sizeof(midz_model_rec) == 968, "Zeus model journal layout");
static char s_cap_dir[400];
static uint32_t s_cap_pal[256];
static bool s_cap_pal_valid = false;
// set by load_pal_table; checked (cheaply) per quad instead of a 1 KB
// memcmp per quad, which cost ~1 ms/frame at 6k quads
static bool s_pal_dirty = true;
// waveram dirty span since the last screen_hook flush (live renderer)
static uint32_t s_wave_lo = ~0u, s_wave_hi = 0;

struct midz_quad_rec
{
	uint32_t frame, numverts, texdata, tex_src;
	uint32_t texwidth, solidcolor, transcolor, srcAlpha, dstAlpha, flags;
	int32_t zbuf_min;
	uint32_t rr04, yscale;
	int32_t clip[4];
	float verts[8][6];    // x, y, p0(z 12.12), p1(u/z), p2(v/z), p3(1/z)
};

static void midz_dump(const char *dir, const char *name, const void *data, size_t bytes)
{
	char path[512];
	snprintf(path, sizeof(path), "%s/%s", dir, name);
	FILE *f = fopen(path, "wb");
	if (f)
	{
		fwrite(data, 1, bytes, f);
		fclose(f);
	}
}

// ════════════════════════════════════════════════════════════════════════
// MIDZ_GL: in-process GL renderer for Zeus2 (Cruis'n Exotica) - the live
// half of the renderer-replacement arc. Mirrors the midvunit_v.cpp
// overlay: an owned NOACTIVATE popup over MAME's window and a render
// thread consuming an in-process ring of the SAME records MIDZ_CAPTURE
// writes (types 1-4) plus waveram dirty spans (5) and per-frame ticks
// carrying the display base (6). The GPU pipeline is the oracle-verified
// gpu/zeus_renderer.py port; shaders are generated into
// zeus2_gl_shaders.h by harness/gen_shaders.py - never hand-edit them.
// Env: MIDZ_GL=1 enables; MIDZ_GL_SCALE (falls back to MIDV_GL_SCALE),
// MIDZ_GL_CRT (falls back to MIDV_GL_CRT), MIDZ_GL_SNAP, MIDZ_GL_LOG,
// MIDV_GL_STATEFILE (crt persist for the launcher).
#ifdef _WIN32
namespace mzgl {

#define MZGL_E(n, v) constexpr unsigned n = v;
MZGL_E(FRAGMENT_SHADER, 0x8B30) MZGL_E(VERTEX_SHADER, 0x8B31)
MZGL_E(COMPILE_STATUS, 0x8B81) MZGL_E(LINK_STATUS, 0x8B82)
MZGL_E(ARRAY_BUFFER, 0x8892) MZGL_E(STREAM_DRAW, 0x88E0)
MZGL_E(FRAMEBUFFER, 0x8D40) MZGL_E(COLOR_ATTACHMENT0, 0x8CE0)
MZGL_E(DEPTH_ATTACHMENT, 0x8D00)
MZGL_E(FRAMEBUFFER_COMPLETE, 0x8CD5) MZGL_E(TEXTURE0, 0x84C0)
MZGL_E(R8UI, 0x8232) MZGL_E(R32UI, 0x8236) MZGL_E(RED_INTEGER, 0x8D94)
MZGL_E(RGBA8, 0x8058) MZGL_E(RGBA, 0x1908)
MZGL_E(DEPTH_COMPONENT24, 0x81A6) MZGL_E(DEPTH_COMPONENT, 0x1902)
MZGL_E(GLDEPTH_TEST, 0x0B71) MZGL_E(GLBLEND, 0x0BE2)
MZGL_E(GLSCISSOR_TEST, 0x0C11)
MZGL_E(GLLEQUAL, 0x0203) MZGL_E(GLALWAYS, 0x0207)
MZGL_E(GLONE, 1) MZGL_E(GLSRC_ALPHA, 0x0302)
#undef MZGL_E

typedef unsigned uint;
typedef ptrdiff_t glsizeiptr;

struct GL
{
	HMODULE dll = nullptr;
	HGLRC (WINAPI *CreateContext)(HDC);
	BOOL (WINAPI *DeleteContext)(HGLRC);
	BOOL (WINAPI *MakeCurrent)(HDC, HGLRC);
	PROC (WINAPI *GetProc)(LPCSTR);
	HGLRC (WINAPI *CreateContextAttribs)(HDC, HGLRC, const int *);
	BOOL (WINAPI *SwapIntervalEXT)(int);
	void (WINAPI *Viewport)(int, int, int, int);
	void (WINAPI *ClearColor)(float, float, float, float);
	void (WINAPI *ClearDepth)(double);
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
	void (WINAPI *DepthFunc)(unsigned);
	void (WINAPI *DepthMask)(unsigned char);
	void (WINAPI *ColorMask)(unsigned char, unsigned char, unsigned char, unsigned char);
	void (WINAPI *BlendFunc)(unsigned, unsigned);
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
	int (WINAPI *GetAttribLocation)(uint, const char *);
	void (WINAPI *GenBuffers)(int, uint *);
	void (WINAPI *BindBuffer)(unsigned, uint);
	void (WINAPI *BufferData)(unsigned, glsizeiptr, const void *, unsigned);
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
		L(Viewport, "glViewport") L(ClearColor, "glClearColor")
		L(ClearDepth, "glClearDepth") L(Clear, "glClear")
		L(GenTextures, "glGenTextures") L(BindTexture, "glBindTexture")
		L(TexParameteri, "glTexParameteri") L(TexImage2D, "glTexImage2D")
		L(TexSubImage2D, "glTexSubImage2D") L(PixelStorei, "glPixelStorei")
		L(DrawArrays, "glDrawArrays") L(Enable, "glEnable")
		L(Disable, "glDisable") L(Scissor, "glScissor")
		L(ReadPixels, "glReadPixels") L(GetError, "glGetError")
		L(DepthFunc, "glDepthFunc") L(DepthMask, "glDepthMask")
		L(ColorMask, "glColorMask") L(BlendFunc, "glBlendFunc")
		L(CreateShader, "glCreateShader") L(ShaderSource, "glShaderSource")
		L(CompileShader, "glCompileShader") L(GetShaderiv, "glGetShaderiv")
		L(GetShaderInfoLog, "glGetShaderInfoLog") L(CreateProgram, "glCreateProgram")
		L(AttachShader, "glAttachShader") L(LinkProgram, "glLinkProgram")
		L(GetProgramiv, "glGetProgramiv") L(GetProgramInfoLog, "glGetProgramInfoLog")
		L(UseProgram, "glUseProgram") L(GetUniformLocation, "glGetUniformLocation")
		L(Uniform1i, "glUniform1i") L(Uniform1f, "glUniform1f")
		L(Uniform2f, "glUniform2f") L(Uniform4f, "glUniform4f")
		L(GetAttribLocation, "glGetAttribLocation") L(GenBuffers, "glGenBuffers")
		L(BindBuffer, "glBindBuffer") L(BufferData, "glBufferData")
		L(GenVertexArrays, "glGenVertexArrays") L(BindVertexArray, "glBindVertexArray")
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

// ---- in-process ring (single producer: emu thread; single consumer) ----
constexpr size_t RING = 64u << 20;
static uint8_t *s_ringbuf = nullptr;
static std::atomic<uint64_t> s_rw{0}, s_rr{0};
static std::atomic<bool> s_on{false}, s_stopz{false}, s_donez{true};
static std::atomic<uint32_t> s_presented_frame{0};
// Host-only diagnostics. A single atomic preserves the phase/start-time pair
// when the emulation thread observes backpressure from the GL consumer.
static bool s_phase_trace = false;
static std::atomic<uint64_t> s_phase_stamp{0};
enum { PHASE_OTHER, PHASE_READBACK, PHASE_FILE, PHASE_SWAP, PHASE_STALL, PHASE_COUNT };
static const char *phase_name(unsigned phase)
{
	static const char *names[] = { "other", "readback", "file", "swap", "stall" };
	return phase < PHASE_COUNT ? names[phase] : "unknown";
}
static uint64_t begin_phase(unsigned phase)
{
	if (!s_phase_trace) return 0;
	const uint64_t now = GetTickCount64();
	s_phase_stamp.store((now << 8) | phase, std::memory_order_relaxed);
	return now;
}
static std::atomic<int> s_zpause{0};
static std::atomic<double> s_secs{0.0};   // machine time, published by screen_update for the GL thread
static std::thread s_thread;
struct FrameTick { uint32_t base, frame; double seconds; };

static std::atomic<uint32_t> s_drops_quad{0}, s_drops_state{0};   // ring overflow accounting (midz_gl.log)
static std::atomic<bool> s_wave_resync{false};   // a waveram span was lost: re-emit the FULL waveram at the next flush

static void ring_push2(uint32_t type, const void *p1, uint32_t n1,
		const void *p2, uint32_t n2)
{
	if (!s_on.load(std::memory_order_relaxed) || s_stopz.load(std::memory_order_relaxed))
		return;
	uint32_t const bytes = n1 + n2;
	uint64_t w = s_rw.load(std::memory_order_relaxed);
	uint64_t r = s_rr.load(std::memory_order_acquire);
	uint64_t const need = 8 + ((uint64_t(bytes) + 7) & ~7ull);
	if (w - r + need > RING)
	{
		const uint64_t initial_r = r, started = GetTickCount64();
		// Quads, clears and direct writes all mutate persistent framebuffer/depth
		// state. Losing any one of them cannot be repaired by a texture refresh.
		for (int i = 0; i < 500 && w - r + need > RING; ++i)
		{
			Sleep(1);
			r = s_rr.load(std::memory_order_acquire);
		}
		const uint64_t stamp = s_phase_stamp.load(std::memory_order_relaxed);
		const uint64_t now = GetTickCount64();
		const char *phase = stamp ? phase_name(unsigned(stamp & 255)) : "unmeasured";
		const uint64_t phase_ms = stamp ? now - (stamp >> 8) : 0;
		if (s_phase_trace && now - started >= 50)
			osd_printf_info("MIDZ stream wait: presented=%u type=%u need=%llu queued=%llu consumer_bytes=%llu wait_ms=%llu phase=%s phase_ms=%llu\n",
				s_presented_frame.load(), type, (unsigned long long)need, (unsigned long long)(w-r),
				(unsigned long long)(r-initial_r), (unsigned long long)(now-started), phase, (unsigned long long)phase_ms);
		if (w - r + need > RING)
		{
			s_drops_state.fetch_add(1);
			s_stopz.store(true);
			osd_printf_error("MIDZ render stream failed: consumer timeout; native presentation fallback; "
				"presented=%u type=%u need=%llu queued=%llu capacity=%llu consumer_bytes=%llu wait_ms=%llu phase=%s phase_ms=%llu\n",
				s_presented_frame.load(), type, (unsigned long long)need, (unsigned long long)(w-r),
				(unsigned long long)RING, (unsigned long long)(r-initial_r),
				(unsigned long long)(now-started), phase, (unsigned long long)phase_ms);
			return;
		}
	}
	auto put = [&](const void *src, uint32_t n)
	{
		uint32_t const o = uint32_t(w % RING);
		uint32_t const first = std::min(n, uint32_t(RING - o));
		memcpy(s_ringbuf + o, src, first);
		if (n > first)
			memcpy(s_ringbuf, (const uint8_t *)src + first, n - first);
		w += n;
	};
	uint32_t hdr[2] = { type, bytes };
	put(hdr, 8);
	if (n1) put(p1, n1);
	if (n2) put(p2, n2);
	w = (w + 7) & ~7ull;
	s_rw.store(w, std::memory_order_release);
}

static FILE *s_zlog;
static void zlogf(const char *fmt, ...)
{
	if (!s_zlog) return;
	va_list ap; va_start(ap, fmt);
	vfprintf(s_zlog, fmt, ap);
	fprintf(s_zlog, "\n");
	fflush(s_zlog);
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

static uint zcompile(GL &gl, unsigned type, const char *src)
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
		zlogf("shader compile failed:\n%.*s", n, buf);
		return 0;
	}
	return sh;
}

static uint zlink(GL &gl, const char *vs, const char *fs)
{
	uint p = gl.CreateProgram();
	uint v = zcompile(gl, VERTEX_SHADER, vs);
	uint f = zcompile(gl, FRAGMENT_SHADER, fs);
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
		zlogf("link failed:\n%.*s", n, buf);
		return 0;
	}
	return p;
}

void thread_main();   // defined after the record structs it consumes

}   // namespace mzgl
#endif   // _WIN32

static void midz_rec(uint32_t type, const void *payload, uint32_t bytes)
{
	if (s_cap_rec)
	{
		fwrite(&type, 4, 1, s_cap_rec);
		fwrite(&bytes, 4, 1, s_cap_rec);
		fwrite(payload, 1, bytes, s_cap_rec);
	}
#ifdef _WIN32
	mzgl::ring_push2(type, payload, bytes, nullptr, 0);
#endif
}

#ifdef _WIN32
namespace mzgl {

void thread_main()
{
	struct DoneGuard { ~DoneGuard() { s_donez.store(true); } } done_guard;
	if (std::getenv("MIDZ_GL_LOG"))
		s_zlog = fopen("midz_gl.log", "w");
	auto envi = [](const char *a, const char *b, int dflt) -> int
	{
		if (const char *v = std::getenv(a)) return atoi(v);
		if (b) if (const char *v = std::getenv(b)) return atoi(v);
		return dflt;
	};
	int const S = std::max(1, std::min(4, envi("MIDZ_GL_SCALE", "MIDV_GL_SCALE", 3)));
	bool crt = envi("MIDZ_GL_CRT", "MIDV_GL_CRT", 0) != 0;
	const char *snapdir = std::getenv("MIDZ_GL_SNAP");
	// crusnexo touches FB rows 0..800 (pages at 0 and 400); 1024 rows
	// halve texture memory/traffic vs the full 2048-row address space, and
	// the masks below wrap the (never-observed) high addresses harmlessly
	constexpr int CW = 512, CH = 1024, DISPH = 400;
	// 16:9 widescreen: render into a wider canvas (all geometry shifted by
	// MARGIN via the VS) and present 3D scenes at 16:9, 2D screens cropped
	// to 4:3. MIDZ_GL_MARGIN=0 (or MIDV_GL_MARGIN=0) forces 4:3.
	int MARGIN = envi("MIDZ_GL_MARGIN", "MIDV_GL_MARGIN", 88);
	MARGIN = std::max(0, std::min(120, MARGIN));
	int const CWW = CW + 2 * MARGIN;
	int const fw = CWW * S, fh = CH * S;

	HWND parent = nullptr;
	for (int i = 0; i < 100 && !parent; i++) { Sleep(100); parent = find_mame_window(); }
	if (!parent) { zlogf("no MAME window found"); return; }

	// owned top-level popup, NOT a child (see midvunit_v.cpp for the war
	// stories: gdi caches its window DC; never activate the overlay)
	WNDCLASSA wc = {};
	wc.lpfnWndProc = [](HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) -> LRESULT
	{
		switch (msg)
		{
		case WM_SETCURSOR:
			SetCursor(nullptr);
			return TRUE;
		case WM_LBUTTONDOWN:
		case WM_RBUTTONDOWN:
		case WM_MBUTTONDOWN:
			SetForegroundWindow(GetWindow(hwnd, GW_OWNER));
			return 0;
		}
		return DefWindowProcA(hwnd, msg, wp, lp);
	};
	wc.hInstance = GetModuleHandleA(nullptr);
	wc.lpszClassName = "MidzGLOverlay";
	RegisterClassA(&wc);
	// the overlay covers the MONITOR, not MAME's window: MAME's gdi window
	// stays small (its software stretch to 4K cost ~3% emulation speed),
	// it just holds keyboard focus and DirectInput foreground under us
	auto monitor_rect = [&]() -> RECT
	{
		MONITORINFO mi{};
		mi.cbSize = sizeof(mi);
		GetMonitorInfoA(MonitorFromWindow(parent, MONITOR_DEFAULTTONEAREST), &mi);
		return mi.rcMonitor;
	};
	RECT rc = monitor_rect();
	HWND child = CreateWindowExA(
		WS_EX_NOACTIVATE | WS_EX_TOOLWINDOW,
		"MidzGLOverlay", "", WS_POPUP | WS_VISIBLE,
		rc.left, rc.top, rc.right - rc.left, rc.bottom - rc.top,
		parent, nullptr, wc.hInstance, nullptr);
	if (!child) { zlogf("overlay window failed"); return; }

	HDC dc = GetDC(child);
	PIXELFORMATDESCRIPTOR pfd = {};
	pfd.nSize = sizeof(pfd); pfd.nVersion = 1;
	pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
	pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32;
	SetPixelFormat(dc, ChoosePixelFormat(dc, &pfd), &pfd);
	GL gl;
	if (!gl.load_base()) { zlogf("opengl32 load failed"); return; }
	HGLRC legacy = gl.CreateContext(dc);
	gl.MakeCurrent(dc, legacy);
	if (!gl.load_rest()) { zlogf("GL function resolution failed"); return; }
	if (gl.CreateContextAttribs)
	{
		const int attrs[] = { 0x2091, 4, 0x2092, 3, 0x9126, 1, 0 };
		HGLRC core = gl.CreateContextAttribs(dc, nullptr, attrs);
		if (core)
		{
			gl.MakeCurrent(dc, core);
			gl.DeleteContext(legacy);
			gl.load_rest();
		}
	}
	// perf attribution (2026-08-22): d3d-no-overlay 100.00%, gdi-no-overlay
	// 99.48%, gdi+overlay ~97.5% - the ~2% overlay cost is independent of
	// scale (1..4), vsync, FB size and the emit path; needs an ETW/GPUView
	// session to pin down. MIDZ_GL_VSYNC=0 runs the overlay unthrottled.
	bool const vsync = envi("MIDZ_GL_VSYNC", nullptr, 1) != 0;
	if (gl.SwapIntervalEXT) gl.SwapIntervalEXT(vsync ? 1 : 0);

	uint prog = zlink(gl, MZGL_VS, MZGL_FS);
	uint present = zlink(gl, MZGL_PRESENT_VS, MZGL_PRESENT_FS);
	uint menuprog = zlink(gl, MZGL_MENU_VS, MZGL_MENU_FS);
	if (!prog || !present) return;

	gl.PixelStorei(0x0CF5 /*UNPACK_ALIGNMENT*/, 1);
	auto make_tex = [&](int w, int h, unsigned ifmt, unsigned fmt, unsigned type) -> uint
	{
		uint t; gl.GenTextures(1, &t);
		gl.BindTexture(0x0DE1, t);
		gl.TexParameteri(0x0DE1, 0x2801, 0x2600);   // MIN NEAREST
		gl.TexParameteri(0x0DE1, 0x2800, 0x2600);   // MAG NEAREST
		gl.TexParameteri(0x0DE1, 0x2802, 0x812F);
		gl.TexParameteri(0x0DE1, 0x2803, 0x812F);
		gl.TexImage2D(0x0DE1, 0, int(ifmt), w, h, 0, fmt, type, nullptr);
		return t;
	};
	uint waveTex = make_tex(4096, 4096, R8UI, RED_INTEGER, 0x1401);
	uint palTex = make_tex(256, 256, R32UI, RED_INTEGER, 0x1405);
	uint fbTex = make_tex(fw, fh, RGBA8, RGBA, 0x1401);
	uint depthTex = make_tex(fw, fh, DEPTH_COMPONENT24, DEPTH_COMPONENT, 0x1405);
	uint fbo;
	gl.GenFramebuffers(1, &fbo);
	gl.BindFramebuffer(FRAMEBUFFER, fbo);
	gl.FramebufferTexture2D(FRAMEBUFFER, COLOR_ATTACHMENT0, 0x0DE1, fbTex, 0);
	gl.FramebufferTexture2D(FRAMEBUFFER, DEPTH_ATTACHMENT, 0x0DE1, depthTex, 0);
	if (gl.CheckFramebufferStatus(FRAMEBUFFER) != FRAMEBUFFER_COMPLETE)
		zlogf("fbo incomplete");
	gl.Viewport(0, 0, fw, fh);
	gl.ClearColor(0, 0, 0, 1);
	gl.ClearDepth(1.0);
	gl.Clear(0x4100);   // COLOR | DEPTH
	gl.BindFramebuffer(FRAMEBUFFER, 0);

	uint vao, vbo_f, vbo_u, vao_empty;
	gl.GenVertexArrays(1, &vao);
	gl.GenVertexArrays(1, &vao_empty);
	gl.GenBuffers(1, &vbo_f);
	gl.GenBuffers(1, &vbo_u);
	gl.BindVertexArray(vao);
	gl.BindBuffer(ARRAY_BUFFER, vbo_f);
	{
		const char *fattr[] = { "in_pos", "in_rowbase", "in_p" };
		int const fsize[] = { 2, 1, 4 };
		int off = 0;
		for (int i = 0; i < 3; i++)
		{
			int loc = gl.GetAttribLocation(prog, fattr[i]);
			if (loc >= 0)
			{
				gl.EnableVertexAttribArray(loc);
				gl.VertexAttribPointer(loc, fsize[i], 0x1406, 0, 7 * 4,
					(const void *)(uintptr_t)(off * 4));
			}
			off += fsize[i];
		}
	}
	gl.BindBuffer(ARRAY_BUFFER, vbo_u);
	{
		const char *uattr[] = { "in_meta0", "in_meta1", "in_meta2" };
		int const usize[] = { 4, 4, 2 };
		int off = 0;
		for (int i = 0; i < 3; i++)
		{
			int loc = gl.GetAttribLocation(prog, uattr[i]);
			if (loc >= 0)
			{
				gl.EnableVertexAttribArray(loc);
				gl.VertexAttribIPointer(loc, usize[i], 0x1405, 10 * 4,
					(const void *)(uintptr_t)(off * 4));
			}
			off += usize[i];
		}
	}
	gl.UseProgram(prog);
	gl.Uniform2f(gl.GetUniformLocation(prog, "uCanvas"), float(CWW), float(CH));
	gl.Uniform1f(gl.GetUniformLocation(prog, "uMargin"), float(MARGIN));
	gl.Uniform1i(gl.GetUniformLocation(prog, "waveram"), 0);
	gl.Uniform1i(gl.GetUniformLocation(prog, "palTex"), 1);
	gl.UseProgram(present);
	gl.Uniform1i(gl.GetUniformLocation(present, "fbTex"), 3);
	gl.Uniform1i(gl.GetUniformLocation(present, "uScale"), S);
	gl.Uniform1f(gl.GetUniformLocation(present, "uSrcH"), float(DISPH));
	int const uBaseRow = gl.GetUniformLocation(present, "uBaseRow");
	int const uCrt = gl.GetUniformLocation(present, "uCrt");

	// menu (labels shared with the V-Unit overlay)
	struct Label { uint tex; int w, h; };
	auto make_label = [&](const unsigned char *px, int w, int h) -> Label
	{
		uint t;
		gl.GenTextures(1, &t);
		gl.BindTexture(0x0DE1, t);
		gl.TexParameteri(0x0DE1, 0x2801, 0x2601);
		gl.TexParameteri(0x0DE1, 0x2800, 0x2601);
		gl.TexParameteri(0x0DE1, 0x2802, 0x812F);
		gl.TexParameteri(0x0DE1, 0x2803, 0x812F);
		gl.TexImage2D(0x0DE1, 0, 0x8229 /*R8*/, w, h, 0, 0x1903, 0x1401, px);
		return Label{ t, w, h };
	};
	Label lb_title = make_label(MVMENU_TITLE, MVMENU_TITLE_W, MVMENU_TITLE_H);
	Label lb_resume = make_label(MVMENU_RESUME, MVMENU_RESUME_W, MVMENU_RESUME_H);
	Label lb_crt_on = make_label(MVMENU_CRT_ON, MVMENU_CRT_ON_W, MVMENU_CRT_ON_H);
	Label lb_crt_off = make_label(MVMENU_CRT_OFF, MVMENU_CRT_OFF_W, MVMENU_CRT_OFF_H);
	Label lb_exit = make_label(MVMENU_EXIT, MVMENU_EXIT_W, MVMENU_EXIT_H);
	Label lb_hint = make_label(MVMENU_HINT, MVMENU_HINT_W, MVMENU_HINT_H);
	auto cheat_title = cruisn::menu_text_bitmap("CHEATS");
	Label lb_cheats = make_label(cheat_title.pixels.data(), cheat_title.width, cheat_title.height);
	cruisn::pause_cheats cheat_menu;
	Label text_labels[16]{};
	std::string text_values[16];
	bool left_prev = false, right_prev = false;
	int const mRect = gl.GetUniformLocation(menuprog, "uRect");
	int const mScreen = gl.GetUniformLocation(menuprog, "uScreen");
	int const mColor = gl.GetUniformLocation(menuprog, "uColor");
	int const mSolid = gl.GetUniformLocation(menuprog, "uSolid");
	gl.UseProgram(menuprog);
	gl.Uniform1i(gl.GetUniformLocation(menuprog, "uTex"), 0);
	bool menu_open = false;
	int menu_sel = 0;
	bool f9_prev = false, esc_prev = false, up_prev = false,
		down_prev = false, ret_prev = false;

	// ---- streaming batch state (port of gpu/zeus_renderer.py) ----
	std::vector<float> fdata;
	std::vector<uint32_t> udata;
	struct Batch
	{
		int first, count;
		bool blend, dtest, dwrite, cmask;
		int rowbase, clip[4];
	};
	std::vector<Batch> batches;
	static uint8_t *wave_mirror = (uint8_t *)calloc(1, 16u << 20);
	uint32_t pal_slot = 0;
	bool const palette_trace = std::getenv("MIDZ_PALETTE_GUARD") != nullptr;
	bool const palette_guard = envi("MIDZ_PALETTE_GUARD", nullptr, 0) != 0;
	cruisn::zeus_palette_lifetime palette_life;
	uint64_t palette_conflicts = 0, palette_flushes = 0, palette_frame_conflicts = 0;
	unsigned palette_log_frames = 0;
	bool const margin_trace = std::getenv("MIDZ_GL_MARGIN_PAGE_CLEAR") != nullptr;
	bool const margin_page_clear = envi("MIDZ_GL_MARGIN_PAGE_CLEAR", nullptr, 0) != 0;
	uint64_t margin_clears = 0, margin_expansions = 0;
	bool const sky_trace=std::getenv("MIDZ_SKY_REPEAT")!=nullptr;
	bool const sky_enabled=envi("MIDZ_SKY_REPEAT",nullptr,0)!=0;
	bool sky_open=false;
	std::vector<midz_quad_rec> sky_quads;
	std::vector<cruisn::zeus_sky_tile> sky_tiles;
	if (sky_enabled) { sky_quads.reserve(64); sky_tiles.reserve(64); }
	uint64_t sky_groups=0,sky_accepted=0,sky_copied=0,sky_budget_rejected=0;
	uint32_t zb38 = 0x1900000;
	uint64_t presents = 0, n_quads = 0;
	int snap_n = 0;

	auto vert = [&](float x, float y, float rowbase, const float *p,
			const uint32_t *meta)
	{
		fdata.insert(fdata.end(), { x, y, rowbase, p[0], p[1], p[2], p[3] });
		udata.insert(udata.end(), meta, meta + 10);
	};
	auto want_batch = [&](bool blend, bool dtest, bool dwrite, bool cmask,
			int rowbase, const int32_t *clip)
	{
		int const nv = int(fdata.size() / 7);
		if (!batches.empty())
		{
			Batch &b = batches.back();
			if (b.blend == blend && b.dtest == dtest && b.dwrite == dwrite
				&& b.cmask == cmask && b.rowbase == rowbase
				&& b.clip[0] == clip[0] && b.clip[1] == clip[1]
				&& b.clip[2] == clip[2] && b.clip[3] == clip[3])
				return;
			b.count = nv - b.first;
		}
		batches.push_back({ nv, 0, blend, dtest, dwrite, cmask, rowbase,
			{ int(clip[0]), int(clip[1]), int(clip[2]), int(clip[3]) } });
	};
	auto flush = [&]()
	{
		if (palette_trace) palette_life.clear();
		if (fdata.empty()) { batches.clear(); return; }
		if (!batches.empty())
			batches.back().count = int(fdata.size() / 7) - batches.back().first;
		gl.UseProgram(prog);
		gl.BindVertexArray(vao);
		gl.BindBuffer(ARRAY_BUFFER, vbo_f);
		gl.BufferData(ARRAY_BUFFER, fdata.size() * 4, fdata.data(), STREAM_DRAW);
		gl.BindBuffer(ARRAY_BUFFER, vbo_u);
		gl.BufferData(ARRAY_BUFFER, udata.size() * 4, udata.data(), STREAM_DRAW);
		gl.BindFramebuffer(FRAMEBUFFER, fbo);
		gl.Viewport(0, 0, fw, fh);
		gl.Enable(GLDEPTH_TEST);
		gl.Enable(GLSCISSOR_TEST);
		gl.BlendFunc(GLONE, GLSRC_ALPHA);
		gl.ActiveTexture(TEXTURE0);
		gl.BindTexture(0x0DE1, waveTex);
		gl.ActiveTexture(TEXTURE0 + 1);
		gl.BindTexture(0x0DE1, palTex);
		for (Batch const &b : batches)
		{
			if (!b.count) continue;
			if (b.blend) gl.Enable(GLBLEND); else gl.Disable(GLBLEND);
			gl.DepthFunc(b.dtest ? GLLEQUAL : GLALWAYS);
			gl.DepthMask(b.dwrite ? 1 : 0);
			gl.ColorMask(b.cmask, b.cmask, b.cmask, b.cmask);
			// widescreen: allow the full canvas width (margins) but keep
			// the cliprect's vertical bounds. The game's horizontal clip
			// (0..511) shifted by MARGIN would cut the margins off.
			gl.Scissor(0, (b.rowbase + b.clip[1]) * S, fw,
				(b.clip[3] - b.clip[1] + 1) * S);
			gl.DrawArrays(0x0004, b.first, b.count);
		}
		gl.Disable(GLBLEND);
		gl.Disable(GLSCISSOR_TEST);
		gl.DepthMask(1);
		gl.ColorMask(1, 1, 1, 1);
		gl.BindFramebuffer(FRAMEBUFFER, 0);
		fdata.clear();
		udata.clear();
		batches.clear();
	};
	bool had_quads_iter = false, had_writes_iter = false, wide_mode = false;
	auto add_quad = [&](const midz_quad_rec &r)
	{
		if (palette_trace) palette_life.use(pal_slot);
		++n_quads;
		had_quads_iter = true;
		bool const blend = (r.flags & 2) != 0;
		bool const dtest = (r.flags & 8) != 0;
		bool const dwrite = ((r.flags & 16) != 0) && !(r.flags & 128);
		want_batch(blend, dtest, dwrite, true, int(r.rr04), r.clip);
		uint32_t const meta[10] = { r.flags, (r.tex_src * 8) & 0xFFFFFF,
			r.texwidth, r.texdata & 0xffff, r.transcolor, r.solidcolor,
			pal_slot, r.srcAlpha, r.dstAlpha, uint32_t(r.zbuf_min) };
		float const rb = float(r.rr04);
		for (uint32_t i = 2; i < r.numverts && i < 8; i++)
			for (uint32_t vi : { 0u, i - 1, i })
			{
				const float *v = r.verts[vi];
				float const p[4] = { v[2], v[3], v[4], v[5] };
				vert(v[0], v[1], rb, p, meta);
			}
	};
	auto sky_tile = [](const midz_quad_rec &r)
	{
		cruisn::zeus_sky_tile t;
		t.state={{r.texdata,r.tex_src,r.texwidth,r.solidcolor,r.transcolor,r.srcAlpha,r.dstAlpha,
			r.flags,uint32_t(r.zbuf_min),r.rr04,r.yscale,uint32_t(r.clip[0]),uint32_t(r.clip[1]),
			uint32_t(r.clip[2]),uint32_t(r.clip[3])}};
		for (unsigned i=0;i<4;++i) for (unsigned j=0;j<6;++j) t.v[i][j]=r.verts[i][j];
		return t;
	};
	auto finish_sky = [&]()
	{
		sky_open=false;
		if (sky_tiles.empty()) return;
		++sky_groups;
		auto const plan=cruisn::zeus_sky_repeat(sky_tiles,unsigned(MARGIN));
		if (plan.accepted) {
			++sky_accepted;
			for (auto const &copy:plan.copies) {
				auto q=sky_quads[copy.index];
				for (unsigned i=0;i<4;++i) q.verts[i][0]+=copy.shift;
				add_quad(q); ++sky_copied;
			}
		}
		sky_tiles.clear(); sky_quads.clear();
	};
	// raw fills (fast clears / frame writes) go through the same pipeline
	// as FLAG_RAW rectangles so ordering with quads is exact
	int32_t const fullclip[4] = { 0, 0, CW - 1, CH - 1 };
	auto add_rect = [&](int x0, int y0, int x1, int y1, uint32_t rgb24,
			int32_t depth, bool dwrite, bool cmask)
	{
		want_batch(false, false, dwrite, cmask, 0, fullclip);
		uint32_t const meta[10] = { 256u | (dwrite ? 16u : 0u), 0, 0, 0, 0,
			rgb24, 0, 0, 0, 0 };
		float const p[4] = { float(depth), 0.0f, 0.0f, 1.0f };
		float const fx0 = float(x0), fy0 = float(y0);
		float const fx1 = float(x1), fy1 = float(y1);
		float const cs[6][2] = { { fx0, fy0 }, { fx1, fy0 }, { fx1, fy1 },
			{ fx0, fy0 }, { fx1, fy1 }, { fx0, fy1 } };
		for (auto const &c : cs)
			vert(c[0], c[1], 0.0f, p, meta);
	};
	auto add_span = [&](uint32_t addr, uint32_t n, uint32_t rgb24,
			int32_t depth, bool dwrite, bool cmask)
	{
		if (cmask)
			had_writes_iter = true;   // 2D screen signal (frame writes)
		while (n)
		{
			uint32_t const row = addr / CW, x = addr % CW;
			uint32_t const take = std::min(n, CW - x);
			add_rect(int(x), int(row), int(x + take), int(row + 1),
				rgb24, depth, dwrite, cmask);
			addr = (addr + take) & (CW * CH - 1);
			n -= take;
		}
	};

	std::vector<uint8_t> rec;
    uint32_t completed_frame = 0;
    double completed_seconds = 0;
    uint64_t phase_calls[PHASE_COUNT]{}, phase_total[PHASE_COUNT]{}, phase_max[PHASE_COUNT]{};
    auto finish_phase = [&](unsigned phase, uint64_t started)
    {
        if (!s_phase_trace) return;
        const uint64_t elapsed = GetTickCount64() - started;
        ++phase_calls[phase]; phase_total[phase] += elapsed;
        phase_max[phase] = std::max(phase_max[phase], elapsed);
        begin_phase(PHASE_OTHER);
        if (elapsed >= 50)
            zlogf("slow phase: completed_frame=%u phase=%s elapsed_ms=%llu",
                completed_frame, phase_name(phase), (unsigned long long)elapsed);
    };
    // Explicit diagnostic fault injection. Physical outputs stay off in replay.
    int const stop_frame = envi("MIDZ_GL_STOP_FRAME",nullptr,-1);
    int const stall_frame = envi("MIDZ_GL_STALL_FRAME",nullptr,-1);
    int const stall_ms = envi("MIDZ_GL_STALL_MS",nullptr,0);
    bool stall_applied = false;
    FILE *snap_index = nullptr;
    if (snapdir) {
        char path[512]; snprintf(path,sizeof(path),"%s/captures.csv",snapdir);
        snap_index=fopen(path,"w");
        if (snap_index) fprintf(snap_index,"file,present,last_received_frame,width,height,quads,dropped_messages,completed_frame\n");
    }
	zlogf("MZGL up: scale %d fb %dx%d crt=%d", S, fw, fh, int(crt));

	int const menu_test_frame = (std::getenv("MIDV_FFB") && strcmp(std::getenv("MIDV_FFB"), "0") == 0 &&
		std::getenv("MIDV_CHEAT_MENU_TEST_FRAME")) ? atoi(std::getenv("MIDV_CHEAT_MENU_TEST_FRAME")) : -1;
	int menu_test_step = -1, menu_test_saved = -1;
	ULONGLONG menu_test_next = 0;
	while (IsWindow(parent) && !s_stopz.load())
	{
		MSG msg;
		while (PeekMessageA(&msg, child, 0, 0, PM_REMOVE)) DispatchMessageA(&msg);

		int menu_test_key = 0;
		if (menu_test_frame >= 0 && completed_frame >= uint32_t(menu_test_frame) &&
			menu_test_step < 9 && GetTickCount64() >= menu_test_next) {
			static int const keys[] = { VK_ESCAPE, VK_DOWN, VK_DOWN, VK_RETURN, VK_RIGHT,
				VK_ESCAPE, VK_ESCAPE, VK_ESCAPE, VK_DOWN, VK_RETURN };
			menu_test_key = keys[++menu_test_step]; menu_test_next = GetTickCount64() + 350;
			zlogf("menu test step=%d key=%d completed_frame=%u", menu_test_step, menu_test_key, completed_frame);
		}
		bool ui_changed = false;
		bool const f9 = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
		if (f9 && !f9_prev)
			crt = !crt;
		f9_prev = f9;
		{
			bool const fg = (GetForegroundWindow() == parent);
			auto edge = [&](int vk, bool &prev) -> bool
			{
				bool const down = fg && (GetAsyncKeyState(vk) & 0x8000) != 0;
				bool const e = down && !prev;
				prev = down;
				return e || menu_test_key == vk;
			};
			bool const escape = edge(VK_ESCAPE, esc_prev);
			bool const up = edge(VK_UP, up_prev), dn = edge(VK_DOWN, down_prev);
			bool const ok = edge(VK_RETURN, ret_prev);
			bool const left = edge(VK_LEFT, left_prev), right = edge(VK_RIGHT, right_prev);
			ui_changed = ui_changed || escape || up || dn || ok || left || right;
			if (escape && menuprog) {
				if (menu_open && cheat_menu.open) cheat_menu.open = false;
				else {
					menu_open = !menu_open;
					if (menu_open) { cheat_menu.begin_pause();  }
					else cheat_menu.resume();
				}
			}
			if (menu_open && cheat_menu.open) {
				if (up) cheat_menu.move(-1);
				if (dn) cheat_menu.move(1);
				if (left || right || ok) cheat_menu.change(left ? -1 : right ? 1 : 0, ok);
			} else if (menu_open) {
				if (up) menu_sel = (menu_sel + 3) % 4;
				if (dn) menu_sel = (menu_sel + 1) % 4;
				if (ok) {
					if (menu_sel == 0) { cheat_menu.resume(); menu_open = false; }
					else if (menu_sel == 1) { crt = !crt; }
					else if (menu_sel == 2) cheat_menu.open = true;
					else { cheat_menu.cancel(); PostMessageA(parent, WM_CLOSE, 0, 0); menu_open = false; }
				}
			}
			s_zpause.store(menu_open ? 1 : 0);
		}
		rc = monitor_rect();
		RECT crc; GetWindowRect(child, &crc);
		if (IsIconic(parent))
			ShowWindow(child, SW_HIDE);
		else
		{
			if (!IsWindowVisible(child))
				ShowWindow(child, SW_SHOWNA);
			if (crc.left != rc.left || crc.top != rc.top ||
				crc.right != rc.right || crc.bottom != rc.bottom)
				SetWindowPos(child, nullptr, rc.left, rc.top,
					rc.right - rc.left, rc.bottom - rc.top,
					SWP_NOACTIVATE | SWP_NOZORDER);
		}

		bool frame_ready = false;
		// ---- drain the ring ----
		uint64_t w = s_rw.load(std::memory_order_acquire);
		uint64_t r = s_rr.load(std::memory_order_relaxed);
		auto ring_get = [&](void *dst, uint32_t n)
		{
			uint32_t const o = uint32_t(r % RING);
			uint32_t const first = std::min(n, uint32_t(RING - o));
			memcpy(dst, s_ringbuf + o, first);
			if (n > first) memcpy((uint8_t *)dst + first, s_ringbuf, n - first);
			r += n;
		};
		while (r < w)
		{
			uint32_t hdr[2];
			ring_get(hdr, 8);
			rec.resize(hdr[1]);
			if (hdr[1]) ring_get(rec.data(), hdr[1]);
			r = (r + 7) & ~7ull;
			// Finish before changing palettes/uploads/pages or presenting. Stored
			// tiles retain the current material only within this uninterrupted span.
			if (sky_enabled && hdr[0]!=1 && !sky_quads.empty()) finish_sky();
			switch (hdr[0])
			{
			case 1:
				if (rec.size() >= sizeof(midz_quad_rec)) {
					auto const &q=*(const midz_quad_rec *)rec.data();
					if (sky_enabled && sky_open) {
						auto const t=sky_tile(q);
						if (q.numverts==4 && cruisn::zeus_sky_candidate(t)) {
							if (sky_tiles.size()<64) { sky_tiles.push_back(t); sky_quads.push_back(q); }
							else { sky_tiles.clear(); sky_quads.clear(); sky_open=false; ++sky_budget_rejected; }
						} else finish_sky();
					}
					add_quad(q);
				}
				break;
			case 2:
			{
				uint32_t const next_slot = (pal_slot + 1) & 255;
				if (palette_trace && palette_life.conflicts(next_slot))
				{
					++palette_conflicts; ++palette_frame_conflicts;
					if (palette_guard) { flush(); ++palette_flushes; }
				}
				pal_slot = next_slot;
				gl.ActiveTexture(TEXTURE0 + 1);
				gl.BindTexture(0x0DE1, palTex);
				gl.TexSubImage2D(0x0DE1, 0, 0, int(pal_slot), 256, 1,
					RED_INTEGER, 0x1405, rec.data());
				break;
			}
			case 3:
			{
				uint32_t const *p = (const uint32_t *)rec.data();
				// a frame-sized fast clear is the frame boundary: the game
				// only clears its 512-wide region, so clear the 16:9 margins
				// too (else they accumulate garbage from degenerate
				// near-plane-clipped quads). Flush prior geometry first.
				// Scope the clear to the rows of the page being cleared —
				// the canvas holds BOTH page-flip buffers, and a full-height
				// clear wipes the DISPLAYED page's margins the moment the
				// game starts the next frame (margins showed black on every
				// present; the game really does render past 4:3).
				if (MARGIN > 0 && p[1] > uint32_t(CW * 4))
				{
					auto const span = cruisn::zeus_margin_clear(p[0], p[1], margin_page_clear);
					uint32_t const row0 = span.row, nrows = span.count;
					++margin_clears;
					if (span.expanded) ++margin_expansions;
					if (sky_enabled) sky_open=true;
					flush();
					gl.BindFramebuffer(FRAMEBUFFER, fbo);
					gl.Viewport(0, 0, fw, fh);
					gl.Enable(GLSCISSOR_TEST);
					gl.ClearColor(0, 0, 0, 1);
					gl.ClearDepth(1.0);
					gl.Scissor(0, int(row0 * S), MARGIN * S, int(nrows * S));
					gl.Clear(0x4100);
					gl.Scissor(fw - MARGIN * S, int(row0 * S), MARGIN * S, int(nrows * S));
					gl.Clear(0x4100);
					gl.Disable(GLSCISSOR_TEST);
					gl.BindFramebuffer(FRAMEBUFFER, 0);
				}
				add_span(p[0] & (CW * CH - 1), std::min(p[1], uint32_t(CW * CH)),
					p[2] & 0xffffff, int32_t(p[3]), true, true);
				break;
			}
			case 4:
			{
				uint32_t const *p = (const uint32_t *)rec.data();
				uint32_t const addr = p[0] & (CW * CH - 1);
				uint32_t const r57 = p[1], r58 = p[2], r59 = p[3],
					r5a = p[4], r5e = p[5];
				if (r57 & 0x1)
					add_span(addr, 1, r58 & 0xffffff, 0, false, true);
				if (r5e & 0x20)
				{
					if (r57 & 0x4)
						add_span(addr + 1, 1, r5a & 0xffffff, 0, false, true);
				}
				else
				{
					if (r57 & 0x4)
						add_span(addr + 1, 1, r59 & 0xffffff, 0, false, true);
					if (r57 & 0x10)
						add_span(addr, 1, 0, int32_t(r5a), true, false);
				}
				break;
			}
			case 5:
			{
				// waveram dirty span: draw everything queued first - those
				// quads must sample the texture as it was
				flush();
				uint32_t const *p = (const uint32_t *)rec.data();
				uint32_t const off = p[0], len = p[1];
				if (off + len <= (16u << 20) && rec.size() >= 8 + len)
				{
					memcpy(wave_mirror + off, rec.data() + 8, len);
					int const row0 = int(off / 4096), row1 = int((off + len - 1) / 4096);
					gl.ActiveTexture(TEXTURE0);
					gl.BindTexture(0x0DE1, waveTex);
					gl.TexSubImage2D(0x0DE1, 0, 0, row0, 4096, row1 - row0 + 1,
						RED_INTEGER, 0x1401, wave_mirror + size_t(row0) * 4096);
				}
				break;
			}
			case 6:
                if (rec.size() == sizeof(FrameTick)) {
                    FrameTick tick; memcpy(&tick,rec.data(),sizeof(tick));
                    zb38=tick.base; completed_frame=tick.frame;
                    completed_seconds=tick.seconds; frame_ready=true;
                    if (palette_frame_conflicts) {
                        if (palette_log_frames++ < 64)
                            zlogf("palette lifetime completed_frame=%u conflicts=%llu guard=%d",
                                tick.frame, (unsigned long long)palette_frame_conflicts, int(palette_guard));
                        palette_frame_conflicts = 0;
                    }
                }
				break;
			}
            if (frame_ready) break; // never consume the next frame before presenting this one
		}
		s_rr.store(r, std::memory_order_release);
		flush();

		if (!frame_ready && !menu_open && !ui_changed) { Sleep(1); continue; }
        if (frame_ready && !stall_applied && stall_frame > 0 && completed_frame >= uint32_t(stall_frame))
        {
            stall_applied = true;
            const uint64_t began = GetTickCount64();
            begin_phase(PHASE_STALL);
            Sleep(stall_ms);
            const uint64_t elapsed = GetTickCount64()-began;
            finish_phase(PHASE_STALL, began);
            fprintf(stderr,"MIDZ_GL_STALL_APPLIED frame=%u milliseconds=%d elapsed_ms=%llu\n",
                completed_frame, stall_ms, (unsigned long long)elapsed);
        }
        if (frame_ready && stop_frame >= 0 && completed_frame >= uint32_t(stop_frame)) {
            zlogf("diagnostic consumer stop at completed frame %u",completed_frame);
            break;
        }
		// ---- present ----
		// 3D scenes present 16:9 (full wide canvas); 2D screens (menus,
		// high scores - drawn via frame writes) present 4:3, cropped to the
		// center 512. Hysteresis: quads flip to wide IMMEDIATELY, but
		// dropping to 4:3 needs a sustained run of quad-free write
		// iterations - a mid-frame iteration catching only HUD frame
		// writes otherwise flickered the aspect (seen as width "wobble").
		static int s_no_quad_writes = 0;
		if (frame_ready && had_quads_iter)
		{
			wide_mode = (MARGIN > 0);
			s_no_quad_writes = 0;
		}
		else if (frame_ready && had_writes_iter && ++s_no_quad_writes >= 30)
			wide_mode = false;
		if (frame_ready) had_quads_iter = had_writes_iter = false;

		int const cw = rc.right - rc.left, ch = rc.bottom - rc.top;
		gl.BindFramebuffer(FRAMEBUFFER, 0);
		gl.Disable(GLDEPTH_TEST);
		gl.Disable(GLSCISSOR_TEST);
		gl.Viewport(0, 0, cw, ch);
		gl.ClearColor(0, 0, 0, 1);
		gl.Clear(0x4000);
		// PAR ~1.0: 512x400 4:3-ish; 16:9 spans the full CWW canvas
		float const aspect = wide_mode ? (16.0f / 9.0f) : (4.0f / 3.0f);
		int vw = cw, vh = int(cw / aspect + 0.5f);
		if (vh > ch) { vh = ch; vw = int(ch * aspect + 0.5f); }
		gl.UseProgram(present);
		gl.Uniform1i(uBaseRow, int((zb38 >> 16) & (CH - 1)));
		gl.Uniform1i(uCrt, crt ? 1 : 0);
		gl.Uniform1f(gl.GetUniformLocation(present, "uSampW"),
			wide_mode ? float(CWW) : 512.0f);
		gl.Uniform1f(gl.GetUniformLocation(present, "uSampX0"),
			wide_mode ? 0.0f : float(MARGIN));
		gl.ActiveTexture(TEXTURE0 + 3);
		gl.BindTexture(0x0DE1, fbTex);
		gl.Viewport((cw - vw) / 2, (ch - vh) / 2, vw, vh);
		gl.BindVertexArray(vao_empty);
		gl.DrawArrays(0x0004, 0, 3);

		if (menu_open)
		{
			gl.Viewport(0, 0, cw, ch);
			gl.Enable(0x0BE2);
			gl.BlendFunc(0x0302, 0x0303);
			gl.UseProgram(menuprog);
			gl.Uniform2f(mScreen, float(cw), float(ch));
			auto mrect = [&](Label const *L, float x, float y, float w2, float h2,
					float r2, float g2, float b2, float a2)
			{
				gl.Uniform4f(mRect, x, y, w2, h2);
				gl.Uniform4f(mColor, r2, g2, b2, a2);
				gl.Uniform1i(mSolid, L ? 0 : 1);
				if (L)
				{
					gl.ActiveTexture(TEXTURE0);
					gl.BindTexture(0x0DE1, L->tex);
				}
				gl.DrawArrays(0x0005, 0, 4);
			};
			auto mlabel = [&](Label const &L, float cy, float px,
					float r2, float g2, float b2)
			{
				float const h2 = px, w2 = L.w * px / L.h;
				mrect(&L, (cw - w2) / 2.0f, cy, w2, h2, r2, g2, b2, 1.0f);
			};
			float const sc = ch / 1080.0f;
			mrect(nullptr, 0, 0, float(cw), float(ch), 0, 0, 0, 0.55f);
			if (cheat_menu.open) {
				int slot = 0;
				cruisn::draw_pause_cheats(cheat_menu, [&](std::string const &value, float y, float px, bool selected) {
					auto &label = text_labels[slot];
					if (!label.tex || text_values[slot] != value) {
						auto bitmap = cruisn::menu_text_bitmap(value);
						if (!label.tex) label = make_label(bitmap.pixels.data(), bitmap.width, bitmap.height);
						else {
							gl.ActiveTexture(TEXTURE0); gl.BindTexture(0x0DE1, label.tex);
							gl.TexImage2D(0x0DE1, 0, 0x8229, bitmap.width, bitmap.height, 0, 0x1903, 0x1401, bitmap.pixels.data());
							label.w = bitmap.width; label.h = bitmap.height;
						}
						text_values[slot] = value;
					}
					++slot;
					float const height = std::min(px * sc, float(cw) * .90f * label.h / label.w);
					mlabel(label, ch * y, height, selected ? 1.f : .85f, selected ? .72f : .85f, selected ? .20f : .90f);
				});
			} else {
			mlabel(lb_title, ch * 0.24f, 72 * sc, 1.0f, 0.72f, 0.20f);
			Label const *items[4] = { &lb_resume, crt ? &lb_crt_on : &lb_crt_off, &lb_cheats, &lb_exit };
			for (int i = 0; i < 4; i++)
			{
				bool const sel = (i == menu_sel);
				mlabel(*items[i], ch * (0.42f + 0.09f * i), 44 * sc,
					sel ? 1.0f : 0.85f, sel ? 0.72f : 0.85f, sel ? 0.20f : 0.90f);
			}
			mlabel(lb_hint, ch * 0.86f, 22 * sc, 0.75f, 0.75f, 0.80f);
			}
			gl.Disable(0x0BE2);
		}

		++presents;
		// MIDZ_GL_SNAP_EVERY=<n> presents between snapshots (default 150; 1 =
		// every present, for flicker / partial-frame hunts)
		static int s_snap_every = -1;
		if (s_snap_every < 0)
		{
			const char *e = std::getenv("MIDZ_GL_SNAP_EVERY");
			s_snap_every = (e && atoi(e) > 0) ? atoi(e) : 150;
		}
		static long s_snap_from = -1;   // MIDZ_GL_SNAP_FROM=<present>: start snapshots there
		if (s_snap_from < 0)
		{
			const char *e = std::getenv("MIDZ_GL_SNAP_FROM");
			s_snap_from = e ? atol(e) : 0;
		}
		static int s_snap_max = -1;     // MIDZ_GL_SNAP_MAX=<n>: stop after n snapshots (dense runs)
		if (s_snap_max < 0)
		{
			const char *e = std::getenv("MIDZ_GL_SNAP_MAX");
			s_snap_max = (e && atoi(e) > 0) ? atoi(e) : 1000000;
		}
		static double s_snap_from_sec = -1.0;   // MIDZ_GL_SNAP_FROM_SEC=<machine seconds>: present cadence varies run to run, game time does not
		if (s_snap_from_sec < 0)
		{
			const char *e = std::getenv("MIDZ_GL_SNAP_FROM_SEC");
			s_snap_from_sec = e ? atof(e) : 0.0;
		}
		static long s_snap_base = -1;
		if (s_snap_base < 0 && completed_seconds >= s_snap_from_sec)
			s_snap_base = presents;
		int const first_frame = envi("MIDZ_GL_SNAP_FIRST",nullptr,0);
        int const last_frame = envi("MIDZ_GL_SNAP_LAST",nullptr,2147483647);
        bool const menu_test_capture = menu_test_step >= 0 && menu_test_step < 9 && menu_test_saved != menu_test_step;
        if (snapdir && (menu_test_capture || (!menu_open && snap_index && frame_ready && s_snap_base >= 0 && presents >= s_snap_from &&
            completed_frame >= uint32_t(first_frame) && completed_frame <= uint32_t(last_frame) &&
            (completed_frame % s_snap_every) == 0 && snap_n < s_snap_max)))
		{
			std::vector<uint8_t> px(size_t(cw) * ch * 3);
			// This destination has tight RGB rows. The default four-byte pack
			// alignment pads odd-width rows and can write beyond the allocation.
			gl.PixelStorei(0x0D05 /*GL_PACK_ALIGNMENT*/, 1);
			const uint64_t read_began = begin_phase(PHASE_READBACK);
			gl.ReadPixels(0, 0, cw, ch, 0x80E0, 0x1401, px.data());
			finish_phase(PHASE_READBACK, read_began);
			const uint64_t file_began = begin_phase(PHASE_FILE);
			char path[512];
			if (menu_test_capture) snprintf(path, sizeof(path), "%s\\menu_%02d.bmp", snapdir, menu_test_step);
			else snprintf(path, sizeof(path), "%s\\mzgl_%03d.bmp", snapdir, snap_n++);
			FILE *f = fopen(path, "wb");
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
				std::vector<uint8_t> rowbuf(rowsz, 0);
				for (int y = 0; y < ch; y++)
				{
					memcpy(rowbuf.data(), &px[size_t(y) * cw * 3], cw * 3);
					fwrite(rowbuf.data(), 1, rowsz, f);
				}
                fclose(f);
                if (menu_test_capture) {
                    menu_test_saved = menu_test_step;
                    zlogf("menu snapshot step=%d open=%d selected=%d crt=%d completed_frame=%u new_frame=%d",
                        menu_test_step,int(menu_open),menu_sel,int(crt),completed_frame,int(frame_ready));
                    zlogf("cheat menu snapshot step=%d submenu=%d entries=%u pending=%u",
                        menu_test_step,int(cheat_menu.open),unsigned(cheat_menu.rows.size()),unsigned(cheat_menu.pending.size()));
                } else {
                fprintf(snap_index,"mzgl_%03d.bmp,%llu,%u,%d,%d,%llu,%u,%u\n", snap_n-1,
                    (unsigned long long)presents,completed_frame,cw,ch,(unsigned long long)n_quads,
                    s_drops_quad.load()+s_drops_state.load(),completed_frame);
                fflush(snap_index);
                }
			}
			finish_phase(PHASE_FILE, file_began);
		}
		const uint64_t swap_began = begin_phase(PHASE_SWAP);
		SwapBuffers(dc);
		finish_phase(PHASE_SWAP, swap_began);
		if (frame_ready) s_presented_frame.store(completed_frame);
		if (!vsync)
			Sleep(4);   // ~4 ms pace: plenty of presents, no busy spin
	}

	if (char const *sf = std::getenv("MIDV_GL_STATEFILE"))
	{
		FILE *f = fopen(sf, "w");
		if (f)
		{
			fprintf(f, "crt=%d\n", crt ? 1 : 0);
			fclose(f);
		}
	}
	if (snap_index) fclose(snap_index);
	if (s_phase_trace)
		for (unsigned phase = PHASE_READBACK; phase < PHASE_COUNT; ++phase)
			fprintf(stderr,"MIDZ_GL_TIMING phase=%s calls=%llu total_ms=%llu max_ms=%llu\n",
				phase_name(phase), (unsigned long long)phase_calls[phase],
				(unsigned long long)phase_total[phase], (unsigned long long)phase_max[phase]);
	if (palette_trace)
		fprintf(stderr,"MIDZ_PALETTE_RESULT guard=%d conflicts=%llu flushes=%llu\n",
			int(palette_guard),(unsigned long long)palette_conflicts,(unsigned long long)palette_flushes);
	if (margin_trace)
		fprintf(stderr,"MIDZ_MARGIN_RESULT page=%d clears=%llu expanded=%llu\n",
			int(margin_page_clear),(unsigned long long)margin_clears,(unsigned long long)margin_expansions);
	if (sky_trace)
		fprintf(stderr,"MIDZ_SKY_REPEAT_RESULT enabled=%d groups=%llu accepted=%llu copied=%llu budget_rejected=%llu\n",
			int(sky_enabled),(unsigned long long)sky_groups,(unsigned long long)sky_accepted,
			(unsigned long long)sky_copied,(unsigned long long)sky_budget_rejected);
	s_zpause.store(0);
	zlogf("ring drops: quads %u, state(spans/pal/tick) %u", s_drops_quad.load(), s_drops_state.load());
	zlogf("exit after %llu presents, %llu quads",
		(unsigned long long)presents, (unsigned long long)n_quads);
	gl.MakeCurrent(nullptr, nullptr);
	ReleaseDC(child, dc);
	DestroyWindow(child);
}

void start()
{
	if (s_on.load())
		return;
	if (!s_ringbuf)
		s_ringbuf = (uint8_t *)malloc(RING);
	if (!s_ringbuf)
		return;
	s_stopz.store(false);
	s_donez.store(false);
	s_presented_frame.store(0);
	s_phase_trace = std::getenv("MIDZ_GL_LOG") != nullptr;
	s_phase_stamp.store(0);
	begin_phase(PHASE_OTHER);
	s_on.store(true);
	s_thread = std::thread(thread_main);
}

void stop()
{
	if (!s_on.load())
		return;
	// Diagnostic-only: finish already queued captures before tearing resources
	// down. No extra emulated frame, guest write or physical output is generated.
	if (const char *drain = std::getenv("MIDZ_GL_DRAIN_FRAME"))
		if (std::getenv("MIDV_FFB") && !strcmp(std::getenv("MIDV_FFB"),"0")) {
			char *end = nullptr; const auto target = strtoul(drain,&end,10);
			if (*drain >= '0' && *drain <= '9' && !*end && target <= 1000000) {
				const auto began = GetTickCount64();
				while (!s_donez.load() && s_presented_frame.load() < target && GetTickCount64()-began < 10000) Sleep(5);
				fprintf(stderr,"MIDZ capture drain: target=%lu presented=%u wait_ms=%llu complete=%u\n",
					target,s_presented_frame.load(),(unsigned long long)(GetTickCount64()-began),unsigned(s_presented_frame.load()>=target));
			}
		}
	s_stopz.store(true);
	if (s_thread.joinable())
		s_thread.join();
	s_on.store(false);
}

}   // namespace mzgl
#endif   // _WIN32

void zeus2_device::midz_screen_hook(bool completed)
{
#ifdef _WIN32
    if (midz_live && (mzgl::s_donez.load() || mzgl::s_stopz.load())) {
        midz_live = false;
        osd_printf_error("MIDZ render stream failed: GL stopped; restoring CPU polygon rasterization\n");
    }
	if (midz_live)
	{
		// Esc-menu pause: block the emu thread here while the menu is open,
		// which freezes emulation and sound. The overlay thread keeps
		// drawing the menu and clears s_zpause on Resume/Exit, so this
		// loop always exits. (machine().pause() was unreliable: its resume
		// relied on screen_update being called again while paused, which
		// MAME does not do - the menu got stuck paused.) Pump messages so
		// the window stays responsive; Exit clears s_zpause before its
		// WM_CLOSE, so we fall through and the close is handled.
		while (mzgl::s_zpause.load())
		{
			MSG msg;
			while (PeekMessageA(&msg, nullptr, 0, 0, PM_REMOVE))
			{
				TranslateMessage(&msg);
				DispatchMessageA(&msg);
			}
			Sleep(15);
		}
		// flush the waveram dirty span, then the per-frame display tick
		if (mzgl::s_wave_resync.exchange(false))
		{
			s_wave_lo = 0;               // a span was lost to a full ring: resend everything
			s_wave_hi = 16u << 20;
		}
		if (s_wave_hi > s_wave_lo)
		{
			uint32_t hdr2[2] = { s_wave_lo, s_wave_hi - s_wave_lo };
			mzgl::ring_push2(5, hdr2, 8,
				(const uint8_t *)m_waveram.get() + s_wave_lo, hdr2[1]);
			s_wave_lo = ~0u;
			s_wave_hi = 0;
		}
		if (completed) {
            mzgl::FrameTick tick{m_zeusbase[0x38], uint32_t(screen().frame_number()), machine().time().as_double()};
            mzgl::ring_push2(6, &tick, sizeof(tick), nullptr, 0);
        }
	}
#endif
	if (s_cap_state == -2)
	{
		const char *dir = std::getenv("MIDZ_CAPTURE");
		if (!dir)
		{
			s_cap_state = -1;
			return;
		}
		strncpy(s_cap_dir, dir, sizeof(s_cap_dir) - 1);
		if (const char *f = std::getenv("MIDZ_CAPTURE_FRAME"))
			s_cap_frame = strtoul(f, nullptr, 10);
		if (const char *q = std::getenv("MIDZ_CAPTURE_MINQUADS"))
			s_cap_minq = strtoul(q, nullptr, 10);
		s_cap_state = 0;
	}
	if (s_cap_state < 0 || s_cap_state == 2)
		return;
	uint32_t const frame = uint32_t(screen().frame_number());
	uint32_t const fq = s_cap_fq;
	s_cap_fq = 0;
	size_t const fbcount = WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2;
	if (s_cap_state == 0 && frame >= s_cap_frame - 1
		&& (s_cap_minq == 0 || fq >= s_cap_minq))
	{
		poly->wait("MIDZ_CAP_PRE");
		midz_dump(s_cap_dir, "pre_color.bin", m_frameColor.get(), fbcount * 4);
		midz_dump(s_cap_dir, "pre_depth.bin", m_frameDepth.get(), fbcount * 4);
		char path[512];
		snprintf(path, sizeof(path), "%s/records.bin", s_cap_dir);
		s_cap_rec = fopen(path, "wb");
		if (const char *models = std::getenv("MIDZ_CAPTURE_MODELS"); models && !strcmp(models, "1"))
		{
			const char *force = std::getenv("MIDV_FFB");
			if (m_system != CRUSNEXO || !force || strcmp(force, "0"))
				fatalerror("Zeus model capture requires Exotica and MIDV_FFB=0");
			snprintf(path, sizeof(path), "%s/models.bin", s_cap_dir);
			s_cap_models = fopen(path, "wb");
			if (!s_cap_models) fatalerror("Cannot create Zeus model journal");
			s_cap_model_count = s_cap_model_bytes = 0;
		}
		s_cap_pal_valid = false;
		midz_cap = true;
		s_cap_state = 1;
		fprintf(stderr, "MIDZ capture: recording frames %u..%u\n", frame, s_cap_frame);
	}
	else if (s_cap_state == 1
		&& ((s_cap_minq == 0) ? (frame >= s_cap_frame)
			: (s_cap_recq >= s_cap_minq || ++s_cap_frames_rec > 10)))
	{
		poly->wait("MIDZ_CAP_POST");
		midz_cap = false;
		midz_dump(s_cap_dir, "post_color.bin", m_frameColor.get(), fbcount * 4);
		midz_dump(s_cap_dir, "post_depth.bin", m_frameDepth.get(), fbcount * 4);
		midz_dump(s_cap_dir, "waveram.bin", m_waveram.get(), WAVERAM0_WIDTH * WAVERAM0_HEIGHT * 8);
		midz_dump(s_cap_dir, "pal_table.bin", m_pal_table, sizeof(m_pal_table));
		if (s_cap_rec)
		{
			fclose(s_cap_rec);
			s_cap_rec = nullptr;
		}
		char path[512];
		if (s_cap_models)
		{
			if (fclose(s_cap_models)) fatalerror("Cannot close Zeus model journal");
			s_cap_models = nullptr;
			snprintf(path, sizeof(path), "%s/models.json", s_cap_dir);
			FILE *f = fopen(path, "w");
			if (!f) fatalerror("Cannot create Zeus model completion receipt");
			const int written = fprintf(f, "{\"schema\":1,\"complete\":true,\"models\":%u,\"bytes\":%u,\"quads\":%u}\n",
				s_cap_model_count, s_cap_model_bytes, s_cap_recq);
			const int closed = fclose(f);
			if (written <= 0 || closed) fatalerror("Cannot complete Zeus model receipt");
		}
		snprintf(path, sizeof(path), "%s/regs.txt", s_cap_dir);
		if (FILE *f = fopen(path, "w"))
		{
			fprintf(f, "yScale %d\npalSize %d\ncliprect %d %d %d %d\nsystem %d\n",
				m_yScale, m_palSize, zeus_cliprect.min_x, zeus_cliprect.min_y,
				zeus_cliprect.max_x, zeus_cliprect.max_y, m_system);
			for (int i = 0; i < 0x80; i++)
				fprintf(f, "zb%02x %08X\n", i, m_zeusbase[i]);
			for (int i = 0; i < 0x50; i++)
				fprintf(f, "rr%02x %08X\n", i, m_renderRegs[i]);
			fclose(f);
		}
		s_cap_state = 2;
		fprintf(stderr, "MIDZ capture complete in %s\n", s_cap_dir);
	}
}

void zeus2_device::midz_cap_quad(int numverts, const void *verts,
		const zeus2_poly_extra_data &extra, uint32_t texdata)
{
	if (!(midz_cap && s_cap_rec) && !midz_live)
		return;
	if (midz_cap)
		++s_cap_recq;
	if (s_pal_dirty || !s_cap_pal_valid)
	{
		memcpy(s_cap_pal, m_pal_table, sizeof(s_cap_pal));
		s_cap_pal_valid = true;
		s_pal_dirty = false;
		midz_rec(2, s_cap_pal, sizeof(s_cap_pal));
	}
	auto const *v = reinterpret_cast<const z2_poly_vertex *>(verts);
	midz_quad_rec r = {};
	// frame number matters only to the offline capture; the live overlay
	// ignores it, and screen().frame_number() per quad (~6.5k/frame) is a
	// measurable emu-thread cost
	r.frame = midz_cap ? uint32_t(screen().frame_number()) : 0;
	r.numverts = numverts;
	r.texdata = texdata;
	r.tex_src = extra.tex_src;
	r.texwidth = extra.texwidth;
	r.solidcolor = extra.solidcolor;
	r.transcolor = extra.transcolor;
	r.srcAlpha = extra.srcAlpha;
	r.dstAlpha = extra.dstAlpha;
	r.flags = (extra.solid_enable ? 1 : 0) | (extra.blend_enable ? 2 : 0)
		| (extra.depth_min_enable ? 4 : 0) | (extra.depth_test_enable ? 8 : 0)
		| (extra.depth_write_enable ? 16 : 0) | (extra.depth_clear_enable ? 32 : 0)
		| (extra.texture_alpha ? 64 : 0) | (extra.texture_rgb555 ? 128 : 0)
		| (extra.depth_floor_enable ? cruisn::zeus_policy::QuadDepthFloor : 0);
	r.zbuf_min = extra.zbuf_min;
	r.rr04 = m_renderRegs[0x4];
	r.yscale = uint32_t(m_yScale);
	r.clip[0] = zeus_cliprect.min_x;
	r.clip[1] = zeus_cliprect.min_y;
	r.clip[2] = zeus_cliprect.max_x;
	r.clip[3] = zeus_cliprect.max_y;
	for (int i = 0; i < numverts && i < 8; i++)
	{
		r.verts[i][0] = v[i].x;
		r.verts[i][1] = v[i].y;
		for (int p = 0; p < 4; p++)
			r.verts[i][2 + p] = v[i].p[p];
	}
	midz_rec(1, &r, sizeof(r));
}

void zeus2_device::midz_cap_clear(uint32_t addr, uint32_t numPixels, uint32_t color, int32_t depth)
{
	if (!(midz_cap && s_cap_rec) && !midz_live)
		return;
	uint32_t p[4] = { addr, numPixels, color, uint32_t(depth) };
	midz_rec(3, p, sizeof(p));
}

void zeus2_device::midz_capture_framewrite()
{
	if (!s_cap_rec && !midz_live)
		return;
	uint32_t p[6] = { frame_addr_from_phys_addr(m_zeusbase[0x51]),
		m_zeusbase[0x57], m_zeusbase[0x58], m_zeusbase[0x59],
		m_zeusbase[0x5a], m_zeusbase[0x5e] };
	midz_rec(4, p, sizeof(p));
}

void zeus2_device::midz_wave_dirty(uint32_t addr41)
{
	uint32_t const block = (addr41 % WAVERAM0_WIDTH)
		+ ((addr41 >> 16) % WAVERAM0_HEIGHT) * WAVERAM0_WIDTH;
	uint32_t const off = block * 8;
	if (off < s_wave_lo) s_wave_lo = off;
	if (off + 8 > s_wave_hi) s_wave_hi = off + 8;
}

void zeus2_device::device_stop()
{
#ifdef _WIN32
	mzgl::stop();
#endif
#if DUMP_WAVE_RAM
	std::string fileName = "waveram_";
	fileName += machine().system().name;
	fileName += ".bin";
	std::ofstream myfile;
	myfile.open(fileName.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);

	if (myfile.is_open())
		myfile.write((char *)m_waveram.get(), WAVERAM0_WIDTH * WAVERAM0_HEIGHT * 2 * sizeof(uint32_t));
	myfile.close();
#endif

#if TRACK_REG_USAGE
{
	reg_info *info;
	int regnum;

	for (regnum = 0; regnum < 0x80; regnum++)
	{
		printf("Register %02X\n", regnum);
		if (regread_count[regnum] == 0)
			printf("\tNever read\n");
		else
			printf("\tRead %d times\n", regread_count[regnum]);

		if (regwrite_count[regnum] == 0)
			printf("\tNever written\n");
		else
		{
			printf("\tWritten %d times\n", regwrite_count[regnum]);
			for (info = regdata[regnum]; info != nullptr; info = info->next)
				printf("\t%08X\n", info->value);
		}
	}

	for (regnum = 0; regnum < 0x100; regnum++)
		if (subregwrite_count[regnum] != 0)
		{
			printf("Sub-Register %02X (%d writes)\n", regnum, subregwrite_count[regnum]);
			for (info = subregdata[regnum]; info != nullptr; info = info->next)
				printf("\t%08X\n", info->value);
		}
}
#endif

}



/*************************************
 *
 *  Video update
 *
 *************************************/

uint32_t zeus2_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	mzgl::s_secs.store(machine().time().as_double());   // POC: game time for the overlay's snapshot trigger
	midv_trace_wheelpos(machine(), ":ANALOG3");   // POC: FFB trace, input half (Exotica steering)
	// Wait until configuration is completed before transfering anything
	if (!(m_zeusbase[0x10] & 0x20))
		return 0;

	midz_screen_hook(cliprect.max_y >= screen.visible_area().max_y);

	// POC scoping stats (MIDZ_STATS=1): quads/frame profile to stderr
	if (s_mz_stats > 0)
	{
		if (s_mz_quads > s_mz_qmax) s_mz_qmax = s_mz_quads;
		if ((++s_mz_frames % 300) == 0)
			fprintf(stderr, "MIDZ frame %u: quads this frame %u, max %u\n",
					s_mz_frames, s_mz_quads, s_mz_qmax);
		s_mz_quads = 0;
	}

	int x, y;

	//if (machine().input().code_pressed(KEYCODE_DOWN)) { zbase += machine().input().code_pressed(KEYCODE_LSHIFT) ? 0x10 : 1; popmessage("Zbase = %f", (double)zbase); }
	//if (machine().input().code_pressed(KEYCODE_UP)) { zbase -= machine().input().code_pressed(KEYCODE_LSHIFT) ? 0x10 : 1; popmessage("Zbase = %f", (double)zbase); }

	/* normal update case */
	//if (!machine().input().code_pressed(KEYCODE_W))
	if (1)
	{
		for (y = cliprect.min_y; y <= cliprect.max_y; y++)
		{
			uint32_t *colorptr = &m_frameColor[frame_addr_from_xy(0, y, false)];
			std::copy(colorptr + cliprect.min_x, colorptr + cliprect.max_x + 1, &bitmap.pix(y, cliprect.min_x));
		}
	}

	/* waveram drawing case */
	else
	{
		const void *base;

		if (machine().input().code_pressed(KEYCODE_DOWN)) yoffs += machine().input().code_pressed(KEYCODE_LSHIFT) ? 0x1000 : 40;
		if (machine().input().code_pressed(KEYCODE_UP)) yoffs -= machine().input().code_pressed(KEYCODE_LSHIFT) ? 0x1000 : 40;
		if (machine().input().code_pressed(KEYCODE_LEFT) && texel_width > 4) { texel_width >>= 1; while (machine().input().code_pressed(KEYCODE_LEFT)) ; }
		if (machine().input().code_pressed(KEYCODE_RIGHT) && texel_width < 512) { texel_width <<= 1; while (machine().input().code_pressed(KEYCODE_RIGHT)) ; }

		if (yoffs < 0) yoffs = 0;
		if (1) {
			//base = waveram0_ptr_from_expanded_addr(yoffs << 8);
			//base = waveram0_ptr_from_expanded_addr(yoffs);
			base = WAVERAM_BLOCK0(yoffs);
		}
		else
			base = (void *)&m_frameColor[yoffs << 6];

		int xoffs = screen.visible_area().min_x;
		for (y = cliprect.min_y; y <= cliprect.max_y; y++)
		{
			uint32_t *const dest = &bitmap.pix(y);
			for (x = cliprect.min_x; x <= cliprect.max_x; x++)
			{
				if (1) {
					uint8_t tex = get_texel_8bit_4x2((uint64_t *)base, y, x, texel_width);
					dest[x] = (tex << 16) | (tex << 8) | tex;
				}
				else {
					dest[x] = ((uint32_t *)(base))[((y * WAVERAM1_WIDTH)) + x - xoffs];
				}
			}
		}
		popmessage("offs = %06X base = %08X", yoffs, base);
	}

	return 0;
}



/*************************************
 *
 *  Core read handler
 *
 *************************************/

uint32_t zeus2_device::zeus2_r(offs_t offset)
{
	int logit = (offset != 0x00 && offset != 0x01 &&
		offset != 0x18 && offset != 0x19 && offset != 0x48 && offset != 0x49
		&& offset != 0x54 && offset != 0x58 && offset != 0x59 && offset != 0x5a
		);
	logit &= LOG_REGS;
	uint32_t result = m_zeusbase[offset];
#if TRACK_REG_USAGE
	regread_count[offset]++;
#endif

	switch (offset)
	{
		case 0x00:
			// STATUS0
			// 0x20 IFIFO Empty
			// 0x80 IFIFO Full
			result = 0x20;
			break;

		case 0x01:
			// STATUS1
			// 0x00000004 VBLANK
			// 0x40000000 Slave Error
			// 0x80000000 Master Error
			// 0x00000010 DP_ACTIVE
			// 0x00000020 TP_ACTIVE
			// 0x00000040 RP_ACTIVE
			// 0x00040000 EXPY_ACTIVE0
			// 0x00080000 EXPY_ACTIVE1
			/* bit  $000C0070 are tested in a loop until 0 */
			/* bits $00080000 is tested in a loop until 0 */
			/* bit  $00000004 is tested for toggling; probably VBLANK */
			result = 0x00;
			if (screen().vblank())
				result |= 0x04;
			break;

		case 0x07:
			/* this is needed to pass the self-test in thegrid */
			result = 0x10451998;
			break;

		case 0x18:
			// IFIFO Read Counter
			break;

		case 0x19:
			// MFIFO Read Counter
			break;

		case 0x54:
			// VCOUNT upper 16 bits
			//result = (screen().vpos() << 16) | screen().vpos();
			result = (screen().vpos() << 16);
			break;
	}

	if (logit)
		logerror("%s:zeus2_r(%02X) = %08X\n", machine().describe_context(), offset, result);

	return result;
}



/*************************************
 *
 *  Core write handler
 *
 *************************************/

void zeus2_device::zeus2_w(offs_t offset, uint32_t data)
{
	int logit = (offset != 0x08 &&
					 offset != 0x18 && offset != 0x19 && (offset != 0x20 || data != 0) &&
					 offset != 0x35 && offset != 0x36 && offset != 0x37 &&
					 offset != 0x48 && offset != 0x49 && offset != 0x40 && offset != 0x41 && offset != 0x4e &&
					 offset != 0x50 && offset != 0x51 && offset != 0x57 && offset != 0x58 && offset != 0x59 && offset != 0x5a && offset != 0x5e
		);
	logit &= LOG_REGS;
	if (logit)
		logerror("%s:zeus2_w", machine().describe_context());
	zeus2_register32_w(offset, data, logit);
}



/*************************************
 *
 *  Handle register writes
 *
 *************************************/

void zeus2_device::zeus2_register32_w(offs_t offset, uint32_t data, int logit)
{
	uint32_t oldval = m_zeusbase[offset];

#if TRACK_REG_USAGE
regwrite_count[offset]++;
if (regdata_count[offset] < 256)
{
	reg_info **tailptr;

	for (tailptr = &regdata[offset]; *tailptr != nullptr; tailptr = &(*tailptr)->next)
		if ((*tailptr)->value == data)
			break;
	if (*tailptr == nullptr)
	{
		*tailptr = alloc_or_die(reg_info);
		(*tailptr)->next = nullptr;
		(*tailptr)->value = data;
		regdata_count[offset]++;
	}
}
#endif

	/* writes to register $CC need to force a partial update */
//  if ((offset & ~1) == 0xcc)
//      screen().update_partial(screen().vpos());

	/* always write to low word? */
	m_zeusbase[offset] = data;

	/* log appropriately */
	if (logit) {
		logerror("(%02X) = %08X", offset, data);
	}
	/* handle the update */
	zeus2_register_update(offset, oldval, logit);
}



/*************************************
 *
 *  Update state after a register write
 *
 *************************************/

void zeus2_device::zeus2_register_update(offs_t offset, uint32_t oldval, int logit)
{
	/* handle the writes; only trigger on low accesses */
	switch (offset)
	{
	case 0x08:
		zeus_fifo[zeus_fifo_words++] = m_zeusbase[0x08];
		if (zeus2_fifo_process(zeus_fifo, zeus_fifo_words))
			zeus_fifo_words = 0;

		/* set the interrupt signal to indicate we can handle more */
		// Not sure how much to time to put here
		int_timer->adjust(attotime::from_usec(20));
		break;

	case 0x10:
		// BITS 11 - 10 COL SIZE / BANK FOR WR1
		// BITS 9 - 8   COL SIZE / BANK FOR WR0
		if (logit) logerror("\tSys Setup");
		if (m_zeusbase[0x10] & 0x20)
		{
			m_yScale = (((m_zeusbase[0x39] >> 16) & 0xfff) < 0x100) ? 0 : 1;
			int hor = ((m_zeusbase[0x34] & 0xffff) - (m_zeusbase[0x33] >> 16)) << m_yScale;
			int ver = ((m_zeusbase[0x35] & 0xffff) + 1) << m_yScale;
			// POC: stock MAME debug popmessage - flashes "reg[30]: ...
			// Screen: 512H X 400V" at the bottom on every boot. Silenced
			// for the collection (kept as logerror for diagnostics).
			logerror("reg[30]: %08X Screen: %dH X %dV yScale: %d\n", m_zeusbase[0x30], hor, ver, m_yScale);
			int vtotal = (m_zeusbase[0x37] & 0xffff) << m_yScale;
			int htotal = (m_zeusbase[0x34] >> 16) << m_yScale;
			//rectangle visarea((m_zeusbase[0x33] >> 16) << m_yScale, htotal - 1, 0, (m_zeusbase[0x35] & 0xffff) << m_yScale);
			rectangle visarea(0, hor - 1, 0, ver - 1);
			screen().configure(htotal, vtotal, visarea, HZ_TO_ATTOSECONDS(ZEUS2_VIDEO_CLOCK / 4.0 / (htotal * vtotal)));
			zeus_cliprect = visarea;
			zeus_cliprect.max_x -= zeus_cliprect.min_x;
			zeus_cliprect.min_x = 0;
			// Startup vblank timer
			vblank_timer->adjust(attotime::from_usec(1));
		}
		break;

	case 0x11:
		if (logit) logerror("\tHost Interface Setup");
		break;

	case 0x12:
		if (logit) logerror("\tPLL Setup");
		break;

	case 0x13:
		if (logit) logerror("\tZeus Test Out");
		break;

	case 0x20:
		zeus2_pointer_write(m_zeusbase[0x20] >> 24, (m_zeusbase[0x20] & 0xffffff), logit);
		break;

	case 0x22:
		if (logit) logerror("\tRend Setup 0");
		break;

	case 0x23:
		if (logit) logerror("\tRend Setup 1");
		break;

	case 0x24:
		// 0x601 == test mode
		if (logit) logerror("\tRend Setup 4");
		break;

	case 0x2a:
		// 0x000000c0 = bilinear off
		if (logit) logerror("\tRend Force Off");
		break;

	case 0x2b:
		if (logit) logerror("\tRend Force On");
		break;

	case 0x2c:
		if (logit) logerror("\tRend AE Flag");
		break;

	case 0x2d:
		if (logit) logerror("\tRend AF Flag");
		break;

	case 0x2f:
		if (logit) logerror("\tPixel Proc Setup");
		break;


	case 0x30:
		if (logit) logerror("\tCRT Controller Setup = %08X", m_zeusbase[offset]);
		break;

	case 0x31:
		if (logit) logerror("\tDotClk Sel 1 : DotClk Sel 2 = %08X", m_zeusbase[offset]);
		break;
	case 0x32:
		if (logit) logerror("\tHSync End = %i HSync Start = %i", (m_zeusbase[offset] >> 16), m_zeusbase[offset] & 0xffff);
		break;
	case 0x33:
		if (logit) logerror("\tHBlank End = %i Update Start = %i", (m_zeusbase[offset] >> 16), m_zeusbase[offset] & 0xffff);
		break;
	case 0x34:
		if (logit) logerror("\tHTotal = %i HBlank Start = %i", (m_zeusbase[offset] >> 16), m_zeusbase[offset] & 0xffff);
		break;
	case 0x35:
		if (logit) logerror("\tVSync Start = %i VBlank Start = %i", (m_zeusbase[offset] >> 16), m_zeusbase[offset] & 0xffff);
		break;
	case 0x36:
		if (logit) logerror("\tVTotal = %i VSync End = %i", (m_zeusbase[offset] >> 16), m_zeusbase[offset] & 0xffff);
		break;
	case 0x37:
		if (logit) logerror("\tVTotal = %i", m_zeusbase[offset]);
		break;
	case 0x38:
	{
		uint32_t temp = m_zeusbase[0x38];
		m_zeusbase[0x38] = oldval;
		screen().update_partial(screen().vpos());
		log_fifo = machine().input().code_pressed(KEYCODE_L) | ALWAYS_LOG_FIFO;
		m_zeusbase[0x38] = temp;
#ifdef _WIN32
		// live overlay: the display-base flip must land IN ORDER with the
		// quad/clear stream - a once-per-frame sample lets the overlay keep
		// presenting a page the game is already clearing (seen as flashing)
		if (midz_live)
			mzgl::ring_push2(6, &m_zeusbase[0x38], 4, nullptr, 0);
#endif
	}
	break;
	case 0x39:
		if (logit) logerror("\tLine Length = %i FIFO AF = %i FIFO AE = %i", (m_zeusbase[offset] >> 16) & 0xfff, (m_zeusbase[offset] >> 8) & 0xff, m_zeusbase[offset] & 0xff);
		break;

	case 0x40: case 0x41: case 0x48: case 0x49: case 0x4e:
		if (offset == 0x41) {
			// mwskinsa (atlantis) writes 0xffffffff and expects 0x1fff03ff to be read back
			m_zeusbase[0x41] &= 0x1fff03ff;
		}
		/*
		m_zeusbase[0x4e] :
		    bit 0 - 1 : which register triggers write through
		    bit 3 : enable write through via these registers
		    bit 4 : seems to be set during reads, when 0x41 is used for latching
		    bit 6 : enable autoincrement on write through
		*/
		if ((offset & 0xf) == (m_zeusbase[0x4e] & 0xf)) {
			// If the address is auto-increment then don't load new value
			// In reality address is latched by a write to 0x40
			if (offset == 0x41 && (m_zeusbase[0x4e] & 0x40)) {
				m_zeusbase[0x41] = oldval;
			}

			int code = (m_zeusbase[0x40] >> 16) & 0xf;
			switch (code) {
			case 0:
				// NOP
				break;
			case 1:
				// Special mode register write
				//if (logit)
				logerror("\t-- Special Mode Write: [4e]: %08X [40]: %08X [41]: %08x [48]: %08x [49]: %08x\n", code, m_zeusbase[0x4e], m_zeusbase[0x40], m_zeusbase[0x41], m_zeusbase[0x48], m_zeusbase[0x49]);
				break;
			case 2:
			{
				/* Read waveram0 */
				//if ((m_zeusbase[0x4e] & 0x20))
				//{
				const void *src = waveram0_ptr_from_expanded_addr(m_zeusbase[0x41]);
				m_zeusbase[0x48] = WAVERAM_READ32(src, 0);
				m_zeusbase[0x49] = WAVERAM_READ32(src, 1);

				if (m_zeusbase[0x4e] & 0x40)
				{
					m_zeusbase[0x41]++;
					m_zeusbase[0x41] += (m_zeusbase[0x41] & 0x400) << 6;
					m_zeusbase[0x41] &= ~0xfc00;
				}
				//}
			}
			break;
			case 4:
			{
				// Load pal table from RGB555
				if (logit)
					logerror("\t-- pal table rgb555 load: control: %08X addr: %08X", m_zeusbase[0x40], m_zeusbase[0x41]);
				poly->wait("PAL_TABLE_WRITE");
				// blocknum = (addr % WAVERAM0_WIDTH) + ((addr >> 16) % WAVERAM0_HEIGHT) * WAVERAM0_WIDTH;
				m_curPalTableSrc = (m_zeusbase[0x41] % WAVERAM0_WIDTH) + ((m_zeusbase[0x41] >> 16) % WAVERAM0_HEIGHT) * WAVERAM0_WIDTH;
				void *dataPtr = waveram0_ptr_from_expanded_addr(m_zeusbase[0x41]);
				load_pal_table(dataPtr, m_zeusbase[0x40], 0, logit);
			}
			break;
			case 5:
			{
				//if (m_zeusbase[0x41] == 0x266) {
				//  logit = 1;
				//  log_fifo = 1;
				//}
				// Zeus microcode burst from waveram
				if (logit)
					logerror("\t-- ucode load: control: %08X addr: %08X", m_zeusbase[0x40], m_zeusbase[0x41]);
				// Load ucode from waveram
				poly->wait("UCODE_LOAD");
				void *dataPtr = waveram0_ptr_from_expanded_addr(m_zeusbase[0x41]);
				load_ucode(dataPtr, m_zeusbase[0x40], logit);
				if (((m_zeusbase[0x40] >> 24) & 0xff) >= 0xc0) {
					// Light table load
					if (logit) logerror("\t-- light table loaded");
				}
				else {
					m_curUCodeSrc = m_zeusbase[0x41];
					m_useZOffset = false;
					// Zeus Quad Size
					if (m_system == THEGRID) {
						switch (m_curUCodeSrc) {
						case 0x000000037:
							// pm4dl: mfifo quads dynamic light
							zeus_quad_size = 14;
							if (logit) logerror(" pm4dl quad size %d\n", zeus_quad_size);
							break;
						case 0x0000000b3:
							// pm4sl: mfifo quads static light
							zeus_quad_size = 12;
							if (logit) logerror(" pm4sl quad size %d\n", zeus_quad_size);
							break;
						case 0x00000015d:
							// pm4nl: mfifo quads no light
							m_useZOffset = true;
							zeus_quad_size = 10;
							if (logit) logerror(" pm4nl quad size %d\n", zeus_quad_size);
							break;
						case 0x0000001d5:
							// pm4nluv: mfifo quads no light (tga and uv variants)
							m_useZOffset = true;
							zeus_quad_size = 10;
							if (logit) logerror(" pm4nluv pm4nl_tga quad size %d\n", zeus_quad_size);
							break;
						case 0x000000266:
							// pm3dli: mfifo trimesh dynamic light interpolation
							zeus_quad_size = 8;
							if (logit) logerror(" pm3dli quad size %d\n", zeus_quad_size);
							break;
						case 0x0000002e9:
							// px3sl: xfifo trimesh static light
							zeus_quad_size = 10;
							if (logit) logerror(" px3sl quad size %d\n", zeus_quad_size);
							break;
						case 0x000000343:
							// blit24: The Grid direct rendering
							// Really should be 128 but this works
							zeus_quad_size = 512;
							if (logit) logerror(" blit24 quad size %d\n", zeus_quad_size);
							break;
						default:
							zeus_quad_size = 10;
							logerror(" unknown quad size from source [41]=%08X\n", m_curUCodeSrc);
							break;
						}
					}
					else if (m_system == CRUSNEXO) {
						switch (m_curUCodeSrc) {
						case 0x0000000c0:
							zeus_quad_size = 10;
							if (logit) logerror(" c0 quad size %d\n", zeus_quad_size);
							break;
						case 0x000000136:
							zeus_quad_size = 14;
							if (logit) logerror(" 136 quad size %d\n", zeus_quad_size);
							break;
						case 0x0000001ba:
							zeus_quad_size = 10;
							if (logit) logerror(" 1ba quad size %d\n", zeus_quad_size);
							break;
						case 0x00000022b:
							zeus_quad_size = 12;
							if (logit) logerror(" 22b quad size %d\n", zeus_quad_size);
							break;
						case 0x00000029b:
							zeus_quad_size = 12;
							if (logit) logerror(" 29b quad size %d\n", zeus_quad_size);
							break;
						case 0x000000324:
							zeus_quad_size = 12;
							if (logit) logerror(" 324 quad size %d\n", zeus_quad_size);
							break;
						default:
							zeus_quad_size = 10;
							logerror(" unknown quad size from source [41]=%08X\n", m_curUCodeSrc);
							break;
						}

					}
					else {
						switch (m_curUCodeSrc) {
						case 0x000000000:
							zeus_quad_size = 14;
							if (logit) logerror(" 00 quad size %d\n", zeus_quad_size);
							break;
						case 0x00000002d:
							zeus_quad_size = 14;
							if (logit) logerror(" 2d quad size %d\n", zeus_quad_size);
							break;
						case 0x0000001aa:
							// Direct frame buffer access, render data is rgb565
							zeus_quad_size = 14;
							if (logit) logerror(" 1aa quad size %d\n", zeus_quad_size);
							break;
						default:
							zeus_quad_size = 14;
							logerror(" unknown quad size from source [41]=%08X\n", m_curUCodeSrc);
							break;
						}

					}
				}
				if (1 && logit) {
					uint32_t *wavePtr = (uint32_t*)waveram0_ptr_from_expanded_addr(m_zeusbase[0x41]);
					uint32_t waveData;
					int size = m_zeusbase[0x40] & 0xff;
					logerror("\n Setup size=%d [40]=%08X [41]=%08X [4e]=%08X\n", zeus_quad_size, m_zeusbase[0x40], m_zeusbase[0x41], m_zeusbase[0x4e]);
					for (int i = 0; i <= size; ++i) {
						if (m_zeusbase[0x41] < 0xc0) {
							waveData = wavePtr[1];
							logerror(" %08X", waveData);
							waveData = wavePtr[0];
							logerror("%08X", waveData);
						}
						else {
							waveData = wavePtr[0];
							logerror(" %08X", waveData);
							waveData = wavePtr[1];
							logerror(" %08X", waveData);
						}
						wavePtr += 2;
						if (0 && (i + 1) % 16 == 0)
							logerror("\n");
					}
				}
			}
			break;
			//case 6: {
			//  // Zeus model fifo burst from waveram
			//}
			//break;
			case 9:
			{
				// Waveram Write
				void *dest = waveram0_ptr_from_expanded_addr(m_zeusbase[0x41]);
				WAVERAM_WRITE32(dest, 0, m_zeusbase[0x48]);
				WAVERAM_WRITE32(dest, 1, m_zeusbase[0x49]);
				if (midz_live)
					midz_wave_dirty(m_zeusbase[0x41]);
				if (logit)
					logerror("\t[41]=%08X [4E]=%08X", m_zeusbase[0x41], m_zeusbase[0x4e]);

				if (m_zeusbase[0x4e] & 0x40)
				{
					m_zeusbase[0x41]++;
					m_zeusbase[0x41] += (m_zeusbase[0x41] & 0x400) << 6;
					m_zeusbase[0x41] &= ~0xfc00;
				}
			}
			break;
			default:
			{
				//if (logit)
				logerror("\t-- unknown code (%d): [4e]: %08X [40]: %08X [41]: %08x [48]: %08x [49]: %08x\n", code, m_zeusbase[0x4e], m_zeusbase[0x40], m_zeusbase[0x41], m_zeusbase[0x48], m_zeusbase[0x49]);
			}
			break;
			}
		}
		break;

	case 0x50: case 0x51: case 0x58: case 0x59: case 0x5a: case 0x5b: case 0x5e:
		// m_zeusbase[0x5e]:
		// bits 3:0 select which offset triggers an access
		// bit 4:   clear error
		// bit 5:   24 bit (1) / 32 bit mode (0)
		// bit 6:   enable autoincrement wave address
		// bit 7    autoincrement desination address?
		// bit 8    autoincrement row/col address by 1 (0) or 2 (1)
		if ((offset & 0xf) == (m_zeusbase[0x5e] & 0xf)) {

			int code = (m_zeusbase[0x50] >> 16) & 0xf;

			// If the address is auto-increment then don't load new value
			// In reality address is latched by a write to 0x50
			if (offset == 0x51 && (m_zeusbase[0x5e] & 0x40)) {
				m_zeusbase[0x51] = oldval;
			}

			switch (code) {
			case 0:
				// NOP
				break;
			case 1:
				// SGRAM Special Mode Register Write
				if (m_zeusbase[0x51] == 0x00200000) {
					// SGRAM Mask Register
					if ((m_zeusbase[0x58] & m_zeusbase[0x59] & m_zeusbase[0x5a]) != 0xffffffff)
						logerror("zeus2_register_update: Warning! Mask Register not equal to 0xffffffff\n");
				}
				if (m_zeusbase[0x51] == 0x00400000) {
					// SGRAM Color Register.  R5E bit 5 packs 24 bit color with 24 bit depth,
					// otherwise it is 32 bit color with 16 bit depth.
					bool mode24 = m_zeusbase[0x5e] & 0x20;
					if (mode24) {
						m_fill_color = m_zeusbase[0x58] & 0xffffff;
						m_fill_depth = ((m_zeusbase[0x59] & 0xffff) << 8) | (m_zeusbase[0x58] >> 24);
					}
					else {
						m_fill_color = m_zeusbase[0x58];
						m_fill_depth = (m_zeusbase[0x5a] & 0xffff) << 8;
					}
					if (m_zeusbase[0x58] != m_zeusbase[mode24 ? 0x5a : 0x59])
						logerror("zeus2_register_update: Warning! Different fill colors are set.\n");
					if (logit)
						logerror(" -- Setting fill color = %06X depth = %06X ", m_fill_color, m_fill_depth);
				}
				break;
			case 0x2:
				frame_read();
				break;
			case 0x8:
			{
				// Fast clear
				// Atlantis: 0x00983FFF => clear entire frame buffer, 0x00981FFF => clear one frame
				// crusnexo: 0x007831FF => clear one frame
				// thegrid:  0x008831FF => clear one frame
				// thegrid:  0x0079FFFF => clear entire frame buffer at 51=0 then 51=00800000, only seen at initial tests in thegrid
				uint32_t addr = frame_addr_from_phys_addr(m_zeusbase[0x51]);
				uint32_t numPixels = (m_zeusbase[0x50] & 0xffff) + 1;
				numPixels *= 0x10;
				if (m_zeusbase[0x50] & 0x10000) {
					addr = 0x0;
					numPixels = WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 8;
				}
				if (logit)
					logerror(" -- Clearing buffer: numPixels: %08X addr: %08X reg51: %08X", numPixels, addr, m_zeusbase[0x51]);
				if (midz_cap || midz_live)
					midz_cap_clear(addr, numPixels, m_fill_color, m_fill_depth);
				for (int count = 0; count < numPixels; count++) {
					// Crusn wraps the frame buffer during fill so need to mask address
					m_frameColor[(addr + count) & (WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2 - 1)] = m_fill_color;
					m_frameDepth[(addr + count) & (WAVERAM1_WIDTH * WAVERAM1_HEIGHT * 2 - 1)] = m_fill_depth;
				}
			}
			break;
			case 0x9:
			{
				// Fast fill from local regs
				uint32_t numDWords = (m_zeusbase[0x50] & 0xffff) + 1;
				// Set autoincrement
				if (numDWords>1)
					m_zeusbase[0x5e] |= 0x40;
				if (logit && numDWords > 1)
					logerror(" -- Filling buffer: numDWords: %08X addr: %08X reg50: %08X reg5e: %08X\n", numDWords, m_zeusbase[0x51], m_zeusbase[0x50], m_zeusbase[0x5e]);
				for (int dword = 0; dword < numDWords; dword++)
					frame_write();
			}
			break;
			default:
				logerror("unknown code = %x offset = %x", code, offset);
				break;
			}
		}
		break;

		// 0x60, 0x61, 0x62 Translation matrix, set using fifo command

	case 0x60: case 0x61: case 0x62: case 0x63:
		zeus_trans[offset & 3] = convert_float(m_zeusbase[offset]);
		if (logit)
			logerror("\tMAC Trans%d = %8.2f", offset & 3, reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x64:
		if (logit)
			logerror("\tMAC Offset");
		break;

	case 0x65:
		if (logit)
			logerror("\tMAC Offset");
		break;

	case 0x66:
		if (logit)
			logerror("\tMultiply Offset");
		break;

	case 0x67:
		if (logit)
			logerror("\tMath MAC Setup");
		break;

	case 0x68:
		if (logit)
			logerror("\tALU Float Offset");
		break;

	case 0x6A:
		if (logit)
			logerror("\tALU RegC X_OFF = %8.2f", reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x6B:
		if (logit)
			logerror("\tALU RegD Y_OFF = %8.2f", reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x6c:
		if (logit)
			logerror("\tALU Inv Offset");
		break;

	case 0x6f:
		if (logit)
			logerror("\tLight Table Setup Page: %02X Mask: %02X", (m_zeusbase[offset] >> 8) & 0xff, m_zeusbase[offset] & 0xff);
		break;

	case 0x76:
		if (logit)
			logerror("\tMath Comp Reg0 XClip = %8.2f", reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x77:
		if (logit)
			logerror("\tMath Comp Reg1 YClip = %8.2f", reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x78:
		if (logit)
			logerror("\tMath Comp Reg2 ZClip = %8.2f", reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x79:
		if (logit)
			logerror("\tMath Comp Reg3 YRange = %8.2f", reinterpret_cast<float&>(m_zeusbase[offset]));
		break;

	case 0x7a:
		if (logit)
			logerror("\tMath Conditional Branch Setup");
		break;

	case 0x7c:
		if (logit)
			logerror("\tMath Compare Setup 1");
		break;

	case 0x7D:
		if (logit)
			logerror("\tMath FIFO AF / AE Input FIFO AF / AE");
		break;

	case 0x7f:
		if (logit)
			logerror("\tMath Setup Reg");
		break;

	}
	if (logit)
		logerror("\n");
}

/*************************************
*  Load pal table from waveram
*************************************/
void zeus2_device::load_pal_table(void *wavePtr, uint32_t ctrl, int type, int logit)
{
	s_pal_dirty = true;   // capture/live: pal record before the next quad
	int count = ctrl & 0xffff;
	m_palSize = (count + 1) * 4;
	uint32_t addr = (ctrl >> 24) << 1;
	uint32_t *tablePtr = &m_pal_table[addr];
	if (type == 0) {
		// Convert from RGB555
		uint16_t *src = (uint16_t*)wavePtr;
		for (int i = 0; i <= count; ++i) {
			*tablePtr++ = conv_rgb555_to_rgb32(*src++);
			*tablePtr++ = conv_rgb555_to_rgb32(*src++);
			*tablePtr++ = conv_rgb555_to_rgb32(*src++);
			*tablePtr++ = conv_rgb555_to_rgb32(*src++);
		}
	}
	else {
		// Raw Copy
		uint32_t *src = (uint32_t*)wavePtr;
		for (int i = 0; i <= count; ++i) {
			*tablePtr++ = *src++;
			*tablePtr++ = *src++;
		}
	}
	if (logit) {
		logerror("\ntable: ");
		tablePtr = &m_pal_table[addr];
		for (int i = 0; i < (count+1)*4; ++i) {
			logerror(" %08X", *tablePtr++);
			if (0 && (i + 1) % 16 == 0)
				logerror("\n");
		}
		logerror("\n");
	}
}
/*************************************
*  Load microcode from waveram
*************************************/
void zeus2_device::load_ucode(void *wavePtr, uint32_t ctrl, int logit)
{
	int count = ctrl & 0xffff;
	uint32_t addr = (ctrl >> 24) << 1;
	uint32_t *src = (uint32_t*)wavePtr;
	uint32_t *tablePtr = &m_ucode[addr];
	for (int i = 0; i <= count; ++i) {
		*tablePtr++ = *src++;
		*tablePtr++ = *src++;
	}
}

/*************************************
 *
 *  Process the FIFO
 *
 *************************************/

void zeus2_device::zeus2_pointer_write(uint8_t which, uint32_t value, int logit)
{
#if TRACK_REG_USAGE
subregwrite_count[which]++;
if (subregdata_count[which] < 256)
{
	reg_info **tailptr;

	for (tailptr = &subregdata[which]; *tailptr != nullptr; tailptr = &(*tailptr)->next)
		if ((*tailptr)->value == value)
			break;
	if (*tailptr == nullptr)
	{
		*tailptr = alloc_or_die(reg_info);
		(*tailptr)->next = nullptr;
		(*tailptr)->value = value;
		subregdata_count[which]++;
	}
}
#endif
	if (which<0x50)
		m_renderRegs[which] = value;

	switch (which)
	{
		case 0x01:
			// Limit to 12 bits
			m_renderRegs[which] &= 0xfff;
			zeus_cliprect.max_x = m_renderRegs[which];
			if (logit)
				logerror("\t(R%02X) = %4i Rend XClip", which & 0xfff, value);
			break;

		case 0x02:
			// Limit to 12 bits
			m_renderRegs[which] &= 0xfff;
			zeus_cliprect.max_y = m_renderRegs[which];
			if (logit)
				logerror("\t(R%02X) = %4i Rend YClip", which & 0xfff, value);
			break;

		case 0x03:
			if (logit)
				logerror("\t(R%02X) = %06x Rend XOffset", which, value);
			break;

		case 0x04:
			if (logit)
				logerror("\t(R%02X) = %06x Rend YOffset", which, value);
			break;

		case 0x05:
			zeus_texbase = value % (WAVERAM0_HEIGHT * WAVERAM0_WIDTH);
			if (logit)
				logerror("\t(R%02X)  texbase = %06x", which, zeus_texbase);
			break;

		case 0x07:
			if (logit)
				logerror("\t(R%02X)  Texel Mask = %06x", which, value);
			break;

		case 0x08:
			{
				//int blockNum = ((m_renderRegs[0x9] >> 16) * 1024 + (m_renderRegs[0x9] & 0xffff));
				int blockNum = m_renderRegs[0x9];
				void *dataPtr = (void *)(&m_waveram[blockNum * 2]);
				if (logit)
					logerror("\t(R%02X) = %06x PAL Control Load Table Byte Addr = %08X", which, value, blockNum * 8);
				m_curPalTableSrc = m_renderRegs[0x9];
				load_pal_table(dataPtr, m_renderRegs[0x8], 0, logit);
			}
			break;

		case 0x09:
			if (logit) logerror("\t(R%02X) = %06x PAL Addr", which, value);
			break;

		case 0x0a:
			if (logit) logerror("\t(R%02X) = %4i Pixel ALU IntA", which, value);
			break;

		case 0x0b:
			if (logit) logerror("\t(R%02X) = %4i Pixel ALU IntB (Obj Light Color)", which, value);
			break;

		case 0x0c:
			if (logit) logerror("\t(R%02X) = %4i Pixel ALU IntC (Translucency FG)", which, value);
			break;

		case 0x0d:
			if (logit) logerror("\t(R%02X) = %4i Pixel ALU IntD (Translucency BG)", which, value);
			break;

		case 0x11:
			if (logit) logerror("\t(R%02X)  Texel Setup = %06x", which, value);
			break;

		case 0x12:
			if (logit) logerror("\t(R%02X)  Pixel FIFO Setup = %06x", which, value);
			break;

		case 0x14:
			if (logit) logerror("\t(R%02X) = %06x ZBuf Control", which, value);
			break;

		case 0x15:
			//m_zbufmin = value & 0xffffff;
			if (logit) logerror("\t(R%02X) = %06X ZBuf Min", which, value);
			break;

		case 0x40:
			// 0x004000 no shading
			// 0x024004 gouraud shading
			if (logit) logerror("\t(R%02X) = %06x Pixel ALU Control", which, value);
			break;

		case 0xff:
			// Reset???
			if (logit) logerror("\tRender Reset");
			break;

		default:
			if (logit) logerror("\t(R%02X) = %06x", which, value);
			break;


#if 0
		case 0x0c:
		case 0x0d:
			// These seem to have something to do with blending.
			// There are fairly unique 0x0C,0x0D pairs for various things:
			// Car reflection on initial screen: 0x40, 0x00
			// Additively-blended "flares": 0xFA, 0xFF
			// Car windshields (and drivers, apparently): 0x82, 0x7D
			// Other minor things: 0xA4, 0x100
			break;
#endif
	}
}

/*************************************
 *  Process the FIFO
 *************************************/

// MIDZ_PCLOG=<file>: histogram of game-CPU PCs that submit FIFO commands
// (keyed by command byte). Locates the game routine that sends geometry so
// the widescreen work can trace its callers. Inert when unset.
static void midz_pclog(running_machine &machine, int cmd)
{
	static const char *path = std::getenv("MIDZ_PCLOG");
	if (!path)
		return;
	static std::map<uint64_t, uint64_t> counts;
	static uint64_t total = 0;
	auto *cpu = dynamic_cast<cpu_device *>(machine.root_device().subdevice("maincpu"));
	if (!cpu)
		return;
	counts[(uint64_t(cmd) << 32) | uint32_t(cpu->pcbase())]++;
	if ((++total & 0x3ff) == 0)
	{
		FILE *f = fopen(path, "w");
		if (f)
		{
			for (auto &kv : counts)
				fprintf(f, "%02x %05x %llu\n", int(kv.first >> 32), unsigned(kv.first & 0xffffffff), (unsigned long long)kv.second);
			fclose(f);
		}
	}
}

bool zeus2_device::zeus2_fifo_process(const uint32_t *data, int numwords)
{
	int dataoffs = 0;

	// Increment ififo counter
	m_zeusbase[0x18]++;

	/* handle logging */
	switch (data[0] >> 24)
	{
		// 0x00: write 32-bit value to low registers
	case 0x00:
		// Ignore the all zeros commmand
		if (((data[0] >> 16) & 0x7f) == 0x0) {
			if (log_fifo && (data[0] & 0xfff) != 0x2c0)
				log_fifo_command(data, numwords, " -- ignored\n");
			return true;
		}
		// Drop through to 0x05 command
		[[fallthrough]];
	/* 0x05: write 32-bit value to low registers */
	case 0x05:
		if (numwords < 2)
			return false;
		if (log_fifo)
			log_fifo_command(data, numwords, " -- reg32");
		if (((data[0] >> 16) & 0x7f) != 0x08)
			zeus2_register32_w((data[0] >> 16) & 0x7f, data[1], log_fifo);
		break;

		/* 0x08: set matrix and point (thegrid) */
		case 0x08:
			if (numwords < 14)
				return false;
			zeus_trans[3] = convert_float(data[1]);
			dataoffs = 1;
			[[fallthrough]];
		/* 0x07: set matrix and point (crusnexo) */
		case 0x07:
			if (numwords < 13)
				return false;

			/* extract the matrix from the raw data */
			zeus_matrix[0][0] = convert_float(data[dataoffs + 1]);
			zeus_matrix[0][1] = convert_float(data[dataoffs + 2]);
			zeus_matrix[0][2] = convert_float(data[dataoffs + 3]);
			zeus_matrix[1][0] = convert_float(data[dataoffs + 4]);
			zeus_matrix[1][1] = convert_float(data[dataoffs + 5]);
			zeus_matrix[1][2] = convert_float(data[dataoffs + 6]);
			zeus_matrix[2][0] = convert_float(data[dataoffs + 7]);
			zeus_matrix[2][1] = convert_float(data[dataoffs + 8]);
			zeus_matrix[2][2] = convert_float(data[dataoffs + 9]);

			/* extract the translation point from the raw data */
			zeus_trans[0] = convert_float(data[dataoffs + 10]);
			zeus_trans[1] = convert_float(data[dataoffs + 11]);
			zeus_trans[2] = convert_float(data[dataoffs + 12]);

			if (log_fifo)
			{
				log_fifo_command(data, numwords, "\n");
				logerror("\t\tmatrix ( %8.2f %8.2f %8.2f ) ( %8.2f %8.2f %8.2f ) ( %8.2f %8.2f %8.2f )\n\t\ttrans_vector %8.2f %8.2f %8.5f %8.2f\n",
						(double) zeus_matrix[0][0], (double) zeus_matrix[0][1], (double) zeus_matrix[0][2],
						(double) zeus_matrix[1][0], (double) zeus_matrix[1][1], (double) zeus_matrix[1][2],
						(double) zeus_matrix[2][0], (double) zeus_matrix[2][1], (double) zeus_matrix[2][2],
						(double) zeus_trans[0], (double) zeus_trans[1], (double) zeus_trans[2], (double)zeus_trans[3]);
			}
			break;

		/* 0x15: set point only (thegrid) */
		/* 0x16: set point only (crusnexo) */
		// 0x10: atlantis
		case 0x10:
		case 0x15:
		case 0x16:
			if (numwords < 4)
				return false;

			/* extract the translation point from the raw data */
			zeus_trans[0] = convert_float(data[1]);
			zeus_trans[1] = convert_float(data[2]);
			zeus_trans[2] = convert_float(data[3]);

			if (log_fifo)
			{
				log_fifo_command(data, numwords, "\n");
				logerror("\t\ttrans_vector %8.2f %8.2f %8.2f %8.2f\n",
					(double)zeus_trans[0], (double)zeus_trans[1], (double)zeus_trans[2], (double)zeus_trans[3]);
			}
			break;

		// 0x1c: thegrid (3 words)
		// 0x14: atlantis
		case 0x14:
		case 0x1c:
			if (m_system == THEGRID) {
				if (numwords < 3)
					return false;
				zeus_light[1] = convert_float(data[1]);
				zeus_light[2] = convert_float(data[2]);
				if (log_fifo)
				{
					log_fifo_command(data, numwords, " -- Set static fade\n");
					logerror("\t\tlight_vector %8.2f %8.2f %8.2f\n",
						(double)zeus_light[0], (double)zeus_light[1], (double)zeus_light[2]);
				}
				break;
			}
			[[fallthrough]];
		// 0x1b: thegrid
		// 0x1c: crusnexo (4 words)
		case 0x1b:
			if (numwords < 4)
				return false;
			/* extract the translation point from the raw data */
			zeus_light[0] = convert_float(data[1]);
			zeus_light[1] = convert_float(data[2]);
			zeus_light[2] = convert_float(data[3]);
			if (log_fifo)
			{
				log_fifo_command(data, numwords, " -- Set light vector\n");
				logerror("\t\tlight_vector %8.2f %8.2f %8.2f\n",
					(double)zeus_light[0], (double)zeus_light[1], (double)zeus_light[2]);

			}
			break;

		// thegrid
		case 0x1d:
			if (numwords < 2)
				return false;
			zeus_light[2] = convert_float(data[1]);
			if (log_fifo)
			{
				log_fifo_command(data, numwords, " -- Set zoffset\n");
				logerror("\t\tdata %8.5f\n", (double)convert_float(data[1]));
			}
			break;

		/* 0x23: render model in waveram (thegrid) */
		/* 0x24: render model in waveram (crusnexo) */
		// 0x17: ??? (atlantis)
		case 0x17:
		case 0x23:
		case 0x24:
			if (numwords < 2)
				return false;
			if (log_fifo)
				log_fifo_command(data, numwords, "");
			midz_pclog(machine(), data[0] >> 24);
			zeus2_draw_model(data[1], data[0] & 0xffff, log_fifo);
			break;

		// 0x2d; set direct render pixels location (atlantis)
		case 0x2d:
			if (numwords < 2)
				return false;
			if (log_fifo)
				log_fifo_command(data, numwords, "\n");
			// Need to figure how the 0x40 gets there
			m_zeusbase[0x5e] = (data[0] << 16) | 0x40;
			m_zeusbase[0x51] = data[1];
			//zeus2_draw_model(data[1], data[0] & 0xff, log_fifo);
			break;

		/* 0x31: sync pipeline? (thegrid) */
		/* 0x32: sync pipeline? (crusnexo) */
		// 0x25 ?? (atlantis)
		case 0x25:
		case 0x31:
		case 0x32:
			poly->wait("REND_WAIT");
			if (log_fifo)
				log_fifo_command(data, numwords, " wait for renderer idle \n");
			break;

		/* 0x38: direct render quad (crusnexo) */
		// 0x38: direct write to frame buffer (atlantis)
		case 0x38:
			if (m_curUCodeSrc == 0x1aa) {
				if (numwords < 3)
					return false;
				// mwskins direct write to frame buffer
				m_zeusbase[0x58] = conv_rgb555_to_rgb32((uint16_t)data[1]);
				m_zeusbase[0x59] = conv_rgb555_to_rgb32((uint16_t)(data[1] >> 16));
				frame_write();
				m_zeusbase[0x58] = conv_rgb555_to_rgb32((uint16_t)data[2]);
				m_zeusbase[0x59] = conv_rgb555_to_rgb32((uint16_t)(data[2] >> 16));
				frame_write();
				if (((m_zeusbase[0x51] & 0xff) == 2) && log_fifo)
					log_fifo_command(data, numwords, "\n");
			}
			else if (numwords < 12) {
				return false;
				//print_fifo_command(data, numwords, "\n");
				if (log_fifo)
					log_fifo_command(data, numwords, "\n");
			}
			break;

			// thegrid
		case 0xb7:
			if (numwords < 2)
				return false;
			if (log_fifo)
			{
				log_fifo_command(data, numwords, " -- Set interp factor\n");
				logerror("\t\tdata %8.5f\n", (double)convert_float(data[1]));
			}
			break;

		default:
			if (1 || data[0] != 0x2c0)
			{
				printf("Unknown command %08X\n", data[0]);
				if (log_fifo)
					log_fifo_command(data, numwords, "\n");
			}
			break;
	}
	return true;
}

/*************************************
 *  Draw a model in waveram
 *************************************/

void zeus2_device::zeus2_draw_model(uint32_t baseaddr, uint16_t count, int logit)
{
	midz_model_rec captured;
	std::vector<uint32_t> captured_words;
	const bool capture_model = s_cap_models && midz_cap;
	if (capture_model)
	{
		if (++s_cap_model_count > 4096 || count > 0xc800)
			fatalerror("Zeus model journal exceeds bounded record count");
		captured = {};
		captured.version = m_upstream_render ? 2 : 1;
		captured.reserved = m_upstream_render;
		captured.frame = uint32_t(screen().frame_number());
		captured.id = s_cap_model_count; captured.baseaddr = baseaddr; captured.count = count;
		captured.quad_size = zeus_quad_size; captured.system = m_system;
		captured.first_quad = s_cap_recq; captured.ucode = m_curUCodeSrc;
		captured.palette = m_curPalTableSrc; captured.texture = zeus_texbase;
		captured.yscale = m_yScale; captured.zoffset = m_useZOffset;
		captured.raw_words = baseaddr ? 2 * (uint32_t(count) + 1) : 0;
		captured.time = machine().time().as_double();
		memcpy(captured.matrix, zeus_matrix, sizeof(captured.matrix));
		memcpy(captured.translation, zeus_trans, sizeof(captured.translation));
		memcpy(captured.light, zeus_light, sizeof(captured.light));
		for (unsigned i=0;i<0x80;++i) captured.regs[i] = m_zeusbase[i];
		for (unsigned i=0;i<0x50;++i) captured.render[i] = m_renderRegs[i];
		const uint32_t block = (baseaddr % WAVERAM0_WIDTH) + ((baseaddr >> 16) % WAVERAM0_HEIGHT) * WAVERAM0_WIDTH;
		if (uint64_t(block) * 2 + captured.raw_words > uint64_t(WAVERAM0_WIDTH) * WAVERAM0_HEIGHT * 2)
			fatalerror("Zeus model journal source outside WaveRAM");
		const uint32_t bytes = 8 + sizeof(captured) + 4 * captured.raw_words;
		if (uint64_t(s_cap_model_bytes) + bytes > 64 * 1024 * 1024)
			fatalerror("Zeus model journal exceeds bounded byte count");
		s_cap_model_bytes += bytes;
		const uint32_t *source = static_cast<const uint32_t *>(waveram0_ptr_from_expanded_addr(baseaddr));
		captured_words.assign(source, source + captured.raw_words);
	}
	uint32_t databuffer[512];
	int databufcount = 0;
	int model_done = false;
	uint32_t texdata = 0;

	if (logit)
		logerror(" -- model @ %08X, len %04X, palSrc %08x, rendSrc %08x\n", baseaddr, count, m_curPalTableSrc, m_curUCodeSrc);

	if (count > 0xc800)
		fatalerror("Extreme count\n");

	while (baseaddr != 0 && !model_done)
	{
		const void *base = waveram0_ptr_from_expanded_addr(baseaddr);
		int curoffs;

		/* reset the objdata address */
		baseaddr = 0;

		/* loop until we run out of data */
		for (curoffs = 0; curoffs <= count; curoffs++)
		{
			int countneeded = 2;
			uint8_t cmd;

			/* accumulate 2 words of data */
			databuffer[databufcount++] = WAVERAM_READ32(base, curoffs * 2 + 0);
			databuffer[databufcount++] = WAVERAM_READ32(base, curoffs * 2 + 1);

			/* if this is enough, process the command */
			cmd = databuffer[0] >> 24;

			if ((cmd == 0x38) || (cmd == 0x2d) || (cmd == 0xa7) || (cmd == 0xaf)) {
				countneeded = zeus_quad_size;
			}
			if (databufcount == countneeded)
			{
				// Increment mfifo counter
				m_zeusbase[0x19] += databufcount;

				/* handle logging of the command */
				if (logit)
				{
					if (cmd != 0x00 || (cmd == 0x00 && curoffs == count)) {
						logerror("\t");
						// Limit logging to 16 words
						for (int offs = 0; offs < databufcount && offs < 16; offs++)
							logerror("%08X ", databuffer[offs]);
						logerror("-- ");
					}
				}

				/* handle the command */
				switch (cmd)
				{
					case 0x00: // crusnexo
						if (logit && curoffs == count)
							logerror(" end cmd 00\n");
						[[fallthrough]];
					case 0x21:  /* thegrid */
					case 0x22:  /* crusnexo */
						// Sets 0x68 (uv float offset) and texture line and mode
						// In reality this sets internal registers that are used in the
						// zeus2 microcode to set these registers
						m_zeusbase[0x68] = (databuffer[0] >> 16) & 0xff;
						texdata = databuffer[1];
						if (logit)
							logerror(" (0x68)=%02X texMode=%08X\n", m_zeusbase[0x68], texdata);
						break;

					case 0x31:  /* thegrid */
						poly->wait("REND_WAIT");
						if (logit)
							logerror("wait for renderer not active\n");
						break;

					case 0x29:  // atlantis
					case 0x35:  /* thegrid */
					case 0x36:  /* crusnexo */
						if (logit)
							logerror("reg32");
						zeus2_register32_w((databuffer[0] >> 16) & 0x7f, databuffer[1], logit);
						break;

					case 0x2d:  // atlantis
						poly->zeus2_draw_quad(databuffer, texdata, logit);
						break;

					case 0x38:  /* crusnexo/thegrid */
						if (m_system==THEGRID && m_curUCodeSrc==0x00000343) {
							if (logit)
								logerror("direct write [57]=%08X [51]==%08X\n", m_zeusbase[0x57], m_zeusbase[0x51]);
							// Direct write to frame buffer
							for (int subIndex = 0; subIndex < zeus_quad_size / 2; ++subIndex) {
								m_zeusbase[0x5a] = databuffer[subIndex * 2 + 1] & 0xffffff;
								m_zeusbase[0x58] = databuffer[subIndex * 2 + 0] & 0xffffff;
								frame_write();
							}
						}
						else {
							poly->zeus2_draw_quad(databuffer, texdata, logit);
						}
						break;

					// thegrid: triangle mesh, pm3dli
					case 0xa7:
					case 0xaf:
						if (1 || logit)
							logerror(" unknown triangle data\n");
						break;

					default:
						if (logit)
							logerror("unknown model data\n");
						break;
				}

				/* reset the count */
				databufcount = 0;
			}
		}
		// Log unused data
		if (databufcount != 0) {
			if (logit)
			{
				logerror("\t");
				for (int offs = 0; offs < databufcount; offs++)
					logerror("%08X ", databuffer[offs]);
				logerror("-- Unused data\n");
			}
		}
	}
	if (capture_model)
	{
		captured.last_quad = s_cap_recq;
		const uint32_t prefix[2] = {0x31534d5a, uint32_t(sizeof(captured) + 4 * captured_words.size())};
		if (fwrite(prefix, sizeof(prefix), 1, s_cap_models) != 1 ||
			fwrite(&captured, sizeof(captured), 1, s_cap_models) != 1 ||
			(!captured_words.empty() && fwrite(captured_words.data(), 4, captured_words.size(), s_cap_models) != captured_words.size()))
			fatalerror("Cannot write Zeus model journal");
	}
}

/*************************************
 *  Draw a quad
 *************************************/
void zeus2_renderer::zeus2_draw_quad(const uint32_t *databuffer, uint32_t texdata, int logit)
{
	z2_poly_vertex vert[4];

	if (s_mz_stats < 0)
		s_mz_stats = std::getenv("MIDZ_STATS") ? 1 : 0;
	if (s_mz_stats)
		++s_mz_quads;
	if (s_cap_state >= 0)
		++s_cap_fq;   // per-frame count for the min-quads capture trigger

	if (logit) {
		m_state->logerror("quad %d", m_state->zeus_quad_size);
#if PRINT_TEX_INFO
		m_state->logerror(" %s\n", m_state->tex_info());
#else
		m_state->logerror("\n");
#endif
	}
	//if (machine().input().code_pressed(KEYCODE_Q) && (m_state->m_renderRegs[0x5] != 0x1fdf00)) return;
	//if (machine().input().code_pressed(KEYCODE_E) && (m_state->m_renderRegs[0x5] != 0x07f540)) return;
	//if (machine().input().code_pressed(KEYCODE_R) && (m_state->m_renderRegs[0x5] != 0x081580)) return;
	//if (machine().input().code_pressed(KEYCODE_T) && (m_state->m_renderRegs[0x5] != 0x14db00)) return;
	//if (machine().input().code_pressed(KEYCODE_Y) && (m_state->m_renderRegs[0x5] != 0x14d880)) return;
	//if (machine().input().code_pressed(KEYCODE_Q) && (texdata & 0xffff) == 0x119) return;
	//if (machine().input().code_pressed(KEYCODE_E) && (texdata & 0xffff) == 0x01d) return;
	//if (machine().input().code_pressed(KEYCODE_R) && (texdata & 0xffff) == 0x11d) return;
	//if (machine().input().code_pressed(KEYCODE_T) && (texdata & 0xffff) == 0x05d) return;
	//if (machine().input().code_pressed(KEYCODE_Y) && (texdata & 0xffff) == 0x0dd) return;
	//if (machine().input().code_pressed(KEYCODE_U) && (texdata & 0xffff) == 0x119) return;
	//if (machine().input().code_pressed(KEYCODE_I) && (texdata & 0xffff) == 0x119) return;
	//if (machine().input().code_pressed(KEYCODE_O) && (texdata & 0xffff) == 0x119) return;
	//if (machine().input().code_pressed(KEYCODE_L) && (texdata & 0x100)) return;
	//if (m_state->m_texmodeReg != 0x9b) return;

	// PZr = (PZ * M22) + (PY * M21) + (PX * M20) + 0
	// AZp = (AZ * M22) + (AY * M21) + (AX * M20) + TZ
	// AYp = (AZ * M12) + (AY * M11) + (AX * M10) + TY
	// AXp = (AZ * M02) + (AY * M01) + (AX * M00) + TX

	/* extract raw x,y,z */
	if (m_state->m_atlantis) {
			// Atlantis quad 14
		texdata = databuffer[1];
		vert[0].x = (int16_t)databuffer[2];
		vert[0].y = (int16_t)databuffer[3];
		vert[0].p[0] = (int16_t)databuffer[4];
		vert[0].p[1] = ((databuffer[5] >> 0) & 0xff);
		vert[0].p[2] = ((databuffer[5] >> 8) & 0xff);

		vert[1].x = (int16_t)(databuffer[2] >> 16);
		vert[1].y = (int16_t)(databuffer[3] >> 16);
		vert[1].p[0] = (int16_t)(databuffer[4] >> 16);
		vert[1].p[1] = ((databuffer[5] >> 16) & 0xff);
		vert[1].p[2] = ((databuffer[5] >> 24) & 0xff);

		vert[2].x = (int16_t)databuffer[6];
		vert[2].y = (int16_t)databuffer[7];
		vert[2].p[0] = (int16_t)databuffer[8];
		vert[2].p[1] = ((databuffer[9] >> 0) & 0xff);
		vert[2].p[2] = ((databuffer[9] >> 8) & 0xff);

		vert[3].x = (int16_t)(databuffer[6] >> 16);
		vert[3].y = (int16_t)(databuffer[7] >> 16);
		vert[3].p[0] = (int16_t)(databuffer[8] >> 16);
		vert[3].p[1] = ((databuffer[9] >> 16) & 0xff);
		vert[3].p[2] = ((databuffer[9] >> 24) & 0xff);
	}
	else {
		//printf("R40: %06X\n", m_state->m_renderRegs[0x40]);
		vert[0].x = (int16_t)databuffer[2];
		vert[0].y = (int16_t)databuffer[3];
		vert[0].p[0] = (int16_t)databuffer[6];
		vert[0].p[1] = (databuffer[1] >> 0) & 0x3ff;
		vert[0].p[2] = (databuffer[1] >> 16) & 0x3ff;

		vert[1].x = (int16_t)(databuffer[2] >> 16);
		vert[1].y = (int16_t)(databuffer[3] >> 16);
		vert[1].p[0] = (int16_t)(databuffer[6] >> 16);
		vert[1].p[1] = (databuffer[4] >> 0) & 0x3ff;
		vert[1].p[2] = (databuffer[4] >> 10) & 0x3ff;

		vert[2].x = (int16_t)databuffer[8];
		vert[2].y = (int16_t)databuffer[9];
		vert[2].p[0] = (int16_t)databuffer[7];
		vert[2].p[1] = (databuffer[4] >> 20) & 0x3ff;
		vert[2].p[2] = (databuffer[5] >> 0) & 0x3ff;

		vert[3].x = (int16_t)(databuffer[8] >> 16);
		vert[3].y = (int16_t)(databuffer[9] >> 16);
		vert[3].p[0] = (int16_t)(databuffer[7] >> 16);
		vert[3].p[1] = (databuffer[5] >> 10) & 0x3ff;
		vert[3].p[2] = (databuffer[5] >> 20) & 0x3ff;
	}
	int unknown[8];
	float unknownFloat[4];
	if (m_state->zeus_quad_size == 14) {
		// buffer 10-13 ???? 00000000 1FF7FC00 00000000 1FF7FC00 -- mwskinsa quad 14
		/* 10:13 16 bit coordinates */
		unknown[0] = (int16_t)databuffer[10];
		unknown[1] = (int16_t)(databuffer[10] >> 16);
		unknown[2] = (int16_t)databuffer[11];
		unknown[3] = (int16_t)(databuffer[11] >> 16);
		unknown[4] = (int16_t)databuffer[12];
		unknown[5] = (int16_t)(databuffer[12] >> 16);
		unknown[6] = (int16_t)databuffer[13];
		unknown[7] = (int16_t)(databuffer[13] >> 16);
		unknownFloat[0] = m_state->convert_float(databuffer[10]);
		unknownFloat[1] = m_state->convert_float(databuffer[11]);
		unknownFloat[2] = m_state->convert_float(databuffer[12]);
		unknownFloat[3] = m_state->convert_float(databuffer[13]);
	}

	int logextra = 0;

	int intScale = m_state->m_zeusbase[0x66] - 0x8e;
	float fScale = pow(2.0f, intScale);
	int intUVScale = m_state->m_zeusbase[0x68] - 0x9d;
	float uvScale = pow(2.0f, intUVScale);
	for (int i = 0; i < 4; i++)
	{
		float x = vert[i].x;
		float y = vert[i].y;
		float z = vert[i].p[0];
		if (1) {
		  x *= fScale;
		  y *= fScale;
		  z *= fScale;
		}
#if PRINT_TEX_INFO
		if (logit && i == 0) {
			m_state->check_tex(texdata, z, m_state->zeus_matrix[2][2], m_state->zeus_trans[2]);
		}
#endif
		vert[i].x =    x * m_state->zeus_matrix[0][0] + y * m_state->zeus_matrix[0][1] + z * m_state->zeus_matrix[0][2];
		vert[i].y =    x * m_state->zeus_matrix[1][0] + y * m_state->zeus_matrix[1][1] + z * m_state->zeus_matrix[1][2];
		vert[i].p[0] = x * m_state->zeus_matrix[2][0] + y * m_state->zeus_matrix[2][1] + z * m_state->zeus_matrix[2][2];

		vert[i].x += m_state->zeus_trans[0];
		vert[i].y += m_state->zeus_trans[1];
		vert[i].p[0] += m_state->zeus_trans[2];

		//vert[i].p[1] += ((texdata >> 8) & 0x1) ? 1.0f : 0.0f;
		vert[i].p[1] *= uvScale;
		vert[i].p[2] *= uvScale;
		vert[i].p[2] += (texdata >> 16);
		vert[i].p[1] *= 256.0f;
		vert[i].p[2] *= 256.0f;
		vert[i].p[3] = 0.0f;



		if (logextra & logit)
		{
			m_state->logerror("\t\t(%f,%f,%f) (%02X,%02X)\n",
				(double)vert[i].x, (double)vert[i].y, (double)vert[i].p[0],
				(int)(vert[i].p[1] / 256.0f), (int)(vert[i].p[2] / 256.0f));
		}
	}
	if (0 && logextra & logit && m_state->zeus_quad_size == 14) {
		m_state->logerror("unknown: int16: %d %d %d %d %d %d %d %d float: %f %f %f %f\n",
			unknown[0], unknown[1], unknown[2], unknown[3], unknown[4], unknown[5], unknown[6], unknown[7],
			unknownFloat[0], unknownFloat[1], unknownFloat[2], unknownFloat[3]);
	}

	// Near-plane clip straddling quads instead of rejecting them (which dropped the nearest
	// crusnexo road segment), matching the Zeus 1 renderer.
	float clipVal = reinterpret_cast<float&>(m_state->m_zeusbase[0x78]);
	z2_poly_vertex clipvert[8];
	int numverts = zclip_if_less<4>(4, vert, clipvert, clipVal);
	if (numverts < 3)
		return;

	float xOrigin = reinterpret_cast<float&>(m_state->m_zeusbase[0x6a]);
	float yOrigin = reinterpret_cast<float&>(m_state->m_zeusbase[0x6b]);

	// crusn seems to use a different z scale
	float zRound = (m_state->m_system == m_state->CRUSNEXO) ? 2.0f : 0.5f;

	float oozBase = 1 << m_state->m_zeusbase[0x6c];
	for (int i = 0; i < numverts; i++)
	{
		// Clamp to zero if negative
		if (clipvert[i].p[0] < 0)
			clipvert[i].p[0] = 0.0f;

		float ooz = oozBase / (clipvert[i].p[0] + zRound);

		clipvert[i].x *= ooz;
		clipvert[i].y *= ooz;
		// Perspective-correct texturing: carry u/z, v/z and 1/z for the per-pixel divide.
		clipvert[i].p[1] *= ooz;
		clipvert[i].p[2] *= ooz;
		clipvert[i].p[3] = ooz;

		clipvert[i].x += xOrigin;
		clipvert[i].y += yOrigin;
		// The Grid adds zoffset for objects with no light
		if (m_state->m_useZOffset)
			clipvert[i].p[0] += m_state->zeus_light[2];

		clipvert[i].p[0] *= 4096.0f;  // 12.12

		if (logextra & logit)
			m_state->logerror("\t\t\tTranslated=(%f,%f, %f) scale = %f\n", (double)clipvert[i].x, (double)clipvert[i].y, (double)clipvert[i].p[0], ooz);
	}
	// Slow HSR
	// ((AYs - BYs) * (BXs - CXs)) - ((AXs - BXs) * (BYs - CYs))
	if (1) {
		float slowHSR = ((clipvert[0].y - clipvert[1].y) * (clipvert[1].x - clipvert[2].x) - (clipvert[0].x - clipvert[1].x) * (clipvert[1].y - clipvert[2].y));
		if (slowHSR >= 0)
			return;
	}

	// live overlay: our GL renderer draws every quad itself, so MAME's CPU
	// rasterization would render the whole game a second time into a
	// framebuffer nobody displays. When live, fill a LOCAL raster-state
	// struct for the capture (never touching poly_manager's object ring)
	// and skip render_triangle_fan below - recovers the emu-thread margin
	// that pushed crusnexo under 100% beside ambient load. The offline
	// capture path (midz_cap without live) still rasterizes: the oracle
	// diffs against the CPU framebuffer.
	zeus2_poly_extra_data local_extra;
	static bool const native_diagnostic = std::getenv("MIDZ_GL_NATIVE") && atoi(std::getenv("MIDZ_GL_NATIVE")) == 1;
	bool const gl_only = m_state->midz_live && !native_diagnostic;
	zeus2_poly_extra_data& extra = gl_only
		? local_extra : this->object_data().next();

	extra.ucode_src = m_state->m_curUCodeSrc;
	extra.tex_src = m_state->zeus_texbase;
	int texmode = texdata & 0xffff;
	extra.texwidth = 0x20 << ((texmode >> 2) & 3);
	extra.solidcolor = m_state->m_zeusbase[0x00] & 0x7fff;
	// Flat solid-color fill: texmode bits 10-11 both set (same as Zeus 1)
	extra.solid_enable = ((texmode & 0x0c00) == 0x0c00);
	extra.transcolor = (texmode & 0x180) ? 0 : 0x100;
	extra.texbase = WAVERAM_BLOCK0_EXT(m_state->zeus_texbase);
	extra.depth_min_enable = true;// (m_state->m_renderRegs[0x14] & 0x008000);
	extra.depth_floor_enable = (m_state->m_upstream_render & cruisn::zeus_policy::DepthFloor) != 0;
	extra.zbuf_min = util::sext(m_state->m_renderRegs[0x15], 24);
	const auto policy = cruisn::zeus_policy::material(m_state->m_upstream_render,texmode,
		m_state->m_renderRegs[0x14],m_state->m_renderRegs[0x40],m_state->m_renderRegs[0x0c]);
	extra.depth_test_enable = policy.depth_test;
	//extra.depth_test_enable &= !(m_state->m_renderRegs[0x14] & 0x008000);
	extra.depth_write_enable = policy.depth_write;
	extra.depth_clear_enable = (m_state->m_renderRegs[0x14] & 0x000c00);
	// 021e0e = blend with texture alpha for type 2, 020202 blend src / dst alpha
	extra.blend_enable = policy.blend;
	// Clamp translucency (1.8 fixed, 0x100=1.0) to 0x100: scale8() takes a uint8_t, so >=0x100 would truncate to near-black.
	extra.srcAlpha = policy.source_alpha;
	extra.dstAlpha = std::min<uint32_t>(m_state->m_renderRegs[0x0d], 0x100);
	extra.texture_alpha = false;
	extra.texture_rgb555 = false;
	switch (texmode & 0x3) {
	case 0:
		extra.get_texel = m_state->get_texel_4bit_2x2;
		extra.texwidth >>= 1;
		break;
	case 1:
		extra.get_texel = m_state->get_texel_8bit_4x2;
		break;
	case 2:
		// Seems to select texture with embedded alpha
		if (texmode & 0x80) {
			// Texel , Alpha
			extra.get_texel = m_state->get_texel_8bit_2x2_alpha;
			extra.texture_alpha = true;
			extra.get_alpha = m_state->get_alpha_8bit_2x2_alpha;
		}
		else {
			extra.texture_rgb555 = true;
		}
		break;
	default:
		m_state->logerror("unknown texel type");
		extra.get_texel = m_state->get_texel_8bit_2x2;
		break;
	}

	m_state->midz_cap_quad(numverts, clipvert, extra, texdata);
	if (gl_only)
		return;   // see local_extra above - GL overlay renders this quad
	render_triangle_fan<4>(m_state->zeus_cliprect, render_delegate(&zeus2_renderer::render_poly_8bit, this), numverts, clipvert);
}



/*************************************
*  Rasterizers
*************************************/

// Blend srcColor into a frame buffer pixel and update depth; shared by the solid-fill and textured paths.
static inline void zeus2_write_pixel(uint32_t &colorpix, int32_t &depthpix, rgb_t srcColor,
	bool blend_enable, int32_t srcAlpha, int32_t dstAlpha, bool depth_write_enable, int32_t depthVal)
{
	if (blend_enable) {
		// If src alpha is 0 don't write
		if (srcAlpha == 0x00)
			return;
		rgb_t dstColor = colorpix;
		if (srcAlpha != 0x100)
			srcColor.scale8(srcAlpha);
		if (dstAlpha == 0x100)
			srcColor += dstColor;
		else
			srcColor += dstColor.scale8(dstAlpha);
	}
	colorpix = srcColor;
	if (depth_write_enable)
		depthpix = depthVal; // Should limit to 24 bits
}

void zeus2_renderer::render_poly_8bit(int32_t scanline, const extent_t& extent, const zeus2_poly_extra_data& object, int threadid)
{
	int32_t curz = extent.param[0].start;
	// Perspective-correct texturing: params 1/2 hold u/z and v/z, param 3 holds 1/z.
	float curupz = extent.param[1].start;
	float curvpz = extent.param[2].start;
	float curooz = extent.param[3].start;
	int32_t dzdx = extent.param[0].dpdx;
	float dupzdx = extent.param[1].dpdx;
	float dvpzdx = extent.param[2].dpdx;
	float doozdx = extent.param[3].dpdx;
	const void *texbase = object.texbase;
	//const void *palbase = object.palbase;
	uint16_t transcolor = object.transcolor;
	int32_t srcAlpha = object.srcAlpha;
	int32_t dstAlpha = object.dstAlpha;
	bool depth_write_enable = object.depth_write_enable;
	int texwidth = object.texwidth;
	// RGB555 solidcolor expanded to RGB32
	uint32_t solidColor = ((object.solidcolor & 0x7c00) << 9) | ((object.solidcolor & 0x3e0) << 6) | ((object.solidcolor & 0x1f) << 3);
	int x;

	uint32_t addr = m_state->frame_addr_from_xy(0, scanline, true);
	int32_t *depthptr = &m_state->m_frameDepth[addr];
	uint32_t *colorptr = &m_state->m_frameColor[addr];
	int32_t curDepthVal;

	for (x = extent.startx; x < extent.stopx; x++)
	{
		if (object.depth_clear_enable) {
			//curDepthVal = object.zbuf_min;
			curDepthVal = 0xffffff;
		} else if (object.depth_min_enable) {
			curDepthVal = cruisn::zeus_policy::depth(object.depth_floor_enable ? cruisn::zeus_policy::DepthFloor : 0,
				curz,object.zbuf_min);
		}
		else {
			curDepthVal = curz;
		}
		//if (curz < object.zbuf_min)
		//  curDepthVal = object.zbuf_min;
		//else
		//  curDepthVal = curz;
		if (curDepthVal < 0)
			curDepthVal = 0;
		bool depth_pass = true;
		if (object.depth_test_enable) {
			if (curDepthVal > depthptr[x])
				depth_pass = false;
		}
		if (depth_pass) {
			// Perspective divide for texel coords; clamp guards the clipped near edge.
			float oozInv = 1.0f / curooz;
			int32_t curu = (int32_t)(curupz * oozInv);
			int32_t curv = (int32_t)(curvpz * oozInv);
			int u0 = (curu >> 8);
			int v0 = (curv >> 8);
			if (u0 < 0) u0 = 0;
			if (v0 < 0) v0 = 0;
			int u1 = (u0 + 1);
			int v1 = (v0 + 1);
			if (object.solid_enable) {
				zeus2_write_pixel(colorptr[x], depthptr[x], solidColor, object.blend_enable,
					srcAlpha, dstAlpha, depth_write_enable, curDepthVal);
			}
			else if (object.texture_rgb555) {
				// Rendering for textures with direct color
				rgb_t srcColor = m_state->get_rgb555(texbase, v0, u0, texwidth);
				colorptr[x] = srcColor;
			}
			else if (object.texture_alpha) {
				// Rendering for textures with embedded alpha
				// To bilinear filter or not to bilinear filter
				if (0) {
					// Add rounding
					u0 += (curu >> 7) & 1;
					v0 += (curv >> 7) & 1;
					uint8_t texel0 = object.get_texel(texbase, v0, u0, texwidth);
					srcAlpha = object.get_alpha(texbase, v0, u0, texwidth);
					if (srcAlpha != 0) {
						rgb_t srcColor = m_state->m_pal_table[texel0];
						rgb_t dstColor = colorptr[x];
						dstAlpha = 0xff - srcAlpha;
						srcColor.scale8(srcAlpha);
						srcColor += dstColor.scale8(dstAlpha);
						colorptr[x] = srcColor;
						if (depth_write_enable)
							depthptr[x] = curDepthVal; // Should limit to 24 bits
					}
				}
				else {
					uint8_t texel0 = object.get_texel(texbase, v0, u0, texwidth);
					uint8_t texel1 = object.get_texel(texbase, v0, u1, texwidth);
					uint8_t texel2 = object.get_texel(texbase, v1, u0, texwidth);
					uint8_t texel3 = object.get_texel(texbase, v1, u1, texwidth);
					uint8_t alpha0 = object.get_alpha(texbase, v0, u0, texwidth);
					uint8_t alpha1 = object.get_alpha(texbase, v0, u1, texwidth);
					uint8_t alpha2 = object.get_alpha(texbase, v1, u0, texwidth);
					uint8_t alpha3 = object.get_alpha(texbase, v1, u1, texwidth);
					if (1)
					{
						// Calculate source alpha
						//srcAlpha = ((uint32_t)alpha0 + (uint32_t)alpha1 + (uint32_t)alpha2 + (uint32_t)alpha3) >> 2;
						uint32_t uFactor = curu & 0xff;
						uint32_t vFactor = curv & 0xff;
						srcAlpha = ((alpha0 * (256 - uFactor) + alpha1 * (uFactor)) * (256 - vFactor) + (alpha2 * (256 - uFactor) + alpha3 * (uFactor)) * (vFactor)) >> 16;
						if (srcAlpha != 0) {
							uint32_t color0 = m_state->m_pal_table[texel0];
							uint32_t color1 = m_state->m_pal_table[texel1];
							uint32_t color2 = m_state->m_pal_table[texel2];
							uint32_t color3 = m_state->m_pal_table[texel3];
							rgb_t filtered = rgbaint_t::bilinear_filter(color0, color1, color2, color3, curu, curv);
							rgb_t dstColor = colorptr[x];
							dstAlpha = 0x100 - srcAlpha;
							filtered.scale8(srcAlpha);
							filtered += dstColor.scale8(dstAlpha);
							colorptr[x] = filtered;
							if (depth_write_enable)
								depthptr[x] = curDepthVal; // Should limit to 24 bits
						}
					}
				}
			// Rendering for textures with no transparent color
			} else if (1 || transcolor == 0x100) {
				uint8_t texel0 = object.get_texel(texbase, v0, u0, texwidth);
				uint8_t texel1 = object.get_texel(texbase, v0, u1, texwidth);
				uint8_t texel2 = object.get_texel(texbase, v1, u0, texwidth);
				uint8_t texel3 = object.get_texel(texbase, v1, u1, texwidth);
				if ((texel0 != transcolor) && (texel1 != transcolor) && (texel2 != transcolor) && (texel3 != transcolor))
				//if (1)
				{
					uint32_t color0 = m_state->m_pal_table[texel0];
					uint32_t color1 = m_state->m_pal_table[texel1];
					uint32_t color2 = m_state->m_pal_table[texel2];
					uint32_t color3 = m_state->m_pal_table[texel3];
					rgb_t srcColor = rgbaint_t::bilinear_filter(color0, color1, color2, color3, curu, curv);
					zeus2_write_pixel(colorptr[x], depthptr[x], srcColor, object.blend_enable,
						srcAlpha, dstAlpha, depth_write_enable, curDepthVal);
				}
			// Rendering for textures with transparent color
			//} else {
			//  // Add rounding
			//  u0 += (curu >> 7) & 1;
			//  v0 += (curv >> 7) & 1;
			//  uint8_t texel0 = object.get_texel(texbase, v0, u0, texwidth);
			//  if (texel0 != transcolor) {
			//      uint32_t color0 = m_state->m_pal_table[texel0];
			//      colorptr[x] = color0;
			//      if (object.depth_write_enable)
			//          depthptr[x] = curz; // Should limit to 24 bits
			//  }
			}
		}
		curz += dzdx;
		curupz += dupzdx;
		curvpz += dvpzdx;
		curooz += doozdx;
	}
}

/*************************************
 *  Debugging tools
 *************************************/

void zeus2_device::log_fifo_command(const uint32_t *data, int numwords, const char *suffix)
{
	int wordnum;
	logerror("Zeus cmd %02X :", data[0] >> 24);
	for (wordnum = 0; wordnum < numwords; wordnum++)
		logerror(" %08X", data[wordnum]);
	logerror("%s", suffix);
}

void zeus2_device::print_fifo_command(const uint32_t *data, int numwords, const char *suffix)
{
	int wordnum;
	printf("Zeus cmd %02X :", data[0] >> 24);
	for (wordnum = 0; wordnum < numwords; wordnum++)
		printf(" %08X", data[wordnum]);
	printf("%s", suffix);
}

void zeus2_device::log_render_info(uint32_t texdata)
{
	logerror("-- RMode0 R40 = %08X texdata = %08X", m_renderRegs[0x40], texdata);
	logerror("\n-- RMode1 ");
	for (int i = 1; i <= 0x9; ++i)
		logerror(" R%02X=%06X", i, m_renderRegs[i]);
	for (int i = 0xa; i <= 0x15; ++i)
		logerror(" R%02X=%06X", i, m_renderRegs[i]);
	logerror("\n-- RMode2 ");
	for (int i = 0x63; i <= 0x6f; ++i)
		logerror(" %02X=%08X", i, m_zeusbase[i]);
	logerror("\n");
}

#if PRINT_TEX_INFO
#include <iomanip>

void zeus2_device::check_tex(uint32_t &texmode, float &zObj, float &zMat, float &zOff)
{
	if (tex_map.count(zeus_texbase) == 0) {
		std::string infoStr;
		std::stringstream infoStream;
		infoStream << "tex=0x" << std::setw(8) << std::setfill('0') << std::hex << zeus_texbase << " ";
		//infoStream << "pal=0x" << std::setw(4) << std::setfill('0') << (m_curPalTableSrc >> 16) << ", 0x" << std::setw(4) << (m_curPalTableSrc & 0xffff) << " ";
		infoStream << "pal=0x" << std::setw(8) << std::setfill('0') << m_curPalTableSrc << " ";
		infoStream << "texdata=" << std::setw(8) << std::hex << texmode << " ";
		infoStream << "68(uvFloat)=" << std::setw(2) << std::hex << m_zeusbase[0x68] << " ";
		infoStream << "(6c)=" << m_zeusbase[0x6c] << " ";
		infoStream << "(63)=" << std::setw(6) << std::dec << reinterpret_cast<float&>(m_zeusbase[0x63]) << " ";
		//infoStream << "zObj=" << std::setw(6) << std::dec << zObj << " ";
		//infoStream << "zMat=" << std::setw(6) << std::dec << zMat << " ";
		infoStream << "zOff=" << std::setw(6) << std::dec << zOff << " ";
		infoStream << "R40=" << std::setw(6) << std::hex << m_renderRegs[0x40] << " ";
		infoStream << "R14=" << m_renderRegs[0x14] << " ";
		infoStream << "R15=" << m_renderRegs[0x15] << " ";
		//infoStream << "R0A=" << std::setw(2) << m_renderRegs[0x0a] << " ";
		infoStream << "R0B(LC)=" << std::setw(6) << m_renderRegs[0x0b] << " ";
		infoStream << "R0C(FGD)=" << std::setw(3) << m_renderRegs[0x0c] << " ";
		infoStream << "R0D(BGD)=" << std::setw(3) << m_renderRegs[0x0d] << " ";
		infoStr += infoStream.str();
		infoStr += tex_info();

		tex_map.insert(std::pair<uint32_t, std::string>(zeus_texbase, infoStr));
		osd_printf_info("%s\n", infoStr);
	}
}

std::string zeus2_device::tex_info(void)
{
	std::string retVal;
	if (m_system == CRUSNEXO) {
		switch (zeus_texbase) {
		// crusnexo
		case 0x01fc00:      retVal = "credits / insert coin"; break;
		case 0x1dc000:      retVal = "copywrite text"; break;
		case 0x0e7400:      retVal = "tire"; break;
		case 0x0e8800:      retVal = "star behind tire"; break;
		case 0x0e6800:      retVal = "crusn exotica text"; break;
		case 0x02a400:      retVal = "Yellow Letters / Numbers"; break;
		case 0x1fd000:      retVal = "Star burst in license plate screen"; break;
		case 0x1e9800:      retVal = "Red Letter in license plate screen"; break;
		case 0x0c1c00:      retVal = "Car parts"; break;
		case 0x0006f000:    retVal = "license plate background"; break;
		case 0x0006f400:    retVal = "blue on white license plate names"; break;
		case 0x00047800:    retVal = "baby body"; break;
		case 0x000e7000:    retVal = "crusn exotica yellow glow"; break;
		case 0x0002b000:    retVal = "blue crusn stencil behind leader list"; break;
		case 0x001f4800:    retVal = "number keypad"; break;
		case 0x001e7800:    retVal = "register now logo"; break;
		case 0x001e5000:    retVal = "blue start game / enter code / earn miles"; break;
		case 0x0001c800:    retVal = "black letters silver back track select / crusn"; break;
		case 0x001df400:    retVal = "first place / free race logo"; break;
		case 0x001ddc00:    retVal = "secret car logo"; break;
		case 0x0006e800:    retVal = "???"; break;
		case 0x001f1c00:    retVal = "black 0-9 on silver background"; break;
		case 0x001ec800:    retVal = "black on silver holland/amazon/sahara"; break;
		case 0x001f7800:    retVal = "license plate background white"; break;
		case 0x001f7000:    retVal = "red Hot Times writing"; break;
		case 0x001eb000:    retVal = "black numbers 0-10 on silver background"; break;
		case 0x00100800:    retVal = "sunset and stars sky background"; break;
		case 0x00108c00:    retVal = "asphalt surface"; break;
		case 0x0010c000:    retVal = "wood surface?"; break;
		case 0x0010e400:    retVal = "palm tree"; break;
		case 0x00118c00:    retVal = "highway green signs"; break;
		case 0x000f6400:    retVal = "glowing feather?"; break;
		case 0x00112c00:    retVal = "fancy street lamps"; break;
		case 0x00032400:    retVal = "lady driver body"; break;
		case 0x000b5400:    retVal = "blue firebird car"; break;
		case 0x00089c00:    retVal = "brown hummer car"; break;
		case 0x00110c00:    retVal = "oak tree"; break;
		case 0x00115400:    retVal = "welcome to las vegas sign"; break;
		case 0x000f3c00:    retVal = "star or headlight?"; break;
		case 0x00127400:    retVal = "another (lod) star or headlight?"; break;
		default: retVal = "Unknown"; break;
		}
	}
	else if (m_system == MWSKINS) {
		switch (zeus_texbase) {
		// mwskinsa
		case 0x1fdf00:      retVal = "Skins Tip Box, s=256"; break;
		case 0x07f540:      retVal = "Left main intro"; break;
		case 0x081580:      retVal = "Right main intro"; break;
		case 0x14db00:      retVal = "silver letter b, s=64"; break;
		case 0x14d880:      retVal = "letter a"; break;
		case 0x14e000:      retVal = "letter d"; break;
		case 0x0014dd80:    retVal = "silver letter c, s=64"; break;
		case 0x0014fb80:    retVal = "silver letter o, s=64"; break;
		case 0x0014ec80:    retVal = "silver letter i, s=64"; break;
		case 0x0014f900:    retVal = "silver letter n, s=64"; break;
		case 0x00150580:    retVal = "silver letter s, s=64"; break;
		case 0x00150800:    retVal = "silver letter t, s=64"; break;
		case 0x00150300:    retVal = "silver letter r, s=64"; break;
		case 0x0014e780:    retVal = "silver letter g, s=64"; break;
		case 0x00153280:    retVal = "silver letter C, s=64"; break;
		case 0x0014e280:    retVal = "silver letter e, s=64"; break;
		case 0x0014b800:    retVal = "silver letter O, s=64"; break;
		case 0x00152d80:    retVal = "silver letter A, s=64"; break;
		case 0x0014f680:    retVal = "silver letter m, s=64"; break;
		case 0x00142b40:    retVal = "Black Screen?"; break;
		case 0x00004740:    retVal = "picture bridge over water, s=256"; break;
		case 0x00005c80:    retVal = "picture water shore, s=256"; break;
		case 0x000030c0:    retVal = "left leaderboard background graphics, s=256"; break;
		case 0x00003c00:    retVal = "right leaderboard background graphics, s=256"; break;
		case 0x00040bc0:    retVal = "extreme mode, s=128, t=8alpha"; break;
		case 0x001602a0:    retVal = "photo black hat, sunglasses, gautee, s=64, t=8"; break;
		case 0x00091630:    retVal = "photo wild eye guy, s=64"; break;
		case 0x00159d80:    retVal = "white M s=32, t=4"; break;
		case 0x0015a080:    retVal = "white 9 s=32, t=4"; break;
		case 0x00159f00:    retVal = "white P s=32, t=4"; break;
		case 0x00145a40:    retVal = "white crossbar? s=32, t=4"; break;
		case 0x00145c40:    retVal = "white crossbar2? s=32, t=4"; break;
		case 0x00159300:    retVal = "white _ s=32, t=4"; break;
		case 0x00158d00:    retVal = "white 1 s=32, t=4"; break;
		case 0x00158e80:    retVal = "white 4 s=32, t=4"; break;
		case 0x0001c080:    retVal = "scorecard background, s=256, t=8alpha"; break;
		default: retVal = "Unknown"; break;
		}
	}
	else {
		switch (zeus_texbase) {
		// thegrid
		case 0x000116c8:    retVal = "letter L, s=16, t=4a"; break;
		case 0x00011668:    retVal = "letter O, s=16, t=4a"; break;
		case 0x00011828:    retVal = "letter A, s=16, t=4a"; break;
		case 0x000117c8:    retVal = "letter D, s=16, t=4a"; break;
		case 0x00011728:    retVal = "letter I, s=16, t=4a"; break;
		case 0x00011688:    retVal = "letter N, s=16, t=4a"; break;
		case 0x00011768:    retVal = "letter G, s=16, t=4a"; break;
		case 0x00155b40:    retVal = "green 1010, s=256, t=8"; break;
		case 0x0014db80:    retVal = "The Grid logo, s=256, t=8alpha"; break;
		case 0x0014f280:    retVal = "Searching fo, s=256, t=8alpha"; break;
		case 0x00150500:    retVal = "or, s=64, t=8alpha"; break;
		case 0x000c4400:    retVal = "P, s=32, t=8alpha"; break;
		case 0x000c3ba0:    retVal = "U, s=32, t=8alpha"; break;
		case 0x000c3c00:    retVal = "S, s=32, t=8alpha"; break;
		case 0x000c3c60:    retVal = "H, s=32, t=8alpha"; break;
		case 0x000c39c0:    retVal = "T, s=32, t=8alpha"; break;
		case 0x000c4c70:    retVal = "A, s=32, t=8alpha"; break;
		case 0x000c4070:    retVal = "R, s=32, t=8alpha"; break;
		case 0x000c4460:    retVal = "O, s=32, t=8alpha"; break;
		case 0x000c47a0:    retVal = "E, s=32, t=8alpha"; break;
		case 0x000c48f0:    retVal = "C, s=32, t=8alpha"; break;
		case 0x000c3de0:    retVal = "V, s=32, t=8alpha"; break;
		case 0x000c4650:    retVal = "I, s=32, t=8alpha"; break;
		case 0x000c3a20:    retVal = "N, s=32, t=8alpha"; break;
		case 0x000c4fd0:    retVal = "0, s=32, t=8alpha"; break;
		case 0x000c4290:    retVal = "., s=32, t=8alpha"; break;
		case 0x000c4f70:    retVal = "2, s=32, t=8alpha"; break;
		case 0x000c5030:    retVal = "1, s=32, t=8alpha"; break;
		case 0x000c3ec0:    retVal = "/, s=32, t=8alpha"; break;
		case 0x000c4df0:    retVal = "6, s=32, t=8alpha"; break;
		case 0x00150d00:    retVal = "System 1, s=128, t=8"; break;
		case 0x00151360:    retVal = "System 2, s=128, t=8"; break;
		case 0x001519c0:    retVal = "System 3, s=128, t=8"; break;
		case 0x00152020:    retVal = "System 4, s=128, t=8"; break;
		case 0x00152680:    retVal = "System 5, s=128, t=8"; break;
		case 0x00152ce0:    retVal = "System 6, s=128, t=8"; break;
		case 0x001509c0:    retVal = "READY!, s=128, t=8alpha"; break;
		case 0x000c2d10:    retVal = "6, s=32, t=8alpha"; break;
		case 0x000c30d0:    retVal = "0, s=32, t=8alpha"; break;
		case 0x000c2db0:    retVal = "5, s=32, t=8alpha"; break;
		case 0x000c2b30:    retVal = "9, s=32, t=8alpha"; break;
		case 0x000c2bd0:    retVal = "8, s=32, t=8alpha"; break;
		case 0x000c2c70:    retVal = "7, s=32, t=8alpha"; break;
		case 0x000c2e50:    retVal = "4, s=32, t=8alpha"; break;
		case 0x000c2ef0:    retVal = "3, s=32, t=8alpha"; break;
		case 0x000c2f90:    retVal = "2, s=32, t=8alpha"; break;
		case 0x000c3030:    retVal = "1, s=32, t=8alpha"; break;
		case 0x0014fb80:    retVal = "Brownish circle, s=64, t=8_4x2"; break;
		case 0x0014fd80:    retVal = "Midsize Dark circle rainbow edge, s=256, t=8_4x2"; break;
		case 0x00150580:    retVal = "Dark circle rainbow edge, s=256, t=8_4x2"; break;
		case 0x00012fb0:    retVal = "Red dots, s=16, t=4_2x2"; break;
		case 0x0013b500:    retVal = "Flash with purple outer edge, s=128, t=8_alpha"; break;
		case 0x001c6220:    retVal = "Yellow console?, s=128, t=8_4x2"; break;
		case 0x001c58e0:    retVal = "White/Red Fabric?, s=128, t=8_4x2"; break;
		case 0x001c8b10:    retVal = "White Fabric with yellow band LOD0?, s=64, t=8_4x2"; break;
		case 0x001c6880:    retVal = "White Fabric with yellow band LOD1?, s=128, t=8_4x2"; break;
		case 0x001c76e0:    retVal = "Chiller face, s=128, t=8_4x2"; break;
		case 0x0018cc80:    retVal = "Green grid square 10101, s=256, t=8_4x2"; break;
		case 0x00187780:    retVal = "Left logo The Grid, s=256, t=8_alpha"; break;
		case 0x0018a200:    retVal = "Right logo the Grid, s=256, t=8_alpha"; break;
		case 0x0003cc00:    retVal = "CREDITS, s=256, t=8_alpha"; break;
		case 0x0003e780:    retVal = "INSERT COINS, s=256, t=8_alpha"; break;
		case 0x0003fe00:    retVal = "White 1, s=32, t=8_alpha"; break;
		case 0x0003fbc0:    retVal = "White 2, s=64, t=8_alpha"; break;
		case 0x0003d580:    retVal = "PRESS START, s=256, t=8_alpha"; break;
		//case 0x00154740:    retVal = "Chiller Face, s=128, t=16rgb"; break;
		//case 0x00153340:    retVal = "Ike Face, s=128, t=16rgb"; break;
		case 0x00154740:    retVal = "Just Play, s=256, t=8_4x2"; break;
		case 0x00153340:    retVal = "Enter Name, s=256, t=8_4x2"; break;
		case 0x00130d00:    retVal = "Cyrus highlight screen, s=128, t=8_4x2"; break;
		case 0x00139700:    retVal = "Green Welcome To, s=128, t=4_2x2"; break;
		case 0x0012f300:    retVal = "More people in stands, s=64, t=8_4x2"; break;
		case 0x0012fd00:    retVal = "Even more people in stands, s=64, t=8_4x2"; break;
		case 0x0003ff20:    retVal = "0, s=64, t=8_alpha"; break;
		case 0x00130300:    retVal = "People in stands, s=64, t=8_4x2"; break;
		case 0x0007c8e0:    retVal = "Greenish blob, s=64, t=8_alpha"; break;
		case 0x0015c940:    retVal = "Red +"; break;
		case 0x0015bf40:    retVal = "Blue circle with green outline"; break;
		case 0x0015c740:    retVal = "Radiation symbol"; break;
		case 0x0015cb80:    retVal = "Grey square"; break;
		case 0x0015d380:    retVal = "Green circle inside grey square"; break;
		case 0x00159f40:    retVal = "Shinny green square"; break;
		case 0x001a6340:    retVal = "Yellow ski tip"; break;
		case 0x001a65a0:    retVal = "Metal vest"; break;
		case 0x001a6a00:    retVal = "Head hole metal vest"; break;
		case 0x001a6b70:    retVal = "Yellow WES badge"; break;
		case 0x001a6140:    retVal = "Backwards Yellow WES badge"; break;
		case 0x001a6d70:    retVal = "Maybe stomach"; break;
		case 0x001a6e60:    retVal = "Maybe back"; break;
		case 0x001a6f20:    retVal = "Hand with black glove"; break;
		case 0x001a7090:    retVal = "Wes Face"; break;
		case 0x001a72c0:    retVal = "Dark red strip"; break;
		case 0x001a7340:    retVal = "Wes shoulder pad"; break;
		case 0x001a7460:    retVal = "Orange circle"; break;
		case 0x001a5e20:    retVal = "Wes belt"; break;
		case 0x001a5f40:    retVal = "Wes orange strip on side"; break;
		case 0x001a7770:    retVal = "Grey something"; break;
		case 0x001a74e0:    retVal = "Grey maybe top of boot"; break;
		case 0x001a76e0:    retVal = "Grey hexagon"; break;
		case 0x001a7800:    retVal = "Belt pouches"; break;
		case 0x0015a340:    retVal = "Green shinny block"; break;
		default: retVal = "Unknown"; break;
		}
	}
	return retVal;
}
#endif
