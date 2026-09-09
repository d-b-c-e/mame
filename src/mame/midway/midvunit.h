// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Driver for Midway V-Unit games

**************************************************************************/
#ifndef MAME_MIDWAY_MIDVUNIT_H
#define MAME_MIDWAY_MIDVUNIT_H

#pragma once

#include "midwayic.h"

#include "dcs.h"

#include "bus/ata/ataintf.h"
#include "machine/adc0844.h"
#include "machine/timer.h"
#include "machine/watchdog.h"
#include "video/poly.h"

#include "emupal.h"
#include "screen.h"


struct midvunit_object_data
{
	uint16_t *    destbase = nullptr;
	uint8_t *     texbase = 0;
	uint16_t      pixdata = 0;
	uint8_t       dither = 0;
};

// POC: re-assert MIDV_PATCH entries the game's own startup ROM re-copy
// reverted (defined in midvunit.cpp, called from screen_update)
void midv_patches_tick(uint32_t *ram);
// POC: built-in force feedback - the drivers hand over the signed motor byte
// (see mvffb in midvunit_v.cpp); inert unless MIDV_FFB=1
void midv_ffb_source(int raw, int adapted, uint64_t frame, double seconds, bool game_invert = false);
void midv_ffb_write(int f, bool game_invert = false);
void midv_ffb_cancel();
void midv_ffb_game_active(bool active);

class midvunit_base_state;

class midvunit_renderer : public poly_manager<float, midvunit_object_data, 2>
{
public:
	midvunit_renderer(midvunit_base_state &state);
	void process_dma_queue();
	void make_vertices_inclusive(vertex_t *vert);

private:
	void render_flat(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
	void render_tex(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
	void render_textrans(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
	void render_textransmask(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);

	midvunit_base_state &m_state;
};

class midvunit_base_state : public driver_device
{
public:
	void observe_numeric_hud();
	uint16_t m_page_control = 0;
	uint16_t m_dma_data[16]{};
	uint8_t m_video_changed = 0;

	memory_share_creator<uint16_t> m_videoram;
	required_shared_ptr<uint32_t> m_textureram;
	required_device<screen_device> m_screen;

protected:
	static inline constexpr XTAL MIDVUNIT_VIDEO_CLOCK = 33.333333_MHz_XTAL;

	midvunit_base_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_videoram(*this, "videoram", 0x200000, ENDIANNESS_LITTLE)
		, m_textureram(*this, "textureram")
		, m_screen(*this, "screen")
		, m_maincpu(*this, "maincpu")
		, m_watchdog(*this, "watchdog")
		, m_palette(*this, "palette")
		, m_timer(*this, "timer%u", 0U)
		, m_dcs(*this, "dcs")
		, m_paletteram(*this, "paletteram")
		, m_ram_base(*this, "ram_base")
		, m_fastram(*this, "fastram")
		, m_tms320c31_control(*this, "320c31_control")
	{ }

	virtual void device_post_load() override;
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void video_start() override ATTR_COLD;
	void mvgl_exit();   // POC: stop the GL overlay thread before teardown
	void reset_display_assets();
	void scenery_start();
	void scenery_tick();
	void scenery_exit();
	void world_distance_start();
	void world_distance_tick();
	void world_distance_exit();
	void world_host_start();
	void world_host_exit();
	void world_host_submit(const std::vector<std::array<uint16_t,16>> &quads);
	void usa_distance_start();
	void usa_distance_tick();
	void usa_distance_exit();
	void offroad_distance_start();
	void offroad_distance_tick();
	void offroad_distance_reset();
	void offroad_distance_exit();

	void cmos_protect_w(uint32_t data);
	void dma_queue_w(uint32_t data);
	uint32_t dma_queue_entries_r();
	uint32_t dma_trigger_r(offs_t offset);
	void page_control_w(uint32_t data);
	uint32_t page_control_r();
	void video_control_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	uint32_t scanline_r();
	void videoram_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	uint32_t videoram_r(offs_t offset);
	void paletteram_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	void textureram_w(offs_t offset, uint32_t data);
	uint32_t textureram_r(offs_t offset);
	void control_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	void sound_w(uint32_t data);
	uint32_t tms320c31_control_r(offs_t offset);
	void tms320c31_control_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	uint32_t generic_speedup_r(offs_t offset);

	uint32_t screen_update(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);

	TIMER_CALLBACK_MEMBER(scanline_timer_cb);
	TIMER_CALLBACK_MEMBER(eoi_timer_cb);

	void midvcommon(machine_config &config);

	required_device<tms320c31_device> m_maincpu;
	required_device<watchdog_timer_device> m_watchdog;
	required_device<palette_device> m_palette;
	required_device_array<timer_device, 2> m_timer;
	required_device<dcs_audio_device> m_dcs;
public:
	// POC live bridge reads this from midvunit_renderer (see midvunit_v.cpp)
	required_shared_ptr<uint32_t> m_paletteram;
protected:
	required_shared_ptr<uint32_t> m_ram_base;
	// POC: second work-RAM bank (0x400000) - exposed for the telemetry RAM
	// hunts; the live player physics does not live in m_ram_base
	optional_shared_ptr<uint32_t> m_fastram;
	required_shared_ptr<uint32_t> m_tms320c31_control;

	uint8_t m_cmos_protected = 0;
	uint16_t m_control_data = 0;
	double m_timer_rate = 0;
	uint32_t *m_generic_speedup = nullptr;
	uint16_t m_video_regs[16]{};
	uint8_t m_dma_data_index = 0;
	emu_timer *m_scanline_timer = nullptr;
	emu_timer *m_eoi_timer = nullptr;
	std::unique_ptr<midvunit_renderer> m_poly;
	// Optional World 2.4 scenery extension. Taps never write guest RAM.
	memory_passthrough_handler m_scenery_far_tap, m_scenery_reciprocal_tap, m_scenery_activation_tap;
	uint32_t m_scenery_tree = 0;
	uint8_t m_scenery_mode = 0;
	uint8_t m_scenery_lead = 8;
	std::array<uint32_t, 5001> m_scenery_reciprocal{};
	uint64_t m_scenery_mountains = 0, m_scenery_trees = 0, m_scenery_forests = 0, m_scenery_reads = 0;
	uint64_t m_scenery_activations = 0;
	uint32_t m_scenery_max_index = 0;
	FILE *m_scenery_log = nullptr;
	// Explicit global experiment; projection table in host memory, no model IDs.
	memory_passthrough_handler m_distance_far_tap, m_distance_reciprocal_tap, m_distance_pending_tap;
	uint32_t m_distance_far = 0, m_distance_lead = 0, m_distance_max_index = 0, m_distance_revision = 24;
	std::vector<uint32_t> m_distance_reciprocal;
	uint64_t m_distance_far_tests = 0, m_distance_extra_tests = 0, m_distance_reads = 0, m_distance_pending = 0;
	FILE *m_distance_log = nullptr;
	memory_passthrough_handler m_host_scene_tap;
	FILE *m_host_scene_log = nullptr, *m_host_quad_log = nullptr;
	uint32_t m_host_mode = 0, m_host_first = 0, m_host_last = 0xffffffff, m_host_far = 80000;
	double m_host_previous_scene_log_us = 0;
	memory_passthrough_handler m_usa_far_tap, m_usa_reciprocal_tap, m_usa_residency_tap;
	uint32_t m_usa_far = 0, m_usa_max_index = 0;
	bool m_usa_residency = false;
	std::vector<uint32_t> m_usa_reciprocal;
	uint64_t m_usa_far_tests = 0, m_usa_extra_tests = 0, m_usa_reads = 0, m_usa_effect_reads = 0;
	uint64_t m_usa_pending = 0, m_usa_removal = 0;
	FILE *m_usa_distance_log = nullptr;
	memory_passthrough_handler m_offroad_far_tap, m_offroad_clip_tap, m_offroad_ceiling_tap, m_offroad_table_tap;
	uint32_t m_offroad_multiplier = 0;
	uint64_t m_offroad_last_frame = uint64_t(-1);
	std::vector<uint32_t> m_offroad_reciprocal;
	uint64_t m_offroad_far_tests = 0, m_offroad_stock_rejects = 0, m_offroad_rejects = 0, m_offroad_extra = 0;
	uint64_t m_offroad_clip_reads = 0, m_offroad_ceiling_reads = 0;
	struct offroad_projection_count {
		uint64_t reads = 0, extended = 0, upper = 0;
		int32_t minimum = INT32_MAX, maximum = INT32_MIN;
	};
	std::array<offroad_projection_count,59> m_offroad_counts{};
	FILE *m_offroad_log = nullptr, *m_offroad_projection_log = nullptr;
};

class midvunit_state : public midvunit_base_state
{
public:
	void midvunit(machine_config &config);

	DECLARE_INPUT_CHANGED_MEMBER(gear_button);
	DECLARE_INPUT_CHANGED_MEMBER(shift_button);

	// POC telemetry: current latched shifter bitmask (0x2000=1st .. 0x0400=4th,
	// 0=neutral) - the Forza-packet gear byte mirrors the real shifter
	uint16_t shifter_state() const { return m_shifter_state; }

protected:
	midvunit_state(const machine_config &mconfig, device_type type, const char *tag)
		: midvunit_base_state(mconfig, type, tag)
		, m_adc(*this, "adc")
		, m_nvram(*this, "nvram")
		, m_optional_drivers(*this, "lamp%u", 0U)
		, m_wheel_motor(*this, "wheel_motor")
		, m_in0(*this, "IN0")
		, m_in1(*this, "IN1")
		, m_dsw(*this, "DSW")
		, m_conf(*this, "CONF")
	{ }

	virtual void machine_start() override ATTR_COLD;

	uint32_t port0_r();
	uint32_t adc_r();
	void adc_w(uint32_t data);
	void cmos_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	uint32_t cmos_r(offs_t offset);
	uint32_t wheel_board_r();
	void wheel_board_w(uint32_t data);
	uint32_t intcs_r();
	uint32_t comcs_r(offs_t offset);
	void comcs_w(offs_t offset, uint32_t data);
	void set_input(const char *s);

	uint16_t comm_bus_out();
	uint16_t comm_bus_in();

	void midvunit_map(address_map &map) ATTR_COLD;

	required_device<adc0844_device> m_adc;

	required_shared_ptr<uint32_t> m_nvram;

	output_finder<8> m_optional_drivers;
	output_finder<> m_wheel_motor;
	required_ioport m_in0;
	required_ioport m_in1;
	required_ioport m_dsw;
	required_ioport m_conf;

	uint8_t m_adc_shift = 0;
	uint16_t m_last_port0 = 0;
	uint16_t m_shifter_state = 0;
	uint8_t m_galil_input_index = 0;
	uint8_t m_galil_input_length = 0;
	const char *m_galil_input = nullptr;
	uint16_t m_galil_output_index = 0;
	char m_galil_output[450]{};
	uint8_t m_wheel_board_output = 0;
	uint32_t m_wheel_board_last = 0;
	uint32_t m_wheel_board_u8_latch = 0;
	uint8_t m_comm_flags = 0;
	uint16_t m_comm_data = 0;
};

class crusnusa_state : public midvunit_state
{
public:
	crusnusa_state(const machine_config &mconfig, device_type type, const char *tag)
		: midvunit_state(mconfig, type, tag)
		, m_motion(*this, "MOTION")
	{ }

	void init_crusnu40();
	void init_crusnu21();
	void init_crusnusa();

	ioport_value motion_r();

protected:
	void init_crusnusa_common(offs_t speedup);

	required_ioport m_motion;
};

class crusnwld_state : public midvunit_state
{
public:
	crusnwld_state(const machine_config &mconfig, device_type type, const char *tag)
		: midvunit_state(mconfig, type, tag)
		, m_midway_serial_pic2(*this, "serial_pic2")
	{ }

	void crusnwld(machine_config &config);
	void offroadc(machine_config &config);

	void init_crusnwld();
	void init_offroadc();

protected:
	virtual void machine_start() override ATTR_COLD;

	void crusnwld_control_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	uint32_t crusnwld_serial_status_r();
	uint32_t crusnwld_serial_data_r();
	void crusnwld_serial_data_w(uint32_t data);
	uint32_t bit_data_r(offs_t offset);
	void bit_reset_w(uint32_t data);
	void init_crusnwld_common(offs_t speedup);

	void crusnwld_map(address_map &map) ATTR_COLD;
	void offroadc_map(address_map &map) ATTR_COLD;

	required_device<midway_serial_pic2_device> m_midway_serial_pic2;

	uint16_t m_bit_index = 0;
};

class midvplus_state : public midvunit_base_state
{
public:
	midvplus_state(const machine_config &mconfig, device_type type, const char *tag)
		: midvunit_base_state(mconfig, type, tag)
		, m_midway_ioasic(*this, "ioasic")
		, m_ata(*this, "ata")
		, m_fastram_base(*this, "fastram_base")
		, m_midvplus_misc(*this, "midvplus_misc")
	{ }

	void midvplus(machine_config &config);

	void init_wargods();

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	uint32_t midvplus_misc_r(offs_t offset);
	void midvplus_misc_w(offs_t offset, uint32_t data, uint32_t mem_mask = ~0);
	void midvplus_xf1_w(uint8_t data);

	void midvplus_map(address_map &map) ATTR_COLD;

	required_device<midway_ioasic_device> m_midway_ioasic;
	required_device<ata_interface_device> m_ata;
	required_shared_ptr<uint32_t> m_fastram_base;
	required_shared_ptr<uint32_t> m_midvplus_misc;

	int m_lastval = 0;
};

#endif // MAME_MIDWAY_MIDVUNIT_H
