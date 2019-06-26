// license:BSD-3-Clause
// copyright-holders:hap
// thanks-to:Berger
/******************************************************************************

Novag Savant, chess computer with touchscreen. It was followed by Savant II and
Savant Royale on similar hardware. The chess engine is MyChess by David Kittinger.

Hardware overview:
- Zilog Z80B @ 6MHz
- 24KB ROM(3*TMM2364) + sockets for expansion (populated in Savant II)
- 4KB RAM(8*MM2114N-2L), 256x4 battery-backed RAM(MWS5101)
- Fairchild 3870 @ 4MHz (2KB internal ROM)
- HLCD0538, HLCD0539, LCD screen with 8*8 touch-sensitive overlay
- external ports for optional chess clock and printer

The display (both the LCD and the sensors) didn't last long, probably none exist
anymore in original working order.

TODO:
- doesn't work, Z80 side stops communicating and gets in an infinite loop
- hook up inputs
- internal artwork

******************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "cpu/f8/f8.h"
#include "machine/f3853.h"
#include "video/hlcd0538.h"
#include "video/pwm.h"
#include "machine/sensorboard.h"
#include "machine/nvram.h"
#include "sound/dac.h"
#include "sound/volt_reg.h"
#include "screen.h"
#include "speaker.h"

// internal artwork
//#include "novag_savant.lh" // clickable


namespace {

class savant_state : public driver_device
{
public:
	savant_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_maincpu(*this, "maincpu"),
		m_mcu(*this, "mcu"),
		m_psu(*this, "psu"),
		m_lcd1(*this, "lcd1"),
		m_lcd2(*this, "lcd2"),
		m_display(*this, "display"),
		m_board(*this, "board"),
		m_dac(*this, "dac"),
		m_nvram(*this, "nvram"),
		m_inputs(*this, "IN.%u", 0)
	{ }

	// machine drivers
	void savant(machine_config &config);

protected:
	virtual void machine_start() override;

private:
	// devices/pointers
	required_device<cpu_device> m_maincpu;
	required_device<cpu_device> m_mcu;
	required_device<f38t56_device> m_psu;
	required_device<hlcd0538_device> m_lcd1;
	required_device<hlcd0539_device> m_lcd2;
	required_device<pwm_display_device> m_display;
	required_device<sensorboard_device> m_board;
	required_device<dac_bit_interface> m_dac;
	required_shared_ptr<u8> m_nvram;
	required_ioport_array<3> m_inputs;

	// address maps
	void main_map(address_map &map);
	void main_io(address_map &map);
	void mcu_map(address_map &map);
	void mcu_io(address_map &map);

	// I/O handlers
	DECLARE_READ8_MEMBER(nvram_r);
	DECLARE_READ8_MEMBER(stall_r);
	DECLARE_WRITE8_MEMBER(stall_w);
	DECLARE_READ8_MEMBER(mcustatus_r);

	DECLARE_WRITE64_MEMBER(lcd1_output_w);
	DECLARE_WRITE64_MEMBER(lcd2_output_w);

	DECLARE_READ8_MEMBER(databus_r);
	DECLARE_WRITE8_MEMBER(databus_w);
	DECLARE_READ8_MEMBER(control_r);
	DECLARE_WRITE8_MEMBER(control_w);
	DECLARE_WRITE8_MEMBER(lcd_w);
	DECLARE_READ8_MEMBER(input_r);

	u8 m_inp_mux;
	u8 m_databus;
	u8 m_control;
	u64 m_lcd_data;
};

void savant_state::machine_start()
{
	// zerofill
	m_inp_mux = 0;
	m_databus = 0;
	m_control = 0;
	m_lcd_data = 0;

	// register for savestates
	save_item(NAME(m_inp_mux));
	save_item(NAME(m_databus));
	save_item(NAME(m_control));
	save_item(NAME(m_lcd_data));
}



/******************************************************************************
    Devices, I/O
******************************************************************************/

// Z80 side

READ8_MEMBER(savant_state::nvram_r)
{
	// nvram is only d0-d3
	return m_nvram[offset] & 0xf;
}

READ8_MEMBER(savant_state::stall_r)
{
	// TODO: seen in disasm
	return 0;
}

WRITE8_MEMBER(savant_state::stall_w)
{
	// any access to port C0 puts the Z80 into WAIT, sets BUSRQ, and sets MCU EXT INT
	m_databus = data;
	m_psu->ext_int_w(1);
	m_maincpu->set_input_line(Z80_INPUT_LINE_WAIT, ASSERT_LINE);
	m_maincpu->set_input_line(Z80_INPUT_LINE_BUSRQ, ASSERT_LINE);
}

READ8_MEMBER(savant_state::mcustatus_r)
{
	// d0: MCU P1.2
	return BIT(~m_control, 2);
}


// 3870 side

WRITE64_MEMBER(savant_state::lcd1_output_w)
{
	// uses C1-C24
	m_lcd_data = m_lcd_data << 24 | (data >> 8 & 0xffffff);
	m_display->matrix(data & 0xff, m_lcd_data);
}

WRITE64_MEMBER(savant_state::lcd2_output_w)
{
	// uses C6-C32
	m_lcd_data = data >> 5 & 0x7ffffff;
}

READ8_MEMBER(savant_state::databus_r)
{
	return ~m_databus;
}

WRITE8_MEMBER(savant_state::databus_w)
{
	m_databus = ~data;
}

READ8_MEMBER(savant_state::control_r)
{
	return m_control;
}

WRITE8_MEMBER(savant_state::control_w)
{
	// d0: clear EXT INT, clear Z80 WAIT
	if (~data & m_control & 1)
	{
		m_psu->ext_int_w(0);
		m_maincpu->set_input_line(Z80_INPUT_LINE_WAIT, CLEAR_LINE);
	}

	// d1: clear Z80 BUSRQ
	if (~data & m_control & 2)
	{
		//m_maincpu->set_state_int(Z80_A, m_databus);
		m_maincpu->set_input_line(Z80_INPUT_LINE_BUSRQ, CLEAR_LINE);
	}

	// d2: return data for Z80

	// d3: speaker out
	m_dac->write(BIT(data, 3));

	// d4: LCD pins
	m_lcd2->lcd_w(BIT(~data, 4));
	m_lcd1->lcd_w(BIT(~data, 4));

	// d5-d7: keypad mux

	m_control = data;
}

WRITE8_MEMBER(savant_state::lcd_w)
{
	// d0: HLCD0538 data
	// d4: HLCD0539 data
	m_lcd1->data_w(BIT(~data, 0));
	m_lcd2->data_w(BIT(~data, 4));

	// STROBE pin to LCD chips CLK
	m_lcd1->clk_w(1); m_lcd1->clk_w(0);
	m_lcd2->clk_w(1); m_lcd2->clk_w(0);

	// also touchscreen input mux
	m_inp_mux = bitswap<8>(data,7,3,6,2,5,1,4,0);
}

READ8_MEMBER(savant_state::input_r)
{
	u8 data = 0;

	// read touchscreen
	for (int i = 0; i < 8; i++)
		if (BIT(m_inp_mux, i))
			data |= m_board->read_rank(i);

	data = bitswap<8>(data,0,1,2,3,7,6,5,4);

	// read keypad
	for (int i = 0; i < 3; i++)
		if (BIT(m_control >> 5, i))
			data |= m_inputs[i]->read();

	return data;
}



/******************************************************************************
    Address Maps
******************************************************************************/

void savant_state::main_map(address_map &map)
{
	map(0x0000, 0x5fff).rom();
	map(0xc000, 0xcfff).ram();
	map(0xd000, 0xd0ff).mirror(0x0300).ram().r(FUNC(savant_state::nvram_r)).share("nvram");
}

void savant_state::main_io(address_map &map)
{
	map.global_mask(0xff);
	map(0xc0, 0xc0).mirror(0x38).rw(FUNC(savant_state::stall_r), FUNC(savant_state::stall_w));
	map(0xc1, 0xc1).mirror(0x38).unmapw(); // clock
	map(0xc2, 0xc2).mirror(0x38).unmapw(); // printer
	map(0xc3, 0xc3).mirror(0x38).unmapr(); // printer
	map(0xc4, 0xc4).mirror(0x38).r(FUNC(savant_state::mcustatus_r));
	map(0xc5, 0xc5).mirror(0x38).unmapw(); // printer
}

void savant_state::mcu_map(address_map &map)
{
	map.global_mask(0x7ff);
	map(0x0000, 0x07ff).rom();
}

void savant_state::mcu_io(address_map &map)
{
	map(0x00, 0x00).rw(FUNC(savant_state::databus_r), FUNC(savant_state::databus_w));
	map(0x01, 0x01).rw(FUNC(savant_state::control_r), FUNC(savant_state::control_w));
	map(0x04, 0x07).rw(m_psu, FUNC(f38t56_device::read), FUNC(f38t56_device::write));
}



/******************************************************************************
    Input Ports
******************************************************************************/

static INPUT_PORTS_START( savant )
	PORT_START("IN.0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_1)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_2)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_3)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_4)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_5)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_6)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_7)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_8)

	PORT_START("IN.1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_Q)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_W)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_E)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_R)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_T)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_Y)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_U)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_I)

	PORT_START("IN.2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_A)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_S)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_D)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_F)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_G)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_H)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_J)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYPAD) PORT_CODE(KEYCODE_K)
INPUT_PORTS_END



/******************************************************************************
    Machine Drivers
******************************************************************************/

void savant_state::savant(machine_config &config)
{
	/* basic machine hardware */
	Z80(config, m_maincpu, 6_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &savant_state::main_map);
	m_maincpu->set_addrmap(AS_IO, &savant_state::main_io);

	F8(config, m_mcu, 4_MHz_XTAL/2);
	m_mcu->set_addrmap(AS_PROGRAM, &savant_state::mcu_map);
	m_mcu->set_addrmap(AS_IO, &savant_state::mcu_io);
	m_mcu->set_irq_acknowledge_callback("psu", FUNC(f38t56_device::int_acknowledge));

	F38T56(config, m_psu, 4_MHz_XTAL/2);
	m_psu->set_int_vector(0x20);
	m_psu->int_req_callback().set_inputline(m_mcu, F8_INPUT_LINE_INT_REQ);
	m_psu->write_a().set(FUNC(savant_state::lcd_w));
	m_psu->read_b().set(FUNC(savant_state::input_r));

	config.m_perfect_cpu_quantum = subtag("mcu");

	SENSORBOARD(config, m_board).set_type(sensorboard_device::BUTTONS);
	m_board->set_ui_enable(false); // no chesspieces
	m_board->set_delay(attotime::never);

	NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_1);

	/* video hardware */
	HLCD0538(config, m_lcd1).write_cols().set(FUNC(savant_state::lcd1_output_w));
	HLCD0539(config, m_lcd2).write_cols().set(FUNC(savant_state::lcd2_output_w));

	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_SVG));
	screen.set_refresh_hz(60);
	screen.set_size(958, 1080);
	screen.set_visarea_full();

	PWM_DISPLAY(config, m_display).set_size(8, 24+27);
	//config.set_default_layout(layout_novag_savant);

	/* sound hardware */
	SPEAKER(config, "speaker").front_center();
	DAC_1BIT(config, m_dac).add_route(ALL_OUTPUTS, "speaker", 0.25);
	VOLTAGE_REGULATOR(config, "vref").add_route(0, "dac", 1.0, DAC_VREF_POS_INPUT);
}



/******************************************************************************
    ROM Definitions
******************************************************************************/

ROM_START( savant )
	ROM_REGION( 0x10000, "maincpu", ROMREGION_ERASE00 )
	ROM_LOAD("5605_1f_orange.u13", 0x0000, 0x2000, CRC(1feb8e29) SHA1(1c9a9f6ca7b02ea002ea9dc07b0870b3b0a4cdb9) ) // TMM2364
	ROM_LOAD("5606_1g_white.u14",  0x2000, 0x2000, CRC(9d95cf14) SHA1(eb29774d05c2f17b54363417297e535b4217e222) ) // "
	ROM_LOAD("5607_1e_blue.u15",   0x4000, 0x2000, CRC(131fb097) SHA1(2423d369cc9d9c62e6040e7c34941833f7ec7aa0) ) // "
	// 3 more ROM sockets not populated(yellow, red, gold), manual mentions possible expansion

	ROM_REGION( 0x0800, "mcu", 0 )
	ROM_LOAD("sl90547.u29", 0x0000, 0x0800, CRC(6fbf2aa0) SHA1(18e673ba5b806b397dd3d350525b5467c25a0d94) )

	ROM_REGION( 763850, "screen", 0)
	ROM_LOAD("savant.svg", 0, 763850, CRC(f29a5ca4) SHA1(9fabfb86e6235057b60232e987872a645ee4112e) )
ROM_END

} // anonymous namespace



/******************************************************************************
    Drivers
******************************************************************************/

//    YEAR  NAME    PARENT CMP MACHINE  INPUT   CLASS         INIT        COMPANY, FULLNAME, FLAGS
CONS( 1981, savant, 0,      0, savant,  savant, savant_state, empty_init, "Novag", "Savant", MACHINE_SUPPORTS_SAVE | MACHINE_NOT_WORKING )
