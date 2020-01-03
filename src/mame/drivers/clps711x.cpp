// license:BSD-3-Clause
// copyright-holders:Ash Wolf
/***************************************************************************

Cirrus Logic CL-PS7110 and CL-PS7111 system-on-chip

Includes:
- ARM710a CPU
- Multiple ROM support
- 2KB SRAM
- Serial interfaces
- GPIO ports
- SIR (infrared)
- LCD controller
- Timers
- RTC
- Synchronous serial interface
- CL-PS7110: One UART
- CL-PS7111: Two UARTs
- CL-PS7111: Support for CL-PS6700 PC Card controllers

CL-PS7110 Memory Map:
- 00000000..0FFFFFFF : ROM Bank 0 (CS0)
- 10000000..1FFFFFFF : ROM Bank 1 (CS1)
- 20000000..2FFFFFFF : Expansion (CS2)
- 30000000..3FFFFFFF : Expansion (CS3)
- 40000000..4FFFFFFF : Expansion (CS4)
- 50000000..5FFFFFFF : Expansion (CS5)
- 60000000..6FFFFFFF : Expansion (CS6)
- 70000000..7FFFFFFF : Expansion (CS7)
- 80000000..80000FFF : Internal Registers
- 80001000..BFFFFFFF : Unmapped
- C0000000..CFFFFFFF : DRAM Bank 0
- D0000000..DFFFFFFF : DRAM Bank 1
- E0000000..EFFFFFFF : DRAM Bank 2
- F0000000..FFFFFFFF : DRAM Bank 3

CL-PS7111 Memory Map:
- 00000000..0FFFFFFF : ROM Bank 0 (NCS[0])
- 10000000..1FFFFFFF : ROM Bank 1 (NCS[1])
- 20000000..2FFFFFFF : Expansion (NCS[2])
- 30000000..3FFFFFFF : Expansion (NCS[3])
- 40000000..4FFFFFFF : PCMCIA-0 (NCS[4])
- 50000000..5FFFFFFF : PCMCIA-1 (NCS[5])
- 60000000..600007FF : On-chip SRAM
- 60000800..6FFFFFFF : Unmapped
- 70000000..7000007F : Boot ROM
- 70000080..7FFFFFFF : Unmapped
- 80000000..80001FFF : Internal Registers
- 80002000..BFFFFFFF : Unmapped
- C0000000..CFFFFFFF : DRAM Bank 0
- D0000000..DFFFFFFF : DRAM Bank 1
- E0000000..FFFFFFFF : Unmapped

***************************************************************************/


#include "emu.h"
#include "cpu/arm7/arm7.h"
#include "cpu/arm7/arm7core.h"
#include "emupal.h"
#include "screen.h"
#include "speaker.h"

#include "logmacro.h"

#define LOG_GPIO            (1U << 1)
#define LOG_REG_VERBOSE     (1U << 2)
#define LOG_REG_UNHANDLED   (1U << 3)
#define LOG_PCMCIA          (1U << 4)
#define LOG_TIMER           (1U << 5)

#define LOGGPIO(...)            LOGMASKED(LOG_GPIO, __VA_ARGS__)
#define LOGREG_VERBOSE(...)     LOGMASKED(LOG_REG_VERBOSE, __VA_ARGS__)
#define LOGREG_UNHANDLED(...)   LOGMASKED(LOG_REG_UNHANDLED, __VA_ARGS__)
#define LOGPCMCIA(...)          LOGMASKED(LOG_PCMCIA, __VA_ARGS__)
#define LOGTIMER(...)           LOGMASKED(LOG_TIMER, __VA_ARGS__)

#undef VERBOSE
#define VERBOSE LOG_GENERAL

// Interrupts
#define CLPS711x_EXTFIQ 0
#define CLPS711x_BLINT  1
#define CLPS711x_WEINT  2
#define CLPS711x_MCINT  3
#define CLPS711x_CSINT  4
#define CLPS711x_EINT1  5
#define CLPS711x_EINT2  6
#define CLPS711x_EINT3  7
#define CLPS711x_TC1OI  8
#define CLPS711x_TC2OI  9
#define CLPS711x_RTCMI  10
#define CLPS711x_TINT   11
#define CLPS711x_UTXINT 12
#define CLPS711x_URXINT 13
#define CLPS711x_UMSINT 14
#define CLPS711x_SSEOTI 15

#define CLPS7111_KBDINT 16
#define CLPS7111_UTXINT2 28
#define CLPS7111_URXINT2 29

// Internal Registers
// 0x80000000 range
// Ports A, B, E: 0 = input, 1 = output
// Ports C, D:    0 = output, 1 = input
// 7110 has A,B,C,D and 4 bits in E
// 7111 has A,B,D and 3 bits in E
#define CLPS711x_GPIO_ABCD_VALUES     0x000
#define CLPS711x_GPIO_ABCD_DIRECTIONS 0x040
	#define CLPS711x_GPIO_ABCD_A_MASK             0x000000FF
	#define CLPS711x_GPIO_ABCD_B_MASK             0x0000FF00
	#define CLPS711x_GPIO_ABCD_C_MASK             0x00FF0000
	#define CLPS711x_GPIO_ABCD_D_MASK             0xFF000000
	#define CLPS7110_GPIO_ABCD_VALID_MASK         0xFFFFFFFF
	#define CLPS7111_GPIO_ABCD_VALID_MASK         0xFF00FFFF
	#define CLPS711x_GPIO_ABCD_INPUTS_BY_DEFAULT_MASK 0xFFFF0000
#define CLPS711x_GPIO_E_VALUES        0x080
#define CLPS711x_GPIO_E_DIRECTIONS    0x0C0
	#define CLPS7110_GPIO_E_VALID_MASK            0x0000000F
	#define CLPS7111_GPIO_E_VALID_MASK            0x00000007

#define CLPS711x_SYSCON   0x100
	#define CLPS711x_SYSCON_KSCAN_MASK   0x0000000F
	#define CLPS711x_SYSCON_KSCAN_SHIFT  0
		#define CLPS711x_SYSCON_KSCAN_ALL_HIGH 0
		#define CLPS711x_SYSCON_KSCAN_ALL_LOW  1
		#define CLPS711x_SYSCON_KSCAN_ALL_COL0 8
		#define CLPS711x_SYSCON_KSCAN_ALL_COL1 9
		#define CLPS711x_SYSCON_KSCAN_ALL_COL2 10
		#define CLPS711x_SYSCON_KSCAN_ALL_COL3 11
		#define CLPS711x_SYSCON_KSCAN_ALL_COL4 12
		#define CLPS711x_SYSCON_KSCAN_ALL_COL5 13
		#define CLPS711x_SYSCON_KSCAN_ALL_COL6 14
		#define CLPS711x_SYSCON_KSCAN_ALL_COL7 15
	#define CLPS711x_SYSCON_TC1M         0x00000010
	#define CLPS711x_SYSCON_TC1S         0x00000020
	#define CLPS711x_SYSCON_TC2M         0x00000040
	#define CLPS711x_SYSCON_TC2S         0x00000080
	#define CLPS711x_SYSCON_UARTEN       0x00000100
	#define CLPS711x_SYSCON_BZTOG        0x00000200
	#define CLPS711x_SYSCON_BZMOD        0x00000400
	#define CLPS711x_SYSCON_DBGEN        0x00000800
	#define CLPS711x_SYSCON_LCDEN        0x00001000
	#define CLPS711x_SYSCON_CDENTX       0x00002000
	#define CLPS711x_SYSCON_CDENRX       0x00004000
	#define CLPS711x_SYSCON_SIREN        0x00008000
	#define CLPS711x_SYSCON_ADCKSEL_MASK 0x00030000
	#define CLPS711x_SYSCON_ADCKSEL_SHIFT 16
	#define CLPS711x_SYSCON_EXCKEN       0x00040000
	#define CLPS711x_SYSCON_WAKEDIS      0x00080000
	#define CLPS711x_SYSCON_IRTXM        0x00100000
	#define CLPS7111_SYSCON_ALWAYS_1     0x00800000
#define CLPS711x_SYSFLG   0x140
	#define CLPS711x_SYSFLG_MCDR         0x00000001
	#define CLPS711x_SYSFLG_DCDET        0x00000002
	#define CLPS711x_SYSFLG_WUDR         0x00000004
	#define CLPS711x_SYSFLG_WUON         0x00000008
	#define CLPS711x_SYSFLG_DID_MASK     0x000000F0
	#define CLPS711x_SYSFLG_DID_SHIFT    4
	#define CLPS711x_SYSFLG_CTS          0x00000100
	#define CLPS711x_SYSFLG_DSR          0x00000200
	#define CLPS711x_SYSFLG_DCD          0x00000400
	#define CLPS711x_SYSFLG_UBUSY        0x00000800
	#define CLPS711x_SYSFLG_NBFLG        0x00001000
	#define CLPS711x_SYSFLG_RSTFLG       0x00002000
	#define CLPS711x_SYSFLG_PFFLG        0x00004000
	#define CLPS711x_SYSFLG_CLDFLG       0x00008000
	#define CLPS711x_SYSFLG_RTCDIV_MASK  0x003F0000
	#define CLPS711x_SYSFLG_RTCDIV_SHIFT 16
	#define CLPS711x_SYSFLG_URXFE        0x00400000
	#define CLPS711x_SYSFLG_UTXFF        0x00800000
	#define CLPS711x_SYSFLG_CRXFE        0x01000000
	#define CLPS711x_SYSFLG_CTXFF        0x02000000
	#define CLPS711x_SYSFLG_SSIBUSY      0x04000000
	#define CLPS7110_SYSFLG_BOOT8BIT     0x08000000
	#define CLPS7111_SYSFLG_BOOTBIT0     0x08000000
	#define CLPS7111_SYSFLG_BOOTBIT1     0x10000000
	#define CLPS7111_SYSFLG_IS_7111      0x20000000
	#define CLPS711x_SYSFLG_VERID_MASK   0xC0000000
	#define CLPS711x_SYSFLG_VERID_SHIFT  29
#define CLPS711x_MEMCFG1  0x180
#define CLPS711x_MEMCFG2  0x1C0
	#define CLPS711x_MEMCFG_BUS_WIDTH_MASK   0x03
	#define CLPS711x_MEMCFG_BUS_WIDTH_SHIFT   0
	#define CLPS711x_MEMCFG_RANDOM_WAIT_MASK 0x0C
	#define CLPS711x_MEMCFG_RANDOM_WAIT_SHIFT 2
	#define CLPS711x_MEMCFG_SEQ_WAIT_MASK    0x30
	#define CLPS711x_MEMCFG_SEQ_WAIT_SHIFT    4
	#define CLPS711x_MEMCFG_SQAEN            0x40
	#define CLPS711x_MEMCFG_CLKEN            0x80
#define CLPS711x_DRFPR    0x200
#define CLPS711x_INTSR    0x240
#define CLPS711x_INTMR    0x280
#define CLPS711x_LCDCON   0x2C0
	#define CLPS711x_LCDCON_VBUFSIZE_MASK      0x00001FFF
	#define CLPS711x_LCDCON_VBUFSIZE_SHIFT      0
	#define CLPS711x_LCDCON_LINELENGTH_MASK    0x0007E000
	#define CLPS711x_LCDCON_LINELENGTH_SHIFT    13
	#define CLPS711x_LCDCON_PIXELPRESCALE_MASK 0x01F80000
	#define CLPS711x_LCDCON_PIXELPRESCALE_SHIFT 19
	#define CLPS711x_LCDCON_ACBIAS_MASK        0x3E000000
	#define CLPS711x_LCDCON_ACBIAS_SHIFT        25
	#define CLPS711x_LCDCON_GSEN               0x40000000
	#define CLPS711x_LCDCON_GSMD               0x80000000
#define CLPS711x_TC1D     0x300
#define CLPS711x_TC2D     0x340
#define CLPS711x_RTCDR    0x380
#define CLPS711x_RTCMR    0x3C0
#define CLPS711x_PMPCON   0x400
#define CLPS711x_CODR     0x440
#define CLPS711x_UARTDR   0x480
	#define CLPS711x_UARTDR_DATA_MASK 0x0FF
	#define CLPS711x_UARTDR_FRMERR    0x100
	#define CLPS711x_UARTDR_PARERR    0x200
	#define CLPS711x_UARTDR_OVERR     0x400
#define CLPS711x_UBRLCR   0x4C0
	#define CLPS711x_UBRLCR_BIT_RATE_MASK  0x00000FFF
	#define CLPS711x_UBRLCR_BIT_RATE_SHIFT 0
	#define CLPS711x_UBRLCR_BREAK          0x00001000
	#define CLPS711x_UBRLCR_PRTEN          0x00002000
	#define CLPS711x_UBRLCR_EVENPRT        0x00004000
	#define CLPS711x_UBRLCR_XSTOP          0x00008000
	#define CLPS711x_UBRLCR_FIFOEN         0x00010000
	#define CLPS711x_UBRLCR_WRDLEN_MASK    0x00060000
	#define CLPS711x_UBRLCR_WRDLEN_SHIFT   17
#define CLPS711x_SYNCIO   0x500
	#define CLPS711x_SYNCIO_CONFIG_MASK    0x00FF
	#define CLPS711x_SYNCIO_CONFIG_SHIFT   0
	#define CLPS711x_SYNCIO_FRAMELEN_MASK  0x1F00
	#define CLPS711x_SYNCIO_FRAMELEN_SHIFT 8
	#define CLPS711x_SYNCIO_SMCKEN         0x2000
	#define CLPS711x_SYNCIO_TXFRMEN        0x4000
#define CLPS711x_PALLSW   0x540
#define CLPS711x_PALMSW   0x580
#define CLPS711x_STFCLR   0x5C0
#define CLPS711x_BLEOI    0x600
#define CLPS711x_MCEOI    0x640
#define CLPS711x_TEOI     0x680
#define CLPS711x_TC1EOI   0x6C0
#define CLPS711x_TC2EOI   0x700
#define CLPS711x_RTCEOI   0x740
#define CLPS711x_UMSEOI   0x780
#define CLPS711x_COEOI    0x7C0
#define CLPS711x_HALT     0x800
#define CLPS711x_STDBY    0x840

// CL-PS7111 only
// 0x80001000 range
#define CLPS7111_REG_BASE 0x80001000
#define CLPS7111_FRBADDR  0x000
#define CLPS7111_SYSCON2  0x100
	#define CLPS7111_SYSCON2_CODECEN  0x0001
	#define CLPS7111_SYSCON2_KBD6     0x0002
	#define CLPS7111_SYSCON2_DRAMSZ   0x0004
	#define CLPS7111_SYSCON2_KBWEN    0x0008
	#define CLPS7111_SYSCON2_PCMCIA1  0x0020
	#define CLPS7111_SYSCON2_PCMCIA2  0x0040
	#define CLPS7111_SYSCON2_UART2EN  0x0100
	#define CLPS7111_SYSCON2_OSTB     0x1000
	#define CLPS7111_SYSCON2_CLKENSL  0x2000
	#define CLPS7111_SYSCON2_BUZFREQ  0x4000
#define CLPS7111_SYSFLG2  0x140
	#define CLPS7111_SYSFLG2_CKMODE   0x000040
	#define CLPS7111_SYSFLG2_UBUSY2   0x000800
	#define CLPS7111_SYSFLG2_URXFE2   0x400000
	#define CLPS7111_SYSFLG2_UTXFF2   0x800000
#define CLPS7111_INTSR2   0x240
#define CLPS7111_INTMR2   0x280
#define CLPS7111_UARTDR2  0x480
#define CLPS7111_UBRLCR2  0x4C0
#define CLPS7111_KBDEOI   0x700

#define MAIN_CLOCK XTAL(18'432'000)
static const uint32_t TIMER_FREQ_0 = 2'000;
static const uint32_t TIMER_FREQ_1 = 512'000;


// GPIO mappings are specific to particular devices
// and not to the CL-PS711x as a whole


class clps711x_state : public driver_device
{
public:
	clps711x_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_screen(*this, "screen")
	{
	}

	void osaris(machine_config &config);

	DECLARE_WRITE32_MEMBER( clps711x_reg_w );
	DECLARE_READ32_MEMBER( clps711x_reg_r );
	DECLARE_WRITE32_MEMBER( clps7111_reg_w );
	DECLARE_READ32_MEMBER( clps7111_reg_r );

protected:
	// driver_device overrides
	virtual void machine_start() override;
	virtual void machine_reset() override;

	virtual void video_start() override;

	uint32_t gpio_abcd_read_pins(uint32_t mask);
	uint32_t gpio_e_read_pins(uint32_t mask);
	void gpio_abcd_write_pins(uint32_t mask, uint32_t pins);
	void gpio_e_write_pins(uint32_t mask, uint32_t pins);

	void set_interrupt_state(uint32_t line, bool state);
	void refresh_interrupts();

	void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;

private:
	// screen updates
	uint32_t screen_update(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);

	void osaris_map(address_map &map);

	// devices
	required_device<cpu_device> m_cpu;
	required_device<screen_device> m_screen;

	// port states
	// uint32_t m_abcd_values;
	uint32_t m_abcd_directions;
	uint32_t m_abcd_valid_mask;
	// uint32_t m_e_values;
	uint32_t m_e_directions;
	uint32_t m_e_valid_mask;

	uint32_t m_syscon;
	uint32_t get_sysflg() const;

	uint32_t m_interrupt_states;
	uint32_t m_interrupt_mask;

	emu_timer *m_tick_timer;
	emu_timer *m_counter_timers[2];
	uint32_t m_counter_reload[2];

	void counter_timer_load(int which, uint32_t data);
	void counter_timer_update_settings(int which, uint32_t data);

	uint32_t counter_timer_frequency(int which) const {
		uint32_t mask = (which == 1) ? CLPS711x_SYSCON_TC2S : CLPS711x_SYSCON_TC1S;
		return (m_syscon & mask) ? TIMER_FREQ_1 : TIMER_FREQ_0;
	}


	// LCD
	struct lcd_config_t {
		uint32_t width, height;
		uint32_t pixel_prescale;
		uint32_t ac_prescale;
		uint32_t bpp;
		uint32_t raw_value;
		uint64_t palette;
		uint32_t framebuffer_address;
	};
	lcd_config_t m_lcd_config;

	void configure_lcd(uint32_t raw_value);


	// PC CARD
	DECLARE_READ32_MEMBER( clps7600_reg_r );
	DECLARE_WRITE32_MEMBER( clps7600_reg_w );

	struct clps7600_regs_t {
		uint32_t interrupt_status;
		uint32_t interrupt_mask;
		uint32_t system_interface_config;
		uint32_t card_interface_config;
		uint32_t power_management;
		uint32_t card_power_control;
		uint32_t card_interface_timing_0A;
		uint32_t card_interface_timing_0B;
		uint32_t card_interface_timing_1A;
		uint32_t card_interface_timing_1B;
		uint32_t dma_control;
		uint32_t device_information;
	};
	clps7600_regs_t m_clps7600_regs;
};

READ32_MEMBER( clps711x_state::clps711x_reg_r )
{
	switch ((offset << 2) & 0xFC0)
	{
		case CLPS711x_GPIO_ABCD_VALUES:
			return gpio_abcd_read_pins(mem_mask);
		case CLPS711x_GPIO_ABCD_DIRECTIONS:
			return (m_abcd_directions ^ CLPS711x_GPIO_ABCD_INPUTS_BY_DEFAULT_MASK) & mem_mask;
		case CLPS711x_GPIO_E_VALUES:
			return gpio_e_read_pins(mem_mask);
		case CLPS711x_GPIO_E_DIRECTIONS:
			return m_e_directions & mem_mask;
		case CLPS711x_SYSCON:
			LOGREG_VERBOSE("clps711x_reg_r: %s: read syscon\n", machine().describe_context());
			return m_syscon;
		case CLPS711x_SYSFLG:
			LOGREG_VERBOSE("clps711x_reg_r: %s: read sysflg\n", machine().describe_context());
			return get_sysflg();
		case CLPS711x_INTSR:
			return m_interrupt_states & 0xFFFF;
		case CLPS711x_INTMR:
			return m_interrupt_mask & 0xFFFF;
		case CLPS711x_LCDCON:
			return m_lcd_config.raw_value;
		case CLPS711x_TC1D:
			return m_counter_timers[0]->remaining().as_ticks(counter_timer_frequency(0));
		case CLPS711x_TC2D:
			return m_counter_timers[1]->remaining().as_ticks(counter_timer_frequency(1));
		case CLPS711x_PALLSW:
			return m_lcd_config.palette & 0xFFFFFFFF;
		case CLPS711x_PALMSW:
			return m_lcd_config.palette >> 32;
	}
	LOGREG_UNHANDLED("clps711x_reg_r: %s: read unknown register %08x\n", machine().describe_context(), offset<<2);
	return 0xFFFFFFFF;
}
WRITE32_MEMBER( clps711x_state::clps711x_reg_w )
{
	switch ((offset << 2) & 0xFC0)
	{
		case CLPS711x_GPIO_ABCD_VALUES:
			gpio_abcd_write_pins(mem_mask & m_abcd_valid_mask, data);
			return;
		case CLPS711x_GPIO_ABCD_DIRECTIONS:
			data ^= CLPS711x_GPIO_ABCD_INPUTS_BY_DEFAULT_MASK;
			m_abcd_directions &= ~mem_mask;
			m_abcd_directions |= (data & mem_mask & m_abcd_valid_mask);
			LOGGPIO("clps711x_reg_w: %s: GPIO ABCD output pins now %08x\n", machine().describe_context(), m_abcd_directions);
			return;
		case CLPS711x_GPIO_E_VALUES:
			gpio_e_write_pins(mem_mask & m_e_valid_mask, data);
			return;
		case CLPS711x_GPIO_E_DIRECTIONS:
			m_e_directions &= ~mem_mask;
			m_e_directions |= (data & mem_mask & m_e_valid_mask);
			LOGGPIO("clps711x_reg_w: %s: GPIO E output pins now %x\n", machine().describe_context(), m_e_directions);
			return;
		case CLPS711x_SYSCON:
			if (mem_mask & CLPS711x_SYSCON_TC1S)
				counter_timer_update_settings(0, data);
			if (mem_mask & CLPS711x_SYSCON_TC2S)
				counter_timer_update_settings(1, data);
			m_syscon = data;
			LOGREG_VERBOSE("clps711x_reg_w: %s: syscon now %08x\n", machine().describe_context(), m_syscon);
			return;
		case CLPS711x_INTMR:
			m_interrupt_mask &= ~0xFFFF;
			m_interrupt_mask |= (data & mem_mask & 0xFFFF);
			refresh_interrupts();
			LOGREG_VERBOSE("clps711x_reg_w: %s: interrupt mask now %08x\n", machine().describe_context(), m_interrupt_mask);
			return;
		case CLPS711x_LCDCON:
			LOGREG_VERBOSE("clps711x_reg_w: %s: setting LCDCON to %08x\n", machine().describe_context(), data);
			configure_lcd((m_lcd_config.raw_value & ~mem_mask) | (data & mem_mask));
			return;
		case CLPS711x_TC1D:
			counter_timer_load(0, data);
			return;
		case CLPS711x_TC2D:
			counter_timer_load(1, data);
			return;
		case CLPS711x_PALLSW:
			m_lcd_config.palette &= 0xFFFFFFFF00000000;
			m_lcd_config.palette |= data;
			return;
		case CLPS711x_PALMSW:
			m_lcd_config.palette &= 0x00000000FFFFFFFF;
			m_lcd_config.palette |= ((uint64_t)data << 32);
			return;
		case CLPS711x_BLEOI:  set_interrupt_state(CLPS711x_BLINT,  false); return;
		case CLPS711x_MCEOI:  set_interrupt_state(CLPS711x_MCINT,  false); return;
		case CLPS711x_TEOI:
			set_interrupt_state(CLPS711x_TINT, false);
			set_interrupt_state(CLPS711x_WEINT, false);
			return;
		case CLPS711x_TC1EOI: set_interrupt_state(CLPS711x_TC1OI,  false); return;
		case CLPS711x_TC2EOI: set_interrupt_state(CLPS711x_TC2OI,  false); return;
		case CLPS711x_RTCEOI: set_interrupt_state(CLPS711x_RTCMI,  false); return;
		case CLPS711x_UMSEOI: set_interrupt_state(CLPS711x_UMSINT, false); return;
		case CLPS711x_COEOI:  set_interrupt_state(CLPS711x_CSINT,  false); return;
		case CLPS711x_HALT:
			m_cpu->suspend(SUSPEND_REASON_HALT, true);
			return;
	}
	LOGREG_UNHANDLED("clps711x_reg_w: %s: write unknown register %08x at %08x\n", machine().describe_context(), data, offset<<2);
}
READ32_MEMBER( clps711x_state::clps7111_reg_r )
{
	LOGREG_UNHANDLED("clps7111_reg_r: %s: read %08x\n", machine().describe_context(), offset<<2);
	return 0xFFFFFFFF;
}
WRITE32_MEMBER( clps711x_state::clps7111_reg_w )
{
	LOGREG_UNHANDLED("clps7111_reg_w: %s: write %08x at %08x\n", machine().describe_context(), data, offset<<2);
}


uint32_t clps711x_state::get_sysflg() const
{
	return CLPS7111_SYSFLG_IS_7111;
}



uint32_t clps711x_state::gpio_abcd_read_pins( uint32_t mask )
{
	return 0;
}
uint32_t clps711x_state::gpio_e_read_pins( uint32_t mask )
{
	return 0;
}

void clps711x_state::gpio_abcd_write_pins( uint32_t mask, uint32_t pins )
{
	LOGGPIO("gpio_abcd_write_pins: %s: GPIO pins %08x => %08x\n", machine().describe_context(), mask, pins);
}
void clps711x_state::gpio_e_write_pins( uint32_t mask, uint32_t pins )
{
	LOGGPIO("gpio_e_write_pins: %s: GPIO pins %08x => %08x\n", machine().describe_context(), mask, pins);
}

void clps711x_state::set_interrupt_state( uint32_t line, bool state )
{
	m_interrupt_states &= ~(1 << line);
	if ( state )
		m_interrupt_states |= (1 << line);

	refresh_interrupts();
}

void clps711x_state::refresh_interrupts()
{
	uint32_t masked_states = m_interrupt_states & m_interrupt_mask;

	m_cpu->set_input_line(ARM7_FIRQ_LINE, (masked_states & 0x000F) ? ASSERT_LINE : CLEAR_LINE);
	m_cpu->set_input_line(ARM7_IRQ_LINE,  (masked_states & 0xFFF0) ? ASSERT_LINE : CLEAR_LINE);

	if ( m_interrupt_states && m_cpu->suspended(SUSPEND_REASON_HALT) )
		m_cpu->resume(SUSPEND_REASON_HALT);
}

void clps711x_state::video_start()
{
}

uint32_t clps711x_state::screen_update( screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect )
{
	address_space &space = m_cpu->space(AS_PROGRAM);

	rgb_t palette[16];
	if (m_lcd_config.bpp == 1)
	{
		palette[0] = rgb_t::white();
		palette[1] = rgb_t::black();
	}
	else
	{
		static const uint8_t values[16] = {
			255, 227, 204, 187,
			170, 153, 142, 127,
			127, 113, 102, 85,
			68,  51,  28,  0
		};
		for (int i = 0; i < (1 << m_lcd_config.bpp); i++)
		{
			int raw_val = (m_lcd_config.palette >> (i * 4)) & 0xF;
			palette[i] = rgb_t(values[raw_val], values[raw_val], values[raw_val]);
		}
	}
	

	rectangle to_draw = cliprect;
	to_draw &= bitmap.cliprect();

	int bpp = m_lcd_config.bpp;
	int ppb = 8 / bpp;
	int line_width = (m_lcd_config.width * bpp) / 8;

	for (int y = to_draw.min_y; y <= to_draw.max_y; y++)
	{
		uint32_t line_offset = m_lcd_config.framebuffer_address + line_width * y;

		for (int x = to_draw.min_x; x <= to_draw.max_x; x++)
		{
			uint8_t byte = space.read_byte(line_offset + (x / ppb));
			int shift = (x & (ppb - 1)) * bpp;
			int mask = (1 << bpp) - 1;
			int index = (byte >> shift) & mask;
			bitmap.pix32(y, x) = palette[index];
		}
	}

	return 0;
}

void clps711x_state::osaris_map(address_map &map)
{
	map(0x00000000, 0x007FFFFF).rom().mirror(0x0F800000).region("rom0", 0);
	map(0x4C000000, 0x4C003FFF).rw(FUNC(clps711x_state::clps7600_reg_r), FUNC(clps711x_state::clps7600_reg_w));
	map(0x60000000, 0x600007FF).ram(); // SRAM
	map(0x80000000, 0x80000FFF).rw(FUNC(clps711x_state::clps711x_reg_r), FUNC(clps711x_state::clps711x_reg_w));
	map(0x80001000, 0x80001FFF).rw(FUNC(clps711x_state::clps7111_reg_r), FUNC(clps711x_state::clps7111_reg_w));
	map(0xC0000000, 0xC03FFFFF).ram().mirror(0x0FC00000); // DRAM
	map(0xD0000000, 0xDFFFFFFF).noprw();
	map(0xE0000000, 0xEFFFFFFF).noprw();
	map(0xF0000000, 0xFFFFFFFF).noprw();
}

static INPUT_PORTS_START( osaris )
INPUT_PORTS_END


enum {
	CLPS711x_TIMER_TICK = 0,
	CLPS711x_TIMER_TC1 = 1,
	CLPS711x_TIMER_TC2 = 2
};

void clps711x_state::machine_start()
{
	save_item(NAME(m_abcd_directions));
	save_item(NAME(m_e_directions));
	save_item(NAME(m_syscon));
	save_item(NAME(m_interrupt_states));
	save_item(NAME(m_interrupt_mask));
	save_item(NAME(m_counter_reload[0]));
	save_item(NAME(m_counter_reload[1]));

	m_tick_timer = timer_alloc(CLPS711x_TIMER_TICK);
	m_counter_timers[0] = timer_alloc(CLPS711x_TIMER_TC1);
	m_counter_timers[1] = timer_alloc(CLPS711x_TIMER_TC2);

	save_item(NAME(m_lcd_config.width));
	save_item(NAME(m_lcd_config.height));
	save_item(NAME(m_lcd_config.pixel_prescale));
	save_item(NAME(m_lcd_config.ac_prescale));
	save_item(NAME(m_lcd_config.bpp));
	save_item(NAME(m_lcd_config.raw_value));
	save_item(NAME(m_lcd_config.palette));
	save_item(NAME(m_lcd_config.framebuffer_address));

	save_item(NAME(m_clps7600_regs.interrupt_status));
	save_item(NAME(m_clps7600_regs.interrupt_mask));
	save_item(NAME(m_clps7600_regs.system_interface_config));
	save_item(NAME(m_clps7600_regs.card_interface_config));
	save_item(NAME(m_clps7600_regs.power_management));
	save_item(NAME(m_clps7600_regs.card_power_control));
	save_item(NAME(m_clps7600_regs.card_interface_timing_0A));
	save_item(NAME(m_clps7600_regs.card_interface_timing_0B));
	save_item(NAME(m_clps7600_regs.card_interface_timing_1A));
	save_item(NAME(m_clps7600_regs.card_interface_timing_1B));
	save_item(NAME(m_clps7600_regs.dma_control));
	save_item(NAME(m_clps7600_regs.device_information));
}

void clps711x_state::machine_reset()
{
	m_abcd_directions = CLPS711x_GPIO_ABCD_INPUTS_BY_DEFAULT_MASK & m_abcd_valid_mask;
	m_e_directions = 0;
	m_syscon = 0;
	m_interrupt_states = 0;
	m_interrupt_mask = 0;

	m_tick_timer->adjust(attotime::from_hz(64), 0, attotime::from_hz(64));
	m_counter_timers[0]->adjust(attotime::from_ticks(0xFFFF, TIMER_FREQ_0));
	m_counter_reload[0] = 0xFFFF;
	m_counter_timers[1]->adjust(attotime::from_ticks(0xFFFF, TIMER_FREQ_0));
	m_counter_reload[1] = 0xFFFF;

	m_clps7600_regs.interrupt_status = 0;
	m_clps7600_regs.interrupt_mask = 0;
	m_clps7600_regs.system_interface_config = 0x1F8;
	m_clps7600_regs.card_interface_config = 0;
	m_clps7600_regs.power_management = 0;
	m_clps7600_regs.card_power_control = 0;
	m_clps7600_regs.card_interface_timing_0A = 0x1F00;
	m_clps7600_regs.card_interface_timing_0B = 0;
	m_clps7600_regs.card_interface_timing_1A = 0x1F00;
	m_clps7600_regs.card_interface_timing_1B = 0;
	m_clps7600_regs.dma_control = 0;
	m_clps7600_regs.device_information = 0x40;

	// prefill this with something vaguely reasonable-seeming
	m_lcd_config.raw_value =
		(((m_lcd_config.width / 16) - 1) << CLPS711x_LCDCON_LINELENGTH_SHIFT) |
		((((m_lcd_config.width * m_lcd_config.height) / 128) - 1) << CLPS711x_LCDCON_VBUFSIZE_SHIFT);
	m_lcd_config.width = 320;
	m_lcd_config.height = 200;
	m_lcd_config.bpp = 1;
	m_lcd_config.ac_prescale = 1;
	m_lcd_config.pixel_prescale = 1;
	m_lcd_config.palette = 0;
	m_lcd_config.framebuffer_address = 0xC0000000;
}

void clps711x_state::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	if (id == CLPS711x_TIMER_TICK)
	{
		if (m_interrupt_states & (1 << CLPS711x_TINT))
		{
			LOGTIMER("clps711x_state::device_timer: triggering watchdog interrupt\n");
			set_interrupt_state(CLPS711x_WEINT, true);
		}
		else
		{
			LOGTIMER("clps711x_state::device_timer: triggering timer interrupt\n");
			set_interrupt_state(CLPS711x_TINT, true);
		}
	}
	else if (id == CLPS711x_TIMER_TC1)
	{
		if (m_syscon & CLPS711x_SYSCON_BZMOD)
		{
			LOGTIMER("clps711x_state::device_timer: fired TC1 timer with BZMOD enabled\n");
		}
		else
		{
			LOGTIMER("clps711x_state::device_timer: fired TC1 timer interrupt\n");
			set_interrupt_state(CLPS711x_TC1OI, true);
		}
		uint32_t count = (m_syscon & CLPS711x_SYSCON_TC1M) ? m_counter_reload[0] : 0xFFFF;
		uint32_t freq = counter_timer_frequency(0);
		timer.adjust(attotime::from_ticks(count, freq));
	}
	else if (id == CLPS711x_TIMER_TC2)
	{
		LOGTIMER("clps711x_state::device_timer: fired TC2 timer interrupt\n");
		set_interrupt_state(CLPS711x_TC2OI, true);
		uint32_t count = (m_syscon & CLPS711x_SYSCON_TC2M) ? m_counter_reload[1] : 0xFFFF;
		uint32_t freq = counter_timer_frequency(1);
		timer.adjust(attotime::from_ticks(count, freq));
	}
}

void clps711x_state::counter_timer_load(int which, uint32_t data)
{
	data &= 0xFFFF;
	uint32_t freq = counter_timer_frequency(which);
	attotime atto = attotime::from_ticks(data, freq);
	LOGTIMER("clps711x_state::counter_timer_load: loading TC%d with %d at freq %d, attotime %s\n", which + 1, data, freq, atto.as_string());
	m_counter_timers[which]->adjust(atto);
	m_counter_reload[which] = data;
}

void clps711x_state::counter_timer_update_settings(int which, uint32_t data)
{
	uint32_t clock_flag = (which == 1) ? CLPS711x_SYSCON_TC2S : CLPS711x_SYSCON_TC1S;
	bool old_clock = m_syscon & clock_flag;
	bool new_clock = data & clock_flag;
	if (old_clock != new_clock)
	{
		uint32_t old_freq = old_clock ? TIMER_FREQ_1 : TIMER_FREQ_0;
		uint32_t new_freq = new_clock ? TIMER_FREQ_1 : TIMER_FREQ_0;
		uint32_t remaining = m_counter_timers[which]->remaining().as_ticks(old_freq);
		if (remaining <= 0)
			remaining = 1;
		LOGTIMER("clps711x_state::counter_timer_load: switching TC%d from clock source %d to clock source %d, on %d ticks remaining\n", which + 1, old_freq, new_freq, remaining);
		m_counter_timers[which]->adjust(attotime::from_ticks(remaining, new_freq));
	}
}


READ32_MEMBER( clps711x_state::clps7600_reg_r )
{
	switch ((offset << 2) & 0x7C00) {
		case 0x0000: return m_clps7600_regs.interrupt_status;
		case 0x0400: return m_clps7600_regs.interrupt_mask;
		case 0x1C00: return (m_clps7600_regs.card_interface_config & 0x100) ? 0 : 0x400;
		case 0x2000: return m_clps7600_regs.system_interface_config;
		case 0x2400: return m_clps7600_regs.card_interface_config;
		case 0x2800: return m_clps7600_regs.power_management;
		case 0x2C00: return m_clps7600_regs.card_power_control;
		case 0x3000: return m_clps7600_regs.card_interface_timing_0A;
		case 0x3400: return m_clps7600_regs.card_interface_timing_0B;
		case 0x3800: return m_clps7600_regs.card_interface_timing_1A;
		case 0x3C00: return m_clps7600_regs.card_interface_timing_1B;
		case 0x4000: return m_clps7600_regs.dma_control;
		case 0x4400: return m_clps7600_regs.device_information;
	}
	LOGPCMCIA("clps711x_state::clps7600_reg_r: unknown register %04x\n", offset << 2);
	return 0xFFFFFFFF;
}

WRITE32_MEMBER( clps711x_state::clps7600_reg_w )
{
	LOGPCMCIA("clps711x_state::clps7600_reg_w: write %08x to register %04x\n", data, offset << 2);
	switch ((offset << 2) & 0x7C00) {
		case 0x0400: m_clps7600_regs.interrupt_mask = data; break;
		case 0x2000: m_clps7600_regs.system_interface_config = data; break;
		case 0x2400: m_clps7600_regs.card_interface_config = data; break;
		case 0x2800: m_clps7600_regs.power_management = data; break;
		case 0x2C00: m_clps7600_regs.card_power_control = data; break;
		case 0x3000: m_clps7600_regs.card_interface_timing_0A = data; break;
		case 0x3400: m_clps7600_regs.card_interface_timing_0B = data; break;
		case 0x3800: m_clps7600_regs.card_interface_timing_1A = data; break;
		case 0x3C00: m_clps7600_regs.card_interface_timing_1B = data; break;
		case 0x4000: m_clps7600_regs.dma_control = data; break;
		case 0x4400: m_clps7600_regs.device_information = data; break;
	}
}


void clps711x_state::configure_lcd(uint32_t raw_value)
{
	m_lcd_config.raw_value = raw_value;

	m_lcd_config.width = (raw_value & CLPS711x_LCDCON_LINELENGTH_MASK) >> CLPS711x_LCDCON_LINELENGTH_SHIFT;
	m_lcd_config.width = (m_lcd_config.width + 1) * 16;

	if (raw_value & CLPS711x_LCDCON_GSEN)
		m_lcd_config.bpp = (raw_value & CLPS711x_LCDCON_GSMD) ? 4 : 2;
	else
		m_lcd_config.bpp = 1;

	uint32_t buffer_size = (raw_value & CLPS711x_LCDCON_VBUFSIZE_MASK) >> CLPS711x_LCDCON_VBUFSIZE_SHIFT;
	buffer_size = (buffer_size + 1) * 128;
	m_lcd_config.height = buffer_size / (m_lcd_config.width * m_lcd_config.bpp);

	m_lcd_config.pixel_prescale = (raw_value & CLPS711x_LCDCON_PIXELPRESCALE_MASK) >> CLPS711x_LCDCON_PIXELPRESCALE_SHIFT;
	m_lcd_config.pixel_prescale += 1;

	m_lcd_config.ac_prescale = (raw_value & CLPS711x_LCDCON_ACBIAS_MASK) >> CLPS711x_LCDCON_ACBIAS_SHIFT;

	uint32_t divisor = (m_lcd_config.width * m_lcd_config.height) + (m_lcd_config.height / 2);

	m_screen->set_native_aspect();
	m_screen->configure(
		m_lcd_config.width, m_lcd_config.height,
		rectangle(0, m_lcd_config.width - 1, 0, m_lcd_config.height - 1),
		HZ_TO_ATTOSECONDS(36'864'000 / (m_lcd_config.pixel_prescale * divisor))
	);
}


void clps711x_state::osaris(machine_config &config)
{
	m_abcd_valid_mask = CLPS7111_GPIO_ABCD_VALID_MASK;
	m_e_valid_mask = CLPS7111_GPIO_E_VALID_MASK;

	/* basic machine hardware */
	ARM710A(config, m_cpu, MAIN_CLOCK);
	m_cpu->set_addrmap(AS_PROGRAM, &clps711x_state::osaris_map);

	/* video hardware */
	screen_device &screen(SCREEN(config, "screen", SCREEN_TYPE_LCD));
	// default settings from Osaris ROM
	screen.set_raw(36'864'000 / 8, 320, 0, 320, 200, 0, 200);
	screen.set_screen_update(FUNC(clps711x_state::screen_update));

	/* sound hardware */
	SPEAKER(config, "mono").front_center();
}


/***************************************************************************

  Machine driver(s)

***************************************************************************/

ROM_START( osaris )
	ROM_REGION( 0x800000, "rom0", ROMREGION_ERASE00 | ROMREGION_32BIT | ROMREGION_LE )
	ROM_LOAD( "osaris.bin", 0x000000, 0x800000, CRC(2ea9ff1e) SHA1(8a346f8279b0aef50bd5f2c62a71fa59e53b8318) )
ROM_END

COMP( 1999, osaris,  0,   0,  osaris, osaris, clps711x_state, empty_init, "Oregon Scientific",      "Oregon Scientific Osaris", MACHINE_IS_SKELETON )

