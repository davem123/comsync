#include "timers.h"

// ===========================================================
// Master clock timer initialization
// ===========================================================
void timers_master_init(void)
{
	// MASTERL = Least significant timer (16 bits)
	// MASTERH = Most significant timer (16 bits)

	// With a 32-bit timer (and no prescaler):
	// Resolution = 31.25ns (1 clock cycle)
	// Max period = 31.25ns * 2^32 = 134.2 seconds
	
	// For 100ms period, PER = 0x0030D400 (3.2e6 * (1/32MHz) = 100ms)

	uint32_t period = 0x0030D400;

	// ======================================
	// MASTERL (Least signifcant timer) setup
	// ======================================
	MASTERL.PER = (uint16_t) (period & 0x0000FFFF);

	// Use the system clock for MASTERL
	MASTERL.CTRLA = ( MASTERL.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	// Enable event channel 0
	// Event source = Timer C0 overflow
	EVSYS.CH0MUX |= EVSYS_CHMUX_TCC0_OVF_gc;

	// ======================================
	// MASTERH (Most signifcant timer) setup
	// ======================================
	MASTERH.PER = (uint16_t) ( (period & 0xFFFF0000) >> 16);

	// Use event channel 0 (MASTERL overflow) as the clock for MASTERH
	MASTERH.CTRLA = ( MASTERH.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_EVCH0_gc;

	// Enable overflow interrupt level high
	//MASTERH.INTCTRLA = TC_OVFINTLVL_HI_gc;
	// Disable overflow interrupt
	MASTERH.INTCTRLA = TC_OVFINTLVL_OFF_gc;

	// Restart Timer
	MASTERH.CTRLFSET = TC_CMD_RESTART_gc;

}//end of master_init()

// ===========================================================
// Tau (trigger delay) initialization
// Modifies the registers of timer "MASTERH"
// ===========================================================
void timers_tau_init(	volatile uint16_t *addr_ccN,
						volatile uint8_t *addr_ctrlb,
						volatile uint8_t *addr_intctrlb,
						uint8_t capture_ch_bm,
						uint8_t interrupt_level_bm,
						uint16_t tau
					) {
	
	volatile uint16_t cca_value;

	// Set compare value n
	// n = (tau/128) * 65535
	// 65535/128 = 511.99
	//if (tau >= 127)
//		cca_value = 0xFFFF;
//	else
//		cca_value = tau * 512;
	cca_value = tau;

	//MASTERH.CCn = tau;
	_SFR_MEM16(addr_ccN) = cca_value;

	// Enable capture/compare channel A
	_SFR_MEM16(addr_ctrlb) |= capture_ch_bm;

	//Enable compare channel D interrupt level high
	_SFR_MEM16(addr_intctrlb) |= interrupt_level_bm;

}//end of timers_tau_init()

// ===========================================================
// Clock (pulse) initialization
// Uses a timer in single-slope waveform generation mode
// to produce a single-shot pulse
//
// pulse width (cycles) = TOP - CCA
// TOP = 65535 with a 16-bit timer.
//
// 1. Initialize timer with CCA > PER so that it counts up,
// but never updates the output pin
//
// 2. When a pulse is desired, set CNT == CCA. Output is set immediately,
// and cleared when CNT == TOP.
//
// As long as CCA > PER, only a single pulse will be fired.
//
// Thanks to: http://wp.josh.com/2015/03/12/avr-timer-based-one-shot-explained/
// ===========================================================
void timers_init_clock	(	volatile uint16_t *addr_per,
							volatile uint16_t *addr_cca,
							volatile uint8_t *addr_clockpin,
							volatile uint8_t *addr_ctrla,
							volatile uint8_t *addr_ctrlb,
							volatile uint8_t *addr_ctrld,
							uint8_t clksel_bm
						) {

	// PER controls the PWM period
	_SFR_MEM16(addr_per) = 65534;

	// CCA controls the PWM duty cycle
	_SFR_MEM16(addr_cca) = 65535;

	// Invert the output pin to get a positive pulse
	_SFR_MEM16(addr_clockpin) |= PORT_INVEN_bm;

	// Start CLOCKn with Clk/1 prescaling
	_SFR_MEM16(addr_ctrla) |= clksel_bm;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	_SFR_MEM16(addr_ctrlb) |= (TC_WGMODE_SS_gc | TC0_CCAEN_bm);
	
	// Disable event actions - required for waveform generation mode
	_SFR_MEM16(addr_ctrld) &= TC_EVACT_OFF_gc;
}//end of timers_init_clock()

// ===========================================================
// Update the PER and CCA registers of the specified clock's
// timer, to set the pulse width of the specified clock signal
//
// Parameters: (clock #, pulse width in microseconds)
//
// Example: set_pulse_width(1,350)
// ===========================================================
void timers_set_pulse_width(volatile uint16_t *addr_cca,
							volatile uint16_t *addr_per,
							uint16_t pulse_width) {

	uint16_t pulse_width_us = pulse_width;

	// For a 16-bit timer and 32MHz clock with Clk/1 prescaler:
	// maximum pulse width (us) = (2^16 / F_CPU_MHZ) - 1 = 2047us

	// Reduce too-wide pulses to the maximum width
	if (pulse_width_us > 2047)
		pulse_width_us = 2047;

	// pulse width (cycles) = pulse_width_us * timer_clock(MHz)
	// where timer_clock = F_CPU / prescaler
	
	uint16_t pulse_width_cycles = pulse_width_us * F_CPU_MHZ;
	
	// pulse width (cycles) = TOP - CCA
	uint16_t cca_value = (65535 - pulse_width_cycles);

	// CCA controls the PWM duty cycle
	_SFR_MEM16(addr_cca) = cca_value;
	_SFR_MEM16(addr_per) = cca_value - 1;

}//end of set_pulse_width()

