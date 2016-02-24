#include "timers.h"

// ===========================================================
// Master clock timer initialization
// ===========================================================
void timers_master_init(void)
{
	MASTER.PER = 65535;

	// Start Timer with Clk/64 prescaling
	MASTER.CTRLA = ( MASTER.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;

	// Enable overflow interrupt level high
	//MASTER.INTCTRLA = TC_OVFINTLVL_HI_gc;
	// Disable overflow interrupt
	MASTER.INTCTRLA = TC_OVFINTLVL_OFF_gc;

	// Restart Timer
	MASTER.CTRLFSET = TC_CMD_RESTART_gc;

}//end of master_init()


// ===========================================================
// Tau (trigger delay) initialization
// Modifies the registers of timer "MASTER"
// ===========================================================
void timers_tau_init(	volatile uint16_t *addr_ccN,
						volatile uint8_t *addr_ctrlb,
						volatile uint8_t *addr_intctrlb,
						uint8_t capture_ch_bm,
						uint8_t interrupt_level_bm,
						uint16_t tau
					) {
	
	// Set compare value
	//MASTER.CCn = tau;
	_SFR_MEM16(addr_ccN) = tau;

	// Enable capture/compare channel A
	_SFR_MEM16(addr_ctrlb) |= capture_ch_bm;

	//Enable compare channel D interrupt level high
	_SFR_MEM16(addr_intctrlb) |= interrupt_level_bm;

}//end of timers_tau_init()

// ===========================================================
// Clock1 (first pulse) initialization
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
// Measured delay before setting pin: 330ns
//
// Thanks to: http://wp.josh.com/2015/03/12/avr-timer-based-one-shot-explained/
// ===========================================================
void timers_clock0_init(void) {

	// PER controls the PWM period
	CLOCK0.PER = 65534;

	// CCA controls the PWM duty cycle
	CLOCK0.CCA = 65535;

	// Invert the output pin to get a positive pulse
	CLOCK0PIN |= PORT_INVEN_bm;

	// Start CLOCK0 with Clk/1 prescaling
	CLOCK0.CTRLA = ( CLOCK0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK0.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK0.CTRLB = ( CLOCK0.CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SS_gc | TC0_CCAEN_bm;
}//end of clock0_init()

void timers_clock1_init(void) {

	// PER controls the PWM period
	CLOCK1.PER = 65534;

	// CCA controls the PWM duty cycle
	CLOCK1.CCA = 65535;

	// Invert the output pin to get a positive pulse
	CLOCK1PIN |= PORT_INVEN_bm;

	// Start CLOCK1 with Clk/1 prescaling
	CLOCK1.CTRLA = ( CLOCK1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK1.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK1.CTRLB = ( CLOCK1.CTRLB & ~TC1_WGMODE_gm ) | TC_WGMODE_SS_gc | TC1_CCAEN_bm;
}//end of clock1_init()

// Same as clock0
void timers_clock2_init(void) {

	// PER controls the PWM period
	CLOCK2.PER = 65534;

	// CCA controls the PWM duty cycle
	CLOCK2.CCA = 65535;

	// Invert the output pin to get a positive pulse
	CLOCK2PIN |= PORT_INVEN_bm;

	// Start CLOCK2 with Clk/1 prescaling
	CLOCK2.CTRLA = ( CLOCK2.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK2.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK2.CTRLB = ( CLOCK2.CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SS_gc | TC0_CCAEN_bm;
}//end of clock2_init()

// Same as clock0
void timers_clock3_init(void) {

	// PER controls the PWM period
	CLOCK3.PER = 65534;

	// CCA controls the PWM duty cycle
	CLOCK3.CCA = 65535;

	// Invert the output pin to get a positive pulse
	CLOCK3PIN |= PORT_INVEN_bm;

	// Start CLOCK3 with Clk/1 prescaling
	CLOCK3.CTRLA = ( CLOCK3.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK3.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK3.CTRLB = ( CLOCK3.CTRLB & ~TC1_WGMODE_gm ) | TC_WGMODE_SS_gc | TC1_CCAEN_bm;
}//end of clock3_init()

// ===========================================================
// Update the PER and CCA registers of the specified clock's
// timer, to set the pulse width of the specified clock signal
//
// Parameters: (clock #, pulse width in microseconds)
//
// Example: set_pulse_width(1,350)
// ===========================================================
void timers_set_pulse_width(uint8_t clocknumber, uint16_t pulse_width) {

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


	switch (clocknumber) {
		case 0:
			CLOCK0.CCA = cca_value;
			CLOCK0.PER = cca_value - 1;
			break;
		case 1:
			CLOCK1.CCA = cca_value;
			CLOCK1.PER = cca_value - 1;
			break;
		case 2:
			CLOCK2.CCA = cca_value;
			CLOCK2.PER = cca_value - 1;
			break;
		case 3:
			CLOCK3.CCA = cca_value;
			CLOCK3.PER = cca_value - 1;
			break;
		default:
			break;
	}//end of switch(clocknumber)
}//end of set_pulse_width()

