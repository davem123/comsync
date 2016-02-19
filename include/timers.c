#include "timers.h"

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

// ===========================================================
// Master clock timer initialization
// ===========================================================
void timers_master_init(void)
{
	MASTER.PER = 65535;

	// Start Timer with Clk/1 prescaling
	MASTER.CTRLA = ( MASTER.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	// Enable overflow interrupt level high
	MASTER.INTCTRLA = TC_OVFINTLVL_HI_gc;

	// Restart Timer
	MASTER.CTRLFSET = TC_CMD_RESTART_gc;

}//end of master_init()

// ===========================================================
// CompareA initialization (tau1)
// ===========================================================
void timers_tau1_init(uint16_t tau) {
	
	// Set compare value
	MASTER.CCA = tau;

	// Enable capture/compare channel A
	MASTER.CTRLB = MASTER.CTRLB | TC0_CCAEN_bm;

	//Enable compare channel A interrupt level high
	MASTER.INTCTRLB = ( MASTER.INTCTRLB & ~TC0_CCAINTLVL_gm) | TC_CCAINTLVL_HI_gc;

}

// ===========================================================
// CompareB initialization (tau2)
// ===========================================================
void timers_tau2_init(uint16_t tau) {
	
	// Set compare value
	MASTER.CCB = tau;

	// Enable capture/compare channel A
	MASTER.CTRLB = ( MASTER.CTRLB | TC0_CCBEN_bm);

	//Enable compare channel B interrupt level medium
	MASTER.INTCTRLB = ( MASTER.INTCTRLB & ~TC0_CCBINTLVL_gm) | TC_CCBINTLVL_MED_gc;

}

// ===========================================================
// CompareC initialization (tau3)
// ===========================================================
void timers_tau3_init(uint16_t tau) {
	
	// Set compare value
	MASTER.CCC = tau;

	// Enable capture/compare channel C
	MASTER.CTRLB = ( MASTER.CTRLB | TC0_CCCEN_bm);

	//Enable compare channel C interrupt level low
	MASTER.INTCTRLB = ( MASTER.INTCTRLB & ~TC0_CCCINTLVL_gm) | TC_CCCINTLVL_LO_gc;

}

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
void timers_clock1_init(void) {

	// PER controls the PWM period
	CLOCK1.PER = 65534;

	// CCA controls the PWM duty cycle
	CLOCK1.CCA = 65535;

	// Invert the output pin to get a positive pulse
	CLOCK1PIN |= PORT_INVEN_bm;

	// Start CLOCK1 with Clk/1 prescaling
	CLOCK1.CTRLA = ( CLOCK1.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK1.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK1.CTRLB = ( CLOCK1.CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SS_gc | TC0_CCAEN_bm;
}//end of clock1_init()

// Same as clock1
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

// Same as clock1
void timers_clock3_init(void) {

	// PER controls the PWM period
	CLOCK3.PER = 65534;

	// CCA controls the PWM duty cycle
	CLOCK3.CCA = 65535;

	// Invert the output pin to get a positive pulse
	CLOCK3PIN |= PORT_INVEN_bm;

	// Start CLOCK3 with Clk/1 prescaling
	CLOCK3.CTRLA = ( CLOCK3.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK3.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK3.CTRLB = ( CLOCK3.CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SS_gc | TC0_CCAEN_bm;
}//end of clock3_init()
