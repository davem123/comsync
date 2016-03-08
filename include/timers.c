#include "timers.h"

// ===========================================================
// Master clock timer initialization
// ===========================================================
void timers_master_init(uint32_t period_us)
{
	// 65534 = ~127ms period at clkPER4 = 128MHz and /256 prescaler.
	// 51602 = ~100ms

	uint16_t per_value = ( (float) period_us / MAX_PERIOD_MICROSECONDS) * 65534;

	// Enable hi-res extension for timer C0
	HIRESC.CTRL = HIRES_HREN_TC0_gc;

	// For hi-res operation:
	// The two lsb of the timer/counter period register
	// must be set to zero to ensure correct operation
	// (datasheet p. 186)
	MASTERL.PER = per_value & 0xFFFC;

	// Start Timer with Clk/64 prescaling
	MASTERL.CTRLA = ( MASTERL.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV256_gc;

	// Enable overflow interrupt level high
	//MASTERL.INTCTRLA = TC_OVFINTLVL_HI_gc;
	// Disable overflow interrupt
	MASTERL.INTCTRLA = TC_OVFINTLVL_OFF_gc;

	// Restart Timer
	MASTERL.CTRLFSET = TC_CMD_RESTART_gc;

}//end of timers_master_init()

// ===========================================================
// 32-bit master clock made up of cascading 16-bit timers
// MASTERL and MASTERH
//
// Both timers must be "type 0" because four CCP channels
// are needed for CLOCK[0..4]
//
// MASTERL CCP channels: For tau values < [ 2^16 * (1/F_CPU)]
// MASTERH CCP channels: For tau values > [ 2^16 * (1/F_CPU)]
// ===========================================================
void timers_master_init32(volatile float period_us){

	volatile uint32_t per_value = 0;
	volatile uint16_t periodhigh = 0;
	volatile uint16_t periodlow = 0;

	per_value = (float) period_us / TIMER_PERIOD;

	periodlow = ((uint32_t) per_value >> 0) & 0x0000FFFF;
	periodhigh = ((uint32_t) per_value >> 16) & 0x0000FFFF;

	// ===========================================================
	// LEAST SIGNIFICANT TIMER
	// Only this timer is needed for periods <= 0x0000FFFF
	// ===========================================================
	MASTERL.PER = periodlow;

	// Start Timer with no prescaling
	MASTERL.CTRLA = ( MASTERL.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	// Disable overflow interrupt
	MASTERL.INTCTRLA = TC_OVFINTLVL_OFF_gc;

	// Restart Timer
	MASTERL.CTRLFSET = TC_CMD_RESTART_gc;

	// ===========================================================
	// MOST SIGNIFICANT TIMER
	// If the period is greater than 0x0000FFFF then another 
	// timer (MASTERH) is used to enable 32-bit operation
	// ===========================================================

	if (periodhigh > 0) {

		MASTERH.PER = periodhigh;

		// Select Timer C0 overflow as the source for event channel 0
		EVSYS.CH0MUX |= EVSYS_CHMUX_TCC0_OVF_gc;

		// Start Timer with no prescaling, use event channel 0 as the clock
		MASTERH.CTRLA = ( MASTERH.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_EVCH0_gc;
		
		// Enable overflow interrupt for testing
		//MASTERH.INTCTRLA = RTC_OVFINTLVL_HI_gc;
		// Disable overflow interrupt
		MASTERL.INTCTRLA = TC_OVFINTLVL_OFF_gc;

		// Restart Timer
		MASTERH.CTRLFSET = TC_CMD_RESTART_gc;
	}

}//end of timers_master_init32()

// ===========================================================
// Tau (trigger delay) initialization
// Modifies the registers of timer "MASTER"
// ===========================================================
void timers_tau_init(	volatile uint16_t *addr_ccN,
						volatile uint8_t *addr_ctrlb,
						volatile uint8_t *addr_intctrlb,
						uint8_t capture_ch_bm,
						uint8_t interrupt_level_bm,
						uint32_t tau_us
					) {
	
	volatile uint16_t cca_value;
	
	cca_value = ( (float) tau_us / MAX_PERIOD_MICROSECONDS) * 65534;

	// Resolution (4 counts) = 220ns
	// (Two least significant bits are not used in hi-res mode)
	
	// (tauN + pulsewidthN) MUST be < master period

	// The two lsb of the timer/counter period register
	// must be set to zero to ensure correct operation
	//cca_value = tau & 0xFFFC;

	//MASTER.CCn = tau;
	_SFR_MEM16(addr_ccN) = (cca_value & 0xFFFC);

	// Enable capture/compare channel A
	_SFR_MEM16(addr_ctrlb) |= capture_ch_bm;

	//Enable compare channel D interrupt level high
	_SFR_MEM16(addr_intctrlb) |= interrupt_level_bm;

}//end of timers_tau_init()

// ===========================================================
// Tau (trigger delay) initialization
// Modifies the registers of timer "MASTER"
// ===========================================================
void timers_tau_init32(	uint8_t tau_addr_offset,
						uint32_t tau_us
					) {

	volatile uint8_t *addr_ctrlb, *addr_intctrlb, *addr_ccN;
	volatile uint32_t ccp_value;

	ccp_value = tau_us * F_CPU_MHZ;

	// Use the MASTERH CCP channels for values of
	// ccp_value > 0x0000FFFF (tau > 2.048ms)
	if (ccp_value > 0x0000FFFF) {
		addr_ctrlb = &MASTERH.CTRLB;
		addr_intctrlb = &MASTERH.INTCTRLB;
		addr_ccN = &MASTERH.CTRLA + tau_addr_offset;
	}
	// Use the MASTERL CCP channels for
	// tau values < 2.048ms
	else {
		addr_ctrlb = &MASTERL.CTRLB;
		addr_intctrlb = &MASTERL.INTCTRLB;
		addr_ccN = &MASTERL.CTRLA + tau_addr_offset;
	}

	//MASTER.CCn = tau;
	_SFR_MEM16(addr_ccN) = ccp_value;

	// Enable selected capture/compare channel
//	_SFR_MEM16(addr_ctrlb) |= capture_ch_bm;

	// Enable high-priority interrupt
	// (bitmask is the same for all four CCP channels)
	_SFR_MEM16(addr_intctrlb) |= TC_CCAINTLVL_HI_gc;

}//end of timers_tau_init32()

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
							volatile uint16_t pulse_width_us) {
	
	uint16_t pulse_width_cycles, cca_value;

	// For a 16-bit timer and 32MHz clock with Clk/1 prescaler:
	// maximum pulse width (us) = (2^16 / F_CPU_MHZ) - 1 = 2047us

	// pulse width (cycles) = pulse_width_us * timer_clock(MHz)
	// where timer_clock = F_CPU / prescaler
	
	pulse_width_cycles = pulse_width_us * (F_CPU_MHZ / 8);
	
	// pulse width (cycles) = TOP - CCA
	cca_value = (65535 - pulse_width_cycles);

	// CCA controls the PWM duty cycle
	_SFR_MEM16(addr_cca) = cca_value;
	_SFR_MEM16(addr_per) = cca_value - 1;

}//end of set_pulse_width()

