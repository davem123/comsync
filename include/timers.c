#include "timers.h"
#include "dma.h"

// ===========================================================
// Master clock timer initialization
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
void timers_master_init32(volatile uint32_t period_us){

	volatile uint32_t clock_ticks = 0;
	volatile uint16_t periodhigh = 0;
	volatile uint16_t periodlow = 0;

	clock_ticks = period_us * F_CPU_MHZ;

	//periodlow = ((uint32_t) per_value >> 0) & 0x0000FFFF;
	//periodhigh = ((uint32_t) per_value >> 16) & 0x0000FFFF;

	if (clock_ticks > 0xFFFF) periodhigh = clock_ticks / 0xFFFF;
	periodlow = clock_ticks / (periodhigh + 1);
	

	// ===========================================================
	// LEAST SIGNIFICANT TIMER
	// Only this timer is needed for periods <= 0x0000FFFF
	// ===========================================================
	MASTERL.PER = periodlow;
	//MASTERL.PER = 0xBB80;
	MASTERH.PER = 0;

	// Start Timer with no prescaling
	MASTERL.CTRLA = ( MASTERL.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	// Enable overflow interrupt
	//MASTERL.INTCTRLA = TC_OVFINTLVL_HI_gc;

	// IMPORTANT!!
	//Set CCA = PER so we get a pulse every timer cycle
	MASTERL.CCA = MASTERL.PER;

	// Enable CCA for CLOCK0/TAU0
	MASTERL.CTRLB |= TC0_CCAEN_bm;

	// Enable high-priority interrupt for CCA
	MASTERL.INTCTRLB |= TC_CCAINTLVL_HI_gc;

	// Restart Timer
	MASTERL.CTRLFSET = TC_CMD_RESTART_gc;

	// ===========================================================
	// MOST SIGNIFICANT TIMER
	// If the period is greater than 0x0000FFFF then another 
	// timer (MASTERH) is used to enable 32-bit operation
	// ===========================================================

	if (periodhigh > 0) {

		MASTERH.PER = periodhigh;
		//MASTERH.PER = 1;

		// Select Timer C0 overflow as the source for event channel 0
		EVSYS.CH0MUX |= EVSYS_CHMUX_TCC0_OVF_gc;

		// Start Timer with no prescaling, use event channel 0 as the clock
		MASTERH.CTRLA = ( MASTERH.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_EVCH0_gc;

		// Set event source to event channel 0
		// Set event action to "externally controlled up/down count"
		// Delay event source by once cycle to compensate for carry
		// propagation delay
		//MASTERH.CTRLD |= 0x58;
		
		// Disable overflow interrupt
		MASTERH.INTCTRLA = TC_OVFINTLVL_OFF_gc;
		//MASTERH.INTCTRLA = TC_OVFINTLVL_HI_gc;

		// Disable CCA for CLOCK0/TAU0 on MASTERL
		MASTERL.CTRLB &= ~TC0_CCAEN_bm;
		// Disable high-priority interrupt for CCA on MASTERL
		MASTERL.INTCTRLB &= ~TC_CCAINTLVL_HI_gc;

		// IMPORTANT!!
		//Set CCA = PER so we get a pulse every timer cycle
		MASTERH.CCA = MASTERH.PER;

		// Enable CCA for CLOCK0/TAU0
		MASTERH.CTRLB |= TC0_CCAEN_bm;

		// Enable high-priority interrupt for CCA
		MASTERH.INTCTRLB |= TC_CCAINTLVL_HI_gc;

		// Restart Timer
		MASTERH.CTRLFSET = TC_CMD_RESTART_gc;
	}

	// Set TAU1/CCB as event source for Event Channel 1
	EVSYS.CH1MUX |= EVSYS_CHMUX_TCC0_CCB_gc;

	// Use Event Channel 1 as clock source for COUNTER1
	COUNTER1.CTRLA = ( COUNTER1.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_EVCH1_gc;
	COUNTER1.PER = MASTERH.PER;
	COUNTER1.INTCTRLA = TC_OVFINTLVL_HI_gc;
	
 	// Set TAU2/CCC as event source for Event Channel 2
	EVSYS.CH2MUX |= EVSYS_CHMUX_TCC0_CCC_gc;

	// Use Event Channel 2 as clock source for COUNTER2
	COUNTER2.CTRLA = ( COUNTER2.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_EVCH2_gc;
	COUNTER2.PER = MASTERH.PER;
	COUNTER2.INTCTRLA = TC_OVFINTLVL_HI_gc;

	// Re-initialize DMA in case we switched between low and high timers
	dma_init();

}//end of timers_master_init32()

// ===========================================================
// Tau (trigger delay) initialization
// Modifies the registers of timer "MASTER"
// ===========================================================
void timers_tau_init32(	uint8_t capture_ch_bm,
						uint8_t tau_addr_offset,
						uint32_t tau_us
					) {

	volatile uint8_t *addr_ctrlb, *addr_intctrlb, *addr_ccN;
	volatile uint32_t ccp_value;
	volatile uint8_t ccp_interrupt_bm;

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
	_SFR_MEM16(addr_ctrlb) |= capture_ch_bm;

	// Enable high-priority interrupt
	ccp_interrupt_bm = (0x03 << (tau_addr_offset % 0x28));
	_SFR_MEM16(addr_intctrlb) |= ccp_interrupt_bm;

	// Re-initialize DMA controller in case we switched
	// between MASTERH and MASTERL compare channels
	dma_init();


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

void timers_init_all_clocks(void){

	timers_init_clock	(	&CLOCK0.PER,		//Address of CLOCK0.PER
							&CLOCK0.CCA,		//Address of CLOCK0.CCA
							&CLOCK0PIN,			//Address of CLOCK0PIN
							&CLOCK0.CTRLA,		//Address of CLOCK0.CTRLA
							&CLOCK0.CTRLB,		//Address of CLOCK0.CTRLB
							&CLOCK0.CTRLD,		//Address of CLOCK0.CTRLD
							TC_CLKSEL_DIV8_gc	//Timer prescaler bitmask
						);

	timers_init_clock	(	&CLOCK1.PER,
							&CLOCK1.CCA,
							&CLOCK1PIN,
							&CLOCK1.CTRLA,
							&CLOCK1.CTRLB,
							&CLOCK1.CTRLD,
							TC_CLKSEL_DIV8_gc
						);

	timers_init_clock	(	&CLOCK2.PER,
							&CLOCK2.CCA,
							&CLOCK2PIN,
							&CLOCK2.CTRLA,
							&CLOCK2.CTRLB,
							&CLOCK2.CTRLD,
							TC_CLKSEL_DIV8_gc
						);

	timers_init_clock	(	&CLOCK3.PER,
							&CLOCK3.CCA,
							&CLOCK3PIN,
							&CLOCK3.CTRLA,
							&CLOCK3.CTRLB,
							&CLOCK3.CTRLD,
							TC_CLKSEL_DIV8_gc
						);
}

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

