#include <avr/io.h> 
#include <avr/interrupt.h>
#include <util/delay.h>

#define DEBUG

// Set CPU frequency = 32MHz
#ifndef F_CPU
	#define F_CPU 32000000UL
#endif

// ATXMega_A1-Xplained board description
#include "include/board.h"

// Include Clock system driver from application note AVR1003
#include "include/clksys_driver.h"

// Include delay functions
#include "include/delays.h"

// Include timer and pulse configuration functions
#include "include/timers.h"

// Include USART-related functions
#include "include/usart.h"

// Include DMA controller functions
#include "include/dma.h"

// ===========================================================
// SYSTEM CLOCK
// ===========================================================
void configure_system_clock(void)
{
	CLKSYS_Enable( OSC_RC32MEN_bm );						// Enable internal 32 MHz ring oscillator
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );	// ... and wait until it's stable

//	CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, 1);		// Enable auto-correction - external
//	CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, 0);		// Enable auto-correction - internal

	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );	// Set the 32 MHz ring oscillator as the main clock source.
	CLKSYS_Disable( OSC_RC2MEN_bm | OSC_RC32KEN_bm );		// Disable the other oscillators.
}//end of ConfigureSystemClock()

// ===========================================================
// INTERRUPT HANDLERS
// ===========================================================
ISR(MASTER_OVF_VECT) // MASTER overflow
{
	//PORTD.OUTTGL = 0x08;
}//end of Timer0 ISR

//Tau0/Clock0 is the master clock output.
ISR(TAU0_VECT) // CompareA interrupt vector
{
	// DMA controller does this:
	// CLOCK0.CNT = CLOCK0.CCA;
}//end of CompareD ISR

ISR(TAU1_VECT) // CompareB interrupt vector
{
	// DMA controller does this:
	// DMA.CH1.CTRLA |= DMA_CH_TRFREQ_bm;
}//end of CompareA ISR

ISR(TAU2_VECT) // CompareC interrupt vector
{
	// DMA controller does this:
	// CLOCK2.CNT = CLOCK2.CCA;
}//end of CompareB ISR

ISR(TAU3_VECT) // CompareD interrupt vector
{
	// DMA controller does this:
	// CLOCK3.CNT = CLOCK3.CCA;
}//end of CompareC ISR

ISR(USART_VECT){
	uint8_t rec_char;

	// Wait until the data is received
	//while( !(USART.STATUS & USART_RXCIF_bm))

	//Reading the data clears the interrupt flag
	rec_char = USART.DATA;
	//usart_rxbyte(rec_char);
	
}//end of USART RX ISR

// ===========================================================
// MAIN FUNCTION
// ===========================================================
int main(void)
{

	// Disable all interrupts while initializing
	cli();

	// Configure DIOs

	PORTD.DIR = 0xFF; // all outputs
	PORTD.OUT = 0x00;
	
	PORTF.DIR = 0xFF;
	PORTF.OUT = 0x00;

	LEDPORT.DIR = 0xFF; //Set as ouput 
	LEDPORT.OUT = 0xFF; //Default off for LED

	//Configure System Clock
	configure_system_clock(); //32 MHz

	// Tau0/Master pulse initialization
	timers_tau_init(	&MASTER.CCA,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCAEN_bm,			//Capture channel bitmask
						TC_CCAINTLVL_HI_gc,		//Interrupt level bitmask
						0						//TauN (trigger) delay
					);

	// Tau1-3/slave pulses initialization
	timers_tau_init(	&MASTER.CCB,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCBEN_bm,			//Capture channel bitmask
						TC_CCBINTLVL_HI_gc,		//Interrupt level bitmask
						0						//TauN (trigger) delay
					);
	timers_tau_init(	&MASTER.CCC,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCCEN_bm,			//Capture channel bitmask
						TC_CCCINTLVL_HI_gc,		//Interrupt level bitmask
						0						//TauN (trigger) delay
					);

	timers_tau_init(	&MASTER.CCD,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCDEN_bm,			//Capture channel bitmask
						TC_CCDINTLVL_HI_gc,		//Interrupt level bitmask
						0						//TauN (trigger) delay
					);

	
	timers_init_clock	(	&CLOCK0.PER,
							&CLOCK0.CCA,
							&CLOCK0PIN,
							&CLOCK0.CTRLA,
							&CLOCK0.CTRLB,
							&CLOCK0.CTRLD,
							TC_CLKSEL_DIV1_gc
						);

	timers_init_clock	(	&CLOCK1.PER,
							&CLOCK1.CCA,
							&CLOCK1PIN,
							&CLOCK1.CTRLA,
							&CLOCK1.CTRLB,
							&CLOCK1.CTRLD,
							TC_CLKSEL_DIV1_gc
						);

	timers_init_clock	(	&CLOCK2.PER,
							&CLOCK2.CCA,
							&CLOCK2PIN,
							&CLOCK2.CTRLA,
							&CLOCK2.CTRLB,
							&CLOCK2.CTRLD,
							TC_CLKSEL_DIV1_gc
						);

	timers_init_clock	(	&CLOCK3.PER,
							&CLOCK3.CCA,
							&CLOCK3PIN,
							&CLOCK3.CTRLA,
							&CLOCK3.CTRLB,
							&CLOCK3.CTRLD,
							TC_CLKSEL_DIV1_gc
						);

	timers_set_pulse_width(	&CLOCK0.CCA,
							&CLOCK0.PER,
							1000);
								
	timers_set_pulse_width(	&CLOCK1.CCA,
							&CLOCK1.PER,
							1000);	

	timers_set_pulse_width(	&CLOCK2.CCA,
							&CLOCK2.PER,
							1000);	

	timers_set_pulse_width(	&CLOCK3.CCA,
							&CLOCK3.PER,
							1000);	

	// Initialize master clock
	timers_master_init();

	//Initialize USART
	usart_init();

	//Initialize DMA controller
	dma_init();

	//Interrupts: enable high priority interrupts in PMIC
	PMIC.CTRL |= PMIC_HILVLEN_bm;
	
	//Interrupts: enable medium priority interrupts in PMIC
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	//Interrupts: enable low priority interrupts in PMIC
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// Enable global interrupts once all the setup is done
	sei();

	//Infinite Loop - waiting for USART commands
	while (1)
	{
		#ifdef DEBUG
			asm("nop");
		#endif
	}//end of while() loop

}//end of main()
