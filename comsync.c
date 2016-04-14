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
//
// Configures the system clock and the built-in PLL.
// Using the PLL as the main clock source enables some
// peripherals to run at a higher speed than the CPU.
//
// The higher peripheral clock (ClkPER4) is required for
// high-resolution timer operation.
// ===========================================================
void configure_system_clock(void) {
	
	// Enable 32MHz RC oscillator
	CLKSYS_Enable( OSC_RC32MEN_bm);
	while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );

	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );

	// Use PLL as the main clock now that it's configured
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );

	// Disable the other oscillators.
	CLKSYS_Disable( OSC_RC2MEN_bm | OSC_RC32KEN_bm | OSC_PLLEN_bm );
	
}//end of configure_system_clock_pll()

// ===========================================================
// INTERRUPT HANDLERS
// ===========================================================

// Tau0/Clock0 is the master clock output.

// When the TAU interrupt vector is reached,
// the DMA controller does this:
// CLOCKn.CNT = CLOCKn.CCA;
// which triggers a pulse output

ISR(TAU0L_VECT){} // MASTERL.CCA vector
ISR(TAU1L_VECT){} // MASTERL.CCB vector
ISR(TAU2L_VECT){} // MASTERL.CCC vector
ISR(TAU3L_VECT){} // MASTERL.CCD vector

ISR(TAU0H_VECT){} // MASTERH.CCA vector
ISR(TAU1H_VECT){} // MASTERH.CCB vector
ISR(TAU2H_VECT){} // MASTERH.CCC vector
ISR(TAU3H_VECT){} // MASTERH.CCD vector

ISR(COUNTER1_OVF_VECT){} // COUNTER1 OVF vector
ISR(COUNTER2_OVF_VECT){} // COUNTER2 OVF vector

ISR(USART_VECT){

	uint8_t rec_char;
	//Reading the data clears the interrupt flag
	rec_char = USART.DATA;
	usart_rxbyte(rec_char);
	
}//end of USART RX ISR

// ===========================================================
// MAIN FUNCTION
// ===========================================================
int main(void)
{

	// Disable all interrupts while initializing
	cli();

	// Configure DIOs

	PORTC.DIR = 0xFF; //All outputs
	PORTC.OUT = 0x00;

	PORTD.DIR = 0xFF; //All outputs
	PORTD.OUT = 0x00;
	
	PORTF.DIR = 0xFF; //All outputs
	PORTF.OUT = 0x00;

	LEDPORT.DIR = 0xFF; //All outputs
	LEDPORT.OUT = 0xFF; //Default off for LED

	// Configure System Clock using the PLL for a faster
	// peripheral clock
	// ClkCPU = 32MHz
	// ClkPER2 = 64
	// ClkPER4 = 128MHz
	configure_system_clock();

	// Initialize master clock
	timers_master_init32(9000);

/*	timers_tau_init32(	TAU0_CCA_bm,			//CCP channel bitmask for MASTERx.CTRLB
						TAU0offset,				//Offset from the address of MASTERx.CTRLA (first register address)
						0						//TauN (trigger) delay (us)
					 );
*/
	timers_tau_init32(	TAU1_CCB_bm,			//CCP channel bitmask for MASTERx.CTRLB
						TAU1offset,				//Offset from the address of MASTERx.CTRLA (first register address)
						10						//TauN (trigger) delay (us)
					 );

	timers_tau_init32(	TAU2_CCC_bm,			//CCP channel bitmask for MASTERx.CTRLB
						TAU2offset,				//Offset from the address of MASTERx.CTRLA (first register address)
						20						//TauN (trigger) delay (us)
					 );

	timers_tau_init32(	TAU3_CCD_bm,			//CCP channel bitmask for MASTERx.CTRLB
						TAU3offset,				//Offset from the address of MASTERx.CTRLA (first register address)
						30						//TauN (trigger) delay (us)
					 );

	timers_init_all_clocks();

	timers_set_pulse_width(	&CLOCK0.CCA,
							&CLOCK0.PER,
							200);
								
	timers_set_pulse_width(	&CLOCK1.CCA,
							&CLOCK1.PER,
							200);	

	timers_set_pulse_width(	&CLOCK2.CCA,
							&CLOCK2.PER,
							200);	

	timers_set_pulse_width(	&CLOCK3.CCA,
							&CLOCK3.PER,
							200);	

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
	}//end of while() loop

}//end of main()
