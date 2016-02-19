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


// ============= INITIALIZATION FUNCTIONS ====================
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
	PORTD.OUTTGL = 0x08;
}//end of Timer0 ISR

ISR(TAU1_VECT, ISR_NAKED) // CompareA interrupt vector
{
	CLOCK1.CNT = CLOCK1.CCA;
}//end of CompareA ISR

ISR(TAU2_VECT, ISR_NAKED) // CompareB interrupt vector
{
	CLOCK2.CNT = CLOCK2.CCA;
}//end of CompareB ISR

ISR(TAU3_VECT, ISR_NAKED) // CompareC interrupt vector
{
	CLOCK3.CNT = CLOCK3.CCA;
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

	LEDPORT.DIR = 0xFF; //Set as ouput 
	LEDPORT.OUT = 0xFF; //Default off for LED

	PORTF.DIR = 0xFF;
	PORTF.OUT = 0x00;

	//Configure System Clock
	configure_system_clock(); //32 MHz

	timers_tau1_init(0);
	timers_tau2_init(0);
	timers_tau3_init(0);

	timers_clock1_init();
	timers_clock2_init();
	timers_clock3_init();

	timers_set_pulse_width(1,10);
	timers_set_pulse_width(2,20);
	timers_set_pulse_width(3,30);

	// Initialize master clock
	timers_master_init();

	//Initialize USART
	usart_init();

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
