#include <avr/io.h> 
#include <avr/interrupt.h>

#define false 0
#define true  1

// Set CPU frequency = 32MHz
#define F_CPU 32.0E6

// ATXMega_A1-Xplained board description
#include "board.h"

// Include Clock system driver from application note AVR1003
#include "clksys_driver.h"

// Include avr-gcc delay routines
#include <util/delay.h>

// ===========================================================
// Global variables
// ===========================================================
volatile uint8_t usart_buff0;
volatile uint8_t usart_buff1;

// ===========================================================
// Timers
// ===========================================================
#define Time0_vect	TCC0_OVF_vect
#define TIMER0		TCC0

// ===========================================================
// USART
// ===========================================================
#define USART			USARTC0		//use for USB-Virtual COM Port
#define USART_PORT		PORTC		//use for USB-Virtual COM Port
//#define USART			USARTF0		//use for J1 Header
//#define USART_PORT		PORTF	//use for J1 Header
#define BSCALE_VALUE  4
#define BSEL_VALUE   12	//prescalers for 32MHz clock to get 9600 baudrate 


// ============= INITIALIZATION FUNCTIONS ====================
// ===========================================================
// SYSTEM CLOCK
// ===========================================================
void ConfigureSystemClock(void)
{
	CLKSYS_Enable( OSC_RC32MEN_bm );						// Enable internal 32 MHz ring oscillator
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );	// ... and wait until it's stable

//	CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, 1);		// Enable auto-correction - external
//	CLKSYS_AutoCalibration_Enable(OSC_RC32MCREF_bm, 0);		// Enable auto-correction - internal

	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );	// Set the 32 MHz ring oscillator as the main clock source.
	CLKSYS_Disable( OSC_RC2MEN_bm | OSC_RC32KEN_bm );		// Disable the other oscillators.
}//end of ConfigureSystemClock()


// ===========================================================
// USART
// ===========================================================
void USART_init(void)
{
	USART_PORT.DIRSET   = PIN3_bm;   // Pin 3 (TX) as output.
	USART_PORT.DIRCLR   = PIN2_bm;   // Pin 2 (RX) as input.

	// USART, 8 Data bits, Even Parity, 1 Stop bit, Asynchronous
//	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | false;
//	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc | USART_CMODE_ASYNCHRONOUS_gc;

	// USART, 8 Data bits, No Parity, Asynchronous
	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_CMODE_ASYNCHRONOUS_gc;

	USART.BAUDCTRLA = BSEL_VALUE;	
	USART.BAUDCTRLB = BSCALE_VALUE << 4;	

	// Enable both RX and TX
	USART.CTRLB |= USART_RXEN_bm;
	USART.CTRLB |= USART_TXEN_bm;
}//end of USART_init()


// ===========================================================
// INTERRUPT HANDLERS
// ===========================================================
ISR(Time0_vect) // TIMER0 overflow
{
	// first - disable Timer
	TIMER0.CTRLA = 0;

	//TODO 1: "fire" a pulse on PORTD.0


	//TODO 2: update TIMER0.PERL and TIMER0.PERH with right values


	//TODO 3: restart timer

}//end of ISR()

// ===========================================================
// delay1ms() 1ms delay function
// ===========================================================
// this wrapper function calls _delay_ms with a known value of 1
// if you call _delay_ms(variable) then the floating point library
// is going to be included and your output file gets much larger
// source: http://efundies.com/accurate-delays-with-avr-in-c/
void delay1ms(uint16_t ms) {
    uint16_t i;
    for(i=0;i<ms;i++) _delay_ms(1);
}//end of delay1ms()

// ===========================================================
// MAIN FUNCTION
// ===========================================================
int main( void )
{
	//local variables
	uint8_t rec_char;

	//Variable initialization

	//Configure DIOs

	PORTD.DIR = 0xFF; // all outputs
	PORTD.OUT = 0;

	LEDPORT.DIR = 0xFF; //Set as ouput 
	LEDPORT.OUT = 0xFF; //Default off for LED

	//Configure System Clock
	ConfigureSystemClock(); //32 MHz

	//Initialize USART
	USART_init();

	//Interrupts: enable medium interrupt levels in PMIC and enable global interrupts.
	PMIC.CTRL |= PMIC_HILVLEN_bm;
	
	//Global interrupt enable
	sei();

	//Infinite Loop - waiting for USART commands
	while (1)
	{
		//check USART
		// Wait until the data is received
        while( (USART.STATUS & USART_RXCIF_bm) == 0 ) {}
		// Read out the received data
        rec_char = USART.DATA;

		//test - LED output
		LEDPORT.OUT = ~(rec_char); 

		//TODO: Add Timer/Counter Update
		usart_buff0 = 0;
		usart_buff1 = rec_char;

		TIMER0.PERH = usart_buff1;
		TIMER0.PERL = usart_buff0;

		// Start Timer0 with 1 prescaling
		TIMER0.CTRLA = ( TIMER0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
		// Enable Timer0 overflow interrupt level high
		TIMER0.INTCTRLA = TC_OVFINTLVL_HI_gc;
		// Restart Timer0
		TIMER0.CTRLFSET = TC_CMD_RESTART_gc;


	}//end of while() loop
}//end of main()
