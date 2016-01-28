#include <avr/io.h> 
#include <avr/interrupt.h>

#define false 0
#define true  1

// Set CPU frequency = 32MHz
#ifndef F_CPU
	#define F_CPU 32.0E6
#endif

// ATXMega_A1-Xplained board description
#include "board.h"

// Include Clock system driver from application note AVR1003
#include "clksys_driver.h"

// Include avr-gcc delay routines
#include <util/delay.h>

// ===========================================================
// Global variables
// ===========================================================
volatile uint8_t usart_buffer[] = "EMPTYBUFFEREMPTYBUFFER";
volatile uint8_t usart_counter = 0;

volatile uint8_t pulse_count;
volatile uint16_t pulse_length;
volatile uint16_t pulse_delay;

// ===========================================================
// Timers
// ===========================================================
#define TIMER0_VECT		TCC0_OVF_vect
#define TIMER0			TCC0

// ===========================================================
// USART
// ===========================================================
#define USART			USARTC0				//use for USB-Virtual COM Port
#define USART_PORT		PORTC				//use for USB-Virtual COM Port
#define USART_VECT		USARTC0_RXC_vect	//use for USB-Virtual COM Port

//#define USART			USARTF0				//use for J1 Header
//#define USART_PORT	PORTF				//use for J1 Header
//#define USART_VECT		USARTF0_RXC_vect	//use for USB-Virtual COM Port

#define BSCALE_VALUE  4
#define BSEL_VALUE   12	//prescalers for 32MHz clock to get 9600 baudrate 

// ===========================================================
// General purpose I/O
// ===========================================================
#define PULSEPORT PORTD.OUT

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
// USART
// ===========================================================

//Function prototypes
void usart_rxbyte(uint8_t);
void usart_parsebuffer(void);
void config_triggered_pulse(uint8_t,uint16_t,uint16_t);

// ===========================================================
// USART Initialization
// ===========================================================
void usart_init(void)
{
	// Global interrupts should be disabled during USART initialization
	cli();

	USART_PORT.DIRSET   = PIN3_bm;   // Pin 3 (TX) as output.
	USART_PORT.DIRCLR   = PIN2_bm;   // Pin 2 (RX) as input.

	// USART, 8 Data bits, Even Parity, 1 Stop bit, Asynchronous
//	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | false;
//	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc | USART_CMODE_ASYNCHRONOUS_gc;

	// USART, 8 Data bits, No Parity, Asynchronous
	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_CMODE_ASYNCHRONOUS_gc;

	USART.BAUDCTRLA = BSEL_VALUE;	
	USART.BAUDCTRLB = BSCALE_VALUE << 4;

	// Set receive complete interrupt level
	USART.CTRLA = USART_RXCINTLVL_LO_gc;

	// Enable both RX and TX
	USART.CTRLB |= USART_RXEN_bm;
	USART.CTRLB |= USART_TXEN_bm;

	// Re-enable global interrupts
	sei();

}//end of USART_init()

// ===========================================================
// usart_rxbyte() puts the received byte in the global variable
// usart_buffer[] and calls the processing function if a
// carriage return (0x0D) is received.
// ===========================================================
void usart_rxbyte(uint8_t rxbyte) {

		// Echo the received byte back on the TX pin
		USART.DATA = rxbyte;

		// Display the received byte on the LEDs
		LEDPORT.OUT = ~(rxbyte);

		// Read out the received data
        if (rxbyte == 0x0d) {
			USART.DATA = 0x0a;
			usart_parsebuffer();
		}
		else {
			usart_buffer[usart_counter] = rxbyte;
			usart_counter++;
		}//end of usart if/else

}//end of usart_rxbyte()

// ===========================================================
// usart_parsebuffer() processes the commands received on the
// USART and initiates the requested action.
// ===========================================================
void usart_parsebuffer(void){
	usart_counter = 0;
	unsigned char command = usart_buffer[0];

	switch (command) {

		//TODO:Implement all of these commands
		case 'T':
			config_triggered_pulse(0,0,0);
			break;
		case 'C':
			break;
		case '?':
			break;
		case 'X':
			break;
		default:
			//flash the LEDs
			for (uint8_t i=0; i<10; i++){
				LEDPORT.OUT = 0x00;
				delay1ms(20);
				LEDPORT.OUT = 0xFF;
				delay1ms(20);
			}
			break;
	}//end of switch statement

}//end of usart_parsebuffer()

// ===========================================================
// Timer0 Initialization
// ===========================================================
void timer0_init(void)
{
	TIMER0.PERL = 0xFF;
	TIMER0.PERH = 0xFF;

	// Start Timer0 with Clk/1 prescaling
	TIMER0.CTRLA = ( TIMER0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	// Enable overflow interrupt level high
	TIMER0.INTCTRLA = TC_OVFINTLVL_HI_gc;

	// Restart Timer0
	TIMER0.CTRLFSET = TC_CMD_RESTART_gc;

}//end of Timer0_init()

// ===========================================================
// firepulse() pulse triggering function
// ===========================================================
void firepulse(){

	// Do nothing for the specified delay time
	for (int i=0; i < pulse_delay; i++) asm("nop");

	// Repeat the pulse the specified number of times
	for (int j=0; j < pulse_count; j++){
		
		// Set the pulse pin high
		PULSEPORT = 0x01;
		
		// Don't set it low until the specified pulse length has elapsed
		for (int k=0; k < pulse_length; k++) asm("nop");
		PULSEPORT = 0x00;
	}//end of for loop
}//end of firepulse()

// ===========================================================
// config_triggered_pulse() set the conditions for a train of
// triggered pulses
// ===========================================================
void config_triggered_pulse(uint8_t count, uint16_t length, uint16_t delay){
	pulse_count = count;
	pulse_length = length;
	pulse_delay = delay;
}//end of config_triggered_pulse()

// ===========================================================
// INTERRUPT HANDLERS
// ===========================================================
ISR(TIMER0_VECT) // TIMER0 overflow
{
	// Disable Timer0 while handling the interrupt
	TIMER0.CTRLA = 0;

	// Fire a pulse on PORTD.0
	firepulse();

	//TODO 2: update TIMER0.PERL and TIMER0.PERH with right values
	//TIMER0.PERL = 0xFF;
	//TIMER0.PERH = 0xFF;

	// Restart Timer0
	TIMER0.CTRLA = ( TIMER0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

}//end of Timer0 ISR

ISR(USART_VECT){

	uint8_t rec_char;

	// Wait until the data is received
	while( (USART.STATUS & USART_RXCIF_bm) == 0 ) {}

	rec_char = USART.DATA;
	usart_rxbyte(rec_char);

}//end of USART RX ISR

// ===========================================================
// MAIN FUNCTION
// ===========================================================
int main(void)
{
	//local variables

	//Variable initialization

	//Configure DIOs

	PORTD.DIR = 0xFF; // all outputs
	PORTD.OUT = 0;

	LEDPORT.DIR = 0xFF; //Set as ouput 
	LEDPORT.OUT = 0xFF; //Default off for LED

	//Configure System Clock
	configure_system_clock(); //32 MHz

	//Interrupts: enable high interrupt levels in PMIC
	PMIC.CTRL |= PMIC_HILVLEN_bm;
	
	//Interrupts: enable low interrupt levels in PMIC
	PMIC.CTRL |= PMIC_LOLVLEN_bm;
	
	//Global interrupt enable
	sei();

	// Initialize Timer0
	timer0_init();

	//Initialize USART
	usart_init();

	//Infinite Loop - waiting for USART commands
	while (1)
	{
	}//end of while() loop

}//end of main()
