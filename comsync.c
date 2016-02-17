#include <avr/io.h> 
#include <avr/interrupt.h>

#define DEBUG

#define false 0
#define true  1

// Set CPU frequency = 32MHz
#ifndef F_CPU
	#define F_CPU 32000000UL
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

volatile uint16_t pulse_count = 0;
volatile uint16_t pulse_length;
volatile uint16_t pulse_delay;

volatile uint8_t ccpa;
volatile uint8_t ccpb;
volatile uint8_t ccpc;

// ===========================================================
// Timers
// ===========================================================
#define TIMER0_OVF_VECT		TCC0_OVF_vect
#define TIMER0				TCC0
#define CCPA_VECT			TCC0_CCA_vect
#define CCPB_VECT			TCC0_CCB_vect
#define CCPC_VECT			TCC0_CCC_vect

#define CLOCK1				TCD0
#define CLOCK1_VECT			TCD0_OVF_vect

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
void firepulse(void);

// ===========================================================
// USART Initialization
// ===========================================================
void usart_init(void)
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

	// Set receive complete interrupt level
	USART.CTRLA = USART_RXCINTLVL_LO_gc;

	// Enable both RX and TX
	USART.CTRLB |= USART_RXEN_bm;
	USART.CTRLB |= USART_TXEN_bm;

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
		case 'p':
			CLOCK1.PERH = usart_buffer[1];
			CLOCK1.PERL = usart_buffer[2];
			//flash the LEDs twice
			for (uint8_t i=0; i<2; i++){
				LEDPORT.OUT = 0x00;
				delay1ms(20);
				LEDPORT.OUT = 0xFF;
				delay1ms(20);
			}
			break;
		case 'c':
			CLOCK1.CCAH = usart_buffer[1];
			CLOCK1.CCAL = usart_buffer[2];
			//flash the LEDs thrice
			for (uint8_t i=0; i<2; i++){
				LEDPORT.OUT = 0x00;
				delay1ms(20);
				LEDPORT.OUT = 0xFF;
				delay1ms(20);
			}
			break;
		case 'n':
			CLOCK1.CCAH = usart_buffer[1];
			CLOCK1.CCAL = usart_buffer[2];
			//flash the LEDs... frice?
			for (uint8_t i=0; i<2; i++){
				LEDPORT.OUT = 0x00;
				delay1ms(20);
				LEDPORT.OUT = 0xFF;
				delay1ms(20);
			}
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
	TIMER0.PER = 65535;

	// Start Timer0 with Clk/1 prescaling
	TIMER0.CTRLA = ( TIMER0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;

	// Enable overflow interrupt level high
	TIMER0.INTCTRLA = TC_OVFINTLVL_HI_gc;

	// Restart Timer0
	TIMER0.CTRLFSET = TC_CMD_RESTART_gc;

}//end of Timer0_init()

// ===========================================================
// CompareA initialization (tau1)
// ===========================================================
void ccpa_init(void) {
	
	// Set compare value
	// TODO: dynamically reconfigure this as per user input
	TIMER0.CCA = 0x0001;

	// Enable capture/compare channel A
	TIMER0.CTRLB = TIMER0.CTRLB | TC0_CCAEN_bm;

	//Enable compare channel A interrupt level high
	TIMER0.INTCTRLB = ( TIMER0.INTCTRLB & ~TC0_CCAINTLVL_gm) | TC_CCAINTLVL_HI_gc;

}

// ===========================================================
// CompareB initialization (tau2)
// ===========================================================
void ccpb_init(void) {
	
	// Set compare value
	// TODO: dynamically reconfigure this as per user input
	TIMER0.CCB = 0x0001;

	// Enable capture/compare channel A
	TIMER0.CTRLB = ( TIMER0.CTRLB | TC0_CCBEN_bm);

	//Enable compare channel B interrupt level medium
	TIMER0.INTCTRLB = ( TIMER0.INTCTRLB & ~TC0_CCBINTLVL_gm) | TC_CCBINTLVL_MED_gc;

}

// ===========================================================
// CompareC initialization (tau3)
// ===========================================================
void ccpc_init(void) {
	
	// Set compare value
	// TODO: dynamically reconfigure this as per user input
	TIMER0.CCC = 0x00FF;

	// Enable capture/compare channel C
	TIMER0.CTRLB = ( TIMER0.CTRLB | TC0_CCCEN_bm);

	//Enable compare channel C interrupt level low
	TIMER0.INTCTRLB = ( TIMER0.INTCTRLB & ~TC0_CCCINTLVL_gm) | TC_CCCINTLVL_LO_gc;

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
void clock1_init(void) {

	// PER controls the PWM period
	// TODO: dynamically reconfigure this as per user input
	CLOCK1.PER = 65500;

	// CCA controls the PWM duty cycle
	// TODO: dynamically reconfigure this as per user input
	CLOCK1.CCA = 65505;

	PORTD.PIN0CTRL |= PORT_INVEN_bm;

	// Start CLOCK1 with Clk/1 prescaling
	CLOCK1.CTRLA = ( CLOCK1.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV1_gc;
	
	// Disable event actions - required for waveform generation mode
	CLOCK1.CTRLD &= TC_EVACT_OFF_gc;

	// Enable single-slope generation mode and capture/compare channel A
	// Waveform generator overrides regular port OUT when CCAEN is set.
	CLOCK1.CTRLB = ( CLOCK1.CTRLB & ~TC0_WGMODE_gm ) | TC_WGMODE_SS_gc | TC0_CCAEN_bm;
}

// ===========================================================
// firepulse() pulse triggering function
// ===========================================================
void firepulse(void){
	
	//TODO: put something here or remove the function

}//end of firepulse()

// ===========================================================
// config_triggered_pulse() set the conditions for a train of
// triggered pulses
// ===========================================================
void config_triggered_pulse(uint8_t count, uint16_t length, uint16_t delay){
	pulse_count = count;
	pulse_length = length;
	pulse_delay = delay;

	//TODO: put something here or remove the function

}//end of config_triggered_pulse()

// ===========================================================
// INTERRUPT HANDLERS
// ===========================================================
ISR(TIMER0_OVF_VECT) // TIMER0 overflow
{
}//end of Timer0 ISR

ISR(CCPA_VECT) // CompareA interrupt vector
{	
}//end of CompareA ISR

ISR(CCPB_VECT) // CompareB interrupt vector
{

	PORTD.OUT |=(1<<1);
	delay1ms(1);
	PORTD.OUT &=~(1<<1);

}//end of CompareB ISR

ISR(CCPC_VECT) // CompareC interrupt vector
{

	PORTD.OUT |=(1<<0);
	delay1ms(1);
	PORTD.OUT &=~(1<<0);

}//end of CompareC ISR

ISR(USART_VECT){
	uint8_t rec_char;

	// Wait until the data is received
	//while( !(USART.STATUS & USART_RXCIF_bm))

	//Reading the data clears the interrupt flag
	rec_char = USART.DATA;
	//usart_rxbyte(rec_char);

	PORTD.OUTTGL = 0x08;
	CLOCK1.CNT = 65505;

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

	//Configure System Clock
	configure_system_clock(); //32 MHz


	// Initialize Timer0
	//timer0_init();
//	ccpa_init();
//	ccpb_init();
//	ccpc_init();

	clock1_init();

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
