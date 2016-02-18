#include <avr/io.h> 
#include <avr/interrupt.h>

#define DEBUG

#define false 0
#define true  1

// Set CPU frequency = 32MHz
#ifndef F_CPU
	#define F_CPU 32000000UL
#endif

#define F_CPU_MHZ (uint8_t) (F_CPU / 1.0E6)

// ATXMega_A1-Xplained board description
#include "include/board.h"

// Include Clock system driver from application note AVR1003
#include "include/clksys_driver.h"

// Include avr-gcc delay routines
#include <util/delay.h>

// ===========================================================
// Global variables
// ===========================================================
volatile uint8_t usart_buffer[] = "EMPTYBUFFEREMPTYBUFFER";
volatile uint8_t usart_counter = 0;

// ===========================================================
// Timers
// ===========================================================
#define MASTER_OVF_VECT		TCC0_OVF_vect
#define MASTER				TCC0

#define TAU1_VECT			TCC0_CCA_vect
#define TAU2_VECT			TCC0_CCB_vect
#define TAU3_VECT			TCC0_CCC_vect

#define CLOCK1				TCD0
#define CLOCK2				TCF0
#define CLOCK3				TCF1

#define CLOCK1PIN			PORTD.PIN0CTRL
#define CLOCK2PIN			PORTF.PIN0CTRL
#define CLOCK3PIN			PORTF.PIN4CTRL

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
// Master clock timer initialization
// ===========================================================
void master_init(void)
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
void tau1_init(void) {
	
	// Set compare value
	MASTER.CCA = 0x0000;

	// Enable capture/compare channel A
	MASTER.CTRLB = MASTER.CTRLB | TC0_CCAEN_bm;

	//Enable compare channel A interrupt level high
	MASTER.INTCTRLB = ( MASTER.INTCTRLB & ~TC0_CCAINTLVL_gm) | TC_CCAINTLVL_HI_gc;

}

// ===========================================================
// CompareB initialization (tau2)
// ===========================================================
void tau2_init(void) {
	
	// Set compare value
	MASTER.CCB = 0x0000;

	// Enable capture/compare channel A
	MASTER.CTRLB = ( MASTER.CTRLB | TC0_CCBEN_bm);

	//Enable compare channel B interrupt level medium
	MASTER.INTCTRLB = ( MASTER.INTCTRLB & ~TC0_CCBINTLVL_gm) | TC_CCBINTLVL_MED_gc;

}

// ===========================================================
// CompareC initialization (tau3)
// ===========================================================
void tau3_init(void) {
	
	// Set compare value
	MASTER.CCC = 0x0000;

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
void clock1_init(void) {

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
void clock2_init(void) {

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
void clock3_init(void) {

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

// ===========================================================
// Update the PER and CCA registers of the specified clock's
// timer, to set the pulse width of the specified clock signal
//
// Parameters: (clock #, pulse width in microseconds)
//
// Example: set_pulse_width(1,350)
// ===========================================================
void set_pulse_width(uint8_t clocknumber, uint16_t pulse_width) {

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

	tau1_init();
	tau2_init();
	tau3_init();

	clock1_init();
	clock2_init();
	clock3_init();

	set_pulse_width(1,10);
	set_pulse_width(2,20);
	set_pulse_width(3,30);

	// Initialize master clock
	master_init();

	//Initialize USART
	usart_init();

	
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
