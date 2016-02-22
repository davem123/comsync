#include "usart.h"
#include "board.h"
#include "timers.h"

// ===========================================================
// Global variables
// ===========================================================
volatile uint8_t usart_buffer[] = "EMPTYBUFFEREMPTYBUFFER";
volatile uint8_t usart_counter = 0;

// ===========================================================
// USART Initialization
// ===========================================================
void usart_init(void)
{

	USART_PORT.DIRSET = PIN3_bm;   // Pin 3 (TX) as output.
	USART_PORT.DIRCLR = PIN2_bm;   // Pin 2 (RX) as input.

	// USART, 8 Data bits, Even Parity, 1 Stop bit, Asynchronous
//	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | false;
//	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc | USART_CMODE_ASYNCHRONOUS_gc;

	// USART, 8 Data bits, No Parity, Asynchronous
	USART.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_CMODE_ASYNCHRONOUS_gc;

	USART.BAUDCTRLA = BSEL_VALUE;	
	USART.BAUDCTRLB = BSCALE_VALUE << 4;

	// Set receive complete interrupt level low
	USART.CTRLA |= USART_RXCINTLVL_LO_gc;

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

uint16_t *usart_command_t(void) {

	static uint16_t tau_ints[3];

	uint32_t tau1,tau2,tau3;

	//Convert the ASCII data values to 16-bit integers,
	//then return as a 1D array

	return tau_ints;
}

// ===========================================================
// usart_parsebuffer() processes the commands received on the
// USART and initiates the requested action.
// ===========================================================
void usart_parsebuffer(void){

	// disable USART interrupts while processing so the data
	// doesn't get corrupted
	USART.CTRLA &= USART_RXCINTLVL_OFF_gc;

	unsigned char command = usart_buffer[0];

	switch (command) {

		case 'T':
			usart_command_t();
			break;
		case 'X':
			// disable all of the outputs
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
	
	// Write the next byte to the beginning of the buffer
	usart_counter = 0;

	// Re-enable USART interrupts
	USART.CTRLA |= USART_RXCINTLVL_LO_gc;

}//end of usart_parsebuffer()
