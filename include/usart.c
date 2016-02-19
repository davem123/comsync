#include "usart.h"
#include "board.h"
#include "timers.h"

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
