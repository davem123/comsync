#include "usart.h"
#include "board.h"
#include "timers.h"
#include <string.h>

// ===========================================================
// Global variables
// ===========================================================
char usart_buffer[50];
volatile uint8_t usart_counter = 0;

// ===========================================================
// USART Initialization
// ===========================================================
void usart_init(void)
{

	USART_PORT.DIRSET = PIN3_bm;   // Pin 3 (TX) as output.
	USART_PORT.DIRCLR = PIN2_bm;   // Pin 2 (RX) as input.

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
// usart_buffer[] and calls usart_parsebuffer() if a
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
// Updates the tau1,tau2, and tau3 pulse trigger delays
// with the values in the parameters[] array
// ===========================================================
void usart_update_taus(uint32_t *parameters) {

	volatile uint32_t tau1 = parameters[1];
	volatile uint32_t tau2 = parameters[2];
	volatile uint32_t tau3 = parameters[3];

	timers_tau_init(	&MASTER.CCB,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCBEN_bm,			//Capture channel bitmask
						TC_CCBINTLVL_HI_gc,		//Interrupt level bitmask
						tau1					//Tau1 (trigger) delay (us)
					);
	timers_tau_init(	&MASTER.CCC,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCCEN_bm,			//Capture channel bitmask
						TC_CCCINTLVL_HI_gc,		//Interrupt level bitmask
						tau2					//Tau2 (trigger) delay (us)
					);

	timers_tau_init(	&MASTER.CCD,			//Address of CCP value
						&MASTER.CTRLB,			//Address of CTRLB
						&MASTER.INTCTRLB,		//Address of INTCTRLB
						TC0_CCDEN_bm,			//Capture channel bitmask
						TC_CCDINTLVL_HI_gc,		//Interrupt level bitmask
						tau3					//Tau3 (trigger) delay (us)
					);
}//end of usart_update_taus()

// ===========================================================
// Turns off all of the pulse outputs, including the master
// ===========================================================
void usart_disable_outputs(void) {
	
	MASTER.CTRLA = ( MASTER.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;

}

// ===========================================================
// Update the pulse widths of the three slave clock outputs
// ===========================================================
void usart_update_pulsewidths(uint32_t *parameters) {

	timers_set_pulse_width(	&CLOCK1.CCA,
							&CLOCK1.PER,
							parameters[1]);	

	timers_set_pulse_width(	&CLOCK2.CCA,
							&CLOCK2.PER,
							parameters[2]);	

	timers_set_pulse_width(	&CLOCK3.CCA,
							&CLOCK3.PER,
							parameters[3]);	

}

// ===========================================================
// usart_parsebuffer() processes the commands received on the
// USART and initiates the requested action.
// ===========================================================
void usart_parsebuffer(void){

	char *usart_substring[50];
	char command;

	uint32_t parameter_array[4];

	const char delimiter[] = ",";
	
	int i = -1;

	// disable USART interrupts while processing so the data
	// doesn't get corrupted
	USART.CTRLA &= USART_RXCINTLVL_OFF_gc;

	// Extract first substring (command)
	usart_substring[++i] = strtok(usart_buffer, delimiter);

	// Store the command as single ASCII character
	command = *usart_substring[0];

	// Extract remaining strings, convert to 32-bit long ints,
	// store in parameter_array[]
	while ( (usart_substring[++i] = strtok(NULL, delimiter)) != NULL)
	{
	   parameter_array[i] = atol(usart_substring[i]);
	}

	switch (command) {

		// Set the pulse width of the master clock
		// (CLOCK0)
		case 'M':
			timers_set_pulse_width(	&CLOCK0.CCA,
									&CLOCK0.PER,
									parameter_array[1] );
			break;
		
		// Restart MASTER with the specified period
		// eg. "R,100000" for 100ms
		case 'R':
			timers_master_init(parameter_array[1]);
			break;

		case 'T':
			usart_update_taus(parameter_array);
			break;

		case 'W':
			usart_update_pulsewidths(parameter_array);
			break;

		case 'X':
			usart_disable_outputs();
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

	// Clear the USART buffer by filling it with null characters
	for (int i=0; i<=50; i++){
		usart_buffer[i] = 0;
	}

	// Write the next byte to the beginning of the buffer
	usart_counter = 0;

	// Re-enable USART interrupts
	USART.CTRLA |= USART_RXCINTLVL_LO_gc;

}//end of usart_parsebuffer()
