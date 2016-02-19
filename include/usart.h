#include "avr_compiler.h"
#include "delays.h"

// ===========================================================
// Global variables
// ===========================================================
static volatile uint8_t usart_buffer[] = "EMPTYBUFFEREMPTYBUFFER";
static volatile uint8_t usart_counter = 0;

// ===========================================================
// USART config macros
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
// Function Prototypes
// ===========================================================
void usart_init(void);
void usart_rxbyte(uint8_t rxbyte);
void usart_parsebuffer(void);
