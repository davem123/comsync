#ifndef USART_H
#define USART_H

#include "avr_compiler.h"
#include "delays.h"
#include <board.h>

#define USBCOM
//#define J1COM

#define LEDPORT PORTQ

// ===========================================================
// USART config macros
// ===========================================================
#ifdef USBCOM
	#define USART			USARTE0				//use for USB-Virtual COM Port
	#define USART_PORT		PORTE				//use for USB-Virtual COM Port
	#define USART_VECT		USARTE0_RXC_vect	//use for USB-Virtual COM Port
#endif

#ifdef J1COM
	#define USART			USARTF0				//use for J1 Header
	#define USART_PORT	PORTF					//use for J1 Header
	#define USART_VECT		USARTF0_RXC_vect	//use for J1 Header
#endif

#define BSCALE_VALUE  4
#define BSEL_VALUE   12	//prescalers for 32MHz clock to get 9600 baudrate 

// ===========================================================
// Function Prototypes
// ===========================================================
void usart_init(void);
void usart_rxbyte(uint8_t rxbyte);
void usart_parsebuffer(void);

void usart_update_taus32(uint32_t *parameters);

void usart_disable_outputs32(void);

void usart_update_pulsewidths(uint32_t *parameters);

#endif
