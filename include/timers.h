#ifndef TIMERS_H
#define TIMERS_H

#include "avr_compiler.h"

// Set CPU frequency = 32MHz
#ifndef F_CPU
	#define F_CPU 32000000UL
#endif

#define F_CPU_MHZ (uint8_t) (F_CPU / 1.0E6)

// ===========================================================
// Timer Configuration Macros
// ===========================================================
#define MASTERL_OVF_VECT	TCC0_OVF_vect
#define MASTERH_OVF_VECT	TCD0_OVF_vect
#define MASTERL				TCC0
#define MASTERH				TCD0

#define TAU0L_VECT			TCC0_CCA_vect
#define TAU1L_VECT			TCC0_CCB_vect
#define TAU2L_VECT			TCC0_CCC_vect
#define TAU3L_VECT			TCC0_CCD_vect

#define TAU0H_VECT			TCD0_CCA_vect
#define TAU1H_VECT			TCD0_CCB_vect
#define TAU2H_VECT			TCD0_CCC_vect
#define TAU3H_VECT			TCD0_CCD_vect

#define CLOCK0				TCC1
#define CLOCK1				TCD1
#define CLOCK2				TCE1
#define CLOCK3				TCF1

#define COUNTER1			TCE0
#define COUNTER1_OVF_VECT	TCE0_OVF_vect

#define COUNTER2			TCF0
#define COUNTER2_OVF_VECT	TCF0_OVF_vect

#define CLOCK0PIN			PORTC.PIN4CTRL
#define CLOCK1PIN			PORTD.PIN4CTRL
#define CLOCK2PIN			PORTE.PIN4CTRL
#define CLOCK3PIN			PORTF.PIN4CTRL

#define MAX_PERIOD_MICROSECONDS	127000

// 31.25ns = 1/32MHz
#define TIMER_PERIOD		3.125E-2


// ===========================================================
// Global Variables
// ===========================================================
static const uint8_t TAU0offset = 0x28;
static const uint8_t TAU1offset = 0x2A;
static const uint8_t TAU2offset = 0x2C;
static const uint8_t TAU3offset = 0x2E;

static const uint8_t TAU0_CCA_bm = 0x10;
static const uint8_t TAU1_CCB_bm = 0x20;
static const uint8_t TAU2_CCC_bm = 0x40;
static const uint8_t TAU3_CCD_bm = 0x80;


// ===========================================================
// Function Prototypes
// ===========================================================
void timers_master_init32(uint32_t period_us);

void timers_tau_init32(	uint8_t capture_ch_bm,
						uint8_t tau_addr_offset,
						uint32_t tau_us
					);

void timers_init_clock(	volatile uint16_t *addr_per,
						volatile uint16_t *addr_cca,
						volatile uint8_t *addr_clockpin,
						volatile uint8_t *addr_ctrla,
						volatile uint8_t *addr_ctrlb,
						volatile uint8_t *addr_ctrld,
						uint8_t clksel_bm
					);

void timers_set_pulse_width(volatile uint16_t *addr_cca,
							volatile uint16_t *addr_per,
							uint16_t pulse_width
							);

#endif
