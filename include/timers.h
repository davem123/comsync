#include "avr_compiler.h"

// Set CPU frequency = 32MHz
#ifndef F_CPU
	#define F_CPU 32000000UL
#endif

#define F_CPU_MHZ (uint8_t) (F_CPU / 1.0E6)

// ===========================================================
// Timer Configuration Macros
// ===========================================================
#define MASTER_OVF_VECT		TCC0_OVF_vect
#define MASTER				TCC0

#define TAU0_VECT			TCC0_CCA_vect
#define TAU1_VECT			TCC0_CCB_vect
#define TAU2_VECT			TCC0_CCC_vect
#define TAU3_VECT			TCC0_CCD_vect

#define CLOCK0				TCD0
#define CLOCK1				TCD1
#define CLOCK2				TCF0
#define CLOCK3				TCF1

#define CLOCK0PIN			PORTD.PIN0CTRL
#define CLOCK1PIN			PORTD.PIN4CTRL
#define CLOCK2PIN			PORTF.PIN0CTRL
#define CLOCK3PIN			PORTF.PIN4CTRL

// ===========================================================
// Function Prototypes
// ===========================================================
void timers_master_init(uint16_t period);

void timers_tau_init(	volatile uint16_t *addr_ccN,
						volatile uint8_t *addr_ctrlb,
						volatile uint8_t *addr_intctrlb,
						uint8_t capture_ch_bm,
						uint8_t interrupt_level_bm,
						uint16_t tau
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
