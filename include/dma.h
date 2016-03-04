#ifndef DMA_H
#define DMA_H

#include "avr_compiler.h"

void dma_init(void);

// ===========================================================
// Timer Configuration Macros
// ===========================================================
#define CLOCK0_CCA_ADDR		0x0868	//TCC1
#define CLOCK1_CCA_ADDR		0x0968	//TCD1
#define CLOCK2_CCA_ADDR		0x0A68	//TCE1
#define CLOCK3_CCA_ADDR		0x0B68	//TCF1

#define CLOCK0_CNT_ADDR		0x0860	//TCC1
#define CLOCK1_CNT_ADDR		0x0960	//TCD1
#define CLOCK2_CNT_ADDR		0x0A60	//TCE1
#define CLOCK3_CNT_ADDR		0x0B60	//TCF1

#endif
