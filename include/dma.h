#include "avr_compiler.h"

void dma_init(void);

// ===========================================================
// Timer Configuration Macros
// ===========================================================
#define CLOCK0_CCA_ADDR		0x0928	//TCD0
#define CLOCK1_CCA_ADDR		0x0968	//TCD1
#define CLOCK2_CCA_ADDR		0x0B28	//TCF0
#define CLOCK3_CCA_ADDR		0x0B68	//TCF1

#define CLOCK0_CNT_ADDR		0x0920	//TCD0
#define CLOCK1_CNT_ADDR		0x0960	//TCD1
#define CLOCK2_CNT_ADDR		0x0B20	//TCF0
#define CLOCK3_CNT_ADDR		0x0B60	//TCF1
