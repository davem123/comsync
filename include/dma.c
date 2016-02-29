#include "dma.h"

// ===========================================================
// DMA Controller setup
// ===========================================================

void dma_init(void) {

	// Disable the DMA controller while configuring
	DMA.CTRL &= ~DMA_ENABLE_bm;

	//====================
	//Tau0 DMA setup
	//====================
	// Repeat forever
	DMA.CH0.REPCNT = 0;
	
	// Enable Channel 0, single-shot transfer, 2-byte burst.
	DMA.CH0.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm | DMA_CH_SINGLE_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH0.TRFCNTL = 0x02;
	DMA.CH0.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by TCC0 CCA (TAU0_VECT) interrupt
	DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCA_gc;

	// Source: CLOCK0.CCA (TCD0.CCA)
	DMA.CH0.SRCADDR0 = ( ( (uint16_t) 0x0928) >> 0 ) & 0xFF;
	DMA.CH0.SRCADDR1 = ( ( (uint16_t) 0x0928) >> 8 ) & 0xFF;
	DMA.CH0.SRCADDR2 = 0;

	// Destination: CLOCK0.CNT (TCD0.CNT)
	DMA.CH0.DESTADDR0 = ( ( (uint16_t) 0x0920) >> 0 ) & 0xFF;
	DMA.CH0.DESTADDR1 = ( ( (uint16_t) 0x0920) >> 8 ) & 0xFF;
	DMA.CH0.DESTADDR2 = 0;


	//====================
	//Tau1 DMA setup
	//====================
	// Repeat forever
	DMA.CH1.REPCNT = 0;
	
	// Enable Channel 1, single-shot transfer, 2-byte burst.
	DMA.CH1.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH1.TRFCNTL = 0x02;
	DMA.CH1.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH1.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by TCC0 CCB (TAU1_VECT) interrupt
	DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCB_gc;

	// Source: CLOCK1.CCA (TCD1.CCA)
	DMA.CH1.SRCADDR0 = ( ( (uint16_t) 0x0968) >> 0 ) & 0xFF;
	DMA.CH1.SRCADDR1 = ( ( (uint16_t) 0x0968) >> 8 ) & 0xFF;
	DMA.CH1.SRCADDR2 = 0;

	// Destination: CLOCK1.CNT (TCD1.CNT)
	DMA.CH1.DESTADDR0 = ( ( (uint16_t) 0x0960) >> 0 ) & 0xFF;
	DMA.CH1.DESTADDR1 = ( ( (uint16_t) 0x0960) >> 8 ) & 0xFF;
	DMA.CH1.DESTADDR2 = 0;

	//====================
	//Tau2 DMA setup
	//====================
	// Repeat forever
	DMA.CH2.REPCNT = 0;
	
	// Enable Channel 2, single-shot transfer, 2-byte burst.
	DMA.CH2.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH2.TRFCNTL = 0x02;
	DMA.CH2.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH2.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by TCC0 CCC (TAU2_VECT) interrupt
	DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCC_gc;

	// Source: CLOCK2.CCA (TCF0.CCA)
	DMA.CH2.SRCADDR0 = ( ( (uint16_t) 0x0B28) >> 0 ) & 0xFF;
	DMA.CH2.SRCADDR1 = ( ( (uint16_t) 0x0B28) >> 8 ) & 0xFF;
	DMA.CH2.SRCADDR2 = 0;

	// Destination: CLOCK2.CNT (TCF0.CNT)
	DMA.CH2.DESTADDR0 = ( ( (uint16_t) 0x0B20) >> 0 ) & 0xFF;
	DMA.CH2.DESTADDR1 = ( ( (uint16_t) 0x0B20) >> 8 ) & 0xFF;
	DMA.CH2.DESTADDR2 = 0;

	//====================
	//Tau3 DMA setup
	//====================
	// Repeat forever
	DMA.CH3.REPCNT = 0;
	
	// Enable Channel 3, single-shot transfer, 2-byte burst.
	DMA.CH3.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH3.TRFCNTL = 0x02;
	DMA.CH3.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH3.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by TCC0 CCD (TAU3_VECT) interrupt
	DMA.CH3.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCD_gc;

	// Source: CLOCK3.CCA (TCF1.CCA)
	DMA.CH3.SRCADDR0 = ( ( (uint16_t) 0x0B68) >> 0 ) & 0xFF;
	DMA.CH3.SRCADDR1 = ( ( (uint16_t) 0x0B68) >> 8 ) & 0xFF;
	DMA.CH3.SRCADDR2 = 0;

	// Destination: CLOCK3.CNT (TCF1.CNT)
	DMA.CH3.DESTADDR0 = ( ( (uint16_t) 0x0B60) >> 0 ) & 0xFF;
	DMA.CH3.DESTADDR1 = ( ( (uint16_t) 0x0B60) >> 8 ) & 0xFF;
	DMA.CH3.DESTADDR2 = 0;

	// Enable DMA controller, double buffer enabled on ch0/1 and ch2/3, 
	// Channel priority: Ch0 > Ch1 > Ch2 > Ch3
	DMA.CTRL = DMA_ENABLE_bm | DMA_DBUFMODE_CH01CH23_gc | DMA_PRIMODE_CH0123_gc;
}
