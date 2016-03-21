#include "dma.h"
#include "timers.h"

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
	
	// Enable Channel 0, repeat mode, 2-byte burst.
	DMA.CH0.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm | DMA_CH_SINGLE_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH0.TRFCNTL = 0x02;
	DMA.CH0.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH0.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by Timer overflow interrupt
	// Select the most significant timer
	if (MASTERH.PER > 0) {
		DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_TCD0_CCA_gc;
	}
	else{
		DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCA_gc;
	}

	// Source: CLOCK0.CCA
	DMA.CH0.SRCADDR0 = ( ( (uint16_t) CLOCK0_CCA_ADDR) >> 0 ) & 0xFF;
	DMA.CH0.SRCADDR1 = ( ( (uint16_t) CLOCK0_CCA_ADDR) >> 8 ) & 0xFF;
	DMA.CH0.SRCADDR2 = 0;

	// Destination: CLOCK0.CNT
	DMA.CH0.DESTADDR0 = ( ( (uint16_t) CLOCK0_CNT_ADDR) >> 0 ) & 0xFF;
	DMA.CH0.DESTADDR1 = ( ( (uint16_t) CLOCK0_CNT_ADDR) >> 8 ) & 0xFF;
	DMA.CH0.DESTADDR2 = 0;


	//====================
	//Tau1 DMA setup
	//====================
	// Repeat forever
	DMA.CH1.REPCNT = 0;
	
	// Enable Channel 1, repeat mode, 2-byte burst.
	DMA.CH1.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH1.TRFCNTL = 0x02;
	DMA.CH1.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH1.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by CCB (TAU1_VECT) interrupt
	// Select whichever MASTER timer has CCB enabled
	if (MASTERH.CTRLB & TC0_CCBEN_bm) {
		DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_TCD0_CCB_gc;
	}
	else if (MASTERL.CTRLB & TC0_CCBEN_bm){
		DMA.CH1.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCB_gc;
	}

	// Source: CLOCK1.CCA
	DMA.CH1.SRCADDR0 = ( ( (uint16_t) CLOCK1_CCA_ADDR) >> 0 ) & 0xFF;
	DMA.CH1.SRCADDR1 = ( ( (uint16_t) CLOCK1_CCA_ADDR) >> 8 ) & 0xFF;
	DMA.CH1.SRCADDR2 = 0;

	// Destination: CLOCK1.CNT
	DMA.CH1.DESTADDR0 = ( ( (uint16_t) CLOCK1_CNT_ADDR) >> 0 ) & 0xFF;
	DMA.CH1.DESTADDR1 = ( ( (uint16_t) CLOCK1_CNT_ADDR) >> 8 ) & 0xFF;
	DMA.CH1.DESTADDR2 = 0;

	//====================
	//Tau2 DMA setup
	//====================
	// Repeat forever
	DMA.CH2.REPCNT = 0;
	
	// Enable Channel 2, repeat mode, 2-byte burst.
	DMA.CH2.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH2.TRFCNTL = 0x02;
	DMA.CH2.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH2.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by CCC (TAU2_VECT) interrupt
	// Select whichever MASTER timer has CCC enabled
	if (MASTERH.CTRLB & TC0_CCCEN_bm) {
		DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_TCD0_CCC_gc;
	}
	else if (MASTERL.CTRLB & TC0_CCCEN_bm){
		DMA.CH2.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCC_gc;
	}

	// Source: CLOCK2.CCA
	DMA.CH2.SRCADDR0 = ( ( (uint16_t) CLOCK2_CCA_ADDR) >> 0 ) & 0xFF;
	DMA.CH2.SRCADDR1 = ( ( (uint16_t) CLOCK2_CCA_ADDR) >> 8 ) & 0xFF;
	DMA.CH2.SRCADDR2 = 0;

	// Destination: CLOCK2.CNT
	DMA.CH2.DESTADDR0 = ( ( (uint16_t) CLOCK2_CNT_ADDR) >> 0 ) & 0xFF;
	DMA.CH2.DESTADDR1 = ( ( (uint16_t) CLOCK2_CNT_ADDR) >> 8 ) & 0xFF;
	DMA.CH2.DESTADDR2 = 0;

	//====================
	//Tau3 DMA setup
	//====================
	// Repeat forever
	DMA.CH3.REPCNT = 0;
	
	// Enable Channel 3, repeat mode, 2-byte burst.
	DMA.CH3.CTRLA = DMA_CH_ENABLE_bm | DMA_CH_BURSTLEN_2BYTE_gc | DMA_CH_REPEAT_bm;

	// Set "transfer count" aka number of bytes per block transfer
	DMA.CH3.TRFCNTL = 0x02;
	DMA.CH3.TRFCNTH = 0x00;

	// Reload source and destination address after every block, increment source and destination address
	DMA.CH3.ADDRCTRL = DMA_CH_SRCRELOAD_BLOCK_gc | DMA_CH_SRCDIR_INC_gc | DMA_CH_DESTRELOAD_BLOCK_gc | DMA_CH_DESTDIR_INC_gc;

	// DMA transfer triggered by CCD (TAU3_VECT) interrupt
	// Select whichever MASTER timer has CCD enabled
	if (MASTERH.CTRLB & TC0_CCDEN_bm) {
		DMA.CH3.TRIGSRC = DMA_CH_TRIGSRC_TCD0_CCD_gc;
	}
	else if (MASTERL.CTRLB & TC0_CCDEN_bm){
		DMA.CH3.TRIGSRC = DMA_CH_TRIGSRC_TCC0_CCD_gc;
	}

	// Source: CLOCK3.CCA
	DMA.CH3.SRCADDR0 = ( ( (uint16_t) CLOCK3_CCA_ADDR) >> 0 ) & 0xFF;
	DMA.CH3.SRCADDR1 = ( ( (uint16_t) CLOCK3_CCA_ADDR) >> 8 ) & 0xFF;
	DMA.CH3.SRCADDR2 = 0;

	// Destination: CLOCK3.CNT
	DMA.CH3.DESTADDR0 = ( ( (uint16_t) CLOCK3_CNT_ADDR) >> 0 ) & 0xFF;
	DMA.CH3.DESTADDR1 = ( ( (uint16_t) CLOCK3_CNT_ADDR) >> 8 ) & 0xFF;
	DMA.CH3.DESTADDR2 = 0;

	// Enable DMA controller, double buffer enabled on ch0/1 and ch2/3, 
	// Channel priority: Ch0 > Ch1 > Ch2 > Ch3
	DMA.CTRL = DMA_ENABLE_bm | DMA_DBUFMODE_DISABLED_gc | DMA_PRIMODE_CH0123_gc;
}
