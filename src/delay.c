#include "config.h"

extern volatile uint32_t systick_counter;

uint32_t delayLoopCalibration=1000; /* guesstimate in case delay_init() not called */

void delay_init () {
	// Calibrate our simple spin-loop delay using the SysTick timer
	uint32_t startCalib = systick_counter;
	delay(1<<20);

	// Number of delay loop iterations per 10ms
	delayLoopCalibration =  1<<20 / (systick_counter - startCalib) ;

	// Number of ns per iteration
	//delayLoopCalibration = ((timeTick - startCalib) * 10000000) / 1<<20 ;

#ifdef FEATURE_SCT_TIMER
		Chip_SCT_Init(LPC_SCT);

		/* Stop the SCT before configuration */
		Chip_SCTPWM_Stop(LPC_SCT);

		// Match/capture mode register. (ref UM10800 section 16.6.11, Table 232, page 273)
		// Determines if match/capture operate as match or capture. Want all match.
		LPC_SCT->REGMODE_U = 0;

		// Set SCT Counter to count 32-bits and reset to 0 after reaching MATCH0
		Chip_SCT_Config(LPC_SCT, SCT_CONFIG_32BIT_COUNTER );

		Chip_SCT_ClearControl(LPC_SCT, SCT_CTRL_HALT_L | SCT_CTRL_HALT_H);
#endif

}

void delay_sct_clock_cycles (uint32_t delay) {
	uint32_t start = LPC_SCT->COUNT_U;
	while ( (LPC_SCT->COUNT_U - start) < delay) ;
}

void delay_deinit () {
	Chip_SCTPWM_Stop(LPC_SCT);
	Chip_SCT_DeInit(LPC_SCT);
}

/**
 * Short delay spin-loop.
 */
void delay (uint32_t d) {
	while (--d != 0) {
			__NOP();
	}
}

/**
 * Delay for t_ms milliseconds. Uses the SysTick timer which has a resolution of
 * 10ms, so < 10ms will result in no delay.
 */
void delayMilliseconds(uint32_t t_ms) {
	uint32_t t_cs = t_ms/10;
	if (t_cs==0) {
		return;
	}
	uint32_t end = systick_counter + t_cs;
	while (systick_counter != end) {
		// Sleep inbetween systicks
		__WFI();
	}
}

/**
 * Delay for t_us microseconds.
 */
void delayMicroseconds(uint32_t t_us) {
	// This calculation is going to be noticable for very short delays
	//uint32_t niter = (delayLoopCalibration*10000)/t_us;
	//uint32_t niter = (t_us * 1000) / delayLoopCalibration;
	//delay(niter);

#ifdef FEATURE_SCT_TIMER

	uint32_t clock_cycles = t_us * (Chip_Clock_GetSystemClockRate()/1000000);

	// TODO: can be optimized by triggering interrupt
	// then wait for interrupt instead of tight loop.
	uint32_t start = LPC_SCT->COUNT_U;
	while ( (LPC_SCT->COUNT_U - start) < clock_cycles) ;

#else
	uint32_t niter = t_us*35/10; // manual calibration
	delay(niter);
#endif

}

void delay_nop_loop (uint32_t i) {
	while (--i!=0) {
		__NOP();
	}
}

void loopDelay(uint32_t i) {
	while (--i!=0) {
		__NOP();
	}
}

/**
 * Use WKT (wake up timer) to delay. WKT is configured
 * to run at about 10kHz (+/- 40%).
 */
void wktDelay(uint32_t i) {
	LPC_WKT->COUNT = i;
	__WFI();
}
