#include "LPC8xx.h"

/**
 * Initialize SwitchMatrix.
 *
 * UART RXD on SOIC package pin 19
 * UART TXD on SOIC package pin 5
 */
void SwitchMatrix_Init()
{
    /* Enable SWM clock */
    //LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

    /* Pin Assign 8 bit Configuration */
    /* U0_TXD */
    /* U0_RXD */
    LPC_SWM->PINASSIGN0 = 0xffff0004UL;

    /* Pin Assign 1 bit Configuration */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;

}

#ifdef FEATURE_GPS_ON_USART1
/**
 * In addition to assigning UART0 on the same pins as ISP TXD, RXD, also assign
 * UART1 to pins 8,9 to accommodate GPS receiver.
 *
 * UART0.RXD -> pin 19 (aka PIO0_0, ISP RXD)
 * UART0.TXD -> pin 5 (aka PIO0_4, ISP TXD)
 * UART1.RXD -> pin 8 (aka PIO0_11)
 * UART1.TXD -> pin 9 (aka PIO0_10)
 *
 * all other pins at default.
 */
void SwitchMatrix_GPS_UART1_Init()
{
       /* Enable the clock to the Switch Matrix */
       LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);

       /* Pin Assign 8 bit Configuration */
       /* U0_TXD */
       /* U0_RXD */
       LPC_SWM->PINASSIGN0 = 0xffff0004UL;
       /* U1_TXD */
       /* U1_RXD */
       LPC_SWM->PINASSIGN1 = 0xff0b0affUL;

       /* Pin Assign 1 bit Configuration */
       /* SWCLK */
       /* SWDIO */
       /* RESET */
       LPC_SWM->PINENABLE0 = 0xffffffb3UL;

}
#endif

#ifdef BOARD_V1B_HACK
/**
 * Hack to facilitate PCB v1 bug where via on RXD at PIO0_0 (package pin 19)
 * line touches Vdd rendering it useless (always high). There is an easy work around using
 * a knife to separate the via from Vdd, but in one case the was not done before
 * the LPC812 package was soldered in pace. The via contact is under the LPC812, so
 * making the cut was no longer possible. Rather
 * than scrap the board, the RXD line from the board UART connecter was routed to
 * PIO0_11 instead (package pin 8). So in this case want to route LPC812 UART RXD
 * to there. AFAIK, it is not possible to program this board using ISP mode. SWD
 * must be used. I'm going to call this v1b of the board. JD 20141117.
 */
void SwitchMatrix_LPC812_PCB1b_Init()
{
	// UART0 TXD to PIO0_4 (pin package 5). This is location for ISP programming
	// UART0 RXD rerouted to PIO0_11 (pin package 11).
    LPC_SWM->PINASSIGN0 = 0xffff0b04UL;

    /* Pin Assign 1 bit Configuration */
    /* SWCLK */
    /* SWDIO */
    /* RESET */
    LPC_SWM->PINENABLE0 = 0xffffffb3UL;
}
#endif

