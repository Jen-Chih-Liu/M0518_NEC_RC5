/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/12/25 10:24a $
 * @brief    Show the usage of GPIO interrupt function.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0518.h"


#define PLL_CLOCK   50000000


/**
 * @brief       GPIO PA/PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA/PB default IRQ, declared in startup_M0518.s.
 */
unsigned char temp[64] = {0};
unsigned char ir_time = 0;
unsigned char ir_count = 0;
unsigned char start_flag = 0;
unsigned char ir_ready = 0;
void GPAB_IRQHandler(void)
{

    if (GPIO_GET_INT_FLAG(PA, BIT5))
    {
        GPIO_CLR_INT_FLAG(PA, BIT5);
    }

    if ((ir_time > 50) && (ir_time < 75))
    {
        start_flag = 1;
        goto  first;
    }

    if (start_flag == 1)
    {
        temp[ir_count] = ir_time;
        ir_count = ir_count + 1;

        if (ir_count >= 32)
        {
            ir_count = 0;
            start_flag = 0;
            ir_ready = 1;
        }
    }

first:
    ir_time = 0;
}

unsigned int parserir(void)
{
	  unsigned int ir_data=0;
    unsigned char address = 0;
    unsigned char i_address = 0;
    unsigned char data = 0;
    unsigned char i_data = 0;
    unsigned char i;

    for (i = 0; i < 8; i++)
    {
        address = address << 1;

        if (temp[i] > 8)
            address = address | 0x01;

    }

    for (i = 8; i < 16; i++)
    {
        i_address = i_address << 1;

        if (temp[i] > 8)
            i_address = i_address | 0x01;

    }

    for (i = 16; i < 24; i++)
    {
        data = data << 1;

        if (temp[i] > 8)
            data = data | 0x01;

    }

    for (i = 24; i < 32; i++)
    {
        i_data = i_data << 1;

        if (temp[i] > 8)
            i_data = i_data | 0x01;
    }
			  ir_data=(address<<24)|(i_address<<16)|(data<<8)|i_data;
    
		return ir_data;
}
void TMR1_IRQHandler(void)
{

    /* Clear Timer1 time-out interrupt flag */
    TIMER_ClearIntFlag(TIMER1);

    ir_time++;

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HXT, 0);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();


    /* Configure PB.3 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT5, GPIO_PMD_INPUT);
    GPIO_EnableInt(PA, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPAB_IRQn);

    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 5000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);
    TIMER_Start(TIMER1);


    /* Waiting for interrupts */
    while (1)
    {
        if (ir_ready == 1)
        {
            parserir();
            ir_ready = 0;
				
        }
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
