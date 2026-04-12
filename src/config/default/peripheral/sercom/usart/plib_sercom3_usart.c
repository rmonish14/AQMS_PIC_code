/*******************************************************************************
  SERCOM3 USART PLIB

  PB08 = SERCOM3 PAD0 (TX) via MUX D
  PB09 = SERCOM3 PAD1 (RX) via MUX D
  Baud: 9600 @ GCLK0 = 48 MHz
  
  Uses SERCOM3_REGS (non-secure alias 0x42001000) — same pattern as the
  MCC-generated SERCOM1_SPI driver which also uses non-secure aliases and
  works correctly because the CPU executes in Secure state.
*******************************************************************************/

#include "plib_sercom3_usart.h"
#include "device.h"

void SERCOM3_USART_Initialize(void)
{
    /* PORT: PB08=PAD0(TX), PB09=PAD1(RX) — MUX D = 3 */
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[4] = PORT_PMUX_PMUXE(3) | PORT_PMUX_PMUXO(3);
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[8] = PORT_PINCFG_PMUXEN_Msk;
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[9] = PORT_PINCFG_PMUXEN_Msk;

    /* MCLK: enable APB clock for SERCOM3 (bit 4) */
    MCLK_REGS->MCLK_APBCMASK |= (1u << 4);

    /* GCLK_PCHCTRL[20] already enabled in plib_clock.c */

    /* Reset and configure SERCOM3 as USART */
    SERCOM3_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
    while (SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY);

    /* Selection of the Character Size and Receiver Enable */
    SERCOM3_REGS->USART_INT.SERCOM_CTRLB =
        SERCOM_USART_INT_CTRLB_TXEN_Msk |
        SERCOM_USART_INT_CTRLB_RXEN_Msk |
        SERCOM_USART_INT_CTRLB_CHSIZE(0);  /* 8-bit */

    while (SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY);

    /* 9600 baud */
    SERCOM3_REGS->USART_INT.SERCOM_BAUD = (uint16_t)65326U;

    /* CTRLA: internal clock, LSB first, TX=PAD0, RX=PAD1, async, 8N1, enable */
    SERCOM3_REGS->USART_INT.SERCOM_CTRLA =
        SERCOM_USART_INT_CTRLA_MODE(1)   |
        SERCOM_USART_INT_CTRLA_DORD_Msk  |
        SERCOM_USART_INT_CTRLA_TXPO(0)   |
        SERCOM_USART_INT_CTRLA_RXPO(1)   |
        SERCOM_USART_INT_CTRLA_ENABLE_Msk;

    while (SERCOM3_REGS->USART_INT.SERCOM_SYNCBUSY);
}

bool SERCOM3_USART_WriteIsBusy(void)
{
    return !(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk);
}

bool SERCOM3_USART_Write(uint8_t *pBuffer, const size_t size)
{
    size_t i;
    for (i = 0; i < size; i++)
    {
        while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_DRE_Msk));
        SERCOM3_REGS->USART_INT.SERCOM_DATA = (uint16_t)pBuffer[i];
    }
    while (!(SERCOM3_REGS->USART_INT.SERCOM_INTFLAG & SERCOM_USART_INT_INTFLAG_TXC_Msk));
    return true;
}
