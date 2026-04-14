/*******************************************************************************
  SERCOM5 USART PLIB – ESP-01S WiFi Module Interface

  PB16 = SERCOM5 PAD0 (TX → ESP-01S RX) via MUX D
  PB17 = SERCOM5 PAD1 (RX ← ESP-01S TX) via MUX D
  Baud: 115200 @ GCLK0 = 48 MHz
  8N1, No parity, LSB first

  TrustZone note: Same secure-alias pattern as plib_sercom3_usart.c.
  The CPU runs in Secure state so SERCOM5_REGS (non-secure alias at
  0x42001800) is writable and functional.
*******************************************************************************/

#include "plib_sercom5_usart.h"
#include "device.h"

/* --------------------------------------------------------------------------
   Private helpers
   -------------------------------------------------------------------------- */

/* ~1 ms software loop @ 48 MHz (12000 NOP-equivalent iterations).
   We use our own copy so this file is self-contained. */
static void _s5_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
        for (volatile uint32_t j = 0; j < 12000UL; j++)
            ;
}

/* --------------------------------------------------------------------------
   SERCOM5_USART_Initialize
   -------------------------------------------------------------------------- */
void SERCOM5_USART_Initialize(void)
{
    /* --- PORT: PB16 = PAD0 (TX), PB17 = PAD1 (RX) — MUX D (value 3) ---
       PB16 / PB17 share PMUX[8]  (pin index / 2 = 16/2 = 8)            */
    PORT_SEC_REGS->GROUP[1].PORT_PMUX[8] =
        PORT_PMUX_PMUXE(3) |   /* PB16 → SERCOM5 PAD0 (TX) */
        PORT_PMUX_PMUXO(3);    /* PB17 → SERCOM5 PAD1 (RX) */

    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[16] = PORT_PINCFG_PMUXEN_Msk;
    PORT_SEC_REGS->GROUP[1].PORT_PINCFG[17] = PORT_PINCFG_PMUXEN_Msk;

    /* --- MCLK: enable APB clock for SERCOM5 (bit 6 in APBCMASK) ---
       Pattern confirmed from SERCOM3 (bit 4).  SERCOM4 = bit 5, SERCOM5 = bit 6 */
    MCLK_REGS->MCLK_APBCMASK |= (1u << 6);

    /* --- GCLK: enable GCLK_PCHCTRL[22] for SERCOM5 core clock (GEN 0) ---
       Pattern: SERCOM3 = PCHCTRL[20], SERCOM4 = [21], SERCOM5 = [22]   */
    GCLK_REGS->GCLK_PCHCTRL[22] = 0x40U; /* CHEN | GEN(0) */
    while ((GCLK_REGS->GCLK_PCHCTRL[22] & 0x40U) == 0)
        ;

    /* --- Reset SERCOM5 --- */
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA = SERCOM_USART_INT_CTRLA_SWRST_Msk;
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY)
        ;

    /* --- CTRLB: 8-bit, TX + RX enabled --- */
    SERCOM5_REGS->USART_INT.SERCOM_CTRLB =
        SERCOM_USART_INT_CTRLB_TXEN_Msk |
        SERCOM_USART_INT_CTRLB_RXEN_Msk |
        SERCOM_USART_INT_CTRLB_CHSIZE(0);  /* CHSIZE(0) = 8-bit */
    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY)
        ;

    /* --- BAUD: 115200 @ 48 MHz, 16x oversampling ---
       BAUD = 65536 * (1 - 16 * 115200 / 48000000)
            = 65536 * 0.96160
            = 63019                                                      */
    SERCOM5_REGS->USART_INT.SERCOM_BAUD = (uint16_t)63019U;

    /* --- CTRLA: internal clock, LSB first, TX=PAD0, RX=PAD1, async, enable --- */
    SERCOM5_REGS->USART_INT.SERCOM_CTRLA =
        SERCOM_USART_INT_CTRLA_MODE(1)   |  /* USART with internal clock */
        SERCOM_USART_INT_CTRLA_DORD_Msk  |  /* LSB first                 */
        SERCOM_USART_INT_CTRLA_TXPO(0)   |  /* TX on PAD0 (PB16)         */
        SERCOM_USART_INT_CTRLA_RXPO(1)   |  /* RX on PAD1 (PB17)         */
        SERCOM_USART_INT_CTRLA_ENABLE_Msk;

    while (SERCOM5_REGS->USART_INT.SERCOM_SYNCBUSY)
        ;
}

/* --------------------------------------------------------------------------
   TX helpers
   -------------------------------------------------------------------------- */
bool SERCOM5_USART_WriteIsBusy(void)
{
    return !(SERCOM5_REGS->USART_INT.SERCOM_INTFLAG &
             SERCOM_USART_INT_INTFLAG_DRE_Msk);
}

bool SERCOM5_USART_Write(const uint8_t *pBuffer, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        /* Wait until Data Register Empty */
        while (!(SERCOM5_REGS->USART_INT.SERCOM_INTFLAG &
                 SERCOM_USART_INT_INTFLAG_DRE_Msk))
            ;
        SERCOM5_REGS->USART_INT.SERCOM_DATA = (uint16_t)pBuffer[i];
    }
    /* Wait until Transmit Complete */
    while (!(SERCOM5_REGS->USART_INT.SERCOM_INTFLAG &
             SERCOM_USART_INT_INTFLAG_TXC_Msk))
        ;
    return true;
}

/* --------------------------------------------------------------------------
   RX helpers
   -------------------------------------------------------------------------- */
bool SERCOM5_USART_ReadDataIsReady(void)
{
    return (SERCOM5_REGS->USART_INT.SERCOM_INTFLAG &
            SERCOM_USART_INT_INTFLAG_RXC_Msk) != 0U;
}

uint8_t SERCOM5_USART_ReadByte(void)
{
    return (uint8_t)(SERCOM5_REGS->USART_INT.SERCOM_DATA & 0xFFU);
}

void SERCOM5_USART_FlushRx(void)
{
    while (SERCOM5_REGS->USART_INT.SERCOM_INTFLAG &
           SERCOM_USART_INT_INTFLAG_RXC_Msk)
    {
        (void)SERCOM5_REGS->USART_INT.SERCOM_DATA;
    }
}

uint16_t SERCOM5_USART_ReadLine(char *buf, uint16_t size, uint32_t timeout_ms)
{
    uint16_t   idx    = 0;
    uint32_t   ticks  = 0;
    const uint32_t tick_per_ms = 12000UL; /* ≈ loop iterations per ms  */
    uint32_t   deadline = timeout_ms * tick_per_ms;

    while (idx < (size - 1U))
    {
        if (SERCOM5_USART_ReadDataIsReady())
        {
            uint8_t b = SERCOM5_USART_ReadByte();
            buf[idx++] = (char)b;
            ticks = 0; /* reset timeout on each byte received */
            if (b == '\n')
                break;
        }
        else
        {
            ticks++;
            if (ticks >= deadline)
                break; /* timeout */
        }
    }
    buf[idx] = '\0';
    return idx;
}
