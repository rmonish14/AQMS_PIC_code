// --------------------------------------------------------------------------
// ST7735-library (hw-specific defines and interfaces)
//
// Ported for PIC32CM5164LS00048 (ARM Cortex-M23) using Harmony PLIB.
//
// Pin mapping:
//   SDA (MOSI) -> PA08  (SERCOM1 PAD[0], handled by PLIB)
//   SCK        -> PA09  (SERCOM1 PAD[1], handled by PLIB)
//   CS         -> PA10  (GPIO output)
//   DC         -> PA11  (GPIO output)
//   RESET      -> PA12  (GPIO output)
//
// https://github.com/bablokb/pic-st7735
// --------------------------------------------------------------------------

#ifndef _HW_H
#define _HW_H

// ----------------------------------------------------------------
// Necessary includes for PIC32CM HAL
// ----------------------------------------------------------------
#include <stdint.h>
#include "definitions.h"   /* Pulls in plib_port.h, SERCOM regs, etc. */
#include "delay.h"

// ----------------------------------------------------------------
// Feature flags for the ST7735 library
// ----------------------------------------------------------------
#define TFT_ENABLE_BLACK    /* Use Black-tab (128x160) init sequence */
#define TFT_ENABLE_RESET    /* Use hardware RESET pin                */
#define TFT_ENABLE_TEXT     /* Enable text / drawText support         */
#define TFT_ENABLE_SHAPES   /* Enable shape drawing                   */

// ----------------------------------------------------------------
// SPI write: SYNCHRONOUS single-byte send via SERCOM1.
//
// The Harmony PLIB SERCOM1_SPI_Write() is interrupt-driven and
// non-blocking — it returns before the byte is clocked out.
// The ST7735 library sends bytes one at a time in tight loops,
// so we MUST use a blocking, polling approach instead.
// ----------------------------------------------------------------
static inline void _hw_spiwrite(uint8_t data) {
    /* Wait until the Data Register is Empty (ready for new data) */
    while ((SERCOM1_REGS->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_DRE_Msk) == 0U)
        ;

    /* Write the byte — this starts the SPI clock */
    SERCOM1_REGS->SPIM.SERCOM_DATA = data;

    /* Wait until Transmit Complete (byte fully shifted out) */
    while ((SERCOM1_REGS->SPIM.SERCOM_INTFLAG & SERCOM_SPIM_INTFLAG_TXC_Msk) == 0U)
        ;

    /* Read back dummy data to clear RXC flag and prevent overflow */
    (void)SERCOM1_REGS->SPIM.SERCOM_DATA;
}
#define spiwrite(data)    _hw_spiwrite(data)

// ----------------------------------------------------------------
// GPIO control: CS -> PA10
// ----------------------------------------------------------------
#define tft_cs_low()      GPIO_PA10_Clear()
#define tft_cs_high()     GPIO_PA10_Set()

// ----------------------------------------------------------------
// GPIO control: DC -> PA11
// ----------------------------------------------------------------
#define tft_dc_low()      GPIO_PA11_Clear()
#define tft_dc_high()     GPIO_PA11_Set()

// ----------------------------------------------------------------
// GPIO control: RESET -> PA12
// ----------------------------------------------------------------
#define tft_rst_low()     GPIO_PA12_Clear()
#define tft_rst_high()    GPIO_PA12_Set()

// ----------------------------------------------------------------
// Delay: map library __delay_ms() to our delay_ms()
// ----------------------------------------------------------------
#define __delay_ms(x)     delay_ms((uint8_t)(x))

#endif /* _HW_H */
