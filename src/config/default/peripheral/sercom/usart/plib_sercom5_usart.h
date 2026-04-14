/*******************************************************************************
  SERCOM5 USART PLIB – ESP-01S WiFi Module Interface

  PB16 = SERCOM5 PAD0 (TX → ESP-01S RX) via MUX D
  PB17 = SERCOM5 PAD1 (RX ← ESP-01S TX) via MUX D
  Baud: 115200 @ GCLK0 = 48 MHz  (ESP-01S AT firmware default)

  Uses same TrustZone-safe pattern as plib_sercom3_usart.c:
  SERCOM5_REGS (non-secure alias 0x42001800) works because the CPU
  executes in Secure state and the NONSECC_SERCOM5 fuse = CLEAR.
*******************************************************************************/

#ifndef PLIB_SERCOM5_USART_H
#define PLIB_SERCOM5_USART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Init ---------------------------------------------------------------- */
void SERCOM5_USART_Initialize(void);

/* ---- TX ------------------------------------------------------------------ */
/**
 * Returns true while the TX data register is NOT ready
 * (i.e. "is busy" = caller should wait).
 */
bool SERCOM5_USART_WriteIsBusy(void);

/**
 * Blocking byte-by-byte write; waits for DRE before each byte,
 * then waits for TXC after the last byte. Returns true on success.
 */
bool SERCOM5_USART_Write(const uint8_t *pBuffer, size_t size);

/* ---- RX ------------------------------------------------------------------ */
/**
 * Returns true if an unread byte is waiting in the RX register.
 */
bool SERCOM5_USART_ReadDataIsReady(void);

/**
 * Read one byte (call only after ReadDataIsReady returns true).
 */
uint8_t SERCOM5_USART_ReadByte(void);

/**
 * Read characters into buf until '\n' is found, timeout_ms elapses,
 * or buf is full (size-1).  Always NUL-terminates buf.
 * Returns number of bytes written (excluding NUL).
 */
uint16_t SERCOM5_USART_ReadLine(char *buf, uint16_t size, uint32_t timeout_ms);

/**
 * Discard any data currently in the RX register.
 */
void SERCOM5_USART_FlushRx(void);

#ifdef __cplusplus
}
#endif

#endif /* PLIB_SERCOM5_USART_H */
