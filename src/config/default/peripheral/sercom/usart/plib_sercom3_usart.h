/*******************************************************************************
  SERCOM3 USART PLIB (Secure Wrapper for mplabnew_secure)
  Matches API of MCC-generated plib_sercom3_usart.h so user reference code
  (SERCOM3_USART_Write / SERCOM3_USART_WriteIsBusy) works unchanged.
*******************************************************************************/

#ifndef PLIB_SERCOM3_USART_H
#define PLIB_SERCOM3_USART_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void SERCOM3_USART_Initialize(void);
bool SERCOM3_USART_Write(uint8_t *pBuffer, const size_t size);
bool SERCOM3_USART_WriteIsBusy(void);

#ifdef __cplusplus
}
#endif

#endif /* PLIB_SERCOM3_USART_H */
