/*******************************************************************************
  ESP8266 (ESP-01S) AT Command Driver — Implementation
  
  Hardware: SERCOM5  PB16=TX(to ESP RX)  PB17=RX(from ESP TX)  115200 8N1
*******************************************************************************/

#include "esp8266_at.h"

/* Unity-build: compile SERCOM5 USART PLIB into this translation unit so the
   linker finds SERCOM5_USART_* symbols without adding the file to the
   MPLAB project Makefile manually. Same pattern used in initialization.c
   for SERCOM3.                                                               */
#include "config/default/peripheral/sercom/usart/plib_sercom5_usart.c"
#include <string.h>
#include <stdio.h>

/* ============================================================
   Private delay (self-contained, ~1 ms per loop @ 48 MHz)
   ============================================================ */
static void _esp_delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
        for (volatile uint32_t j = 0; j < 12000UL; j++)
            ;
}

/* ============================================================
   Private: check if 'haystack' contains 'needle'
   ============================================================ */
static bool _contains(const char *haystack, const char *needle)
{
    return strstr(haystack, needle) != NULL;
}

/* ============================================================
   Low-level send helpers
   ============================================================ */
void ESP8266_SendRaw(const char *str)
{
    SERCOM5_USART_Write((const uint8_t *)str, strlen(str));
}

void ESP8266_SendCmd(const char *cmd)
{
    SERCOM5_USART_Write((const uint8_t *)cmd,  strlen(cmd));
    SERCOM5_USART_Write((const uint8_t *)"\r\n", 2U);
}

/* ============================================================
   WaitFor — polls RX lines until keyword found or timeout
   ============================================================ */
ESP_Result_t ESP8266_WaitFor(const char *keyword, uint32_t timeout_ms)
{
    char line[128];
    uint32_t elapsed = 0;
    const uint32_t POLL_MS = 5U;  /* check every 5 ms */

    while (elapsed < timeout_ms)
    {
        uint16_t n = SERCOM5_USART_ReadLine(line, sizeof(line), POLL_MS);
        elapsed += POLL_MS;

        if (n > 0U)
        {
            if (_contains(line, keyword))
                return ESP_OK;
            if (_contains(line, "ERROR") || _contains(line, "FAIL"))
                return ESP_ERROR;
        }
    }
    return ESP_TIMEOUT;
}

/* ============================================================
   ESP8266_Init
   Sequence:
     1. Init SERCOM5
     2. AT — basic echo check (retry 3×)
     3. ATE0 — turn off echo
     4. AT+CWMODE=1 — station only
   ============================================================ */
ESP_Result_t ESP8266_Init(void)
{
    SERCOM5_USART_Initialize();
    _esp_delay_ms(500U);            /* give module time after power-on */
    SERCOM5_USART_FlushRx();

    /* --- step 1: verify communication (up to 3 retries) --- */
    for (uint8_t retry = 0; retry < 3U; retry++)
    {
        ESP8266_SendCmd("AT");
        if (ESP8266_WaitFor("OK", 1000U) == ESP_OK)
            goto at_ok;
        _esp_delay_ms(200U);
    }
    return ESP_TIMEOUT;   /* no response from module */

at_ok:
    /* --- step 2: disable echo so responses are easier to parse --- */
    ESP8266_SendCmd("ATE0");
    ESP8266_WaitFor("OK", 500U);   /* ignore result; optional */

    /* --- step 3: set station mode --- */
    ESP8266_SendCmd("AT+CWMODE=1");
    if (ESP8266_WaitFor("OK", 2000U) != ESP_OK)
        return ESP_ERROR;

    /* --- step 4: allow multiple connections (needed for CIPSEND) --- */
    ESP8266_SendCmd("AT+CIPMUX=0");
    ESP8266_WaitFor("OK", 1000U);

    return ESP_OK;
}

/* ============================================================
   ESP8266_ConnectWiFi
   Sends:  AT+CWJAP="<ssid>","<password>"
   Waits for "WIFI GOT IP" (up to 20 s)
   ============================================================ */
ESP_Result_t ESP8266_ConnectWiFi(const char *ssid, const char *password)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);

    SERCOM5_USART_FlushRx();
    ESP8266_SendCmd(cmd);

    /* First wait for "WIFI CONNECTED" then "WIFI GOT IP" */
    if (ESP8266_WaitFor("WIFI CONNECTED", 15000U) != ESP_OK)
        return ESP_TIMEOUT;

    if (ESP8266_WaitFor("WIFI GOT IP", 10000U) != ESP_OK)
        return ESP_NO_IP;

    return ESP_OK;
}

/* ============================================================
   ESP8266_IsWiFiConnected
   Sends AT+CWJAP? and checks for a quoted SSID in the reply.
   ============================================================ */
bool ESP8266_IsWiFiConnected(void)
{
    SERCOM5_USART_FlushRx();
    ESP8266_SendCmd("AT+CWJAP?");
    /* If connected the reply contains "+CWJAP:" followed by the SSID */
    return (ESP8266_WaitFor("+CWJAP:", 2000U) == ESP_OK);
}

/* ============================================================
   ESP8266_TcpConnect
   AT+CIPSTART="TCP","<host>",<port>
   ============================================================ */
ESP_Result_t ESP8266_TcpConnect(const char *host, uint16_t port)
{
    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "AT+CIPSTART=\"TCP\",\"%s\",%u", host, (unsigned)port);

    SERCOM5_USART_FlushRx();
    ESP8266_SendCmd(cmd);

    /* Expected responses: "CONNECT" then "OK"  */
    return ESP8266_WaitFor("CONNECT", 10000U);
}

/* ============================================================
   ESP8266_TcpSend
   AT+CIPSEND=<len>   → wait for '>'   → send bytes → wait "SEND OK"
   ============================================================ */
ESP_Result_t ESP8266_TcpSend(const uint8_t *data, uint16_t len)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", (unsigned)len);

    SERCOM5_USART_FlushRx();
    ESP8266_SendCmd(cmd);

    /* Wait for the '>' prompt meaning the module is ready to receive data */
    if (ESP8266_WaitFor(">", 3000U) != ESP_OK)
        return ESP_TIMEOUT;

    /* Send the raw bytes immediately after '>' */
    SERCOM5_USART_Write(data, (size_t)len);

    /* Wait for "SEND OK" */
    return ESP8266_WaitFor("SEND OK", 5000U);
}

/* ============================================================
   ESP8266_TcpClose
   ============================================================ */
void ESP8266_TcpClose(void)
{
    ESP8266_SendCmd("AT+CIPCLOSE");
    ESP8266_WaitFor("OK", 2000U);   /* best-effort */
}

/* ============================================================
   ESP8266_TcpRead
   Reads any bytes currently available in the SERCOM5 RX FIFO.
   Non-blocking — returns immediately with whatever is there.
   ============================================================ */
uint16_t ESP8266_TcpRead(uint8_t *buf, uint16_t maxLen)
{
    uint16_t n = 0;
    while (n < maxLen && SERCOM5_USART_ReadDataIsReady())
        buf[n++] = SERCOM5_USART_ReadByte();
    return n;
}
