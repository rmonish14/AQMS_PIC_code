/*******************************************************************************
  ESP8266 (ESP-01S) AT Command Driver
  
  Handles low-level communication with the ESP-01S module running the
  factory Espressif AT firmware via SERCOM5 USART (PB16 TX / PB17 RX).

  Supports:
    - Module initialisation  (AT, RST, mode)
    - WiFi join              (AT+CWJAP)
    - TCP connect/send/close (AT+CIPSTART / AT+CIPSEND / AT+CIPCLOSE)
    - Raw byte send for MQTT packets
*******************************************************************************/

#ifndef ESP8266_AT_H
#define ESP8266_AT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
   Return codes
   ============================================================ */
typedef enum {
    ESP_OK        = 0,   /* Command accepted / response matched           */
    ESP_TIMEOUT   = 1,   /* Did not receive expected response in time     */
    ESP_ERROR     = 2,   /* Module replied with "ERROR" or "FAIL"         */
    ESP_NO_IP     = 3,   /* WiFi connected but no IP assigned yet         */
} ESP_Result_t;

/* ============================================================
   Module init & WiFi
   ============================================================ */

/**
 * @brief  Full hardware + AT initialisation sequence.
 *         - Initialises SERCOM5
 *         - Sends AT (checks comms)
 *         - Resets to factory station mode (CWMODE=1)
 * @return ESP_OK on success, ESP_TIMEOUT / ESP_ERROR otherwise.
 */
ESP_Result_t ESP8266_Init(void);

/**
 * @brief  Connect to an Access Point.
 *         Blocks until "WIFI GOT IP" is received or timeout.
 * @param  ssid      WiFi network name (NUL-terminated)
 * @param  password  WiFi password     (NUL-terminated)
 * @return ESP_OK on success.
 */
ESP_Result_t ESP8266_ConnectWiFi(const char *ssid, const char *password);

/**
 * @brief  Quick check: is the module still associated with an AP?
 *         Sends AT+CWJAP? and checks for a non-empty SSID response.
 * @return true if connected, false otherwise.
 */
bool ESP8266_IsWiFiConnected(void);

/* ============================================================
   TCP / Raw data
   ============================================================ */

/**
 * @brief  Open a TCP connection to host:port.
 * @param  host  IP or hostname string
 * @param  port  Port number (e.g. 1883 for MQTT)
 * @return ESP_OK if "CONNECT" received.
 */
ESP_Result_t ESP8266_TcpConnect(const char *host, uint16_t port);

/**
 * @brief  Send raw bytes over the open TCP connection.
 *         Uses AT+CIPSEND=<len> then streams the data.
 * @param  data  Pointer to byte buffer
 * @param  len   Number of bytes to send
 * @return ESP_OK if "SEND OK" received.
 */
ESP_Result_t ESP8266_TcpSend(const uint8_t *data, uint16_t len);

/**
 * @brief  Close the current TCP connection (AT+CIPCLOSE).
 */
void ESP8266_TcpClose(void);

/**
 * @brief  Read up to maxLen bytes from the TCP receive buffer into buf.
 *         Non-blocking; returns number of bytes actually copied.
 */
uint16_t ESP8266_TcpRead(uint8_t *buf, uint16_t maxLen);

/* ============================================================
   Low-level helpers (used internally; exposed for debugging)
   ============================================================ */

/**
 * @brief  Send a NUL-terminated AT command string (no CR/LF added).
 */
void ESP8266_SendRaw(const char *str);

/**
 * @brief  Send a NUL-terminated string followed by "\r\n".
 */
void ESP8266_SendCmd(const char *cmd);

/**
 * @brief  Wait up to timeout_ms for a line containing keyword.
 * @param  keyword     Fragment to search for in any received line
 * @param  timeout_ms  Max wait time in milliseconds
 * @return ESP_OK if found, ESP_TIMEOUT if not found in time.
 */
ESP_Result_t ESP8266_WaitFor(const char *keyword, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* ESP8266_AT_H */
