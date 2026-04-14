/*******************************************************************************
  WiFi + MQTT Publisher Layer — Implementation

  Builds minimal raw MQTT 3.1.1 packets (CONNECT, PUBLISH, DISCONNECT)
  and sends them over ESP-01S TCP using the AT command driver.

  MQTT packet layout (QoS 0, no auth, remaining-length < 128 bytes):
    CONNECT   : 0x10 | remLen | 0x00 0x04 'MQTT' | 0x04 | 0x02 |
                0x00 0x3C | 0x00 cidLen | <clientId>
    PUBLISH   : 0x30 | remLen | 0x00 topicLen | <topic> | <payload>
    DISCONNECT: 0xE0 0x00
*******************************************************************************/

#include "wifi_mqtt.h"
#include "esp8266_at.h"
#include <string.h>
#include <stddef.h>

/* ---- module state -------------------------------------------------------- */
static bool s_wifi_ready = false;

/* ============================================================
   Private: build MQTT CONNECT packet into buf
   Returns total packet length.
   ============================================================ */
static uint16_t _build_connect(uint8_t *buf)
{
    const char *cid     = MQTT_CLIENT_ID;
    uint16_t    cidLen  = (uint16_t)strlen(cid);

    /* Remaining length = 10 (fixed header fields) + 2 (cid length prefix)
                        + cidLen                                            */
    uint8_t remLen = (uint8_t)(10U + 2U + cidLen);

    uint16_t i = 0;

    /* Fixed header */
    buf[i++] = 0x10U;           /* Packet type: CONNECT */
    buf[i++] = remLen;

    /* Variable header – Protocol Name */
    buf[i++] = 0x00U;
    buf[i++] = 0x04U;
    buf[i++] = 'M';
    buf[i++] = 'Q';
    buf[i++] = 'T';
    buf[i++] = 'T';

    /* Protocol level (3.1.1) */
    buf[i++] = 0x04U;

    /* Connect Flags: Clean Session only */
    buf[i++] = 0x02U;

    /* Keep Alive: 60 seconds (0x003C) */
    buf[i++] = 0x00U;
    buf[i++] = 0x3CU;

    /* Payload – Client Identifier */
    buf[i++] = 0x00U;
    buf[i++] = (uint8_t)cidLen;
    memcpy(buf + i, cid, cidLen);
    i += cidLen;

    return i;
}

/* ============================================================
   Private: build MQTT PUBLISH packet into buf
   topic   – NUL-terminated topic string
   payload – NUL-terminated payload string
   Returns total packet length.
   NOTE: Remaining length < 127 bytes assumed (QoS 0, no packet-id).
   ============================================================ */
static uint16_t _build_publish(uint8_t *buf,
                                const char *topic,
                                const char *payload)
{
    uint16_t topicLen   = (uint16_t)strlen(topic);
    uint16_t payLen     = (uint16_t)strlen(payload);

    /* Remaining length = 2 (topic length prefix) + topicLen + payLen */
    uint8_t remLen = (uint8_t)(2U + topicLen + payLen);

    uint16_t i = 0;

    /* Fixed header – PUBLISH, QoS=0, no retain */
    buf[i++] = 0x30U;
    buf[i++] = remLen;

    /* Topic */
    buf[i++] = (uint8_t)(topicLen >> 8U);
    buf[i++] = (uint8_t)(topicLen & 0xFFU);
    memcpy(buf + i, topic, topicLen);
    i += topicLen;

    /* Payload */
    memcpy(buf + i, payload, payLen);
    i += payLen;

    return i;
}

/* ============================================================
   Private: build MQTT DISCONNECT packet (2 bytes)
   ============================================================ */
static uint16_t _build_disconnect(uint8_t *buf)
{
    buf[0] = 0xE0U;  /* DISCONNECT */
    buf[1] = 0x00U;
    return 2U;
}

/* ============================================================
   wifi_mqtt_is_ready
   ============================================================ */
bool wifi_mqtt_is_ready(void)
{
    return s_wifi_ready;
}

/* ============================================================
   wifi_mqtt_init
   Called once from main() — initialises ESP-01S and joins WiFi.
   ============================================================ */
bool wifi_mqtt_init(void)
{
    s_wifi_ready = false;

    /* 1. Bring up SERCOM5 + ESP-01S */
    if (ESP8266_Init() != ESP_OK)
        return false;   /* Module not responding – skip WiFi silently */

    /* 2. Connect to WiFi AP */
    if (ESP8266_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD) != ESP_OK)
        return false;

    s_wifi_ready = true;
    return true;
}

/* ============================================================
   wifi_mqtt_publish
   Opens TCP → MQTT CONNECT → MQTT PUBLISH → MQTT DISCONNECT → TCP close
   ============================================================ */
bool wifi_mqtt_publish(const char *payload)
{
    /* Check if WiFi was successfully initialised */
    if (!s_wifi_ready)
        return false;

    /* Re-check WiFi link before attempting TCP */
    if (!ESP8266_IsWiFiConnected())
    {
        /* Try to reconnect once */
        if (ESP8266_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD) != ESP_OK)
        {
            s_wifi_ready = false;
            return false;
        }
    }

    /* ---- Open TCP connection to MQTT broker ---- */
    if (ESP8266_TcpConnect(MQTT_BROKER_IP, MQTT_BROKER_PORT) != ESP_OK)
        return false;

    uint8_t  pkt[256];
    uint16_t pktLen;

    /* ---- Send MQTT CONNECT ---- */
    pktLen = _build_connect(pkt);
    if (ESP8266_TcpSend(pkt, pktLen) != ESP_OK)
    {
        ESP8266_TcpClose();
        return false;
    }

    /* Small gap – give broker time to send CONNACK (we ignore it) */
    for (volatile uint32_t d = 0; d < 120000UL; d++);  /* ~10 ms @ 48 MHz */

    /* Flush CONNACK so it does not confuse next WaitFor */
    uint8_t trash[32];
    ESP8266_TcpRead(trash, sizeof(trash));

    /* ---- Send MQTT PUBLISH ---- */
    pktLen = _build_publish(pkt, MQTT_TOPIC, payload);
    if (ESP8266_TcpSend(pkt, pktLen) != ESP_OK)
    {
        ESP8266_TcpClose();
        return false;
    }

    /* ---- Send MQTT DISCONNECT ---- */
    pktLen = _build_disconnect(pkt);
    ESP8266_TcpSend(pkt, pktLen);  /* best-effort */

    /* ---- Close TCP session ---- */
    ESP8266_TcpClose();

    return true;
}
