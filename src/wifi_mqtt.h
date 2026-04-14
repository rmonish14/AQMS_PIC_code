/*******************************************************************************
  WiFi + MQTT Publisher Layer

  Sits on top of esp8266_at.c and provides a single ready-to-use function:
      wifi_mqtt_init()           — called once at startup
      wifi_mqtt_publish(payload) — called from the sensor loop

  MQTT protocol 3.1.1, QoS 0, no authentication.
  Connection flow per publish:
      1. Open TCP to MQTT broker
      2. Send MQTT CONNECT
      3. Send MQTT PUBLISH  (topic + sensor-data payload)
      4. Send MQTT DISCONNECT
      5. Close TCP

  ──────────────────────────────────────────────────────────────
  *** EDIT SECTION BELOW WITH YOUR CREDENTIALS ***
  ──────────────────────────────────────────────────────────────
*******************************************************************************/

#ifndef WIFI_MQTT_H
#define WIFI_MQTT_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
   USER CONFIGURATION — fill these in when you have credentials
   ============================================================ */

/* --- WiFi credentials ---------------------------------------------------- */
#define WIFI_SSID          "MONISH"       /* <-- replace me */
#define WIFI_PASSWORD      "12345678"   /* <-- replace me */

/* --- MQTT broker ---------------------------------------------------------- */
#define MQTT_BROKER_IP     "broker.hivemq.com"       /* <-- e.g. "192.168.1.100"
                                                         or "broker.hivemq.com" */
#define MQTT_BROKER_PORT   (1883U)

/* --- MQTT topic ----------------------------------------------------------- */
#define MQTT_TOPIC         "aqms/PIC32CM_AQI_01/data"   /* matches backend aqms/+/data */

/* --- MQTT client ID (must be unique per device on the broker) ------------- */
#define MQTT_CLIENT_ID     "PIC32CM_AQI_01"

/* --- Publish interval ----------------------------------------------------- */
/* How many sensor-read loop iterations between WiFi publishes.
   Each iteration ≈ 250 ms, so 40 ≈ every 10 seconds.                        */
#define WIFI_PUBLISH_EVERY  40U

/* ============================================================
   Public API
   ============================================================ */

/**
 * @brief  One-time init: initialise ESP-01S, connect to WiFi.
 *         Call from main() before run_display().
 *         Returns true on success; false if ESP or WiFi connection failed
 *         (system continues without WiFi in that case).
 */
bool wifi_mqtt_init(void);

/**
 * @brief  Publish one AQI sensor string to the MQTT broker.
 *         Format:  "O3:x.x CO:x.x NH3:x.x Dust:x.x CO2:x.x AQI:xx"
 *
 * @param  payload  NUL-terminated sensor string (max ~160 chars)
 * @return true if publish succeeded; false on any error.
 */
bool wifi_mqtt_publish(const char *payload);

/**
 * @brief  Returns true if the ESP-01S was successfully initialised
 *         and joined the WiFi network at startup.
 */
bool wifi_mqtt_is_ready(void);

#endif /* WIFI_MQTT_H */
