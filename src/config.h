//
// Created by Krzysiek on 2018-12-13.
//

#ifndef WIFILAMP_CONFIG_H
#define WIFILAMP_CONFIG_H

#define VERSION "2.0.0"

#define SDA_PIN 13
#define SCL_PIN 12

#define ZC_PIN 4
#define PWM_PIN 5

#define ADC_MIN_VALUE 0x0360
#define ADC_DIVIDE 0x0C

#define POLLING_DELAY 5000
#define AVG_BUFFER_SIZE 30
#define LOCK_REPORT_DELAY 500 /* In terms of ADC polls*/

#define ADC_UNLOCK_THRESHOLD 8
#define ADC_ERROR 8

#define FADE_BRIGHTNESS_UP 1
#define FADE_BRIGHTNESS_DOWN 1
#define FADE_DOWN_STEPS 3
#define FADE_UP_STEPS 2

#define COMPILE_WIFI 1
#define COMPILE_DRIVER 2

#ifndef DEVICE_NAME
#define DEVICE_NAME "Lamp - Test"
#endif /* DEVICE_NAME */

#ifndef DEVICE_ID
#define DEVICE_ID "lamp_test"
#endif /* DEVICE_ID */

#ifndef SIGNAL_SENSOR_NAME
#define SIGNAL_SENSOR_NAME "Test - Signal"
#endif /* SIGNAL_SENSOR_NAME */

// Has to be different from the other IDs
#ifndef SIGNAL_SENSOR_ID
#define SIGNAL_SENSOR_ID "test_signal"
#endif /* SIGNAL_SENSOR_ID */
// In Seconds
#ifndef SIGNAL_PUBLISH_PERIOD
#define SIGNAL_PUBLISH_PERIOD 60
#endif /* SIGNAL_PUBLISH_PERIOD */

#ifndef JSON_BUFFER_SIZE
#define JSON_BUFFER_SIZE 256
#endif /* JSON_BUFFER_SIZE */
// In Seconds
#ifndef RESTART_DELAY
#define RESTART_DELAY 30
#endif /* RESTART_DELAY */

#ifndef MQTT_BROKER_HOST
#define MQTT_BROKER_HOST "test.mosquitto.org"
#endif /* MQTT_BROKER_HOST */

#ifndef MQTT_BROKER_PORT
#define MQTT_BROKER_PORT 1883
#endif /* MQTT_BROKER_PORT */

#ifndef MQTT_USER
#define MQTT_USER "esp"
#endif /* MQTT_USER */

#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD "password"
#endif /* MQTT_PASSWORD */

#ifndef MQTT_NODE_ID_PREFIX
#define MQTT_NODE_ID_PREFIX "esp_led"
#endif /* MQTT_NODE_ID_PREFIX */

#ifndef MQTT_DISCOVERY_PREFIX
#define MQTT_DISCOVERY_PREFIX "homeassistant"
#endif /* MQTT_DISCOVERY_PREFIX */

#ifndef COMPILE_MODE
#define COMPILE_MODE COMPILE_WIFI
#endif

#endif //WIFILAMP_CONFIG_H
