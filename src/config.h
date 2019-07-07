//
// Created by Krzysiek on 2018-12-13.
//

#ifndef WIFILAMP_CONFIG_H
#define WIFILAMP_CONFIG_H

#define VERSION_CODE 2
#define VERSION_NAME "1.0"
#define BUILD_DATE (String(__TIME__)+"@" + __DATE__)

#ifndef AP_NAME
#define AP_NAME "Chris' Lamp"
#endif

#ifndef DEVICE_ID
#define DEVICE_ID "iot_2"
#endif

#define SDA_PIN 13
#define SCL_PIN 12

#define ZC_PIN 4
#define PWM_PIN 5

#define ADC_MIN_VALUE 0x0360
#define ADC_DIVIDE 0x0C

#define POLLING_DELAY 5000
#define AVG_BUFFER_SIZE 30
#define LOCK_REPORT_DELAY 100 /* In terms of ADC polls*/

#define ADC_UNLOCK_THRESHOLD 8
#define ADC_ERROR 8

#define FADE_BRIGHTNESS_UP 1
#define FADE_BRIGHTNESS_DOWN 1
#define FADE_DOWN_STEPS 3
#define FADE_UP_STEPS 2

#define COMPILE_WIFI 1
#define COMPILE_DRIVER 2

#define TCP_PORT 80
#define UDP_DISCOVERY_PORT 8888
#define UDP_COM_PORT 88

#define HTTP_REGISTER_URL "https://iot-api.zdul.xyz/register.php"
#define HTTP_REPORT_URL "https://iot-api.zdul.xyz/report_state.php"
#define HTTP_SERVER_HTTPS_FINGERPRINT "01 77 78 5b ee 26 28 11 6f 66 82 4e 6f 02 87 0a c4 c1 34 42"

#ifndef COMPILE_MODE
#define COMPILE_MODE COMPILE_WIFI
#endif

#endif //WIFILAMP_CONFIG_H
