//
// Created by Krzysiek on 2018-12-13.
//

#ifndef WIFILAMP_CONFIG_H
#define WIFILAMP_CONFIG_H

#define VERSION_CODE 1
#define VERSION_NAME "0.1"
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

#define ADC_MIN_VALUE 0x0390
#define ADC_DIVIDE 0x0C

/*
 * Values lower then 100000 (more than 10 per second) may result in WDT resets
 * when a request is issued to the HTTP server
 */
#define POLLING_DELAY 20000
#define POLL_COUNT_REPORT 100
#define ADC_OVERTAKE_THRESHOLD 20
#define ADC_ERROR 4
#define FADE_BRIGHTNESS 1

#define COMPILE_WIFI 1
#define COMPILE_DRIVER 2

#define TCP_PORT 80

#define HTTP_REGISTER_URL "https://iot-api.zdul.xyz/register.php"
#define HTTP_REPORT_URL "https://iot-api.zdul.xyz/report_state.php"
#define HTTP_SERVER_HTTPS_FINGERPRINT "01 77 78 5b ee 26 28 11 6f 66 82 4e 6f 02 87 0a c4 c1 34 42"

#ifndef COMPILE_MODE
#define COMPILE_MODE COMPILE_WIFI
#endif

#endif //WIFILAMP_CONFIG_H
