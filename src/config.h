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

#define POLLING_DELAY 100000
#define ADC_OVERTAKE_THRESHOLD 6
#define FADE_BRIGHTNESS 1

#define COMPILE_WIFI 1
#define COMPILE_DRIVER 2

#define COMPILE_MODE COMPILE_WIFI

#endif //WIFILAMP_CONFIG_H
