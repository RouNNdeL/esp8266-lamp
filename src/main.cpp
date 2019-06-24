#include <Arduino.h>
#include "config.h"

#ifdef SERIAL_DEBUG
#define PRINTLN(a) Serial.println(a)
#define PRINT(a) Serial.print(a)
#else
#define PRINTLN(a)
#define PRINT(a)
#endif

#if COMPILE_MODE == COMPILE_DRIVER

#include "hw_timer.h"
#include "memory.h"
#include "communication.h"

extern "C" {
#include "user_interface.h"
}

uint8_t target_brightness = 0;
volatile uint8_t current_brightness = 0;
volatile uint8_t zc_state = 0;

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF


void ICACHE_FLASH_ATTR dimTimerISR() {
    for(uint8_t i = 0; i < FADE_DOWN_STEPS; i++) {
        if(current_brightness > target_brightness) {
#if FADE_BRIGHTNESS_DOWN
            --current_brightness;
#else
            current_brightness = target_brightness;
#endif
        }
    }
    for(uint8_t i = 0; i < FADE_UP_STEPS; i++) {
        if(current_brightness < target_brightness) {
#if FADE_BRIGHTNESS_UP
            ++current_brightness;
#else
            current_brightness = target_brightness;
#endif
        }
    }

    if(current_brightness == 0) {
        digitalWrite(PWM_PIN, 0);
    } else {
        digitalWrite(PWM_PIN, 1);
        delayMicroseconds(250);
        digitalWrite(PWM_PIN, 0);
    }

    zc_state = 0;
}

void ICACHE_RAM_ATTR zcDetectISR() {
    if(!zc_state) {
        zc_state = 1;

        if(current_brightness < UINT8_MAX && current_brightness > 0) {
            digitalWrite(PWM_PIN, 0);

            uint32_t dimDelay = 30 * (UINT8_MAX - current_brightness) + 400;
            hw_timer_arm(dimDelay);
        } else if(current_brightness == UINT8_MAX) {
            digitalWrite(PWM_PIN, 1);
        }
    }
}


void setup(void) {
    pinMode(ZC_PIN, INPUT_PULLUP);
    pinMode(PWM_PIN, OUTPUT);

    digitalWrite(PWM_PIN, 0);

    Serial.begin(115200);
    PRINTLN("Starting...");

    wifi_set_opmode(NULL_MODE);
    wifi_set_sleep_type(MODEM_SLEEP_T);
    wifi_fpm_open();
    wifi_fpm_do_sleep(FPM_SLEEP_MAX_TIME);


    hw_timer_init(NMI_SOURCE, 0);
    hw_timer_set_func(dimTimerISR);

    attachInterrupt(ZC_PIN, zcDetectISR, RISING);

    /* We do not store the last value in EEPROM so we query the WiFi module for it */
    Serial.print(UART_POLL_BRIGHTNESS);
}

void loop(void) {
    if(Serial.available()) {
        target_brightness = (uint8_t) Serial.read();
    }
}

#elif COMPILE_MODE == COMPILE_WIFI

#include "Wire.h"
#include "hw_timer.h"
#include <IPAddress.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>
#include "memory.h"
#include "communication.h"

WiFiServer server(TCP_PORT);

volatile uint8_t poll_adc = 0;
uint16_t previous_value = 0;
uint8_t reported_val = 0;

uint8_t adc_lock = 0;
uint8_t brightness = 0;
uint8_t previous_brightness = 0;

uint8_t change_counter = 0;
uint16_t avg_buffer[AVG_BUFFER_SIZE] = {0};


void ICACHE_FLASH_ATTR adc_poll() {
    poll_adc = 1;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"

uint16_t get_median() {
    uint16_t temp_buffer[AVG_BUFFER_SIZE];
    memcpy(temp_buffer, avg_buffer, AVG_BUFFER_SIZE);
    uint16_t temp, j, i;
    for(i = 0; i < AVG_BUFFER_SIZE - 1; i++) {
        for(j = i + 1; j < AVG_BUFFER_SIZE; j++) {
            if(temp_buffer[j] < temp_buffer[i]) {
                // swap elements
                temp = temp_buffer[i];
                temp_buffer[i] = temp_buffer[j];
                temp_buffer[j] = temp;
            }
        }
    }

    return temp_buffer[AVG_BUFFER_SIZE / 2];
}

#pragma clang diagnostic pop

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    PRINTLN("");
    PRINTLN("|----------------------|");
    PRINTLN("|---- Begin Startup ---|");
    PRINTLN("| AC Dimmer by RouNdeL |");
    PRINTLN("|----------------------|");

    PRINTLN("Version Name: " + String(VERSION_NAME));
    PRINTLN("Version Code: " + String(VERSION_CODE));
    PRINTLN("Build Date: " + BUILD_DATE);
    PRINTLN("Device Id: " + String(DEVICE_ID));

    WiFiManager manager;
#ifndef SERIAL_DEBUG
    manager.setDebugOutput(0);
#endif /* SERIAL_DEBUG */
    manager.setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    manager.setConfigPortalTimeout(300);
    manager.autoConnect(AP_NAME);

    PRINTLN("|------------------|");
    PRINTLN("| Start TCP Server |");
    PRINTLN("|------------------|");

    server.begin();

    HTTPClient http;
    http.begin(HTTP_REGISTER_URL + String("?device_id=") + DEVICE_ID, HTTP_SERVER_HTTPS_FINGERPRINT);
    http.POST(String(TCP_PORT));
    http.end();

    hw_timer_init(NMI_SOURCE, 0);
    hw_timer_set_func(adc_poll);
    hw_timer_arm(POLLING_DELAY);
}

void loop() {
    WiFiClient client = server.available();
    if(client) {
        if(client.available() > 0) {
            uint8_t command = client.read();
            switch(command) {
                case TCP_RESTART:
                    client.print(TCP_SUCCESS, 0);
                    client.flush();
                    client.stop();
                    delay(250);
                    ESP.restart();
                    break;
                case TCP_DATA:
                    if(client.available() == 2) {
                        uint8_t state = client.read();
                        brightness = client.read();
                        adc_lock = 1;
                        Serial.print(state ? brightness : 0, 0);
                        client.print(TCP_SUCCESS, 0);
                    } else {
                        client.print(TCP_INVALID_REQUEST, 0);
                    }
                    break;
                default:
                    client.print(TCP_INVALID_REQUEST, 0);
            }
            client.flush();
        }
    }

    if(poll_adc) {
        Wire.requestFrom(0x48, 2);
        poll_adc = 0;
        change_counter++;
        hw_timer_arm(POLLING_DELAY);
    }

    while(Wire.available()) {
        char c[2];
        c[0] = Wire.read();
        c[1] = Wire.read();
        uint16_t value = (c[0] << 8) | c[1];

        memmove(avg_buffer + 1, avg_buffer, (AVG_BUFFER_SIZE - 1));
        avg_buffer[0] = value;

        uint16_t median = get_median();
        if(abs(median - previous_value) > ADC_ERROR) {
            previous_value = median;
            if(median >= ADC_MIN_VALUE) {
                median -= ADC_MIN_VALUE;
                median /= ADC_DIVIDE;
            } else {
                median = 0;
            }

            if(median > UINT8_MAX) {
                median = UINT8_MAX;
            }

            if((!adc_lock && median != brightness) || abs(previous_brightness - median) > ADC_UNLOCK_THRESHOLD) {
                change_counter = 0;
                brightness = median;
                previous_brightness = median;
                Serial.print(brightness, 0);
            }
        }

        if(change_counter > LOCK_REPORT_DELAY) {
            adc_lock = 1;
            change_counter = 0;
            if(reported_val != brightness) {
                HTTPClient http;
                http.begin(HTTP_REPORT_URL + String("?device_id=") + DEVICE_ID, HTTP_SERVER_HTTPS_FINGERPRINT);
                http.POST(String(brightness));
                http.end();
                reported_val = brightness;
            }
        }
    }
}

#endif