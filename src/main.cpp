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

extern "C" {
#include "user_interface.h"
}

uint8_t target_brightness = 0;
volatile uint8_t current_brightness = 0;
volatile uint8_t zc_state = 0;

#define FPM_SLEEP_MAX_TIME 0xFFFFFFF


void ICACHE_FLASH_ATTR dimTimerISR() {
#if FADE_BRIGHTNESS
    if(current_brightness > target_brightness) {
        --current_brightness;
    } else if(current_brightness < target_brightness) {
        ++current_brightness;
    }
#else
    current_brightness = target_brightness;
#endif

    if(current_brightness == 0) {
        digitalWrite(PWM_PIN, 0);
    } else {
        digitalWrite(PWM_PIN, 1);
        delayMicroseconds(250);
        digitalWrite(PWM_PIN, 0);
    }

    zc_state = 0;
}

void ICACHE_FLASH_ATTR zcDetectISR() {
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
#include <EEPROM.h>
#include <ESP8266HTTPClient.h>
#include "memory.h"
#include "communication.h"

WiFiServer server(TCP_PORT);

volatile uint8_t poll_adc = 0;
uint8_t previous_value = 0;
uint8_t previous_value_for_buffer = 0;
uint8_t reported_val = 0;

uint8_t state = 0;
uint8_t brightness = 0;

uint8_t adc_locked = 0;
int16_t change_buffer[USER_INTERACTION_BUFFER_SIZE] = {0};
uint8_t change_counter = 0;

uint8_t char2int(char input) {
    if(input >= '0' && input <= '9')
        return input - '0';
    if(input >= 'A' && input <= 'F')
        return input + 10 - 'A';
    if(input >= 'a' && input <= 'f')
        return input + 10 - 'a';
    return 0;
}

void ICACHE_FLASH_ATTR adc_poll() {
    poll_adc = 1;
}

uint8_t sum_change_buffer() {
    int16_t sum = 0;
    for(uint8_t i = 0; i < USER_INTERACTION_BUFFER_SIZE; ++i) {
        sum += change_buffer[i];
    }
    return abs(sum);
}

void setup() {
    Serial.begin(115200);
    Serial.print(state ? brightness : 0, 0);

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
                        state = client.read();
                        brightness = client.read();
                        adc_locked = 1;
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

    while(Serial.available()) {
        if(Serial.read() == UART_POLL_BRIGHTNESS) {
            Serial.print(state ? brightness : 0, 0);
        }
    }

    while(Wire.available()) {
        char c[2];
        c[0] = Wire.read();
        c[1] = Wire.read();
        uint16_t value = (c[0] << 8) | c[1];
        if(value >= ADC_MIN_VALUE) {
            value -= ADC_MIN_VALUE;
            value /= ADC_DIVIDE;
        } else {
            value = 0;
        }

        if(value > UINT8_MAX) {
            value = UINT8_MAX;
        }

        int16_t change = value - previous_value_for_buffer;
        previous_value_for_buffer = value;
        memmove(change_buffer + sizeof(*change_buffer), change_buffer,
                (USER_INTERACTION_BUFFER_SIZE - 1));
        change_buffer[0] = change;

        uint8_t d = abs(value - previous_value);
        if((!adc_locked && (d >= ADC_ERROR || (value != previous_value && (!value || value == UINT8_MAX))))
           || d >= ADC_OVERTAKE_THRESHOLD) {
            adc_locked = 0;
            state = 1;
            brightness = value;
            previous_value = value;
            Serial.print(brightness, 0);
        }

        uint8_t sum = sum_change_buffer();

        if(sum > USER_INTERACTION_LOCK_THRESHOLD){
            change_counter = 0;
        }

        if(change_counter > USER_INTERACTION_LOCK_DELAY ) {
            change_counter = 0;
            if(!adc_locked && reported_val != brightness) {
                /* Lock the ADC after user stopped interacting with the ADC to prevent flicker */
                adc_locked = 1;
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