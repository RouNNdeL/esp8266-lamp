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

extern "C" {
#include "user_interface.h"
}

uint8_t target_brightness = UINT8_MAX;
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
#include "memory.h"

ESP8266WebServer server(80);

uint8_t poll_adc;
uint8_t previous_val;

uint8_t state;
uint8_t brightness;

uint8_t adc_locked;

void init_eeprom() {
    EEPROM.begin(MEMORY_SIZE);
    state = EEPROM.read(ADDRESS_STATE);
    brightness = EEPROM.read(ADDRESS_BRIGHTNESS);
}

void save_eeprom() {
    EEPROM.write(ADDRESS_STATE, state);
    EEPROM.write(ADDRESS_BRIGHTNESS, brightness);
    EEPROM.commit();
}

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

void ICACHE_FLASH_ATTR restart() {
    server.send(200, "text/html",
                F("<p>Your device is restarting...</p><p id=\"a\">It should come online in 15s</p><script>var a=15;setInterval(function(){a--;if(!a)window.location=\"/\";document.getElementById(\"a\").innerHTML=\"It should come online in \"+a+\"s\";},1000)</script>"));
    delay(500);
    ESP.restart();
}

void ICACHE_FLASH_ATTR receive_data() {
    if(server.hasArg("plain") && server.method() == HTTP_PUT && server.arg("plain").length() == MEMORY_SIZE * 2) {
        auto a = server.arg("plain");
        server.send(204);
        state = char2int(a[0]) * 16 + char2int(a[1]);
        brightness = char2int(a[2]) * 16 + char2int(a[3]);
        save_eeprom();
        Serial.print(state ? brightness : 0, 0);
        adc_locked = 1;
    } else {
#ifdef SERIAL_DEBUG
        server.send(400, "text/html", "<h2>HTTP 400 Invalid Request</h2>");
#else
        server.send(400);
#endif /* SERIAL_DEBUG */
    }
}

void setup() {
    init_eeprom();
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

    PRINTLN("|-----------------------|");
    PRINTLN("| Start HTTP Web Server |");
    PRINTLN("|-----------------------|");

    server.on("/data", HTTP_PUT, receive_data);
    server.on("/restart", restart);
    server.begin();

    ArduinoOTA.begin();

    hw_timer_init(NMI_SOURCE, 0);
    hw_timer_set_func(adc_poll);
    hw_timer_arm(POLLING_DELAY);
}

void loop() {
    server.handleClient();
    ArduinoOTA.handle();

    if(poll_adc) {
        Wire.requestFrom(0x48, 2);
        poll_adc = 0;
        hw_timer_arm(POLLING_DELAY);
    }

    while(Wire.available()) {
        char c[2];
        c[0] = Wire.read();
        c[1] = Wire.read();
        uint16_t value = ((c[0] << 8) | c[1]) >> 4;

        if(value > 255)
            value = 255;
        if(!adc_locked || (abs(value - previous_val) > ADC_OVERTAKE_THRESHOLD || (previous_val != value && !value))) {
            adc_locked = 0;
            state = 1;
            brightness = value;
            Serial.print(brightness, 0);
        }
        previous_val = value;
    }
}

#endif