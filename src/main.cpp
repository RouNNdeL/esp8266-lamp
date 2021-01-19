#include <Arduino.h>
#include "config.h"

#ifdef DEBUG_ESP_CUSTOM
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
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncWiFiManager.h>
#include <ArduinoJson.h>

PROGMEM const char* discoveryTemplate = "{"
                                        "\"~\":\"%s\","
                                        "\"name\":\"%s\","
                                        "\"uniq_id\":\"%s\","
                                        "\"cmd_t\":\"~/%s\","
                                        "\"stat_t\":\"~/%s\","
                                        "\"schema\":\"json\","
                                        "\"brightness\": true,"
                                        "\"dev\":{"
                                        "\"mf\":\"Krzysztof Zdulski\","
                                        "\"mdl\":\"Dimmable Lamp\","
                                        "\"sw\":\"" VERSION "\","
                                        "\"name\":\"" DEVICE_NAME "\","
                                        "\"ids\":[\"" DEVICE_ID "\", \"%08x\"],"
                                        "\"cns\":[[\"mac\", \"%s\"]]"
                                        "}"
                                        "}";

PROGMEM const char* discoverySensor = "{"
                                      "\"~\":\"%s\","
                                      "\"device_class\":\"signal_strength\","
                                      "\"name\":\"" SIGNAL_SENSOR_NAME "\","
                                      "\"uniq_id\":\"%s\","
                                      "\"stat_t\":\"~/%s\","
                                      "\"json_attr_t\":\"~/%s\","
                                      "\"unit_of_meas\":\"dBm\","
                                      "\"expire_after\": %d,"
                                      "\"force_update\": true,"
                                      "\"dev\":{"
                                      "\"mf\":\"Krzysztof Zdulski\","
                                      "\"mdl\":\"Dimmable Lamp\","
                                      "\"sw\":\"" VERSION "\","
                                      "\"name\":\"" DEVICE_NAME "\","
                                      "\"ids\":[\"" DEVICE_ID "\", \"%08x\"],"
                                      "\"cns\":[[\"mac\", \"%s\"]]"
                                      "}"
                                      "}";

const char* topicConfigSuffix = "config";
const char* topicSetSuffix = "set";
const char* topicStateSuffix = "state";
const char* topicAttributesSuffix = "attrs";

uint32_t last_state_update = 0;

WiFiClient mqttTcpClient;
PubSubClient mqtt(mqttTcpClient);

AsyncWebServer server(80);
DNSServer dns;

char lightTopicPrefix[100];
char topicSetScan[100];
char sensorTopicPrefix[100];

volatile uint8_t poll_adc = 0;
uint16_t previous_value = 0;
uint8_t reported_val = 0;

uint8_t adc_lock = 0;
uint8_t brightness = 0;
uint8_t state = 0;
uint8_t previous_brightness = 0;
bool update = false;

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
    for (i = 0; i < AVG_BUFFER_SIZE - 1; i++) {
        for (j = i + 1; j < AVG_BUFFER_SIZE; j++) {
            if (temp_buffer[j] < temp_buffer[i]) {
                // swap elements
                temp = temp_buffer[i];
                temp_buffer[i] = temp_buffer[j];
                temp_buffer[j] = temp;
            }
        }
    }

    return temp_buffer[AVG_BUFFER_SIZE / 2];
}


__attribute__ ((always_inline)) static inline void publish(const char* topic, const char* message, bool persist) {
    bool success = mqtt.publish(topic, message, persist);
    if (!success) {
        PRINT("Failed to publish to ");
        PRINTLN(topic);
    } else {
        PRINT("Published to ");
        PRINTLN(topic);
    }
}


void get_topic(char* topic, uint8_t topic_size, char* prefix, const char* device_id, const char* suffix) {
    snprintf(topic, topic_size, "%s/%s", prefix, device_id);
    strncat(topic, "/", topic_size - strlen(topic) - 1);
    strncat(topic, suffix, topic_size - strlen(topic) - 1);
}

void get_ssid(char ssid[33]) {
    struct station_config conf;
    wifi_station_get_config(&conf);
    memcpy(ssid, conf.ssid, sizeof(conf.ssid));
    ssid[32] = 0;
}

void get_ip(char ipStr[16]) {
    struct ip_info ip;
    wifi_get_ip_info(STATION_IF, &ip);
    uint32_t addr = ip.ip.addr;
    snprintf(ipStr, 16, "%u.%u.%u.%u",
             addr & 0xFF, (addr >> 8) & 0xFF,
             (addr >> 16) & 0xFF, (addr >> 24) & 0xFF);
}

void publish_sensor_update() {
    PRINTLN("Publish sensor update");
    sint8 rssi = WiFi.RSSI();

    if (rssi > 10) {
        return;
    }

    char ssid[33];
    char ip[16];
    char topic[120];
    char message[64];

    get_topic(topic, sizeof(topic), sensorTopicPrefix, SIGNAL_SENSOR_ID, topicStateSuffix);
    snprintf(message, sizeof(message), "%d", rssi);
    publish(topic, message, true);

    get_ssid(ssid);
    get_ip(ip);
    snprintf(message, sizeof(message), R"({"rssi":%d,"ssid":"%s","ip":"%s"})", rssi, ssid, ip);

    get_topic(topic, sizeof(topic), sensorTopicPrefix, SIGNAL_SENSOR_ID, topicAttributesSuffix);
    publish(topic, message, true);
}

#pragma clang diagnostic pop

void mqttCallback(char* topic, uint8_t* payload, uint16_t length) {
    PRINT("Received payload for topic: ");
    PRINTLN(topic);
    char deviceId[32];
    sscanf(topic, topicSetScan, deviceId);
    PRINT("Device id: ");
    PRINTLN(deviceId);
    if (!strcmp(deviceId, DEVICE_ID)) {
        StaticJsonDocument<JSON_BUFFER_SIZE> json;
        deserializeJson(json, payload, DeserializationOption::NestingLimit(3));

        if (json.containsKey("brightness") && json["brightness"].is<uint8_t>()) {
            brightness = json["brightness"];

            update = true;
        }

        if (json.containsKey("state") && json["state"].is<char*>()) {
            const char* stateStr = json["state"];
            if (!strcmp(stateStr, "ON")) {
                state = 1;
            } else if (!strcmp(stateStr, "OFF")) {
                state = 0;
            }
            update = true;
        }
    }
}

void sendUpdate() {
    StaticJsonDocument<JSON_BUFFER_SIZE> json;
    json["state"] = state ? "ON" : "OFF";
    json["brightness"] = brightness;

    char topic[120];
    get_topic(topic, sizeof(topic), lightTopicPrefix, DEVICE_ID, topicStateSuffix);

    char message[JSON_BUFFER_SIZE];
    serializeJson(json, message);
    publish(topic, message, true);
}

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    AsyncWiFiManager wifiManager(&server, &dns);
#ifdef DEBUG_ESP_CUSTOM
    wifiManager.setDebugOutput(true);
#endif // DEBUG_ESP_CUSTOM
    wifiManager.autoConnect(DEVICE_NAME);

    //mqttTcpClient.setInsecure();
    mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setBufferSize(512); // Temporally increase buffer size for discovery payloads

    char nodeId[sizeof(MQTT_NODE_ID_PREFIX) + 16];
    snprintf(nodeId, sizeof(nodeId), "%s_%08x", MQTT_NODE_ID_PREFIX, ESP.getChipId());
    mqtt.connect(nodeId, MQTT_USER, MQTT_PASSWORD);

    size_t jsonSize = max(strlen(discoveryTemplate), strlen(discoverySensor)) + 192;
    mqtt.setBufferSize(jsonSize + 64);

    snprintf(lightTopicPrefix, sizeof(lightTopicPrefix), "%s/light/%s", MQTT_DISCOVERY_PREFIX, nodeId);
    snprintf(sensorTopicPrefix, sizeof(sensorTopicPrefix), "%s/sensor/%s", MQTT_DISCOVERY_PREFIX, nodeId);
    snprintf(topicSetScan, sizeof(topicSetScan), "%s/%%[^/]/%s", lightTopicPrefix, topicSetSuffix);

    char topic[120];
    char deviceUid[sizeof(nodeId) + 32];
    char* discoveryJson = static_cast<char*>(malloc(jsonSize));

    char macStr[18] = {0};
    uint8_t mac[6];
    WiFi.macAddress(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    PRINTLN("Starting discovery");
    snprintf(topic, sizeof(topic), "%s/%s", lightTopicPrefix, DEVICE_ID);
    snprintf(deviceUid, sizeof(deviceUid), "%s_%s", nodeId, DEVICE_ID);
    snprintf(discoveryJson, jsonSize, discoveryTemplate, topic, DEVICE_NAME,
             deviceUid, topicSetSuffix, topicStateSuffix, ESP.getChipId(), macStr);


    bool success;
    get_topic(topic, sizeof(topic), lightTopicPrefix, DEVICE_ID, topicSetSuffix);
    success = mqtt.subscribe(topic);
    if (!success) {
        PRINT("Failed to subscribe to topic");
        PRINTLN(topic);
    } else {
        PRINT("subscribed to topic ");
        PRINTLN(topic);
    }

    get_topic(topic, sizeof(topic), lightTopicPrefix, DEVICE_ID, topicConfigSuffix);
    publish(topic, discoveryJson, true);

    // Signal sensor discovery
    snprintf(deviceUid, sizeof(deviceUid), "%s_%s", nodeId, SIGNAL_SENSOR_ID);
    snprintf(topic, sizeof(topic), "%s/%s", sensorTopicPrefix, SIGNAL_SENSOR_ID);

    snprintf(discoveryJson, jsonSize, discoverySensor, topic, deviceUid,
             topicStateSuffix, topicAttributesSuffix, SIGNAL_PUBLISH_PERIOD * 2, ESP.getChipId(), macStr);


    get_topic(topic, sizeof(topic), sensorTopicPrefix, SIGNAL_SENSOR_ID, topicConfigSuffix);
    publish(topic, discoveryJson, true);

    free(discoveryJson);
    mqtt.setBufferSize(256);

    publish_sensor_update();

    hw_timer_init(NMI_SOURCE, 0);
    hw_timer_set_func(adc_poll);
    hw_timer_arm(POLLING_DELAY);
}

void loop() {
    if (millis() > last_state_update + SIGNAL_PUBLISH_PERIOD * 1000) {
        last_state_update = millis();
        publish_sensor_update();
    }

    if (!mqtt.loop()) {
        delay(RESTART_DELAY * 1000);
        ESP.restart();
    }

    if (poll_adc) {
        Wire.requestFrom(0x48, 2);
        poll_adc = 0;
        change_counter++;
        hw_timer_arm(POLLING_DELAY);
    }

    if(update) {
        update = false;
        adc_lock = 1;
        Serial.write(state ? brightness : 0);
        sendUpdate();
    }

    while (Wire.available()) {
        uint8_t c[2];
        c[0] = Wire.read();
        c[1] = Wire.read();
        uint16_t value = (c[0] << 8) | c[1];

        memmove(avg_buffer + 1, avg_buffer, (AVG_BUFFER_SIZE - 1));
        avg_buffer[0] = value;

        uint16_t median = get_median();
        if (abs(median - previous_value) > ADC_ERROR) {
            previous_value = median;
            if (median >= ADC_MIN_VALUE) {
                median -= ADC_MIN_VALUE;
                median /= ADC_DIVIDE;
            } else {
                median = 0;
            }

            if (median > UINT8_MAX) {
                median = UINT8_MAX;
            }

            if ((!adc_lock && median != brightness) || abs(previous_brightness - median) > ADC_UNLOCK_THRESHOLD) {
                change_counter = 0;
                brightness = median;
                previous_brightness = median;
                state = brightness ? 1 : 0;
                Serial.write(brightness);
            }
        }

        if (change_counter > LOCK_REPORT_DELAY) {
            adc_lock = 1;
            change_counter = 0;
            if (reported_val != brightness) {
                sendUpdate();
                reported_val = brightness;
            }
        }
    }
}

#endif