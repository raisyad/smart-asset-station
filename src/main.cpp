#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <time.h>
#include <PubSubClient.h>
#include "secrets.h"

const int RFID_SS_PIN = 5;    // Pin for RFID SS
const int RFID_SCK_PIN = 18;  // Pin for RFID SCK
const int RFID_MOSI_PIN = 23; // Pin for RFID MOSI
const int RFID_MISO_PIN = 19; // Pin for RFID MISO
const int RFID_RST_PIN = 22;  // Pin for RFID RST
const int FEEDBACK_LED_PIN = 2; // Pin for LED ESP32 Configuration (GPIO2)
const int OLED_SDA_PIN = 21;
const int OLED_SCL_PIN = 4;
const int BUZZER_PIN = 15;
const int BUZZER_PWM_CHANNEL = 0;
const int BUZZER_PWM_RESOLUTION_BITS = 8;
const int BUZZER_ON_DUTY = 128;
const int BUZZER_OFF_DUTY = 255;

const unsigned long DUPLICATE_SCAN_WINDOW_MS = 3000;
const unsigned long DISPLAY_RESULT_DURATION_MS = 3000;
const unsigned int BUZZER_FREQUENCY_HZ = 2500;
const size_t MAX_UID_LENGTH = 10;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;
const time_t MIN_VALID_EPOCH = 1700000000;
const char NTP_SERVER_PRIMARY[] = "pool.ntp.org";
const char NTP_SERVER_SECONDARY[] = "time.google.com";
const char MQTT_BROKER_HOST[] = "192.168.1.2";
const uint16_t MQTT_BROKER_PORT = 1883;
const char MQTT_CLIENT_ID[] = "smart-asset-station-01";
const char MQTT_EVENT_TOPIC[] = "smart-asset/station-01/rfid/events";
const char MQTT_STATUS_TOPIC[] = "smart-asset/station-01/status";

enum class TagIdentity {
    Unknown,
    TagOne,
    TagTwo
};

enum class FeedbackType {
    Success,
    Denied
};

enum class FeedbackPhase {
    Idle,
    On,
    Off
};

enum class DisplayState {
    Ready,
    ScanResult
};

struct RegisteredTag {
    const byte* uid;
    size_t uidLength;
    TagIdentity identity;
    const char* name;
};

struct ScanEvent {
    unsigned long eventId;
    byte uid[MAX_UID_LENGTH];
    size_t uidLength;
    MFRC522::PICC_Type cardType;
    const RegisteredTag* matchedTag;
    unsigned long occurredAtMillis;
    time_t occurredAtEpoch;
    bool hasValidTimestamp;
};

byte lastProcessedUid[MAX_UID_LENGTH] = {};
size_t lastProcessedUidLength = 0;
unsigned long lastProcessedScanMillis = 0;
unsigned long nextScanEventId = 1;
unsigned long lastWifiConnectionAttemptMillis = 0;
unsigned long lastMqttConnectionAttemptMillis = 0;

bool hasProcessedUid = false;
bool wasWifiConnected = false;
bool hasConfiguredNtp = false;
bool hasAttemptedMqttConnection = false;
bool wasMqttConnected = false;

FeedbackPhase feedbackPhase = FeedbackPhase::Idle;

unsigned long feedbackPhaseStartedMillis = 0;
unsigned long feedbackOnDurationMs = 0;
unsigned long feedbackOffDurationMs = 0;

byte feedbackTargetPulseCount = 0;
byte feedbackCompletedPulseCount = 0;

DisplayState displayState = DisplayState::Ready;
unsigned long displayStateStartedMillis = 0;

const byte TAG_ONE_UID[] = {
    0xC3, 0xF5, 0x3E, 0x06
};

const byte TAG_TWO_UID[] = {
    0x2C, 0x2A, 0xF0, 0x06
};

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled (
    U8G2_R0,
    U8X8_PIN_NONE
);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const RegisteredTag REGISTERED_TAGS[] = {
    {
        TAG_ONE_UID,
        sizeof(TAG_ONE_UID),
        TagIdentity::TagOne,
        "Tag One"
    },
    {
        TAG_TWO_UID,
        sizeof(TAG_TWO_UID),
        TagIdentity::TagTwo,
        "Tag Two"
    }
};

const size_t REGISTERED_TAG_COUNT = sizeof(REGISTERED_TAGS) / sizeof(REGISTERED_TAGS[0]);

const RegisteredTag* findRegisteredTag(const MFRC522::Uid& scannedUid);

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN); // Create MFRC522 instance
bool uidMatches(const MFRC522::Uid& scannedUid, const byte* expectedUid, size_t expectedLength);

bool isDuplicateScan(const MFRC522::Uid& scannedUid, unsigned long currentMillis);

bool rememberProcessedUid(const MFRC522::Uid& scannedUid, unsigned long currentMillis);

void startFeedback(FeedbackType type, unsigned long currentMillis);

void updateFeedback(unsigned long currentMillis);

void stopFeedback();

void updateRfid(unsigned long currentMillis);

bool buildScanEvent(const MFRC522::Uid& scannedUid, unsigned long currentMillis, ScanEvent& event);

void printScanEvent(const ScanEvent& event);

void handleScanEvent(const ScanEvent& event);

void scanI2cDevices();

void showReadyScreen();

void startScanResultDisplay(const ScanEvent& event);

void updateDisplay(unsigned long currentMillis);

void setFeedbackOutputs(bool active);

void printScanEventJson(const ScanEvent& event);

void startWifiConnection(unsigned long currentMillis);

void updateNetwork(unsigned long currentMillis);

bool formatUtcTimestamp(time_t epoch, char* destination, size_t destinationSize);

void updateMqtt(unsigned long currentMillis);

void setup()
{
    Serial.begin(115200);
    startWifiConnection(millis());
    mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    mqttClient.setKeepAlive(30);
    mqttClient.setSocketTimeout(2);
    ledcSetup(BUZZER_PWM_CHANNEL, BUZZER_FREQUENCY_HZ, BUZZER_PWM_RESOLUTION_BITS);
    ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
    ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_OFF_DUTY);

    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    
    Serial.println("ESP32 RC522 Startup...");
    Serial.println("Checking I2C devices...");
    scanI2cDevices();

    oled.begin();
    showReadyScreen();
    
    pinMode(FEEDBACK_LED_PIN, OUTPUT);
    digitalWrite(FEEDBACK_LED_PIN, LOW);

    SPI.begin(
        RFID_SCK_PIN,
        RFID_MISO_PIN,
        RFID_MOSI_PIN,
        RFID_SS_PIN); // Initialize SPI

    rfid.PCD_Init();

    Serial.println("Checking RC522 Communication...");
    rfid.PCD_DumpVersionToSerial(); // Dump version info to serial
}

void loop()
{
    unsigned long currentMillis = millis();

    updateNetwork(currentMillis);
    updateMqtt(currentMillis);
    updateFeedback(currentMillis);
    updateDisplay(currentMillis);
    updateRfid(currentMillis);
}

bool uidMatches(const MFRC522::Uid& scannedUid, const byte* expectedUid, size_t expectedLength){
    if (expectedUid == nullptr) return false;

    if (scannedUid.size != expectedLength) return false;

    for (size_t index = 0; index < expectedLength; ++index)
    {
        if (scannedUid.uidByte[index] != expectedUid[index])
        {
            return false;
        }
    }
    return true;
}

bool isDuplicateScan(const MFRC522::Uid& scannedUid, unsigned long currentMillis){
    if (!hasProcessedUid) {
        return false;
    }

    if (!uidMatches(scannedUid, lastProcessedUid, lastProcessedUidLength)) {
        return false;
    }

    return currentMillis - lastProcessedScanMillis < DUPLICATE_SCAN_WINDOW_MS;
}

bool rememberProcessedUid(const MFRC522::Uid& scannedUid, unsigned long currentMillis){
    if (scannedUid.size > sizeof(lastProcessedUid)) {
        return false;
    }

    for (size_t index = 0; index < scannedUid.size; ++index) {
        lastProcessedUid[index] = scannedUid.uidByte[index];
    }

    lastProcessedUidLength = scannedUid.size;
    lastProcessedScanMillis = currentMillis;
    hasProcessedUid = true;

    return true;
}

const RegisteredTag* findRegisteredTag(const MFRC522::Uid& scannedUid) {
    for (size_t index = 0; index < REGISTERED_TAG_COUNT; ++index) {
        const RegisteredTag& candidate = REGISTERED_TAGS[index];

        if (uidMatches(scannedUid, candidate.uid, candidate.uidLength)) {
            return &candidate;
        }
    }

    return nullptr;
}

void startFeedback(FeedbackType type, unsigned long currentMillis) {
    switch (type) {
        case FeedbackType::Success:
            feedbackOnDurationMs = 200;
            feedbackOffDurationMs = 0;
            feedbackTargetPulseCount = 1;
            break;

        case FeedbackType::Denied:
            feedbackOnDurationMs = 100;
            feedbackOffDurationMs = 100;
            feedbackTargetPulseCount = 3;
            break;
    }

    feedbackCompletedPulseCount = 0;
    feedbackPhaseStartedMillis = currentMillis; 
    feedbackPhase = FeedbackPhase::On;

    setFeedbackOutputs(true);
}

void updateFeedback(unsigned long currentMillis) {
    if (feedbackPhase == FeedbackPhase::Idle) return;

    unsigned long elapsedMillis = currentMillis - feedbackPhaseStartedMillis;

    if (feedbackPhase == FeedbackPhase::On) {
        if (elapsedMillis < feedbackOnDurationMs) return;

        ++feedbackCompletedPulseCount;

        if (feedbackCompletedPulseCount >= feedbackTargetPulseCount) {
            stopFeedback();
            return;
        }

        setFeedbackOutputs(false);

        feedbackPhase = FeedbackPhase::Off;
        feedbackPhaseStartedMillis = currentMillis;
        return;
    }

    if (feedbackPhase == FeedbackPhase::Off) {
        if (elapsedMillis < feedbackOffDurationMs) return;

        setFeedbackOutputs(true);

        feedbackPhase = FeedbackPhase::On;
        feedbackPhaseStartedMillis = currentMillis; 
    }
}

void stopFeedback(){
    setFeedbackOutputs(false);

    feedbackPhase = FeedbackPhase::Idle;
    feedbackTargetPulseCount = 0;
    feedbackCompletedPulseCount = 0;   
}

void updateRfid(unsigned long currentMillis) {
    if (!rfid.PICC_IsNewCardPresent())
    {
        return; // No new card present
    }

    if (!rfid.PICC_ReadCardSerial())
    {
        Serial.println("Card detected, but UID cannot be read");
        return; // Failed to read card serial
    }

    if (isDuplicateScan(rfid.uid, currentMillis)) {
        Serial.println("Duplicate scan detected. Ignoring.");
        rfid.PICC_HaltA();
        return;
    }

    ScanEvent scanEvent{};

    if (!buildScanEvent(rfid.uid, currentMillis, scanEvent)) {
        Serial.println("Failed to build scan event.");
        rfid.PICC_HaltA(); // Halt PICC
        return;
    }

    if (!rememberProcessedUid(rfid.uid, currentMillis)) {
        Serial.println("UID cannot be stored. Scan ignored.");
        rfid.PICC_HaltA();
        return;
    }
    
    printScanEvent(scanEvent);
    printScanEventJson(scanEvent);
    handleScanEvent(scanEvent);
    rfid.PICC_HaltA(); // Halt PICC
}

bool buildScanEvent(const MFRC522::Uid& scannedUid, unsigned long currentMillis, ScanEvent& event){
    if (scannedUid.size > sizeof(event.uid)) return false;

    event.eventId = nextScanEventId;
    ++nextScanEventId;

    for (size_t index = 0; index < scannedUid.size; ++index) {
        event.uid[index] = scannedUid.uidByte[index];
    }

    event.uidLength = scannedUid.size;
    event.cardType = rfid.PICC_GetType(scannedUid.sak);
    event.matchedTag = findRegisteredTag(scannedUid);
    event.occurredAtMillis = currentMillis;
    event.occurredAtEpoch = time(nullptr);
    event.hasValidTimestamp = event.occurredAtEpoch >= MIN_VALID_EPOCH;

    return true;
}

void printScanEvent(const ScanEvent& event){
    Serial.println("RFID tag detected");

    Serial.print("UID Length : ");
    Serial.print(event.uidLength);
    Serial.println(" bytes");

    Serial.print("UID Value : ");
    for (size_t index = 0; index < event.uidLength; ++index)
    {
        byte uidByte = event.uid[index];

        if (uidByte < 0x10)
        {
            Serial.print('0');
        }

        Serial.print(uidByte, HEX);

        if (index + 1 < event.uidLength)
        {
            Serial.print(':');
        }
    }

    Serial.println();

    Serial.print("Card Type : ");
    Serial.println(rfid.PICC_GetTypeName(event.cardType));

    if (event.matchedTag == nullptr) {
        Serial.println("Tag Identity : Unknown");
        Serial.println("This tag is not registered");
    } else {
        Serial.print("Tag Identity : ");
        Serial.println(event.matchedTag->name);
    
        Serial.println("Tag registered");
    }
    Serial.println("-----------------------------------");
}

void handleScanEvent(const ScanEvent& event){
    startScanResultDisplay(event);

    if (event.matchedTag == nullptr) {
        startFeedback(FeedbackType::Denied, event.occurredAtMillis);
        return;
    }

    startFeedback(FeedbackType::Success, event.occurredAtMillis);
}

void scanI2cDevices(){
    byte detectedDeviceCount = 0;

    for (byte address = 1; address < 127; ++address) {
        Wire.beginTransmission(address);
        byte errorCode = Wire.endTransmission();

        if (errorCode == 0) {
            Serial.print("I2C device detected at address 0x");

            if (address < 0x10) {
                Serial.print('0');
            }

            Serial.println(address, HEX);
            ++detectedDeviceCount;
        }
    }

    if (detectedDeviceCount == 0) {
        Serial.println("No I2C device detected");
    } else {
        Serial.print("Total I2C devices detected: ");
        Serial.println(detectedDeviceCount);
    }
}

void showReadyScreen() {
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 14, "SMART ASSET STATION");

    oled.setFont(u8g2_font_9x15B_tf);
    oled.drawStr(31, 38, "READY");

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(12, 58, "Tap your RFID Tag");

    oled.sendBuffer();
}

void startScanResultDisplay(const ScanEvent& event){
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 12, "RFID SCAN RESULT");

    if (event.matchedTag == nullptr) {
        oled.setFont(u8g2_font_9x15B_tf);
        oled.drawStr(22, 36, "DENIED");

        oled.setFont(u8g2_font_6x12_tf);
        oled.drawStr(17, 56, "Unknown RFID Tag");
    } else {
        oled.setFont(u8g2_font_9x15B_tf);
        oled.drawStr(16, 34, "GRANTED");

        oled.setFont(u8g2_font_6x12_tf);
        oled.drawStr(0, 55, event.matchedTag->name);
    }

    oled.sendBuffer();

    displayState = DisplayState::ScanResult;
    displayStateStartedMillis = event.occurredAtMillis;
}

void updateDisplay(unsigned long currentMillis) {
    if (displayState != DisplayState::ScanResult) {
        return;
    }

    unsigned long elapsedMillis = currentMillis - displayStateStartedMillis;

    if (elapsedMillis < DISPLAY_RESULT_DURATION_MS) return;

    showReadyScreen();
    displayState = DisplayState::Ready;
}

void setFeedbackOutputs(bool active){
    if (active) {
        digitalWrite(FEEDBACK_LED_PIN, HIGH);
        ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_ON_DUTY);
        return;
    }

    digitalWrite(FEEDBACK_LED_PIN, LOW);
    ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_OFF_DUTY);
}

void printScanEventJson(const ScanEvent& event){
    char uidText[MAX_UID_LENGTH * 3] = {};
    size_t position = 0;

    for (size_t index = 0; index < event.uidLength; ++index) {
        int writtenCharacterCount = snprintf(
            uidText + position,
            sizeof(uidText) - position,
            index + 1 < event.uidLength ? "%02X:" : "%02X",
            event.uid[index]
        );

        if (writtenCharacterCount < 0) {
            Serial.println("Failed to format UID");
            return;
        }

        position += static_cast<size_t>(writtenCharacterCount);
    }

    JsonDocument document;

    document["event"] = "rfid_scan";
    document["event_id"] = event.eventId;
    document["uptime_ms"] = event.occurredAtMillis;
    document["uid"] = uidText;
    document["card_type"] = rfid.PICC_GetTypeName(event.cardType);
    
    if (event.matchedTag == nullptr) {
        document["access"] = "denied";
        document["tag_name"] = nullptr;
    } else {
        document["access"] = "granted";
        document["tag_name"] = event.matchedTag->name;
    }

    char timestampText[21] = {};

    if (event.hasValidTimestamp && formatUtcTimestamp(event.occurredAtEpoch, timestampText, sizeof(timestampText))){
        document["timestamp_utc"] = timestampText;
    } else {
        document["timestamp_utc"] = nullptr;
    }

    serializeJson(document, Serial);
    Serial.println();
}

void startWifiConnection(unsigned long currentMillis){
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastWifiConnectionAttemptMillis = currentMillis;
}

void updateNetwork(unsigned long currentMillis) {
    bool isWifiConnected = WiFi.status() == WL_CONNECTED;

    if (isWifiConnected) {
        if (!wasWifiConnected) {
            Serial.println("WiFi Connected");

            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());

            Serial.print("Signal strength: ");
            Serial.print(WiFi.RSSI());
            Serial.println(" dBm");
        }

        wasWifiConnected = true;

        if (!hasConfiguredNtp) {
            configTime(
                0,
                0,
                NTP_SERVER_PRIMARY,
                NTP_SERVER_SECONDARY
            );

            hasConfiguredNtp = true;
            Serial.println("NTP synchronization requested");
        }

        return;
    }

    if (wasWifiConnected) {
        Serial.println("WiFi Disconnected");
    }

    wasWifiConnected = false;

    unsigned long elapsedMillis = currentMillis - lastWifiConnectionAttemptMillis;

    if (elapsedMillis < WIFI_RECONNECT_INTERVAL_MS) {
        return;
    }

    Serial.println("Attempting WiFi Reconnecting...");

    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastWifiConnectionAttemptMillis = currentMillis;
}

bool formatUtcTimestamp(time_t epoch, char* destination, size_t destinationSize) {
    if (destination == nullptr || destinationSize == 0) {
        return false;
    }

    struct tm utcTime {};

    if (gmtime_r(&epoch, &utcTime) == nullptr) {
        return false;
    }

    size_t writtenCharacterCount = strftime(
        destination,
        destinationSize,
        "%Y-%m-%dT%H:%M:%SZ",
        &utcTime
    );

    return writtenCharacterCount > 0;
}

void updateMqtt(unsigned long currentMillis){
    if (WiFi.status() != WL_CONNECTED) {
        if (mqttClient.connected()) {
            mqttClient.disconnect();
        }

        wasMqttConnected = false;
        return;
    }

    if (mqttClient.connected()) {
        if (!wasMqttConnected) {
            Serial.println("MQTT Connected");
        }

        wasMqttConnected = true;
        mqttClient.loop();
        return;
    }

    if (wasMqttConnected) Serial.println("MQTT Disconnected");

    wasMqttConnected = false;

    unsigned long elapsedMillis = currentMillis - lastMqttConnectionAttemptMillis;

    if (hasAttemptedMqttConnection && elapsedMillis < MQTT_RECONNECT_INTERVAL_MS) return;

    hasAttemptedMqttConnection = true;
    lastMqttConnectionAttemptMillis = currentMillis;

    Serial.print("Connecting to MQTT broker: ");
    Serial.print(MQTT_BROKER_HOST);
    Serial.print(':');
    Serial.println(MQTT_BROKER_PORT);

    if (!mqttClient.connect(MQTT_CLIENT_ID)) {
        Serial.print("MQTT Connection failed. State: ");
        Serial.println(mqttClient.state());
        return;
    }

    Serial.println("MQTT Connection established");

    bool published = mqttClient.publish(MQTT_STATUS_TOPIC, "{\"status\":\"online\"}", true);

    if (published) Serial.println("MQTT online status published");
    else Serial.println("Failed to publish MQTT online status");
}