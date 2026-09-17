#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <U8g2lib.h>

const int RFID_SS_PIN = 5;    // Pin for RFID SS
const int RFID_SCK_PIN = 18;  // Pin for RFID SCK
const int RFID_MOSI_PIN = 23; // Pin for RFID MOSI
const int RFID_MISO_PIN = 19; // Pin for RFID MISO
const int RFID_RST_PIN = 22;  // Pin for RFID RST
const int FEEDBACK_LED_PIN = 2; // Pin for LED ESP32 Configuration (GPIO2)
const int OLED_SDA_PIN = 21;
const int OLED_SCL_PIN = 4;
const int BUZZER_PIN = 15;

const unsigned long DUPLICATE_SCAN_WINDOW_MS = 3000;
const unsigned long DISPLAY_RESULT_DURATION_MS = 3000;
const size_t MAX_UID_LENGTH = 10;
const int BUZZER_ON_LEVEL = LOW;
const int BUZZER_OFF_LEVEL = HIGH;

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
    byte uid[MAX_UID_LENGTH];
    size_t uidLength;
    MFRC522::PICC_Type cardType;
    const RegisteredTag* matchedTag;
    unsigned long occurredAtMillis;
};

byte lastProcessedUid[MAX_UID_LENGTH] = {};
size_t lastProcessedUidLength = 0;
unsigned long lastProcessedScanMillis = 0;
bool hasProcessedUid = false;

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

void setup()
{
    Serial.begin(115200);
    digitalWrite(BUZZER_PIN, BUZZER_OFF_LEVEL);
    pinMode(BUZZER_PIN, OUTPUT);

    Serial.println("Testing Passive Buzzer...");
    tone(BUZZER_PIN, 2500);
    delay(500);
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, BUZZER_OFF_LEVEL);

    Serial.println("Buzzer Test Complete");

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

    digitalWrite(FEEDBACK_LED_PIN ,HIGH);
}

void updateFeedback(unsigned long currentMillis) {
    if (feedbackPhase == FeedbackPhase::Idle) return;

    unsigned long elapsedMillis = currentMillis - feedbackPhaseStartedMillis;

    if (feedbackPhase == FeedbackPhase::On) {
        if (elapsedMillis < feedbackOnDurationMs) return;

        digitalWrite(FEEDBACK_LED_PIN ,LOW);
        ++feedbackCompletedPulseCount;

        if (feedbackCompletedPulseCount >= feedbackTargetPulseCount) {
            stopFeedback();
            return;
        }

        feedbackPhase = FeedbackPhase::Off;
        feedbackPhaseStartedMillis = currentMillis;
        return;
    }

    if (feedbackPhase == FeedbackPhase::Off) {
        if (elapsedMillis < feedbackOffDurationMs) return;

        digitalWrite(FEEDBACK_LED_PIN, HIGH);

        feedbackPhase = FeedbackPhase::On;
        feedbackPhaseStartedMillis = currentMillis; 
    }
}

void stopFeedback(){
    digitalWrite(FEEDBACK_LED_PIN ,LOW);

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
    handleScanEvent(scanEvent);
    rfid.PICC_HaltA(); // Halt PICC
}

bool buildScanEvent(const MFRC522::Uid& scannedUid, unsigned long currentMillis, ScanEvent& event){
    if (scannedUid.size > sizeof(event.uid)) return false;

    for (size_t index = 0; index < scannedUid.size; ++index) {
        event.uid[index] = scannedUid.uidByte[index];
    }

    event.uidLength = scannedUid.size;
    event.cardType = rfid.PICC_GetType(scannedUid.sak);
    event.matchedTag = findRegisteredTag(scannedUid);
    event.occurredAtMillis = currentMillis;

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