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

const int RFID_SS_PIN = 5;      // Pin for RFID SS
const int RFID_SCK_PIN = 18;    // Pin for RFID SCK
const int RFID_MOSI_PIN = 23;   // Pin for RFID MOSI
const int RFID_MISO_PIN = 19;   // Pin for RFID MISO
const int RFID_RST_PIN = 22;    // Pin for RFID RST
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
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 10000;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;
const unsigned long TRANSACTION_SESSION_TIMEOUT_MS = 15000;
const size_t MAX_UID_LENGTH = 10;
const size_t MQTT_JSON_BUFFER_SIZE = 384;
const size_t MAX_MQTT_COMMAND_LENGTH = 32;
const size_t MQTT_COMMAND_RESPONSE_BUFFER_SIZE = 256;
const size_t MQTT_TRANSACTION_JSON_BUFFER_SIZE = 512;
const time_t MIN_VALID_EPOCH = 1700000000;

const char NTP_SERVER_PRIMARY[] = "pool.ntp.org";
const char NTP_SERVER_SECONDARY[] = "time.google.com";
const char MQTT_BROKER_HOST[] = "192.168.1.2";
const uint16_t MQTT_BROKER_PORT = 1883;
const char MQTT_CLIENT_ID[] = "smart-asset-station-01";
const char MQTT_EVENT_TOPIC[] = "smart-asset/station-01/rfid/events";
const char MQTT_STATUS_TOPIC[] = "smart-asset/station-01/status";
const char MQTT_COMMAND_TOPIC[] = "smart-asset/station-01/commands";
const char MQTT_COMMAND_RESPONSE_TOPIC[] = "smart-asset/station-01/commands/response";
const char MQTT_TRANSACTION_TOPIC[] = "smart-asset/station-01/asset/transactions";
const char MQTT_ONLINE_PAYLOAD[] = "{\"status\":\"online\"}";
const char MQTT_OFFLINE_PAYLOAD[] = "{\"status\":\"offline\"}";

enum class TagRole
{
    User,
    Asset
};

enum class FeedbackType
{
    Success,
    Denied
};

enum class FeedbackPhase
{
    Idle,
    On,
    Off
};

enum class DisplayState
{
    Ready,
    TransactionPrompt,
    ScanResult
};

enum class StationState
{
    Ready,
    WaitingForBorrowAsset,
    WaitingForReturnUser,
    ShowingResult
};

enum class AssetAvailability
{
    Available,
    Borrowed
};

struct RegisteredTag
{
    const byte *uid;
    size_t uidLength;
    TagRole role;
    const char *entityId;
    const char *name;
};

struct ScanEvent
{
    unsigned long eventId;
    byte uid[MAX_UID_LENGTH];
    size_t uidLength;
    MFRC522::PICC_Type cardType;
    const RegisteredTag *matchedTag;
    unsigned long occurredAtMillis;
    time_t occurredAtEpoch;
    bool hasValidTimestamp;
};

struct AssetRuntimeState
{
    const RegisteredTag *asset;
    AssetAvailability availability;
    const RegisteredTag *borrowedBy;
};

byte lastProcessedUid[MAX_UID_LENGTH] = {};
size_t lastProcessedUidLength = 0;
unsigned long lastProcessedScanMillis = 0;
unsigned long nextScanEventId = 1;
unsigned long nextTransactionEventId = 1;
unsigned long lastWifiConnectionAttemptMillis = 0;
unsigned long lastMqttConnectionAttemptMillis = 0;

bool hasProcessedUid = false;
bool wasWifiConnected = false;
bool hasConfiguredNtp = false;
bool hasAttemptedMqttConnection = false;
bool wasMqttConnected = false;

FeedbackPhase feedbackPhase = FeedbackPhase::Idle;
DisplayState displayState = DisplayState::Ready;
StationState stationState = StationState::Ready;
const RegisteredTag *pendingUser = nullptr;
const RegisteredTag *pendingAsset = nullptr;

unsigned long feedbackPhaseStartedMillis = 0;
unsigned long feedbackOnDurationMs = 0;
unsigned long feedbackOffDurationMs = 0;
unsigned long displayStateStartedMillis = 0;
unsigned long transactionSessionStartedMillis = 0;

byte feedbackTargetPulseCount = 0;
byte feedbackCompletedPulseCount = 0;

const byte TAG_ONE_UID[] = {
    0xC3, 0xF5, 0x3E, 0x06};

const byte TAG_TWO_UID[] = {
    0x2C, 0x2A, 0xF0, 0x06};

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
    U8G2_R0,
    U8X8_PIN_NONE);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const RegisteredTag REGISTERED_TAGS[] = {
    {TAG_ONE_UID,
     sizeof(TAG_ONE_UID),
     TagRole::User,
     "USR-001",
     "Raisyad"},
    {TAG_TWO_UID,
     sizeof(TAG_TWO_UID),
     TagRole::Asset,
     "AST-001",
     "Laptop-01"}};

AssetRuntimeState ASSET_STATES[]{
    {&REGISTERED_TAGS[1],
     AssetAvailability::Available,
     nullptr}};

const size_t ASSET_STATE_COUNT = sizeof(ASSET_STATES) / sizeof(ASSET_STATES[0]);
const size_t REGISTERED_TAG_COUNT = sizeof(REGISTERED_TAGS) / sizeof(REGISTERED_TAGS[0]);

const RegisteredTag *findRegisteredTag(const MFRC522::Uid &scannedUid);

MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN); // Create MFRC522 instance
bool uidMatches(const MFRC522::Uid &scannedUid, const byte *expectedUid, size_t expectedLength);

bool isDuplicateScan(const MFRC522::Uid &scannedUid, unsigned long currentMillis);

bool rememberProcessedUid(const MFRC522::Uid &scannedUid, unsigned long currentMillis);

void startFeedback(FeedbackType type, unsigned long currentMillis);

void updateFeedback(unsigned long currentMillis);

void stopFeedback();

void updateRfid(unsigned long currentMillis);

bool buildScanEvent(const MFRC522::Uid &scannedUid, unsigned long currentMillis, ScanEvent &event);

void printScanEvent(const ScanEvent &event);

void handleScanEvent(const ScanEvent &event);

void scanI2cDevices();

void showReadyScreen();

void updateDisplay(unsigned long currentMillis);

void setFeedbackOutputs(bool active);

bool serializeScanEventJson(const ScanEvent &event, char *destination, size_t destinationSize);

void printScanEventJson(const ScanEvent &event);

bool publishScanEvent(const ScanEvent &event);

void startWifiConnection(unsigned long currentMillis);

void updateNetwork(unsigned long currentMillis);

bool formatUtcTimestamp(time_t epoch, char *destination, size_t destinationSize);

void updateMqtt(unsigned long currentMillis);

void handleMqttMessage(char *topic, byte *payload, unsigned int length);

bool publishCommandResponse(const char *command, const char *status, const char *message);

void showBorrowAssetPrompt(const RegisteredTag &user);

void showReturnUserPrompt(const RegisteredTag &asset);

void showStationMessage(const char *title, const char *firstLine, const char *secondLine, unsigned long currentMillis);

void resetStationSession();

void updateStationSession(unsigned long currentMillis);

AssetRuntimeState *findAssetRuntimeState(const RegisteredTag &asset);

bool publishTransactionEvent(const char* operation, const char* result, const char* reason, const RegisteredTag* actorUser, const RegisteredTag* asset, const RegisteredTag* previousBorrower, unsigned long occurredAtMillis);

void setup()
{
    Serial.begin(115200);
    startWifiConnection(millis());
    mqttClient.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    mqttClient.setCallback(handleMqttMessage);
    mqttClient.setKeepAlive(30);
    mqttClient.setSocketTimeout(2);
    mqttClient.setBufferSize(768);
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
    updateStationSession(currentMillis);
    updateRfid(currentMillis);
}

bool uidMatches(const MFRC522::Uid &scannedUid, const byte *expectedUid, size_t expectedLength)
{
    if (expectedUid == nullptr)
        return false;

    if (scannedUid.size != expectedLength)
        return false;

    for (size_t index = 0; index < expectedLength; ++index)
    {
        if (scannedUid.uidByte[index] != expectedUid[index])
        {
            return false;
        }
    }
    return true;
}

bool isDuplicateScan(const MFRC522::Uid &scannedUid, unsigned long currentMillis)
{
    if (!hasProcessedUid)
    {
        return false;
    }

    if (!uidMatches(scannedUid, lastProcessedUid, lastProcessedUidLength))
    {
        return false;
    }

    return currentMillis - lastProcessedScanMillis < DUPLICATE_SCAN_WINDOW_MS;
}

bool rememberProcessedUid(const MFRC522::Uid &scannedUid, unsigned long currentMillis)
{
    if (scannedUid.size > sizeof(lastProcessedUid))
    {
        return false;
    }

    for (size_t index = 0; index < scannedUid.size; ++index)
    {
        lastProcessedUid[index] = scannedUid.uidByte[index];
    }

    lastProcessedUidLength = scannedUid.size;
    lastProcessedScanMillis = currentMillis;
    hasProcessedUid = true;

    return true;
}

const RegisteredTag *findRegisteredTag(const MFRC522::Uid &scannedUid)
{
    for (size_t index = 0; index < REGISTERED_TAG_COUNT; ++index)
    {
        const RegisteredTag &candidate = REGISTERED_TAGS[index];

        if (uidMatches(scannedUid, candidate.uid, candidate.uidLength))
        {
            return &candidate;
        }
    }

    return nullptr;
}

void startFeedback(FeedbackType type, unsigned long currentMillis)
{
    switch (type)
    {
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

void updateFeedback(unsigned long currentMillis)
{
    if (feedbackPhase == FeedbackPhase::Idle)
        return;

    unsigned long elapsedMillis = currentMillis - feedbackPhaseStartedMillis;

    if (feedbackPhase == FeedbackPhase::On)
    {
        if (elapsedMillis < feedbackOnDurationMs)
            return;

        ++feedbackCompletedPulseCount;

        if (feedbackCompletedPulseCount >= feedbackTargetPulseCount)
        {
            stopFeedback();
            return;
        }

        setFeedbackOutputs(false);

        feedbackPhase = FeedbackPhase::Off;
        feedbackPhaseStartedMillis = currentMillis;
        return;
    }

    if (feedbackPhase == FeedbackPhase::Off)
    {
        if (elapsedMillis < feedbackOffDurationMs)
            return;

        setFeedbackOutputs(true);

        feedbackPhase = FeedbackPhase::On;
        feedbackPhaseStartedMillis = currentMillis;
    }
}

void stopFeedback()
{
    setFeedbackOutputs(false);

    feedbackPhase = FeedbackPhase::Idle;
    feedbackTargetPulseCount = 0;
    feedbackCompletedPulseCount = 0;
}

void updateRfid(unsigned long currentMillis)
{
    if (!rfid.PICC_IsNewCardPresent())
    {
        return; // No new card present
    }

    if (!rfid.PICC_ReadCardSerial())
    {
        Serial.println("Card detected, but UID cannot be read");
        return; // Failed to read card serial
    }

    if (isDuplicateScan(rfid.uid, currentMillis))
    {
        Serial.println("Duplicate scan detected. Ignoring.");
        rfid.PICC_HaltA();
        return;
    }

    ScanEvent scanEvent{};

    if (!buildScanEvent(rfid.uid, currentMillis, scanEvent))
    {
        Serial.println("Failed to build scan event.");
        rfid.PICC_HaltA(); // Halt PICC
        return;
    }

    if (!rememberProcessedUid(rfid.uid, currentMillis))
    {
        Serial.println("UID cannot be stored. Scan ignored.");
        rfid.PICC_HaltA();
        return;
    }

    printScanEvent(scanEvent);
    printScanEventJson(scanEvent);
    publishScanEvent(scanEvent);
    handleScanEvent(scanEvent);
    rfid.PICC_HaltA(); // Halt PICC
}

bool buildScanEvent(const MFRC522::Uid &scannedUid, unsigned long currentMillis, ScanEvent &event)
{
    if (scannedUid.size > sizeof(event.uid))
        return false;

    event.eventId = nextScanEventId;
    ++nextScanEventId;

    for (size_t index = 0; index < scannedUid.size; ++index)
    {
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

void printScanEvent(const ScanEvent &event)
{
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

    if (event.matchedTag == nullptr)
    {
        Serial.println("Tag Identity : Unknown");
        Serial.println("This tag is not registered");
    }
    else
    {
        Serial.print("Tag Identity : ");
        Serial.println(event.matchedTag->name);

        Serial.println("Tag registered");
    }
    Serial.println("-----------------------------------");
}

void handleScanEvent(const ScanEvent &event)
{
    if (stationState == StationState::ShowingResult)
    {
        Serial.println("Station is displaying a result. Scan ignored.");
        return;
    }

    if (event.matchedTag == nullptr)
    {
        Serial.println("Unknown tag. Transaction cancelled.");

        startFeedback(
            FeedbackType::Denied,
            event.occurredAtMillis);

        showStationMessage(
            "ACCESS DENIED",
            "Unknown RFID Tag",
            "Session Cancelled",
            event.occurredAtMillis);

        return;
    }

    const RegisteredTag &tag = *event.matchedTag;

    if (stationState == StationState::Ready)
    {
        if (tag.role == TagRole::User)
        {
            pendingUser = &tag;
            pendingAsset = nullptr;

            transactionSessionStartedMillis = event.occurredAtMillis;
            stationState = StationState::WaitingForBorrowAsset;

            Serial.print("Borrow session started by: ");
            Serial.println(tag.name);

            startFeedback(
                FeedbackType::Success,
                event.occurredAtMillis);

            showBorrowAssetPrompt(tag);
            return;
        }
        AssetRuntimeState *assetState =
            findAssetRuntimeState(tag);

        if (assetState == nullptr)
        {
            Serial.println(
                "Asset runtime state was not found");

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "SYSTEM ERROR",
                "Asset Not Configured",
                "Check Registry",
                event.occurredAtMillis);

            return;
        }

        if (
            assetState->availability ==
            AssetAvailability::Available)
        {
            Serial.print(tag.name);
            Serial.println(" is already available");

            publishTransactionEvent(
                "return",
                "denied",
                "asset_not_borrowed",
                nullptr,
                &tag,
                nullptr,
                event.occurredAtMillis
            );

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "RETURN DENIED",
                tag.name,
                "Not Borrowed",
                event.occurredAtMillis);

            return;
        }

        pendingAsset = &tag;
        pendingUser = nullptr;

        transactionSessionStartedMillis =
            event.occurredAtMillis;

        stationState =
            StationState::WaitingForReturnUser;

        Serial.print(
            "Return session started for asset: ");
        Serial.println(tag.name);

        startFeedback(
            FeedbackType::Success,
            event.occurredAtMillis);

        showReturnUserPrompt(tag);
        return;
    }

    if (stationState == StationState::WaitingForBorrowAsset)
    {
        if (tag.role != TagRole::Asset)
        {
            Serial.println("Borrow session expected an asset tag");

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "BORROW CANCELLED",
                "Expected Asset",
                "Start Again",
                event.occurredAtMillis);
            return;
        }
        AssetRuntimeState *assetState =
            findAssetRuntimeState(tag);

        if (assetState == nullptr)
        {
            Serial.println(
                "Asset runtime state was not found");

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "SYSTEM ERROR",
                "Asset Not Configured",
                "Check Registry",
                event.occurredAtMillis);

            return;
        }

        pendingAsset = &tag;

        if (
            assetState->availability ==
            AssetAvailability::Borrowed)
        {
            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            if (assetState->borrowedBy == pendingUser)
            {
                Serial.print(pendingAsset->name);
                Serial.println(
                    " is already borrowed by this user");
                
                publishTransactionEvent(
                    "borrow",
                    "denied",
                    "already_borrowed_by_user",
                    pendingUser,
                    pendingAsset,
                    assetState->borrowedBy,
                    event.occurredAtMillis
                );

                showStationMessage(
                    "ALREADY BORROWED",
                    pendingAsset->name,
                    "Borrowed By You",
                    event.occurredAtMillis);

                return;
            }

            const char *borrowerName =
                assetState->borrowedBy != nullptr
                    ? assetState->borrowedBy->name
                    : "Unknown User";

            Serial.print(pendingAsset->name);
            Serial.print(" is borrowed by: ");
            Serial.println(borrowerName);

            publishTransactionEvent(
                "borrow",
                "denied",
                "asset_borrowed_by_another_user",
                pendingUser,
                pendingAsset,
                assetState->borrowedBy,
                event.occurredAtMillis
            );

            showStationMessage(
                "ASSET UNAVAILABLE",
                pendingAsset->name,
                borrowerName,
                event.occurredAtMillis);

            return;
        }

        bool transactionPublished =
            publishTransactionEvent(
                "borrow",
                "accepted",
                "success",
                pendingUser,
                pendingAsset,
                nullptr,
                event.occurredAtMillis
            );

        if (!transactionPublished) {
            Serial.println(
                "Borrow cancelled because MQTT publish failed"
            );

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis
            );

            showStationMessage(
                "NETWORK ERROR",
                "Borrow Not Saved",
                "Please Try Again",
                event.occurredAtMillis
            );

            return;
        }

        assetState->availability =
            AssetAvailability::Borrowed;

        assetState->borrowedBy = pendingUser;

        Serial.print("Borrow accepted: ");
        Serial.print(pendingUser->name);
        Serial.print(" -> ");
        Serial.println(pendingAsset->name);

        startFeedback(
            FeedbackType::Success,
            event.occurredAtMillis);

        showStationMessage(
            "BORROW ACCEPTED",
            pendingUser->name,
            pendingAsset->name,
            event.occurredAtMillis);

        return;
    }

    if (stationState == StationState::WaitingForReturnUser)
    {
        if (tag.role != TagRole::User)
        {
            Serial.println("Return session expected a user tag");

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "RETURN CANCELLED",
                "Expected user",
                "Start Again",
                event.occurredAtMillis);

            return;
        }
        pendingUser = &tag;

        AssetRuntimeState *assetState =
            findAssetRuntimeState(*pendingAsset);

        if (assetState == nullptr)
        {
            Serial.println(
                "Asset runtime state was not found");

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "SYSTEM ERROR",
                "Asset Not Configured",
                "Check Registry",
                event.occurredAtMillis);

            return;
        }

        /*
         * Pemeriksaan defensif.
         * Seharusnya kondisi ini sudah ditolak ketika
         * asset menjadi tag pertama.
         */
        if (
            assetState->availability ==
            AssetAvailability::Available)
        {
            Serial.println(
                "Return rejected: asset is available");

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis);

            showStationMessage(
                "RETURN DENIED",
                pendingAsset->name,
                "Not Borrowed",
                event.occurredAtMillis);

            return;
        }
        /*
         * Simpan peminjam lama sebelum borrowedBy
         * dikosongkan.
         */
        const RegisteredTag *originalBorrower =
            assetState->borrowedBy;
        
        bool transactionPublished =
            publishTransactionEvent(
                "return",
                "accepted",
                "success",
                pendingUser,
                pendingAsset,
                originalBorrower,
                event.occurredAtMillis
            );

        if (!transactionPublished) {
            Serial.println(
                "Return cancelled because MQTT publish failed"
            );

            startFeedback(
                FeedbackType::Denied,
                event.occurredAtMillis
            );

            showStationMessage(
                "NETWORK ERROR",
                "Return Not Saved",
                "Please Try Again",
                event.occurredAtMillis
            );

            return;
        }

        Serial.print("Return accepted. Asset: ");
        Serial.print(pendingAsset->name);

        Serial.print(", borrowed by: ");

        if (originalBorrower != nullptr)
        {
            Serial.print(originalBorrower->name);
        }
        else
        {
            Serial.print("Unknown");
        }

        Serial.print(", returned by: ");
        Serial.println(pendingUser->name);

        /*
         * Pengembalian diterima.
         * Aset kembali tersedia.
         */
        assetState->availability =
            AssetAvailability::Available;

        assetState->borrowedBy = nullptr;

        startFeedback(
            FeedbackType::Success,
            event.occurredAtMillis);

        showStationMessage(
            "RETURN ACCEPTED",
            pendingAsset->name,
            pendingUser->name,
            event.occurredAtMillis);

        return;
    }
}

void scanI2cDevices()
{
    byte detectedDeviceCount = 0;

    for (byte address = 1; address < 127; ++address)
    {
        Wire.beginTransmission(address);
        byte errorCode = Wire.endTransmission();

        if (errorCode == 0)
        {
            Serial.print("I2C device detected at address 0x");

            if (address < 0x10)
            {
                Serial.print('0');
            }

            Serial.println(address, HEX);
            ++detectedDeviceCount;
        }
    }

    if (detectedDeviceCount == 0)
    {
        Serial.println("No I2C device detected");
    }
    else
    {
        Serial.print("Total I2C devices detected: ");
        Serial.println(detectedDeviceCount);
    }
}

void showReadyScreen()
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 14, "SMART ASSET STATION");

    oled.setFont(u8g2_font_9x15B_tf);
    oled.drawStr(31, 38, "READY");

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(12, 58, "Tap your RFID Tag");

    oled.sendBuffer();

    displayState = DisplayState::Ready;
}

void updateDisplay(unsigned long currentMillis)
{
    if (displayState != DisplayState::ScanResult)
        return;

    unsigned long elapsedMillis = currentMillis - displayStateStartedMillis;

    if (elapsedMillis < DISPLAY_RESULT_DURATION_MS)
        return;

    resetStationSession();
}

void setFeedbackOutputs(bool active)
{
    if (active)
    {
        digitalWrite(FEEDBACK_LED_PIN, HIGH);
        ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_ON_DUTY);
        return;
    }

    digitalWrite(FEEDBACK_LED_PIN, LOW);
    ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_OFF_DUTY);
}

bool serializeScanEventJson(const ScanEvent &event, char *destination, size_t destinationSize)
{
    if (destination == nullptr || destinationSize == 0)
    {
        return false;
    }

    char uidText[MAX_UID_LENGTH * 3] = {};
    size_t position = 0;

    for (size_t index = 0; index < event.uidLength; ++index)
    {
        int writtenCharacterCount = snprintf(
            uidText + position,
            sizeof(uidText) - position,
            index + 1 < event.uidLength ? "%02X:" : "%02X",
            event.uid[index]);

        if (writtenCharacterCount < 0)
            return false;

        size_t writtenSize = static_cast<size_t>(writtenCharacterCount);

        if (writtenSize >= sizeof(uidText) - position)
            return false;

        position += writtenSize;
    }
    JsonDocument document;

    document["event"] = "rfid_scan";
    document["event_id"] = event.eventId;
    document["device_id"] = MQTT_CLIENT_ID;
    document["uptime_ms"] = event.occurredAtMillis;
    document["uid"] = uidText;
    document["card_type"] = rfid.PICC_GetTypeName(event.cardType);

    if (event.matchedTag == nullptr)
    {
        document["recognized"] = false;
        document["entity_id"] = nullptr;
        document["tag_name"] = nullptr;
        document["tag_role"] = nullptr;
    }
    else
    {
        document["recognized"] = true;
        document["entity_id"] =
            event.matchedTag->entityId;

        document["tag_name"] =
            event.matchedTag->name;

        document["tag_role"] =
            event.matchedTag->role == TagRole::User
                ? "user"
                : "asset";
    }

    char timestampText[21] = {};

    if (event.hasValidTimestamp && formatUtcTimestamp(event.occurredAtEpoch, timestampText, sizeof(timestampText)))
    {
        document["timestamp_utc"] = timestampText;
    }
    else
    {
        document["timestamp_utc"] = nullptr;
    }

    size_t requiredSize = measureJson(document);

    if (requiredSize + 1 > destinationSize)
        return false;

    size_t serializedSize = serializeJson(
        document, destination, destinationSize);

    return serializedSize == requiredSize;
}

void printScanEventJson(const ScanEvent &event)
{
    char payload[MQTT_JSON_BUFFER_SIZE] = {};

    if (!serializeScanEventJson(event, payload, sizeof(payload)))
    {
        Serial.println("Failed to serialize scan event JSON");
        return;
    }

    Serial.println(payload);
}

bool publishScanEvent(const ScanEvent &event)
{
    if (!mqttClient.connected())
    {
        Serial.println("MQTT unavailable. Scan was not published");

        return false;
    }

    char payload[MQTT_JSON_BUFFER_SIZE] = {};

    if (!serializeScanEventJson(event, payload, sizeof(payload)))
    {
        Serial.println("Failed to serialize MQTT scan event");
        return false;
    }

    bool published = mqttClient.publish(
        MQTT_EVENT_TOPIC,
        payload,
        false);

    if (published)
    {
        Serial.println("Scan event published to MQTT");
    }
    else
    {
        Serial.println("Failed to publish scan event to MQTT");
    }

    return published;
}

void startWifiConnection(unsigned long currentMillis)
{
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastWifiConnectionAttemptMillis = currentMillis;
}

void updateNetwork(unsigned long currentMillis)
{
    bool isWifiConnected = WiFi.status() == WL_CONNECTED;

    if (isWifiConnected)
    {
        if (!wasWifiConnected)
        {
            Serial.println("WiFi Connected");

            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());

            Serial.print("Signal strength: ");
            Serial.print(WiFi.RSSI());
            Serial.println(" dBm");
        }

        wasWifiConnected = true;

        if (!hasConfiguredNtp)
        {
            configTime(
                0,
                0,
                NTP_SERVER_PRIMARY,
                NTP_SERVER_SECONDARY);

            hasConfiguredNtp = true;
            Serial.println("NTP synchronization requested");
        }

        return;
    }

    if (wasWifiConnected)
    {
        Serial.println("WiFi Disconnected");
    }

    wasWifiConnected = false;

    unsigned long elapsedMillis = currentMillis - lastWifiConnectionAttemptMillis;

    if (elapsedMillis < WIFI_RECONNECT_INTERVAL_MS)
    {
        return;
    }

    Serial.println("Attempting WiFi Reconnecting...");

    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastWifiConnectionAttemptMillis = currentMillis;
}

bool formatUtcTimestamp(time_t epoch, char *destination, size_t destinationSize)
{
    if (destination == nullptr || destinationSize == 0)
    {
        return false;
    }

    struct tm utcTime{};

    if (gmtime_r(&epoch, &utcTime) == nullptr)
    {
        return false;
    }

    size_t writtenCharacterCount = strftime(
        destination,
        destinationSize,
        "%Y-%m-%dT%H:%M:%SZ",
        &utcTime);

    return writtenCharacterCount > 0;
}

void updateMqtt(unsigned long currentMillis)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (mqttClient.connected())
        {
            mqttClient.disconnect();
        }

        wasMqttConnected = false;
        return;
    }

    if (mqttClient.connected())
    {
        if (!wasMqttConnected)
        {
            Serial.println("MQTT Connected");
        }

        wasMqttConnected = true;
        mqttClient.loop();
        return;
    }

    if (wasMqttConnected)
        Serial.println("MQTT Disconnected");

    wasMqttConnected = false;

    unsigned long elapsedMillis = currentMillis - lastMqttConnectionAttemptMillis;

    if (hasAttemptedMqttConnection && elapsedMillis < MQTT_RECONNECT_INTERVAL_MS)
        return;

    hasAttemptedMqttConnection = true;
    lastMqttConnectionAttemptMillis = currentMillis;

    Serial.print("Connecting to MQTT broker: ");
    Serial.print(MQTT_BROKER_HOST);
    Serial.print(':');
    Serial.println(MQTT_BROKER_PORT);

    if (!mqttClient.connect(MQTT_CLIENT_ID, nullptr, nullptr, MQTT_STATUS_TOPIC, 0, true, MQTT_OFFLINE_PAYLOAD))
    {
        Serial.print("MQTT Connection failed. State: ");
        Serial.println(mqttClient.state());
        return;
    }

    Serial.println("MQTT Connection established");

    bool published = mqttClient.publish(MQTT_STATUS_TOPIC, MQTT_ONLINE_PAYLOAD, true);

    if (published)
        Serial.println("MQTT online status published");
    else
        Serial.println("Failed to publish MQTT online status");

    bool subscribed = mqttClient.subscribe(MQTT_COMMAND_TOPIC);

    if (subscribed)
        Serial.println("Subscribed to MQTT command topic");
    else
        Serial.println("Fail to subs to MQTT command topic");
}

void handleMqttMessage(char *topic, byte *payload, unsigned int length)
{
    Serial.print("MQTT message received on topic: ");
    Serial.println(topic);

    if (strcmp(topic, MQTT_COMMAND_TOPIC) != 0)
    {
        Serial.println("Unknown MQTT topic. Ignoring.");
        return;
    }

    if (length == 0 || length > MAX_MQTT_COMMAND_LENGTH)
    {
        Serial.println("Invalid MQTT command length");
        publishCommandResponse("unknown", "error", "Invalid command length");
        return;
    }

    char command[MAX_MQTT_COMMAND_LENGTH + 1] = {};

    memcpy(command, payload, length);
    command[length] = '\0';

    Serial.print("Command: ");
    Serial.println(command);

    if (strcmp(command, "buzzer_test") == 0)
    {
        startFeedback(FeedbackType::Denied, millis());
        Serial.println("Buzzer test executed");

        publishCommandResponse(command, "success", "Buzzer test executed");
        return;
    }

    if (strcmp(command, "show_ready") == 0)
    {
        resetStationSession();

        Serial.println("Ready screen displayed");
        publishCommandResponse(command, "success", "Ready screen displayed");
        return;
    }

    if (strcmp(command, "device_status") == 0)
    {
        bool published = mqttClient.publish(
            MQTT_STATUS_TOPIC,
            MQTT_ONLINE_PAYLOAD,
            true);

        if (published)
        {
            Serial.println("Device status published");
            publishCommandResponse(command, "success", "Device status published");
        }
        else
        {
            Serial.println("Failed to publish device status");
            publishCommandResponse(command, "error", "Failed to publish device status");
        }

        return;
    }

    Serial.println("Unknown MQTT Command");
    publishCommandResponse(command, "error", "Unknown Command");
}

bool publishCommandResponse(const char *command, const char *status, const char *message)
{
    if (!mqttClient.connected())
    {
        Serial.println("Cannot publish command response: MQTT unavailable");
        return false;
    }

    JsonDocument document;

    document["event"] = "command_response";
    document["device_id"] = MQTT_CLIENT_ID;
    document["command"] = command;
    document["status"] = status;
    document["message"] = message;
    document["uptime_ms"] = millis();

    char payload[MQTT_COMMAND_RESPONSE_BUFFER_SIZE] = {};

    size_t requiredSize = measureJson(document);

    if (requiredSize >= sizeof(payload))
    {
        Serial.println("Command response payload is too large");
        return false;
    }

    size_t serializedSize = serializeJson(
        document,
        payload,
        sizeof(payload));

    if (serializedSize != requiredSize)
    {
        Serial.println("Failed to serialize command response JSON");
        return false;
    }

    bool published = mqttClient.publish(
        MQTT_COMMAND_RESPONSE_TOPIC,
        payload,
        false);

    if (published)
    {
        Serial.print("Command response published: ");
        Serial.println(payload);
    }
    else
    {
        Serial.println("Failed to publish command response");
    }

    return published;
}

void showBorrowAssetPrompt(const RegisteredTag &user)
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 12, "BORROW MODE");

    oled.drawStr(0, 30, "User: ");
    oled.drawStr(36, 30, user.name);

    oled.drawStr(0, 54, "Tap Asset Tag");

    oled.sendBuffer();

    displayState = DisplayState::TransactionPrompt;
}

void showReturnUserPrompt(const RegisteredTag &asset)
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);
    oled.drawStr(0, 12, "RETURN MODE");

    oled.drawStr(0, 30, "Asset: ");
    oled.drawStr(36, 30, asset.name);

    oled.drawStr(0, 54, "Tap User Card");

    oled.sendBuffer();

    displayState = DisplayState::TransactionPrompt;
}

void showStationMessage(const char *title, const char *firstLine, const char *secondLine, unsigned long currentMillis)
{
    oled.clearBuffer();

    oled.setFont(u8g2_font_6x12_tf);

    if (title != nullptr)
    {
        oled.drawStr(0, 12, title);
    }

    if (firstLine != nullptr)
    {
        oled.drawStr(0, 34, firstLine);
    }

    if (secondLine != nullptr)
    {
        oled.drawStr(0, 54, secondLine);
    }

    oled.sendBuffer();

    displayState = DisplayState::ScanResult;
    displayStateStartedMillis = currentMillis;

    stationState = StationState::ShowingResult;

    pendingUser = nullptr;
    pendingAsset = nullptr;
    transactionSessionStartedMillis = 0;
}

void resetStationSession()
{
    pendingUser = nullptr;
    pendingAsset = nullptr;

    transactionSessionStartedMillis = 0;
    stationState = StationState::Ready;

    showReadyScreen();

    Serial.println("Station returned to READY state");
}

void updateStationSession(unsigned long currentMillis)
{
    bool isWaitingForSecondTag =
        stationState == StationState::WaitingForBorrowAsset ||
        stationState == StationState::WaitingForReturnUser;

    if (!isWaitingForSecondTag)
        return;

    unsigned long elapsedMillis = currentMillis - transactionSessionStartedMillis;

    if (elapsedMillis < TRANSACTION_SESSION_TIMEOUT_MS)
        return;

    Serial.println("Transaction session timed out");

    startFeedback(FeedbackType::Denied, currentMillis);
    showStationMessage(
        "SESSION TIMEOUT",
        "No second tag",
        "Please Try Again",
        currentMillis);
}

AssetRuntimeState *findAssetRuntimeState(const RegisteredTag &asset)
{
    if (asset.role != TagRole::Asset)
    {
        return nullptr;
    }

    for (size_t index = 0; index < ASSET_STATE_COUNT; ++index)
    {
        AssetRuntimeState &state = ASSET_STATES[index];

        if (strcmp(state.asset->entityId, asset.entityId) == 0)
        {
            return &state;
        }
    }

    return nullptr;
}

bool publishTransactionEvent(const char* operation, const char* result, const char* reason, const RegisteredTag* actorUser, const RegisteredTag* asset, const RegisteredTag* previousBorrower, unsigned long occurredAtMillis){
    if (!mqttClient.connected())
    {
        Serial.println("Cannot publish transaction event: MQTT unavailable");
        return false;
    }

    if (operation == nullptr || result == nullptr || reason == nullptr || asset == nullptr)
    {
        Serial.println("Cannot publish transaction: invalid data");
        return false;

    }

    JsonDocument document;

    document["event"] = "asset_transaction";
    document["transaction_id"] = nextTransactionEventId++;
    document["device_id"] = MQTT_CLIENT_ID;
    document["operation"] = operation;
    document["result"] = result;
    document["reason"] = reason;

    if (actorUser != nullptr)
    {
        document["actor_user_id"] = actorUser->entityId;
        document["actor_user_name"] = actorUser->name;
    } else {
        document["actor_user_id"] = nullptr;
        document["actor_user_name"] = nullptr;
    }

    document["asset_id"] = asset->entityId;
    document["asset_name"] = asset->name;

    if (previousBorrower != nullptr)
    {
        document["previous_borrower_id"] = previousBorrower->entityId;
        document["previous_borrower_name"] = previousBorrower->name;
    } else {
        document["previous_borrower_id"] = nullptr;
        document["previous_borrower_name"] = nullptr;
    }

    document["uptime_ms"] = occurredAtMillis;

    time_t currentEpoch = time(nullptr);
    char timestampText[21] = {};

    if (currentEpoch >= MIN_VALID_EPOCH && formatUtcTimestamp(currentEpoch, timestampText, sizeof(timestampText)))
    {
        document["timestamp_utc"] = timestampText;
    }
    else
    {
        document["timestamp_utc"] = nullptr;
    }

    char payload[MQTT_TRANSACTION_JSON_BUFFER_SIZE] = {};

    size_t requiredSize = measureJson(document);

    if (requiredSize >= sizeof(payload))
    {
        Serial.println("Transaction event payload is too large");
        return false;
    }

    size_t serializedSize = serializeJson(
        document,
        payload,
        sizeof(payload)
    );

    if (serializedSize != requiredSize)
    {
        Serial.println("Failed to serialize transaction JSON");
        return false;
    }

    Serial.print("Transaction event: ");
    Serial.println(payload);

    bool published = mqttClient.publish(
        MQTT_TRANSACTION_TOPIC,
        payload,
        false
    );

    if (published)
    {
        Serial.println("Transaction event published to MQTT");
    }
    else
    {
        Serial.println("Failed to publish transaction event to MQTT");
    }

    return published;
}