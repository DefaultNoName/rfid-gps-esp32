// Modified ESP32 Sensor Code - BLE Only Communication

#include <Arduino.h>
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <HardwareSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// System States
enum system_state_t
{
    STATE_INIT,
    STATE_BLE_SCANNING,
    STATE_BLE_CONNECTING,
    STATE_BLE_CONNECTED,
    STATE_MODULE_INIT,
    STATE_RUNNING,
    STATE_ERROR
};

// Component status tracking
struct component_status_t
{
    bool ble_ok = false;
    bool rfid_ok = false;
    bool gps_ok = false;
    String last_error = "";
};

// Timing structure for GPS time handling
struct timing_data_t
{
    unsigned long millis_timestamp;
    String gps_time_string;
    bool gps_time_valid;
    unsigned long gps_reference_millis;
    String calculated_time;
};

// RFID MFRC522 pins to ESP32
#define SS_PIN 5
#define RST_PIN 22

// GPS pins - Using HardwareSerial - UART2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// LED indicators
#define BUILTIN_LED_PIN 2
#define RFID_LED_PIN 4

// BLE Configuration - Match bridge device
#define BLE_DEVICE_NAME "Tracker-ESP32#1"
#define BLE_BRIDGE_NAME "ESP-BLE" // Name set in bridge ESP
#define BLE_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_UUID_TX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_UUID_RX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Global variables
system_state_t current_system_state = STATE_INIT;
component_status_t components;

// BLE variables
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
BLEScan *pBLEScan = nullptr;
bool deviceConnected = false;
bool doConnect = false;
std::string deviceAddress = "";

// Timing variables
unsigned long last_ble_attempt = 0;
unsigned long last_rfid_read = 0;
unsigned long last_gps_update = 0;
unsigned long last_json_send = 0;
unsigned long last_health_check = 0;
unsigned long state_timeout = 0;

// Vehicle constants
const char *vehicle_brand = "Toyota";
const char *vehicle_model = "Tamaraw FX";
const char *vehicle_plate = "UAM981";

// Hardware objects
MFRC522DriverPinSimple ss_pin(SS_PIN);
MFRC522DriverSPI driver{ss_pin};
MFRC522 module_rfid{driver};
TinyGPSPlus module_gps;
HardwareSerial serial_gps(2);

// Data storage
String current_rfid = "N/A";
String time_exit = "N/A";
String time_entry = "N/A";
String trip_duration = "N/A";
String current_gps_time = "N/A";
String current_gps_lat = "N/A";
String current_gps_lng = "N/A";
String current_gps_alt = "N/A";
String current_gps_speed = "N/A";
String current_gps_sats = "N/A";
String current_gps_hdop = "N/A";

// Global timing variables
timing_data_t exit_timing;
timing_data_t entry_timing;
unsigned long gps_became_available_at = 0;
bool gps_ever_available = false;

// LED indicators
#define BUILTIN_LED 2
#define EXTERNAL_LED_RFID 4

// Get ESP32 MAC address
String get_phys_addr()
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[13];
    sprintf(mac_str, "%02X%02X%02X%02X%02X%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(mac_str);
}

String phys_addr_str = "";
const char *phys_addr = nullptr;

// BLE Callbacks for device scanning
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        Serial.printf("Found BLE device: %s\n", advertisedDevice.toString().c_str());

        // Look for your bridge device by name
        if (advertisedDevice.getName() == BLE_BRIDGE_NAME)
        {
            Serial.println("Found our bridge device!");
            deviceAddress = advertisedDevice.getAddress().toString();
            doConnect = true;
            advertisedDevice.getScan()->stop();
        }
    }
};

// BLE Client callbacks for connection management
class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pclient)
    {
        Serial.println("Connected to BLE bridge");
        deviceConnected = true;
        components.ble_ok = true;

        // Turn on LED to indicate connection
        digitalWrite(BUILTIN_LED_PIN, LOW); // Active LOW
        digitalWrite(RFID_LED_PIN, HIGH);
    }

    void onDisconnect(BLEClient *pclient)
    {
        Serial.println("Disconnected from BLE bridge");
        deviceConnected = false;
        components.ble_ok = false;

        // Turn off LEDs
        digitalWrite(BUILTIN_LED_PIN, HIGH);
        digitalWrite(RFID_LED_PIN, LOW);

        // Return to scanning state
        current_system_state = STATE_BLE_SCANNING;
    }
};

// BLE transmission function - This is where JSON gets sent!
void sendJsonViaBLE(const String &jsonData)
{
    if (!deviceConnected || pRemoteCharacteristic == nullptr)
    {
        Serial.println("BLE not connected - cannot send JSON");
        return;
    }

    // BLE has a maximum packet size (typically 20 bytes for BLE 4.1)
    // We need to fragment large JSON data
    const size_t maxPacketSize = 20;
    size_t dataLength = jsonData.length();

    Serial.printf("Sending JSON data (%d bytes) via BLE:\n", dataLength);
    Serial.println(jsonData);

    // Send data in chunks
    for (size_t i = 0; i < dataLength; i += maxPacketSize)
    {
        size_t chunkSize = min(maxPacketSize, dataLength - i);
        String chunk = jsonData.substring(i, i + chunkSize);

        try
        {
            pRemoteCharacteristic->writeValue(chunk.c_str(), chunk.length());
            Serial.printf("Sent chunk %d: %s\n", i / maxPacketSize + 1, chunk.c_str());
            delay(50); // Small delay between chunks to avoid overwhelming the bridge
        }
        catch (const std::exception &e)
        {
            Serial.printf("Error sending chunk: %s\n", e.what());
            break;
        }
    }

    // Send end marker to indicate complete JSON
    delay(100);
    pRemoteCharacteristic->writeValue("\n", 1);
    Serial.println("JSON transmission complete");
}

// State machine functions
void handle_init_state()
{
    Serial.println("=== BLE-Only Sensor System Initialization ===");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

    // Get MAC address
    phys_addr_str = get_phys_addr();
    phys_addr = phys_addr_str.c_str();
    Serial.println("Device MAC: " + phys_addr_str);

    // Initialize GPS serial
    serial_gps.begin(115200);

    // Initialize LED pins
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(RFID_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off (active LOW)
    digitalWrite(RFID_LED_PIN, LOW);     // Turn off

    // Move to BLE scanning
    current_system_state = STATE_BLE_SCANNING;
    Serial.println("Moving to BLE scanning state");
}

void handle_ble_scanning()
{
    Serial.println("Initializing BLE and scanning for bridge...");

    // Initialize BLE
    BLEDevice::init(BLE_DEVICE_NAME);

    // Create and configure scanner
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);

    // Start scanning
    Serial.printf("Scanning for bridge device: %s\n", BLE_BRIDGE_NAME);
    pBLEScan->start(30, false); // Scan for 30 seconds

    // Move to connecting state
    current_system_state = STATE_BLE_CONNECTING;
    state_timeout = millis() + 35000; // 35 second timeout
}

void handle_ble_connecting()
{
    // Check if we found a device to connect to
    if (doConnect)
    {
        Serial.printf("Attempting connection to: %s\n", deviceAddress.c_str());

        // Create BLE client
        pClient = BLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback());

        // Connect to the device
        try
        {
            pClient->connect(BLEAddress(deviceAddress));
            Serial.println("Connected to BLE bridge");

            // Get the service
            BLERemoteService *pRemoteService = pClient->getService(BLE_SERVICE_UUID);
            if (pRemoteService == nullptr)
            {
                Serial.println("Failed to find UART service");
                pClient->disconnect();
                current_system_state = STATE_BLE_SCANNING;
                return;
            }

            // Get TX characteristic (for sending data)
            pRemoteCharacteristic = pRemoteService->getCharacteristic(BLE_CHAR_UUID_TX);
            if (pRemoteCharacteristic == nullptr)
            {
                Serial.println("Failed to find TX characteristic");
                pClient->disconnect();
                current_system_state = STATE_BLE_SCANNING;
                return;
            }

            // Get RX characteristic (for receiving data) - optional for your use case
            BLERemoteCharacteristic *pRxCharacteristic = pRemoteService->getCharacteristic(BLE_CHAR_UUID_RX);
            if (pRxCharacteristic != nullptr)
            {
                pRxCharacteristic->registerForNotify([](BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                                        uint8_t *pData, size_t length, bool isNotify)
                                                     { Serial.printf("Received from bridge: %.*s\n", length, pData); });
            }

            // Connection successful
            deviceConnected = true;
            components.ble_ok = true;
            current_system_state = STATE_MODULE_INIT;
            doConnect = false;
        }
        catch (const std::exception &e)
        {
            Serial.printf("Connection failed: %s\n", e.what());
            current_system_state = STATE_BLE_SCANNING;
            doConnect = false;
        }
    }

    // Check for timeout
    if (millis() > state_timeout)
    {
        Serial.println("BLE connection timeout, restarting scan");
        current_system_state = STATE_BLE_SCANNING;
        doConnect = false;
    }
}

void handle_hardware_init()
{
    Serial.println("Initializing hardware modules...");

    // Initialize SPI for RFID
    SPI.begin();
    delay(100);

    // Test RFID
    try
    {
        module_rfid.PCD_Init();
        delay(100);
        byte version = module_rfid.PCD_GetVersion();

        if (version != 0x00 && version != 0xFF)
        {
            Serial.printf("RFID OK - Version: 0x%02X\n", version);
            components.rfid_ok = true;
        }
        else
        {
            Serial.println("RFID not detected");
            components.rfid_ok = false;
        }
    }
    catch (...)
    {
        Serial.println("RFID initialization failed");
        components.rfid_ok = false;
    }

    // Test GPS
    unsigned long start_time = millis();
    bool gps_data_seen = false;

    while (millis() - start_time < 2000)
    {
        if (serial_gps.available())
        {
            serial_gps.read();
            gps_data_seen = true;
            break;
        }
        delay(10);
    }

    components.gps_ok = gps_data_seen;
    Serial.printf("GPS: %s\n", components.gps_ok ? "OK" : "No data");

    // Move to running state
    current_system_state = STATE_RUNNING;
    Serial.println("System ready - BLE communication active");
}

// GPS helper functions
String format_gps_time_to_local(TinyGPSTime &gps_time)
{
    if (!gps_time.isValid())
        return "N/A";

    uint32_t utc_in_seconds = gps_time.hour() * 3600 + gps_time.minute() * 60 + gps_time.second();
    uint32_t local_seconds = (utc_in_seconds + 28800) % 86400; // GMT+8

    uint8_t hours = local_seconds / 3600;
    uint8_t minutes = (local_seconds % 3600) / 60;
    uint8_t seconds = local_seconds % 60;

    char time_str[12];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes, seconds);
    return String(time_str);
}

void handle_gps_reading()
{
    while (serial_gps.available())
    {
        if (module_gps.encode(serial_gps.read()))
        {
            if (module_gps.time.isValid())
            {
                current_gps_time = format_gps_time_to_local(module_gps.time);
                if (!gps_ever_available)
                {
                    gps_became_available_at = millis();
                    gps_ever_available = true;
                }
            }

            if (module_gps.location.isValid())
            {
                current_gps_lat = String(module_gps.location.lat(), 8);
                current_gps_lng = String(module_gps.location.lng(), 8);
            }

            current_gps_alt = module_gps.altitude.isValid() ? String(module_gps.altitude.meters(), 4) : "N/A";
            current_gps_speed = module_gps.speed.isValid() ? String(module_gps.speed.kmph(), 4) : "N/A";
            current_gps_sats = module_gps.satellites.isValid() ? String(module_gps.satellites.value()) : "N/A";
            current_gps_hdop = module_gps.hdop.isValid() ? String(module_gps.hdop.hdop(), 4) : "N/A";
        }
    }
}

// RFID helper functions (simplified from original)
void handle_rfid_reading()
{
    static String last_scanned_uid = "";
    static bool is_on_travel = false;
    static unsigned long last_valid_read = 0;
    static unsigned long start_time_millis = 0;

    if (!module_rfid.PICC_IsNewCardPresent() || !module_rfid.PICC_ReadCardSerial())
    {
        return;
    }

    // Debouncing
    if ((millis() - last_valid_read) < 5000)
    {
        digitalWrite(BUILTIN_LED_PIN, LOW);
        digitalWrite(RFID_LED_PIN, HIGH);
        module_rfid.PICC_HaltA();
        module_rfid.PCD_StopCrypto1();
        return;
    }

    // Read UID
    String new_scanned_uid = "";
    for (byte i = 0; i < module_rfid.uid.size; i++)
    {
        if (module_rfid.uid.uidByte[i] < 0x10)
        {
            new_scanned_uid += "0";
        }
        new_scanned_uid += String(module_rfid.uid.uidByte[i], HEX);
    }
    new_scanned_uid.toUpperCase();

    // Handle scan logic
    if (last_scanned_uid == "" || !is_on_travel)
    {
        // EXIT scan
        last_scanned_uid = new_scanned_uid;
        current_rfid = new_scanned_uid;
        is_on_travel = true;
        start_time_millis = millis();
        time_exit = current_gps_time;
        Serial.println("EXIT: " + new_scanned_uid + " @ " + time_exit);
    }
    else if (new_scanned_uid == last_scanned_uid && is_on_travel)
    {
        // ENTRY scan
        is_on_travel = false;
        time_entry = current_gps_time;

        // Calculate duration
        unsigned long duration_ms = millis() - start_time_millis;
        uint16_t hours = duration_ms / 3600000;
        uint16_t minutes = (duration_ms % 3600000) / 60000;
        uint16_t seconds = (duration_ms % 60000) / 1000;

        char duration_str[12];
        snprintf(duration_str, sizeof(duration_str), "%02d:%02d:%02d", hours, minutes, seconds);
        trip_duration = String(duration_str);

        Serial.println("ENTRY: " + new_scanned_uid + " @ " + time_entry);
        Serial.println("Duration: " + trip_duration);
    }

    last_valid_read = millis();

    // Cleanup
    module_rfid.PICC_HaltA();
    module_rfid.PCD_StopCrypto1();

    digitalWrite(BUILTIN_LED_PIN, HIGH);
    digitalWrite(RFID_LED_PIN, LOW);
}

// Main JSON transmission function
void handle_json_transmission()
{
    // Create JSON document
    JsonDocument doc;
    doc["phys_addr"] = phys_addr;
    doc["brand"] = vehicle_brand;
    doc["model"] = vehicle_model;
    doc["pltNum"] = vehicle_plate;
    doc["uid"] = current_rfid;
    doc["time_EXIT"] = time_exit;
    doc["time_ENTRY"] = time_entry;
    doc["trip_duration"] = trip_duration;
    doc["GPS_time"] = current_gps_time;
    doc["lat"] = current_gps_lat;
    doc["long"] = current_gps_lng;
    doc["alt"] = current_gps_alt;
    doc["spd"] = current_gps_speed;
    doc["sat"] = current_gps_sats;
    doc["hdop"] = current_gps_hdop;

    // Serialize to string
    String json;
    serializeJson(doc, json);

    // Send via BLE
    sendJsonViaBLE(json);
}

void handle_running_state()
{
    unsigned long current_time = millis();

    // Check BLE connection
    if (!deviceConnected)
    {
        Serial.println("BLE disconnected, returning to scanning");
        current_system_state = STATE_BLE_SCANNING;
        return;
    }

    // Handle RFID reading
    if (components.rfid_ok && current_time - last_rfid_read > 100)
    {
        handle_rfid_reading();
        last_rfid_read = current_time;
    }

    // Handle GPS reading
    if (components.gps_ok && current_time - last_gps_update > 500)
    {
        handle_gps_reading();
        last_gps_update = current_time;
    }

    // Handle JSON transmission via BLE
    if (components.ble_ok && current_time - last_json_send > 5000)
    {
        handle_json_transmission();
        last_json_send = current_time;
    }

    // Health check
    if (current_time - last_health_check > 15000)
    {
        Serial.printf("Health: Heap=%d, BLE=%s, RFID=%s, GPS=%s\n",
                      ESP.getFreeHeap(),
                      components.ble_ok ? "OK" : "FAIL",
                      components.rfid_ok ? "OK" : "FAIL",
                      components.gps_ok ? "OK" : "FAIL");
        last_health_check = current_time;
    }
}

void handle_error_state()
{
    Serial.printf("System Error: %s\n", components.last_error.c_str());
    Serial.println("Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    setCpuFrequencyMhz(240);

    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(RFID_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off (active LOW)
    digitalWrite(RFID_LED_PIN, LOW);     // Turn off

    Serial.println("=== Vehicle Tracking System (BLE) ===");
    current_system_state = STATE_INIT;
}

void loop()
{
    switch (current_system_state)
    {
    case STATE_INIT:
        handle_init_state();
        break;
    case STATE_BLE_SCANNING:
        handle_ble_scanning();
        break;
    case STATE_BLE_CONNECTING:
        handle_ble_connecting();
        break;
    case STATE_MODULE_INIT:
        handle_hardware_init();
        break;
    case STATE_RUNNING:
        handle_running_state();
        break;
    case STATE_ERROR:
        handle_error_state();
        break;
    }
    delay(10);
}
