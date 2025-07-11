// Proof-of-Concept Project - ESP32 Port

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
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

// Wi-Fi credentials
const char *ssid = "ESP32-Server";       // Server SSID
const char *password = "01234567890";    // Server Passphrase
const char *server_ip = "192.168.120.1"; // Server IP
const int server_port = 80;
const char *upload_path = "/upload";
IPAddress sensor_ip(192, 168, 120, 100); // Fixed IP for this mcu
IPAddress gateway(192, 168, 120, 1);     // ESP32-AP gateway
IPAddress subnet(255, 255, 255, 0);      // Subnet mask
IPAddress dns(192, 168, 120, 1);         // DNS

// RFID MFRC522 pins to ESP32
#define SS_PIN 5   // GPIO5 (SS/CS pin for RFID)
#define RST_PIN 22 // GPIO22 (RST pin for RFID)
// Note: SPI pins are fixed on ESP8266: SCK=GPIO14(D5), MOSI=GPIO13(D7), MISO=GPIO12(D6)

// GPS pins - Using HardwareSerial - UART2
#define GPS_RX_PIN 16 // GPIO16 (connect to GPS TX)
#define GPS_TX_PIN 17 // GPIO5 (connect to GPS RX)

// LED indicators
#define BUILTIN_LED_PIN 2 // GPIO2 (onboard LED - ESP32 also uses GPIO2)
#define RFID_LED_PIN 4    // GPIO4 (external LED indicator)

// System States for Pseudo-Multitasking
enum system_state_t
{
    STATE_INIT,
    STATE_WIFI_CONNECTING,
    STATE_WIFI_CONNECTED,
    STATE_WIFI_RECONNECTION,
    STATE_BLE_CONNECTING,
    STATE_BLE_CONNECTED,
    STATE_BLE_RECONNECTION,
    STATE_MODULE_INIT,
    STATE_RUNNING,
    STATE_ERROR
};

// Component status tracking
struct component_status_t
{
    bool wifi_ok = false;
    bool ble_ok = false;
    bool rfid_ok = false;
    bool gps_ok = false;
    bool http_ok = false;
    String last_error = "";
};

// Timing structure to handle GPS time availability
struct timing_data_t
{
    unsigned long millis_timestamp;     // Always available - system reference
    String gps_time_string;             // GPS time if available
    bool gps_time_valid;                // Whether GPS time was available at capture
    unsigned long gps_reference_millis; // millis() when GPS became available
    String calculated_time;             // Final calculated time (GPS or estimated)
};

// Global variables
system_state_t current_system_state = STATE_INIT;
component_status_t components;
WiFiClient client;
HTTPClient http;

// Global timing variables
timing_data_t exit_timing;
timing_data_t entry_timing;
unsigned long gps_became_available_at = 0; // When GPS first became valid
bool gps_ever_available = false;

// Hardware variables - Fixed pin assignments
MFRC522DriverPinSimple ss_pin(SS_PIN);
MFRC522DriverSPI driver{ss_pin};
MFRC522 module_rfid{driver};
TinyGPSPlus module_gps;
HardwareSerial serial_gps(2);

// Variables to track BLE connection
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
bool deviceConnected = false;
bool doConnect = false;
std::string deviceAddress = "";

// Timing variables
unsigned long last_wifi_attempt = 0;
unsigned long last_ble_attempt = 0;
unsigned long last_rfid_read = 0;
unsigned long last_gps_update = 0;
unsigned long last_http_send = 0;
unsigned long last_health_check = 0;
unsigned long state_timeout = 0;

// Vehicle constants
const char *vehicle_brand = "Toyota";
const char *vehicle_model = "Tamaraw FX";
const char *vehicle_plate = "UAM981";

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

// Fixed MAC address retrieval for ESP8266
String get_phys_addr()
{
    String mac = WiFi.macAddress();
    mac.replace(":", ""); // Remove colons to match format
    return mac;
}
String phys_addr_str = "";       // Will be set in setup()
const char *phys_addr = nullptr; // Will be set in setup()

// State machine functions
void handle_init_state()
{
    Serial.println("=== System Initialization ===");
    Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

    if (ESP.getFreeHeap() < 30000)
    {
        Serial.println("Insufficient memory available, restarting...");
        delay(1000);
        ESP.restart();
    }

    // Initialize serial communication for GPS @ 115200 baud
    serial_gps.begin(115200);

    // Move to WiFi connection state
    current_system_state = STATE_WIFI_CONNECTING;
    last_wifi_attempt = millis();
    state_timeout = millis() + 30000; // 30 second timeout
    Serial.println("Moving to WiFi connection state");
}

void handle_wifi_connecting()
{
    static bool wifi_started = false;
    static int wifi_attempts = 0;

    if (!wifi_started)
    {
        Serial.println("Starting WiFi Connection...");
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(false);
        WiFi.setSleep(false);
        WiFi.config(sensor_ip, gateway, subnet, dns);
        WiFi.begin(ssid, password);
        wifi_started = true;
        Serial.print("Connecting");
        return;
    }

    // Check WiFi status every 500ms
    if (millis() - last_wifi_attempt > 500)
    {
        wl_status_t status = WiFi.status();

        if (status == WL_CONNECTED)
        {
            Serial.println();
            Serial.printf("WiFi Connected! IP: %s, Signal: %d dBm\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            components.wifi_ok = true;
            current_system_state = STATE_WIFI_CONNECTED;
            return;
        }

        if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL)
        {
            Serial.printf("\nWiFi Connection Failed with Status: %d\n", status);
            components.last_error = "Can't connect to WiFi network, check WiFi config.";
            current_system_state = STATE_ERROR;
            return;
        }

        Serial.print(".");
        last_wifi_attempt = millis();
        wifi_attempts++;

        // Timeout check
        if (millis() > state_timeout)
        {
            Serial.println("\nWiFi connection timeout");
            components.last_error = "WiFi connection timeout.";
            current_system_state = STATE_ERROR;
            return;
        }
    }
}

void handle_wifi_connected()
{
    Serial.println("Initializing hardware...");
    current_system_state = STATE_MODULE_INIT;
    state_timeout = millis() + 10000; // 10 second timeout for hardware initialize
}

// This callback gets triggered when we find BLE devices during scanning
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)
    {
        Serial.print("Found device: ");
        Serial.println(advertisedDevice.toString().c_str());

        // Look for your JDY-16 module by name
        if (advertisedDevice.getName() == "VehicleTracker")
        {
            Serial.println("Found our target device!");
            deviceAddress = advertisedDevice.getAddress().toString();
            doConnect = true;
            advertisedDevice.getScan()->stop(); // Stop scanning once we find our device
        }
    }
};

// Callback for when we successfully connect
class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pclient)
    {
        Serial.println("Connected to JDY-16 module");
        deviceConnected = true;
    }

    void onDisconnect(BLEClient *pclient)
    {
        Serial.println("Disconnected from JDY-16 module");
        deviceConnected = false;
    }
};

void handle_ble_connecting()
{
    // Initialize BLE
    BLEDevice::init("Tracker-ESP32#1");

    // Start scanning for devices
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);   // Scan interval in milliseconds
    pBLEScan->setWindow(449);      // Scan window in milliseconds
    pBLEScan->setActiveScan(true); // Active scanning uses more power but gets more info
    pBLEScan->start(30, false);    // Scan for 30 seconds
}

bool connect_to_bridge_esp()
{
    Serial.print("Attempting to connect to ");
    Serial.println(deviceAddress.c_str());

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the BLE device
    pClient->connect(BLEAddress(deviceAddress));
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after (UART service)
    BLERemoteService *pRemoteService = pClient->getService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    if (pRemoteService == nullptr)
    {
        Serial.print("Failed to find UART service UUID");
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found UART service");

    // Get the characteristic used for sending data
    pRemoteCharacteristic = pRemoteService->getCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    if (pRemoteCharacteristic == nullptr)
    {
        Serial.print("Failed to find TX characteristic UUID");
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found TX characteristic");

    // Set up notifications for receiving data
    BLERemoteCharacteristic *pRxCharacteristic = pRemoteService->getCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
    if (pRxCharacteristic != nullptr)
    {
        pRxCharacteristic->registerForNotify([](BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify)
                                             {
            // This callback is triggered when we receive data from the JDY-16
            Serial.print("Received data: ");
            for (int i = 0; i < length; i++) {
                Serial.print((char)pData[i]);
            }
            Serial.println(); });
    }

    return true;
}

void sendJsonData(String jsonData)
{
    if (deviceConnected && pRemoteCharacteristic != nullptr)
    {
        // BLE can handle the fragmentation automatically
        pRemoteCharacteristic->writeValue(jsonData.c_str(), jsonData.length());
        Serial.println("JSON data sent via BLE");
    }
    else
    {
        Serial.println("Not connected - cannot send data");
    }
};

void handle_hardware_init()
{
    static bool spi_initialized = false;
    static bool rfid_tested = false;
    static bool gps_tested = false;

    // Initialize SPI once
    if (!spi_initialized)
    {
        Serial.println("Initializing SPI...");
        SPI.begin();
        delay(100);
        spi_initialized = true;
        Serial.printf("SPI initialized. Free heap: %d\n", ESP.getFreeHeap());
        return;
    }

    // Test RFID
    if (!rfid_tested)
    {
        Serial.println("Testing RFID module...");

        // Simple test - Initialize without complex operations
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
                Serial.println("RFID not detected, continuing without it");
                components.rfid_ok = false;
            }
        }
        catch (...)
        {
            Serial.println("RFID initialization caused exception, now disabled.");
            components.rfid_ok = false;
        }

        rfid_tested = true;
        Serial.printf("After RFID test. Free heap: %d\n", ESP.getFreeHeap());
        return;
    }

    // Test GPS - just check if data is available
    if (!gps_tested)
    {
        Serial.println("Testing GPS module...");

        unsigned long start_time = millis();
        bool gps_data_seen = false;

        // Quick test - just look for any data for 2 seconds
        while (millis() - start_time < 2000)
        {
            if (serial_gps.available())
            {
                serial_gps.read(); // Just consume the data
                gps_data_seen = true;
                break;
            }
            delay(10);
        }

        if (gps_data_seen)
        {
            Serial.println("GPS data detected");
            components.gps_ok = true;
        }
        else
        {
            Serial.println("No GPS data, continuing without it");
            components.gps_ok = false;
        }

        gps_tested = true;
        Serial.printf("After GPS test. Free heap: %d\n", ESP.getFreeHeap());
        return;
    }

    // All hardware tests complete
    Serial.println("Hardware initialization complete");
    Serial.printf("Status: WiFi=%s, RFID=%s, GPS=%s\n",
                  components.wifi_ok ? "OK" : "FAIL",
                  components.rfid_ok ? "OK" : "FAIL",
                  components.gps_ok ? "OK" : "FAIL");

    components.http_ok = true; // HTTP is always available if WiFi works
    current_system_state = STATE_RUNNING;
}

// Helper function to convert GPS time to local time string (GMT+8)
String format_gps_time_to_local(TinyGPSTime &gps_time)
{
    if (!gps_time.isValid())
    {
        return "N/A";
    }

    // Convert to total seconds since midnight UTC
    uint32_t utc_in_seconds = gps_time.hour() * 3600 + gps_time.minute() * 60 + gps_time.second();

    // Add 8 hours for GMT+8 timezone
    uint32_t local_seconds = (utc_in_seconds + 28800) % 86400; // 86400 = seconds in a day

    // Convert back to hours, minutes, seconds
    uint8_t hours = local_seconds / 3600;
    uint8_t minutes = (local_seconds % 3600) / 60;
    uint8_t seconds = local_seconds % 60;

    // Format as HH:MM:SS string
    char time_str[12]; // "HH:MM:SS" + null terminator
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hours, minutes, seconds);

    return String(time_str);
}

// Handle GPS reading
void handle_gps_reading()
{
    static unsigned long GPS_UPDATE_INTERVAL = 1000;
    static unsigned long lastGPSUpdate = 0;

    while (serial_gps.available())
    {
        if (module_gps.encode(serial_gps.read()))
        {
            unsigned long currentTime = millis();
            if (currentTime - lastGPSUpdate >= GPS_UPDATE_INTERVAL)
            {
                // Check time if valid
                if (!module_gps.time.isValid())
                {
                    Serial.println("Time not valid");
                    lastGPSUpdate = currentTime;
                }
                else
                {
                    // Update time
                    current_gps_time = format_gps_time_to_local(module_gps.time);
                    Serial.println("Current time: " + current_gps_time);
                }

                // Check location if valid
                if (!module_gps.location.isValid())
                {
                    Serial.println("Location not valid");
                    lastGPSUpdate = currentTime;
                }
                else
                {
                    // Update location
                    current_gps_lat = String(module_gps.location.lat(), 8);
                    current_gps_lng = String(module_gps.location.lng(), 8);
                    Serial.println("Lat: " + current_gps_lat);
                    Serial.println("Long: " + current_gps_lng);
                }

                // Update other GPS data
                current_gps_alt = module_gps.altitude.isValid() ? String(module_gps.altitude.meters(), 4) : "N/A";
                current_gps_speed = module_gps.speed.isValid() ? String(module_gps.speed.kmph(), 4) : "N/A";
                current_gps_sats = module_gps.satellites.isValid() ? String(module_gps.satellites.value()) : "N/A";
                current_gps_hdop = module_gps.hdop.isValid() ? String(module_gps.hdop.hdop(), 4) : "N/A";
                lastGPSUpdate = currentTime;
            }
        }
    }
}

// Helper function to calculate trip duration
void calculate_trip_duration(uint32_t start_time_millis)
{
    unsigned long duration_ms = millis() - start_time_millis;

    uint16_t hours = duration_ms / 3600000;
    uint16_t minutes = (duration_ms % 3600000) / 60000;
    uint16_t seconds = (duration_ms % 60000) / 1000;

    char duration_str[12];
    snprintf(duration_str, sizeof(duration_str), "%02d:%02d:%02d", hours, minutes, seconds);
    trip_duration = String(duration_str);

    Serial.println("TRIP DURATION: " + trip_duration);
}
// Helper function to capture current time with GPS fallback
timing_data_t capture_current_time()
{
    timing_data_t timing;

    // Always capture millis timestamp as reference
    timing.millis_timestamp = millis();

    // Check if GPS time is currently available
    if (module_gps.time.isValid())
    {
        // GPS is available - use it directly
        timing.gps_time_string = format_gps_time_to_local(module_gps.time);
        timing.gps_time_valid = true;
        timing.calculated_time = timing.gps_time_string;

        // Track when GPS became available for the first time
        if (!gps_ever_available)
        {
            gps_became_available_at = timing.millis_timestamp;
            gps_ever_available = true;
            Serial.println("GPS became available at millis: " + String(gps_became_available_at));
        }

        timing.gps_reference_millis = gps_became_available_at;
    }
    else
    {
        // GPS not available - we'll calculate later if it becomes available
        timing.gps_time_string = "N/A";
        timing.gps_time_valid = false;
        timing.calculated_time = "Wait for GPS fix";
        timing.gps_reference_millis = 0;
    }

    return timing;
}

// Helper function to calculate time from millis offset when GPS becomes available
String calculate_time_from_offset(unsigned long target_millis, unsigned long gps_reference_millis,
                                  const String &gps_reference_time)
{
    if (gps_reference_time == "N/A" || gps_reference_millis == 0)
    {
        return "N/A";
    }

    // Calculate the time difference in milliseconds
    long time_diff_ms = target_millis - gps_reference_millis;

    // Parse the GPS reference time to get seconds since midnight
    int hours, minutes, seconds;
    if (sscanf(gps_reference_time.c_str(), "%d:%d:%d", &hours, &minutes, &seconds) != 3)
    {
        return "N/A";
    }

    // Convert to total seconds since midnight
    long total_seconds = hours * 3600 + minutes * 60 + seconds;

    // Add the offset (convert ms to seconds)
    total_seconds += time_diff_ms / 1000;

    // Handle day overflow/underflow
    if (total_seconds >= 86400)
    {
        total_seconds %= 86400; // Wrap to next day
    }
    else if (total_seconds < 0)
    {
        total_seconds = 86400 + (total_seconds % 86400); // Wrap to previous day
    }

    // Convert back to hours, minutes, seconds
    int calc_hours = total_seconds / 3600;
    int calc_minutes = (total_seconds % 3600) / 60;
    int calc_seconds = total_seconds % 60;

    // Format as time string
    char time_str[12];
    snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", calc_hours, calc_minutes, calc_seconds);

    return String(time_str);
}

// Helper function to update pending timestamps when GPS becomes available
void update_pending_timestamps()
{
    if (!gps_ever_available)
    {
        return; // GPS still not available
    }

    // Get current GPS time as reference
    String current_gps_time = format_gps_time_to_local(module_gps.time);

    // Update exit time if it was pending
    if (exit_timing.calculated_time == "Wait for GPS fix")
    {
        exit_timing.calculated_time = calculate_time_from_offset(
            exit_timing.millis_timestamp,
            gps_became_available_at,
            current_gps_time);
        Serial.println("Updated exit time: " + exit_timing.calculated_time);
    }

    // Update entry time if it was pending
    if (entry_timing.calculated_time == "Wait for GPS fix")
    {
        entry_timing.calculated_time = calculate_time_from_offset(
            entry_timing.millis_timestamp,
            gps_became_available_at,
            current_gps_time);
        Serial.println("Updated entry time: " + entry_timing.calculated_time);
    }
}

// Modified helper function to handle exit scan
void handle_exit_scan(const String &new_scanned_uid, String &last_new_scanned_uid, bool &is_on_travel,
                      unsigned long &last_valid_read, unsigned long &start_time_millis)
{
    last_new_scanned_uid = new_scanned_uid;
    current_rfid = new_scanned_uid;
    last_valid_read = millis();
    is_on_travel = true;

    Serial.println("Internal millis() Timer started");
    start_time_millis = millis();

    // Capture exit timing with GPS fallback
    exit_timing = capture_current_time();
    time_exit = exit_timing.calculated_time;

    Serial.println("EXIT TIME @ " + time_exit + " - UID: " + new_scanned_uid);

    if (exit_timing.gps_time_valid)
    {
        Serial.println("Exit time captured with GPS");
    }
    else
    {
        Serial.println("Exit time pending GPS availability");
    }
}

// Modified helper function to handle entry scan
void handle_entry_scan(const String &new_scanned_uid, String &last_new_scanned_uid, bool &is_on_travel,
                       unsigned long &last_valid_read, unsigned long start_time_millis)
{
    last_valid_read = millis();
    last_new_scanned_uid = "";
    is_on_travel = false;

    // Capture entry timing with GPS fallback
    entry_timing = capture_current_time();
    time_entry = entry_timing.calculated_time;

    Serial.println("ENTRY TIME @ " + time_entry + " - UID: " + new_scanned_uid);

    if (entry_timing.gps_time_valid)
    {
        Serial.println("Entry time captured with GPS");
    }
    else
    {
        Serial.println("Entry time pending GPS availability");
    }

    // Calculate duration using millis timestamps for accuracy
    calculate_trip_duration(start_time_millis);
}

// Helper function to clean up RFID module and LEDs
void cleanup_rfid()
{
    module_rfid.PICC_HaltA();
    module_rfid.PCD_StopCrypto1();

    delay(1000);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off LED (active LOW)
    digitalWrite(RFID_LED_PIN, LOW);     // Turn off external LED
}

// Enhanced RFID reading function
void handle_rfid_reading()
{
    static String last_new_scanned_uid = "";
    static bool is_on_travel = false;
    static unsigned long last_valid_read = 0;
    static unsigned long start_time_millis = 0;

    // Check if GPS became available and update pending timestamps
    if (module_gps.time.isValid())
    {
        update_pending_timestamps();

        // Update current GPS time for transmission
        current_gps_time = format_gps_time_to_local(module_gps.time);

        // Update the transmitted times if they were pending
        if (time_exit == "Wait for GPS fix")
        {
            time_exit = exit_timing.calculated_time;
        }
        if (time_entry == "Wait for GPS fix")
        {
            time_entry = entry_timing.calculated_time;
        }
    }

    // Original RFID reading logic
    if (!module_rfid.PICC_IsNewCardPresent() || !module_rfid.PICC_ReadCardSerial())
    {
        return;
    }

    // Turn on LED indicators
    digitalWrite(BUILTIN_LED_PIN, LOW); // Active LOW
    digitalWrite(RFID_LED_PIN, HIGH);

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

    // Debouncing - ignore rapid repeated scans
    if ((millis() - last_valid_read) < 5000) // Ignore for 6 seconds
    {
        cleanup_rfid();
        return;
    }

    // Handle different scan scenarios
    if (last_new_scanned_uid == "" || !is_on_travel)
    {
        // First scan or returning from travel - this is EXIT
        handle_exit_scan(new_scanned_uid, last_new_scanned_uid, is_on_travel,
                         last_valid_read, start_time_millis);
    }
    else if (new_scanned_uid == last_new_scanned_uid && is_on_travel)
    {
        // Same card scanned while out - this is ENTRY
        handle_entry_scan(new_scanned_uid, last_new_scanned_uid, is_on_travel,
                          last_valid_read, start_time_millis);
    }
    else if (new_scanned_uid != last_new_scanned_uid && is_on_travel)
    {
        // Different card while authorized person is out - UNAUTHORIZED
        Serial.println("UNAUTHORIZED - Wrong RFID Card. Expected: " +
                       last_new_scanned_uid + ", Got: " + new_scanned_uid);
        last_valid_read = millis();
    }

    cleanup_rfid();
}

void handle_http_transmission()
{
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

    String json;
    serializeJson(doc, json);

    String url = String("http://") + server_ip + ":" + String(server_port) + upload_path;

    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(15000);

    int http_response_code = http.POST(json);
    if (http_response_code == 200)
    {
        // Success - no need to log every successful transmission
    }
    else if (http_response_code > 0)
    {
        Serial.printf("HTTP Error %d\n", http_response_code);
    }
    else
    {
        Serial.printf("HTTP Connection Error: %d\n", http_response_code);
    }

    http.end();
}

void handle_health_check()
{
    Serial.printf("Health: Heap=%d, WiFi=%s (%ddBm), RFID=%s, GPS=%s, HTTP=%s\n",
                  ESP.getFreeHeap(),
                  components.wifi_ok ? "OK" : "FAIL",
                  WiFi.RSSI(),
                  components.ble_ok ? "OK" : "FAIL",
                  components.rfid_ok ? "OK" : "FAIL",
                  components.gps_ok ? "OK" : "FAIL",
                  components.http_ok ? "OK" : "FAIL");

    // ESP8266 memory check - lower threshold
    if (ESP.getFreeHeap() < 50000)
    {
        Serial.println("WARNING: Low memory, restarting system");
        delay(1000);
        ESP.restart();
    }
}

void handle_running_state()
{
    unsigned long current_time = millis();

    // Check WiFi status
    if (WiFi.status() != WL_CONNECTED || WiFi.status() == WL_DISCONNECTED || WiFi.status() == WL_CONNECTION_LOST || WiFi.status() == WL_CONNECT_FAILED)
    {
        Serial.println("WiFi disconnected, returning to connection state");
        components.wifi_ok = false;
        components.http_ok = false;
        current_system_state = STATE_WIFI_CONNECTING;
        last_wifi_attempt = 0;
        state_timeout = millis() + 30000;
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

    // Handle HTTP transmission
    if (components.http_ok && current_time - last_http_send > 5000)
    {
        handle_http_transmission();
        last_http_send = current_time;
    }

    // Health check
    if (current_time - last_health_check > 15000)
    {
        handle_health_check();
        last_health_check = current_time;
    }
}

void handle_error_state()
{
    Serial.printf("System Error: %s\n", components.last_error.c_str());
    Serial.println("Will restart in 5 seconds...");
    delay(5000);
    ESP.restart();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    setCpuFrequencyMhz(240); // 240MHz for ESP32
    delay(1000);

    // Initialize LED pins
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    pinMode(RFID_LED_PIN, OUTPUT);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off builtin LED (active LOW)
    digitalWrite(RFID_LED_PIN, LOW);     // Turn off external LED

    // Ensure WiFi is started before getting MAC address
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    delay(100); // Give WiFi stack time to initialize
    phys_addr_str = get_phys_addr();
    phys_addr = phys_addr_str.c_str(); // ESP8266's MAC Address

    Serial.println("=== Tracking System ===");
    Serial.println("Device UUID: " + phys_addr_str);

    // Start with initialization state
    current_system_state = STATE_INIT;
}

void loop()
{
    // State Machine Execution
    switch (current_system_state)
    {
    case STATE_INIT:
        handle_init_state();
        break;
    case STATE_WIFI_CONNECTING:
        handle_wifi_connecting();
        break;
    case STATE_WIFI_CONNECTED:
        handle_wifi_connected();
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
    // Small delay to prevent watchdog issues
    delay(10);
}
