// Proof-of-Concept Project - ESP8266 Port

//#include <Arduino.h>
#include <core_esp8266_features.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char *ssid = "ESP32_AP";
const char *password = "01234567890";
const char *server_ip = "192.168.120.2";
const int server_port = 80;
const char *upload_path = "/upload";
IPAddress sensor_ip(192, 168, 120, 3); // Fixed IP for sensor
IPAddress gateway(192, 168, 120, 1);   // ESP32-AP gateway
IPAddress subnet(255, 255, 255, 0);    // Subnet mask
IPAddress dns(192, 168, 120, 1);       // DNS

// Hardware pins for ESP8266 (NodeMCU/LoLin (and clones)/Wemos D1 Mini mapping)
// RFID MFRC522 pins - Standard SPI connections
#define SS_PIN 15 // D8 - GPIO15 (SS/CS pin for RFID)
#define RST_PIN 0 // D3 - GPIO0 (RST pin for RFID)
// Note: SPI pins are fixed on ESP8266: SCK=GPIO14(D5), MOSI=GPIO13(D7), MISO=GPIO12(D6)

// GPS pins - Using SoftwareSerial library
#define GPS_RX_PIN 4 // D2 - GPIO4 (connect to GPS TX)
#define GPS_TX_PIN 5 // D1 - GPIO5 (connect to GPS RX)

// HC-05 Bluetooth Module pins - Using SoftwareSerial
#define BT_RX_PIN 10 // S3 - GPIO 10
#define BT_TX_PIN 9  // S2 - GPIO 9
// LED indicators
#define BUILTIN_LED_PIN 2 // GPIO2 (onboard LED - active LOW on ESP8266)
#define RFID_LED_PIN 16   // D0 - GPIO16 (external LED indicator)

// System States for Pseudo-Multitasking
enum system_state_t
{
  STATE_INIT,
  STATE_WIFI_CONNECTING,
  STATE_WIFI_CONNECTED,
  STATE_HARDWARE_INIT,
  STATE_RUNNING,
  STATE_ERROR
};
// States for Time Validity of EXIT/ENTRY
enum time_states
{
  STATE_EXIT_TIME_VALID,
  STATE_EXIT_TIME_INVALID,
  STATE_ENTRY_TIME_VALID,
  STATE_ENTRY_TIME_INVALID
};

// Component status tracking
struct component_status_t
{
  bool wifi_ok = false;
  bool bt_ok = false;
  bool rfid_ok = false;
  bool gps_ok = false;
  bool http_ok = false;
  String last_error = "";
};

// Global variables
system_state_t current_system_state = STATE_INIT;
time_states current_time_state = STATE_EXIT_TIME_VALID;
component_status_t components;
WiFiClient client;
HTTPClient http;

// Hardware variables - Fixed pin assignments
MFRC522DriverPinSimple ss_pin(SS_PIN);
MFRC522DriverSPI driver{ss_pin};
MFRC522 module_rfid{driver};
TinyGPSPlus module_gps;
SoftwareSerial serial_gps(GPS_RX_PIN, GPS_TX_PIN);

// Timing variables
unsigned long last_wifi_attempt = 0;
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
String duration = "N/A";
String current_gps_time = "N/A";
String current_gps_lat = "N/A";
String current_gps_lng = "N/A";
String current_gps_alt = "N/A";
String current_gps_speed = "N/A";
String current_gps_sats = "N/A";
String current_gps_hdop = "N/A";

// Global Time Storage
static uint32_t time_from_gps_converted_to_seconds = 0;

// Fixed MAC address retrieval for ESP8266
String get_phys_addr()
{
  String mac = WiFi.macAddress();
  mac.replace(":", ""); // Remove colons to match format
  return mac;
}
const String phys_addr_str = get_phys_addr();
const char *phys_addr = phys_addr_str.c_str(); // Device UUID

// State machine functions
void handle_init_state()
{
  Serial.println("=== System Initialization ===");
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

  // ESP8266 has less RAM - adjust threshold accordingly
  if (ESP.getFreeHeap() < 10000)
  {
    Serial.println("Insufficient memory available, restarting...");
    delay(1000);
    ESP.restart();
  }

  // Initialize serial communication for GPS @ 9600 baud
  serial_gps.begin(9600);

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
    WiFi.setAutoReconnect(false); // We'll handle reconnection manually
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
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

    if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL || status == WL_WRONG_PASSWORD)
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
  current_system_state = STATE_HARDWARE_INIT;
  state_timeout = millis() + 10000; // 10 second timeout for hardware initialize
}

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

void handle_error_state()
{
  Serial.printf("System Error: %s\n", components.last_error.c_str());
  Serial.println("Will restart in 5 seconds...");
  delay(5000);
  ESP.restart();
}

// Component handlers
void handle_gps_reading()
{
  // Local storage for time
  static uint16_t hours_from_gps_data = 0;
  static uint16_t minutes_from_gps_data = 0;
  static uint16_t seconds_from_gps_data = 0;
  // Local storage for conversion into seconds
  static uint16_t hours_to_seconds = 0;
  static uint16_t minutes_to_seconds = 0;
  static uint16_t seconds_is_seconds = 0;
  static uint16_t total_in_seconds = 0;
  // Local storage for convertion back to HH:MM:SS
  static uint16_t from_seconds_to_hours = 0;
  static uint16_t from_seconds_to_minutes = 0;
  static uint16_t converted_seconds = 0;

  while (serial_gps.available())
  {
    if (module_gps.encode(serial_gps.read()))
    {
      if (module_gps.time.isValid() && module_gps.time.isUpdated())
      {
        // Fetch time from GPS module
        hours_from_gps_data = module_gps.time.hour();
        minutes_from_gps_data = module_gps.time.minute();
        seconds_from_gps_data = module_gps.time.second();
        // Convert the time to seconds
        hours_to_seconds = (hours_from_gps_data * 3600) + 28800; // 28800 seconds is GMT+8
        minutes_to_seconds = minutes_from_gps_data * 60;
        seconds_is_seconds = seconds_from_gps_data;
        total_in_seconds = hours_to_seconds + minutes_to_seconds + seconds_is_seconds; // Added all the converted-seconds
        // Convert again to HH:MM:SS (24-hour format)
        from_seconds_to_hours = (total_in_seconds / 3600) % 24;
        from_seconds_to_minutes = (total_in_seconds % 3600) / 60;
        converted_seconds = total_in_seconds % 60;
        // Convert to String
        char time_str[12];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 from_seconds_to_hours, from_seconds_to_minutes, converted_seconds);
        current_gps_time = String(time_str); // Update the time variable that will be sent to the server
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
  time_from_gps_converted_to_seconds = total_in_seconds; // Store time data in global variable, might use
}

void handle_rfid_reading()
{
  static bool exit_time_valid;                  // Flag for exit time validity
  static bool entry_time_valid;                 // Flag for entry time validity
  static bool is_on_travel = false;             // Track if someone is on a travel
  static String last_scanned_uid = "";          // Persistent storage for UID
  static uint16_t last_valid_read = 0;          // Timer for debouncing
  static uint16_t start_time_to_exit_in_ms = 0; // Start timer
  static uint16_t end_time_to_entry_in_ms = 0;  // End Timer
  static uint16_t computed_duration_in_ms = 0;
  static uint16_t seconds = 0, minutes = 0, hours = 0;
  // Local storage for time
  static uint16_t hours_from_gps_data = 0;
  static uint16_t minutes_from_gps_data = 0;
  static uint16_t seconds_from_gps_data = 0;
  // Local storage for conversion into seconds
  static uint16_t hours_to_seconds = 0;
  static uint16_t minutes_to_seconds = 0;
  static uint16_t seconds_is_seconds = 0;
  static uint16_t total_in_seconds = 0;
  // Local storage for convertion back to HH:MM:SS
  static uint16_t from_seconds_to_hours = 0;
  static uint16_t from_seconds_to_minutes = 0;
  static uint16_t converted_seconds = 0;

  if (module_rfid.PICC_IsNewCardPresent() && module_rfid.PICC_ReadCardSerial())
  {
    digitalWrite(BUILTIN_LED_PIN, LOW); // Turn on LED (active LOW)
    digitalWrite(RFID_LED_PIN, HIGH);   // Turn on external LED

    String new_scanned_uid = "";
    for (byte i = 0; i < module_rfid.uid.size; i++)
    {
      if (module_rfid.uid.uidByte[i] < 0x10)
        new_scanned_uid += "0";
      new_scanned_uid += String(module_rfid.uid.uidByte[i], HEX);
    }
    new_scanned_uid.toUpperCase();

    // Debouncing check - ignore rapid repeated scans
    if ((millis() - last_valid_read) < 2000)
    {
      module_rfid.PICC_HaltA();
      module_rfid.PCD_StopCrypto1();
      digitalWrite(BUILTIN_LED_PIN, HIGH);
      digitalWrite(RFID_LED_PIN, LOW);
      return;
    }

    // First scan ever or after person returned (TIME OUT/EXIT)
    if ((last_scanned_uid == "") || !is_on_travel)
    {
      last_scanned_uid = new_scanned_uid; // Sets the newly scanned UID as last scanned UID
      current_rfid = new_scanned_uid;     // Sets the newly scanned UID as the current UID
      last_valid_read = millis();         // Timer for debouncing - prevent rapid reads
      is_on_travel = true;                // Person is now "out"

      Serial.println("Internal Timer started");
      start_time_to_exit_in_ms = millis(); // Internal timer starts on the first scan ever or after 1 cycle completes

      // Record TIME OUT (now on a travel)
      if (module_gps.time.isValid() && module_gps.time.isUpdated())
      {
        // Fetch time from GPS module
        hours_from_gps_data = module_gps.time.hour();
        minutes_from_gps_data = module_gps.time.minute();
        seconds_from_gps_data = module_gps.time.second();
        // Convert the time to seconds
        hours_to_seconds = (hours_from_gps_data * 3600) + 28800; // 28800 seconds is GMT+8
        minutes_to_seconds = minutes_from_gps_data * 60;
        seconds_is_seconds = seconds_from_gps_data;
        total_in_seconds = hours_to_seconds + minutes_to_seconds + seconds_is_seconds; // Adding all the converted-seconds
        // Convert again to HH:MM:SS (24-hour format)
        from_seconds_to_hours = (total_in_seconds / 3600) % 24;
        from_seconds_to_minutes = (total_in_seconds % 3600) / 60;
        converted_seconds = total_in_seconds % 60;
        // Convert to String
        char time_str[12];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 from_seconds_to_hours, from_seconds_to_minutes, converted_seconds);
        time_exit = String(time_str);
        Serial.println("EXIT TIME @ " + time_exit + " - UID: " + new_scanned_uid);

        // Validity flag
        exit_time_valid = true;
      }
      else
      {
        exit_time_valid = false;
      }
    }
    // Same card scanned while person is "out" (TIME IN/EXIT)
    else if ((new_scanned_uid == last_scanned_uid) && is_on_travel)
    {
      last_valid_read = millis();
      is_on_travel = false; // Person is now "in"

      // Record TIME IN
      if (module_gps.time.isValid() && module_gps.time.isUpdated())
      {
        // Fetch time from GPS module
        hours_from_gps_data = module_gps.time.hour();
        minutes_from_gps_data = module_gps.time.minute();
        seconds_from_gps_data = module_gps.time.second();
        // Convert the time to seconds
        hours_to_seconds = (hours_from_gps_data * 3600) + 28800; // 28800 seconds is GMT+8
        minutes_to_seconds = minutes_from_gps_data * 60;
        seconds_is_seconds = seconds_from_gps_data;
        total_in_seconds = hours_to_seconds + minutes_to_seconds + seconds_is_seconds; // Adding all the converted-seconds
        // Convert again to HH:MM:SS (24-hour format)
        from_seconds_to_hours = (total_in_seconds / 3600) % 24;
        from_seconds_to_minutes = (total_in_seconds % 3600) / 60;
        converted_seconds = total_in_seconds % 60;
        // Convert to String
        char time_str[12];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
                 from_seconds_to_hours, from_seconds_to_minutes, converted_seconds);
        time_entry = String(time_str);
        Serial.println("ENTRY TIME @ " + time_entry + " - UID: " + new_scanned_uid);

        // Validity flag
        entry_time_valid = true;
      }
    }
    // Different card scanned while authorized person is out
    else if ((new_scanned_uid != last_scanned_uid) && is_on_travel)
    {
      Serial.println("UNAUTHORIZED - Wrong RFID Card/Tag. Expected: " + last_scanned_uid + ", Got: " + new_scanned_uid);
      last_valid_read = millis();
      // Don't change the state - unauthorized access
    }

    // Clean up RFID module
    module_rfid.PICC_HaltA();
    module_rfid.PCD_StopCrypto1();

    // ---- Time validity conditions ----
    if (exit_time_valid == false)
    {
      /*
      Will handle back-counting if exit time is not valid or available
      (e.g. no data coming from GPS module itself or skies are blocked) haven't figured out how
      */
      while (!module_gps.time.isValid() && !module_gps.time.isUpdated()) // Attempt to check time data
      {
        // Logic haven't figured how
        // should i copy-paste the logic for time calculation just like in handle_gps_reading and set it as the "time_exit"
        // or use the time_from_gps_converted_to_seconds 
      }

      exit_time_valid = true; // Update the boolean flag if success
    }

    if (entry_time_valid == false)
    {
      /*
      Will handle back-counting if entry time is not valid or available
      (e.g. no data coming from GPS module itself or skies are blocked)

      is block below correct? 

      end_time_to_entry_in_ms = millis();
      computed_duration_in_ms = (end_time_to_entry_in_ms - start_time_to_exit_in_ms);
      hours = (computed_duration_in_ms / 3600000);
      minutes = (computed_duration_in_ms % 3600000) / 60000;
      seconds = (computed_duration_in_ms % 60000) / 1000;
      char final_duration_str[12];
      snprintf(final_duration_str, sizeof(final_duration_str), "%02d:%02d:%02d",
               hours,
               minutes,
               seconds);
      duration = String(final_duration_str);
      Serial.println("DURATION @ " + duration);
      */
     entry_time_valid = true;
    }

    // LED Indicators
    delay(1000);
    digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off LED (active LOW)
    digitalWrite(RFID_LED_PIN, LOW);     // Turn off external LED
  }
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

  http.begin(client, url); // ESP8266HTTPClient syntax
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);

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
                components.rfid_ok ? "OK" : "FAIL",
                components.gps_ok ? "OK" : "FAIL",
                components.http_ok ? "OK" : "FAIL");

  // ESP8266 memory check - lower threshold
  if (ESP.getFreeHeap() < 8000)
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
  if (WiFi.status() != WL_CONNECTED && WiFi.status() == WL_DISCONNECTED)
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
  if (components.gps_ok && current_time - last_gps_update > 1000)
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

void setup()
{
  system_update_cpu_freq(160); // 160MHz for ESP8266
  esp_delay(1000);
  Serial.begin(9600);
  esp_delay(1000);

  // Initialize LED pins
  pinMode(BUILTIN_LED_PIN, OUTPUT);
  pinMode(RFID_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, HIGH); // Turn off builtin LED (active LOW)
  digitalWrite(RFID_LED_PIN, LOW);     // Turn off external LED

  Serial.println("=== ESP8266 RFID-GPS System ===");
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
  case STATE_HARDWARE_INIT:
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
