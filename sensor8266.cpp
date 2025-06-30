// Proof-of-Concept Project

#include <Arduino.h>
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
const char *SSID = "ESP32_AP";
const char *PASSWORD = "01234567890";
const char *serverIP = "192.168.120.1";
const int serverPort = 80;
const char *uploadPath = "/upload";

// Hardware pins
// #define SS_PIN 5       // Slave Select for RFID
// #define RST_PIN 21     // Reset Pin for RFID
#define RX_PIN 4          // GPIO 4 | D2 for GPS
#define TX_PIN 5          // GPIO 5 | D1 for GPS

// System States for Psuedo-Multitasking 
enum SystemState
{
  STATE_INIT,
  STATE_WIFI_CONNECTING,
  STATE_WIFI_CONNECTED,
  STATE_HARDWARE_INIT,
  STATE_RUNNING,
  STATE_ERROR
};

// Component status tracking
struct ComponentStatus
{
  bool wifi_ok = false;
  bool rfid_ok = false;
  bool gps_ok = false;
  bool http_ok = false;
  String last_error = "";
};

// Global variables
SystemState currentState = STATE_INIT;
ComponentStatus components;
WiFiClient client;
HTTPClient http;
// Hardware variables
MFRC522DriverPinSimple slaveSelect(5);
MFRC522DriverSPI driver{slaveSelect};
MFRC522 module_RFID{driver};
TinyGPSPlus module_GPS;
SoftwareSerial SerialGPS(RX_PIN, TX_PIN);

int espBUILT_IN = 2;
int rfidLED_IND = 33;

// Timing variables
unsigned long lastWifiAttempt = 0;
unsigned long lastRFIDRead = 0;
unsigned long lastGPSUpdate = 0;
unsigned long lastHTTPSend = 0;
unsigned long lastHealthCheck = 0;
unsigned long stateTimeout = 0;

// Vehicle constants
const char *VEHICLE_BRAND = "Toyota";
const char *VEHICLE_MODEL = "Tamaraw FX";
const char *VEHICLE_PLATE = "UAM981";

// Data storage
String currentRFID = "N/A";
String currentGPS_time = "N/A";
String currentGPS_lat = "N/A";
String currentGPS_lng = "N/A";
String currentGPS_alt = "N/A";
String currentGPS_speed = "N/A";
String currentGPS_sats = "N/A";
String currentGPS_hdop = "N/A";

// Testing - Get PHYS Address from eFuse as Device UUID
String getPHYS_ADDR()
{
  String mac = WiFi.macAddress();
  mac.replace("", ""); // Remove colons to match your format
  return mac;
}
const String PHYS_ADDR_STR = getPHYS_ADDR();
const char *PHYS_ADDR = PHYS_ADDR_STR.c_str(); // Device UUID

// State machine functions
void handleInitState()
{
  Serial.println("=== System Initialization ===");
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

  // Basic memory check
  if (ESP.getFreeHeap() < 20000)
  {
    Serial.println("Insufficient memory available, restarting...");
    delay(1000);
    ESP.restart();
  }

  // Initialize serial communication for GPS
  SerialGPS.begin(9600);

  // Move to WiFi connection state
  currentState = STATE_WIFI_CONNECTING;
  lastWifiAttempt = millis();
  stateTimeout = millis() + 30000; // 30 second timeout
  Serial.println("Moving to WiFi connection state");
}

void handleWifiConnecting()
{
  static bool wifiStarted = false;
  static int wifiAttempts = 0;

  if (!wifiStarted)
  {
    Serial.println("Starting WiFi connection...");
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(false); // We'll handle reconnection manually
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.begin(SSID, PASSWORD);
    wifiStarted = true;
    Serial.print("Connecting");
    return;
  }

  // Check WiFi status every 500ms
  if (millis() - lastWifiAttempt > 500)
  {
    wl_status_t status = WiFi.status();

    if (status == WL_CONNECTED)
    {
      Serial.println();
      Serial.printf("WiFi Connected! IP: %s, Signal: %d dBm\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      components.wifi_ok = true;
      currentState = STATE_WIFI_CONNECTED;
      return;
    }

    if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL)
    {
      Serial.printf("\nWiFi connection failed with status: %d\n", status);
      components.last_error = "WiFi authentication failed";
      currentState = STATE_ERROR;
      return;
    }

    Serial.print(".");
    lastWifiAttempt = millis();
    wifiAttempts++;

    // Timeout check
    if (millis() > stateTimeout)
    {
      Serial.println("\nWiFi connection timeout");
      components.last_error = "WiFi connection timeout";
      currentState = STATE_ERROR;
      return;
    }
  }
}

void handleWifiConnected()
{
  Serial.println("WiFi connected, initializing hardware...");
  currentState = STATE_HARDWARE_INIT;
  stateTimeout = millis() + 10000; // 10 second timeout for hardware initialize
}

void handleHardwareInit()
{
  static bool spiInitialized = false;
  static bool rfidTested = false;
  static bool gpsTested = false;

  // Initialize SPI once
  if (!spiInitialized)
  {
    Serial.println("Initializing SPI...");
    SPI.begin();
    delay(100);
    spiInitialized = true;
    Serial.printf("SPI initialized. Free heap: %d\n", ESP.getFreeHeap());
    return;
  }

  // Test RFID
  if (!rfidTested)
  {
    Serial.println("Testing RFID module...");

    // Simple test - Initialize without complex operations
    try
    {
      module_RFID.PCD_Init();
      delay(100);
      byte version = module_RFID.PCD_GetVersion();

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
      Serial.println("RFID initialization caused exception, disabling");
      components.rfid_ok = false;
    }

    rfidTested = true;
    Serial.printf("After RFID test. Free heap: %d\n", ESP.getFreeHeap());
    return;
  }

  // Test GPS - just check if data is available
  if (!gpsTested)
  {
    Serial.println("Testing GPS module...");

    unsigned long startTime = millis();
    bool gpsDataSeen = false;

    // Quick test - just look for any data for 2 seconds
    while (millis() - startTime < 2000)
    {
      if (SerialGPS.available())
      {
        SerialGPS.read(); // Just consume the data
        gpsDataSeen = true;
        break;
      }
      delay(10);
    }

    if (gpsDataSeen)
    {
      Serial.println("GPS data detected");
      components.gps_ok = true;
    }
    else
    {
      Serial.println("No GPS data, continuing without it");
      components.gps_ok = false;
    }

    gpsTested = true;
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
  currentState = STATE_RUNNING;
}

void handleErrorState()
{
  Serial.printf("System in error state: %s\n", components.last_error.c_str());
  Serial.println("Restarting in 5 seconds...");
  delay(5000);
  ESP.restart();
}

// Simplified component handlers
void handleRFIDReading()
{
  static String lastUID = "";
  static unsigned long lastValidRead = 0;

  if (module_RFID.PICC_IsNewCardPresent() && module_RFID.PICC_ReadCardSerial())
  {
    digitalWrite(espBUILT_IN, HIGH); // Scan Indicator
    // digitalWrite(rfidLED_IND, HIGH);

    String uid = "";
    for (byte i = 0; i < module_RFID.uid.size; i++)
    {
      if (module_RFID.uid.uidByte[i] < 0x10)
        uid += "0";
      uid += String(module_RFID.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();
    // Simple debouncing
    if (uid != lastUID || (millis() - lastValidRead) > 2000)
    {
      Serial.println("RFID: " + uid);
      currentRFID = uid;
      lastUID = uid;
      lastValidRead = millis();
    }
    module_RFID.PICC_HaltA();
    module_RFID.PCD_StopCrypto1();

    delay(1000);
    digitalWrite(espBUILT_IN, LOW);
    // digitalWrite(rfidLED_IND, LOW);
  }
}

void handleGPSReading()
{
  while (SerialGPS.available())
  {
    if (module_GPS.encode(SerialGPS.read()))
    {
      if (module_GPS.time.isValid())
      {
        char timeStr[9];
        snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d",
                 module_GPS.time.hour(), module_GPS.time.minute(), module_GPS.time.second());
        currentGPS_time = String(timeStr);
      }

      if (module_GPS.location.isValid())
      {
        currentGPS_lat = String(module_GPS.location.lat(), 6);
        currentGPS_lng = String(module_GPS.location.lng(), 6);
      }

      currentGPS_alt = module_GPS.altitude.isValid() ? String(module_GPS.altitude.meters(), 1) : "N/A";
      currentGPS_speed = module_GPS.speed.isValid() ? String(module_GPS.speed.kmph(), 1) : "N/A";
      currentGPS_sats = module_GPS.satellites.isValid() ? String(module_GPS.satellites.value()) : "N/A";
      currentGPS_hdop = module_GPS.hdop.isValid() ? String(module_GPS.hdop.hdop(), 2) : "N/A";
    }
  }
}

void handleHTTPTransmission()
{
  JsonDocument doc;
  doc["phys_addr"] = PHYS_ADDR;
  doc["brand"] = VEHICLE_BRAND;
  doc["model"] = VEHICLE_MODEL;
  doc["pltNum"] = VEHICLE_PLATE;
  doc["uid"] = currentRFID;
  doc["time"] = currentGPS_time;
  doc["lat"] = currentGPS_lat;
  doc["long"] = currentGPS_lng;
  doc["alt"] = currentGPS_alt;
  doc["spd"] = currentGPS_speed;
  doc["sat"] = currentGPS_sats;
  doc["hdop"] = currentGPS_hdop;

  String json;
  serializeJson(doc, json);

  String url = String("http://") + serverIP + ":" + String(serverPort) + uploadPath;

  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);

  int httpResponseCode = http.POST(json);
  if (httpResponseCode == 200)
  {
    // Success - no need to log every successful transmission
  }
  else if (httpResponseCode > 0)
  {
    Serial.printf("HTTP Error %d\n", httpResponseCode);
  }
  else
  {
    Serial.printf("HTTP Connection Error: %d\n", httpResponseCode);
  }

  http.end();
}

void handleHealthCheck()
{
  Serial.printf("Health: Heap=%d, WiFi=%s (%ddBm), RFID=%s, GPS=%s, HTTP=%s\n",
                ESP.getFreeHeap(),
                components.wifi_ok ? "OK" : "FAIL",
                WiFi.RSSI(),
                components.rfid_ok ? "OK" : "FAIL",
                components.gps_ok ? "OK" : "FAIL",
                components.http_ok ? "OK" : "FAIL");

  // Memory check
  if (ESP.getFreeHeap() < 20000)
  {
    Serial.println("WARNING: Low memory, restarting system");
    delay(1000);
    ESP.restart();
  }
}

void handleRunningState()
{
  unsigned long currentTime = millis();

  // Check WiFi status
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected, returning to connection state");
    components.wifi_ok = false;
    components.http_ok = false;
    currentState = STATE_WIFI_CONNECTING;
    lastWifiAttempt = 0;
    stateTimeout = millis() + 30000;
    return;
  }

  // Handle RFID reading
  if (components.rfid_ok && currentTime - lastRFIDRead > 100)
  {
    handleRFIDReading();
    lastRFIDRead = currentTime;
  }

  // Handle GPS reading
  if (components.gps_ok && currentTime - lastGPSUpdate > 1000)
  {
    handleGPSReading();
    lastGPSUpdate = currentTime;
  }

  // Handle HTTP transmission
  if (components.http_ok && currentTime - lastHTTPSend > 5000)
  {
    handleHTTPTransmission();
    lastHTTPSend = currentTime;
  }

  // Health check
  if (currentTime - lastHealthCheck > 15000)
  {
    handleHealthCheck();
    lastHealthCheck = currentTime;
  }
}

void setup()
{
  system_update_cpu_freq(160); // Set CPU Frequency
  Serial.begin(115200);
  delay(1000);

  // LED Scan Indicators
  pinMode(espBUILT_IN, OUTPUT);
  pinMode(rfidLED_IND, OUTPUT);

  Serial.println(PHYS_ADDR_STR); // <--- Testing as Device UUID

  // Start with initialization state
  currentState = STATE_INIT;
}

void loop()
{
  // State Machine Execution
  switch (currentState)
  {
  case STATE_INIT:
    handleInitState();
    break;
  case STATE_WIFI_CONNECTING:
    handleWifiConnecting();
    break;
  case STATE_WIFI_CONNECTED:
    handleWifiConnected();
    break;
  case STATE_HARDWARE_INIT:
    handleHardwareInit();
    break;
  case STATE_RUNNING:
    handleRunningState();
    break;
  case STATE_ERROR:
    handleErrorState();
    break;
  }

  // Small delay to prevent watchdog issues
  delay(10);
}
