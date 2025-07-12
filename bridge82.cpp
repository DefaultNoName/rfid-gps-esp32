#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>

// Communication status and buffers
struct BridgeStatus
{
    bool wifi_connected = false;
    bool ble_connected = false;
    unsigned long last_wifi_check = 0;
    unsigned long last_ble_check = 0;
    unsigned long last_data_forward = 0;
    int json_packets_forwarded = 0;
    int connection_failures = 0;
};

// WiFi configuration - STA mode
IPAddress STA_IP(192, 168, 120, 200); // Fixed IP
IPAddress STA_GW(192, 168, 120, 1);   // ESP32-Server gateway
IPAddress STA_SN(255, 255, 255, 0);   // Subnet mask
IPAddress DNS(192, 168, 120, 1);      // DNS (same as gateway)
#define SSID "ESP32-Server"           // SSID
#define PASSWORD "01234567890"        // Password

// Target server configuration
const char *TARGET_SERVER_IP = "192.168.120.1";
const int TARGET_SERVER_PORT = 80;
const char *TARGET_UPLOAD_PATH = "/upload";

// JDY-16 BLE Module Configuration
// The JDY-16 connects to ESP32 via BLE and sends received data to ESP8266 via UART
#define BLE_TX_PIN 12 // ESP8266 receives data from BLE module
#define BLE_RX_PIN 13 // ESP8266 sends commands to BLE module
#define BLE_STATE_PIN 14
#define BLE_EN_PIN 15
SoftwareSerial Serial_BLE(BLE_TX_PIN, BLE_RX_PIN); // RX, TX pins

// Feedback Pins
#define BUILT_IN_LED 2
#define EXTERNAL_LED 4

// Other definitions
#define CPU_FREQUENCY_MHZ 160
#define WIFI_TIMEOUT_MS 30000

BridgeStatus bridgeStatus;

// JSON data buffer for complete packet assembly
String jsonBuffer = "";
bool receivingJson = false;
unsigned long lastBleDataTime = 0;
const unsigned long BLE_DATA_TIMEOUT = 3000; // 3 seconds timeout for JSON assembly

// Data forwarding via WiFi HTTP
bool forwardDataViaWiFi(const String &jsonData)
{
    if (!bridgeStatus.wifi_connected)
    {
        Serial.println("WiFi not connected, cannot forward data");
        return false;
    }

    WiFiClient client;
    HTTPClient http;

    // Construct the full URL
    String url = String("http://") + TARGET_SERVER_IP + ":" + String(TARGET_SERVER_PORT) + TARGET_UPLOAD_PATH;

    Serial.print("Forwarding JSON to: ");
    Serial.println(url);
    Serial.print("JSON Data: ");
    Serial.println(jsonData);

    // Begin HTTP connection
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(5000); // 5 second timeout

    // Send POST request
    int httpResponseCode = http.POST(jsonData);

    bool success = false;
    if (httpResponseCode == 200)
    {
        Serial.println("Data forwarded successfully via WiFi");
        success = true;
        bridgeStatus.json_packets_forwarded++;

        // Blink LED to indicate successful forwarding
        digitalWrite(BUILT_IN_LED, LOW);
        delay(100);
        digitalWrite(BUILT_IN_LED, HIGH);
    }
    else if (httpResponseCode > 0)
    {
        Serial.printf("HTTP Error: %d\n", httpResponseCode);
        String response = http.getString();
        Serial.println("Response: " + response);
        bridgeStatus.connection_failures++;
    }
    else
    {
        Serial.printf("HTTP Connection Error: %d\n", httpResponseCode);
        bridgeStatus.connection_failures++;
    }

    http.end();
    return success;
}

// Process complete JSON packet
bool processJsonPacket(const String &jsonData)
{
    Serial.println("=== Processing JSON Packet ===");

    // Validate JSON format
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error)
    {
        Serial.print("JSON parsing error: ");
        Serial.println(error.c_str());
        return false;
    }

    // Extract and log key information for debugging
    Serial.println("=== JSON Packet Details ===");
    if (doc["phys_addr"].is<const char *>())
    {
        Serial.println("Device MAC: " + String(doc["phys_addr"].as<const char *>()));
    }
    if (doc["uid"].is<const char *>())
    {
        Serial.println("RFID UID: " + String(doc["uid"].as<const char *>()));
    }
    if (doc["lat"].is<const char *>() && doc["long"].is<const char *>())
    {
        Serial.println("GPS Position: " + String(doc["lat"].as<const char *>()) +
                       ", " + String(doc["long"].as<const char *>()));
    }
    if (doc["time_EXIT"].is<const char *>())
    {
        Serial.println("Exit Time: " + String(doc["time_EXIT"].as<const char *>()));
    }
    if (doc["time_ENTRY"].is<const char *>())
    {
        Serial.println("Entry Time: " + String(doc["time_ENTRY"].as<const char *>()));
    }

    // Forward data via WiFi
    bool success = forwardDataViaWiFi(jsonData);

    if (success)
    {
        Serial.println("JSON packet processed and forwarded successfully");
    }
    else
    {
        Serial.println("Failed to forward JSON packet");
    }

    return success;
}

// Process incoming JSON data from BLE module
void processIncomingBLE()
{
    while (Serial_BLE.available())
    {
        char incomingChar = Serial_BLE.read();
        lastBleDataTime = millis();

        // The ESP32 sends JSON data in chunks followed by a newline (\n) as end marker
        if (incomingChar == '\n')
        {
            // End of JSON transmission detected
            if (jsonBuffer.length() > 0)
            {
                Serial.println("=== Complete JSON Received ===");
                Serial.print("Buffer length: ");
                Serial.println(jsonBuffer.length());
                Serial.println("JSON Data: " + jsonBuffer);

                // Process the complete JSON packet
                bool success = processJsonPacket(jsonBuffer);

                // Clear buffer for next packet
                jsonBuffer = "";
                receivingJson = false;

                // Optional: Send acknowledgment back to BLE module
                if (success)
                {
                    Serial_BLE.println("ACK");
                }
                else
                {
                    Serial_BLE.println("NACK");
                }
            }
        }
        else
        {
            // Accumulate JSON data
            jsonBuffer += incomingChar;
            receivingJson = true;

            // Debug: Print received characters (comment out for production)
            // Serial.print(incomingChar);
        }
    }

    // Handle timeout for incomplete JSON packets
    if (receivingJson && (millis() - lastBleDataTime > BLE_DATA_TIMEOUT))
    {
        Serial.println("BLE data timeout, clearing buffer");
        jsonBuffer = "";
        receivingJson = false;
    }
}

// WiFi connection monitoring and maintenance
void check_wifi_connection()
{
    unsigned long currentTime = millis();

    // Check connection status every 5 seconds
    if (currentTime - bridgeStatus.last_wifi_check > 5000)
    {
        wl_status_t status = WiFi.status();

        if (status == WL_CONNECTED)
        {
            if (!bridgeStatus.wifi_connected)
            {
                Serial.println("WiFi connection restored");
                Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
                digitalWrite(EXTERNAL_LED, HIGH);
                bridgeStatus.wifi_connected = true;
                bridgeStatus.connection_failures = 0;
            }
        }
        else
        {
            if (bridgeStatus.wifi_connected)
            {
                Serial.println("WiFi connection lost");
                digitalWrite(EXTERNAL_LED, LOW);
                bridgeStatus.wifi_connected = false;
            }

            // Attempt reconnection
            Serial.println("Attempting WiFi reconnection...");
            WiFi.reconnect();
            bridgeStatus.connection_failures++;
        }

        bridgeStatus.last_wifi_check = currentTime;
    }
}

// BLE connection monitoring
void check_ble_connection()
{
    unsigned long currentTime = millis();

    // Check BLE connection status every 10 seconds
    if (currentTime - bridgeStatus.last_ble_check > 10000)
    {
        // Simple connectivity check by monitoring recent data activity
        bool dataRecent = (currentTime - lastBleDataTime) < 30000; // Data within last 30 seconds

        if (dataRecent != bridgeStatus.ble_connected)
        {
            bridgeStatus.ble_connected = dataRecent;
            Serial.printf("BLE Status: %s\n", bridgeStatus.ble_connected ? "Connected" : "Disconnected");
        }

        bridgeStatus.last_ble_check = currentTime;
    }
}

// Status reporting function
void reportStatus()
{
    static unsigned long lastReport = 0;

    if (millis() - lastReport > 30000) // Report every 30 seconds
    {
        Serial.println("\n=== Bridge Status Report ===");
        Serial.printf("WiFi: %s", bridgeStatus.wifi_connected ? "Connected" : "Disconnected");
        if (bridgeStatus.wifi_connected)
        {
            Serial.printf(" (IP: %s, Signal: %d dBm)",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
        }
        Serial.println();

        Serial.printf("BLE: %s\n", bridgeStatus.ble_connected ? "Connected" : "Disconnected");
        Serial.printf("JSON Packets Forwarded: %d\n", bridgeStatus.json_packets_forwarded);
        Serial.printf("Connection Failures: %d\n", bridgeStatus.connection_failures);
        Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("Buffer Status: %s (length: %d)\n",
                      receivingJson ? "Receiving" : "Idle", jsonBuffer.length());
        Serial.println("================================\n");

        lastReport = millis();
    }
}

// Print BLE module responses for debugging
void printBLEResponse()
{
    delay(100);
    while (Serial_BLE.available())
    {
        Serial.write(Serial_BLE.read());
    }
}

// Initialize BLE module for proper communication
void initializeBLEModule()
{
    Serial.println("Initializing BLE module...");

    // Power cycle BLE module if enable pin is connected
    if (BLE_EN_PIN != -1)
    {
        pinMode(BLE_EN_PIN, OUTPUT);
        digitalWrite(BLE_EN_PIN, LOW);
        delay(100);
        digitalWrite(BLE_EN_PIN, HIGH);
        delay(2000);
    }

    // Send configuration commands to BLE module
    Serial_BLE.print("AT+IMME1\r\n"); // Manual start mode
    delay(500);
    printBLEResponse();

    Serial_BLE.print("AT+BAUD4\r\n"); // Set baud rate to 9600
    delay(500);
    printBLEResponse();

    Serial_BLE.print("AT+POWE3\r\n"); // Set RF power to maximum
    delay(500);
    printBLEResponse();

    Serial_BLE.print("AT+NAMEESP-BLE\r\n"); // Set device name (matches ESP32 expectation)
    delay(500);
    printBLEResponse();

    Serial_BLE.print("AT+ROLE0\r\n"); // Set as peripheral/slave
    delay(500);
    printBLEResponse();

    Serial_BLE.print("AT+RESET\r\n"); // Reset module
    delay(2000);
    printBLEResponse();

    Serial_BLE.print("AT+START\r\n"); // Start advertising
    delay(1000);
    printBLEResponse();

    Serial.println("BLE module configuration complete");
}

void setup()
{
    Serial.begin(9600);
    delay(100);

    // Set CPU frequency for better performance
    system_update_cpu_freq(CPU_FREQUENCY_MHZ);
    delay(500);

    // Initialize feedback pins
    pinMode(BUILT_IN_LED, OUTPUT);
    pinMode(EXTERNAL_LED, OUTPUT);
    digitalWrite(BUILT_IN_LED, HIGH); // Turn off built-in LED (active LOW)
    digitalWrite(EXTERNAL_LED, LOW);  // Turn off external LED initially

    // LED startup sequence
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(BUILT_IN_LED, LOW);
        digitalWrite(EXTERNAL_LED, HIGH);
        delay(200);
        digitalWrite(BUILT_IN_LED, HIGH);
        digitalWrite(EXTERNAL_LED, LOW);
        delay(200);
    }

    Serial.println("=== ESP8266 Bridge Node Starting ===");
    Serial.println("Purpose: Receive BLE JSON data and forward via WiFi");

    // Initialize WiFi
    Serial.println("Setting up WiFi (STA mode)...");
    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    if (!WiFi.mode(WIFI_STA))
    {
        Serial.println("Cannot set WiFi as STA mode! Restarting...");
        ESP.restart();
    }

    if (!WiFi.config(STA_IP, STA_GW, STA_SN, DNS))
    {
        Serial.println("Cannot set WiFi STA parameters! Restarting...");
        ESP.restart();
    }

    WiFi.begin(SSID, PASSWORD);

    // Wait for WiFi connection with timeout
    Serial.println("Connecting to WiFi...");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_TIMEOUT_MS)
    {
        delay(1000);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi connected successfully!");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
        bridgeStatus.wifi_connected = true;
        digitalWrite(EXTERNAL_LED, HIGH);
    }
    else
    {
        Serial.println("\nWiFi connection failed, continuing with BLE only");
        bridgeStatus.wifi_connected = false;
        digitalWrite(EXTERNAL_LED, LOW);
    }

    // Initialize BLE communication
    Serial.println("Setting up BLE communication...");
    Serial_BLE.begin(9600);
    delay(1000);

    // Initialize BLE module
    initializeBLEModule();

    Serial.println("=== Bridge Node Setup Complete ===");
    Serial.println("Ready to receive JSON data from ESP32 sensor via BLE");
    Serial.println("Commands: 'status', 'reset', or AT+<command>");
    Serial.println("=====================================\n");
}

void loop()
{
    // Monitor and maintain WiFi connection
    check_wifi_connection();

    // Monitor BLE connection status
    check_ble_connection();

    // Process incoming BLE data (JSON packets from ESP32)
    processIncomingBLE();

    // Handle serial monitor commands for debugging
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toLowerCase();

        if (command == "status")
        {
            reportStatus();
        }
        else if (command == "reset")
        {
            Serial.println("Resetting bridge node...");
            ESP.restart();
        }
        else if (command.startsWith("at"))
        {
            // Forward AT commands to BLE module
            Serial.println("Sending to BLE module: " + command);
            Serial_BLE.println(command);
            delay(500);
            printBLEResponse();
        }
        else
        {
            Serial.println("Available commands:");
            Serial.println("  status  - Show bridge status");
            Serial.println("  reset   - Restart bridge");
            Serial.println("  AT+<cmd> - Send AT command to BLE module");
        }
    }

    // Report status periodically
    reportStatus();

    // Small delay to prevent watchdog issues
    delay(10);
}
