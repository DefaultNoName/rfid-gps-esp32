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

/* Don't delete for easier configuration - this is AP mode
IPAddress AP_IP = IPAddress(192, 168, 120, 1); // IP Address
IPAddress AP_GW = IPAddress(192, 168, 120, 1); // Gateway
IPAddress AP_SN = IPAddress(255, 255, 255, 0); // Mask
#define AP_CHANNEL 6                           // Channel
#define AP_MAX_CONNECTIONS 8                   // AP Clients (ESP8266 max is 8)
#define AP_HIDDEN_SSID 0                       // 0 = SSID visible | 1 = SSID hidden
*/

// WiFi configuration - STA mode
IPAddress STA_IP(192, 168, 120, 200); // Fixed IP
IPAddress STA_GW(192, 168, 120, 1);   // ESP32-Server gateway
IPAddress STA_SN(255, 255, 255, 0);   // Subnet mask
IPAddress DNS(192, 168, 120, 1);      // DNS (same as gateway)
#define SSID "ESP32-Server"           // SSID
#define PASSWORD "01234567890"        // Password

// Target server configuration
const char* TARGET_SERVER_IP = "192.168.120.1";
const int TARGET_SERVER_PORT = 80;
const char* TARGET_UPLOAD_PATH = "/upload";

// JDY-16 BLE Module (TI CC2541 IC and ZS-040 breakout board)
#define BLE_TX_PIN 12
#define BLE_RX_PIN 13
#define BLE_STATE_PIN 14
#define BLE_EN_PIN 15
SoftwareSerial Serial_BLE(13, 12);

// Feedback Pins
#define BUILT_IN_LED 2
#define EXTERNAL_LED 4

// other definitions
#define CPU_FREQUENCY_MHZ 160 // Max CPU frequency
#define WIFI_TIMEOUT_MS 30000 // 30 second timeout for WiFi connection

void printResponse()
{
    // Print BLE responses to Serial Monitor
    while (Serial_BLE.available())
    {
        Serial.write(Serial_BLE.read());
    }
}

void check_wifi_connection()
{
    // Check WiFi connection status and reconnect if needed
    static unsigned long lastReconnectAttempt = 0;
    const unsigned long reconnectInterval = 10000;

    while (WiFi.status() != WL_CONNECTED || WiFi.status() == WL_CONNECTION_LOST || WiFi.status() == WL_DISCONNECTED)
    {
        digitalWrite(EXTERNAL_LED, LOW); // Indicate disconnection
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        if (millis() - lastReconnectAttempt > reconnectInterval)
        {
            WiFi.reconnect();
            lastReconnectAttempt = millis();
            delay(1000);
        }
    }
}

BridgeStatus bridgeStatus;

// JSON data buffer for complete packet assembly
String jsonBuffer = "";
bool receivingJson = false;

// Data forwarding via WiFi HTTP
bool forwardDataViaWiFi(const String &jsonData)
{
    if (!bridgeStatus.wifi_connected)
    {
        return false;
    }

    WiFiClient client;
    HTTPClient http;

    // Construct the full URL
    String url = String("http://") + TARGET_SERVER_IP + ":" + String(TARGET_SERVER_PORT) + TARGET_UPLOAD_PATH;

    Serial.print("Forwarding to: ");
    Serial.println(url);

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
    }
    else if (httpResponseCode > 0)
    {
        Serial.printf("HTTP Error: %d\n", httpResponseCode);
        String response = http.getString();
        Serial.println("Response: " + response);
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
    // Validate JSON format
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error)
    {
        Serial.print("JSON parsing error: ");
        Serial.println(error.c_str());
        return false;
    }

    // Extract and log key information
    Serial.println("=== JSON Packet Details ===");
    if (doc.containsKey("phys_addr"))
    {
        Serial.println("Device: " + String(doc["phys_addr"].as<const char *>()));
    }
    if (doc.containsKey("uid"))
    {
        Serial.println("RFID: " + String(doc["uid"].as<const char *>()));
    }
    if (doc.containsKey("lat") && doc.containsKey("long"))
    {
        Serial.println("GPS: " + String(doc["lat"].as<const char *>()) +
                       ", " + String(doc["long"].as<const char *>()));
    }
    if (doc.containsKey("timestamp"))
    {
        Serial.println("Timestamp: " + String(doc["timestamp"].as<unsigned long>()));
    }

    // Forward data via WiFi if available
    if (bridgeStatus.wifi_connected)
    {
        return forwardDataViaWiFi(jsonData);
    }
    else
    {
        Serial.println("WiFi not available, cannot forward data");
        return false;
    }
}

// Process incoming JSON data from BLE
void processIncomingBLE()
{
    while (Serial_BLE.available())
    {
        String incoming = Serial_BLE.readStringUntil('\n');
        incoming.trim();

        // Check for JSON packet delimiters
        if (incoming == "START_JSON")
        {
            receivingJson = true;
            jsonBuffer = "";
            Serial.println("Started receiving JSON packet");
        }
        else if (incoming == "END_JSON")
        {
            if (receivingJson)
            {
                receivingJson = false;
                Serial.println("Complete JSON packet received:");
                Serial.println(jsonBuffer);

                // Process the complete JSON packet
                bool success = processJsonPacket(jsonBuffer);

                // Send acknowledgment back to sensor
                if (success)
                {
                    Serial_BLE.println("ACK");
                }
                else
                {
                    Serial_BLE.println("NACK");
                }

                jsonBuffer = "";
            }
        }
        else if (receivingJson)
        {
            // Accumulate JSON data
            jsonBuffer += incoming;
        }
        else
        {
            // Handle other BLE commands/responses
            Serial.print("BLE: ");
            Serial.println(incoming);
        }
    }
}

// WiFi connection checking
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

    // Check BLE connection status every 3 seconds
    if (currentTime - bridgeStatus.last_ble_check > 3000)
    {
        // Simple connectivity check by monitoring data flow
        // In a real implementation, you might ping the connected device

        // For now, we'll assume BLE is connected if we're receiving data
        // You can enhance this by implementing a heartbeat mechanism

        bridgeStatus.last_ble_check = currentTime;
    }
}

// Status reporting function
void reportStatus()
{
    static unsigned long lastReport = 0;

    if (millis() - lastReport > 30000)
    { // Report every 30 seconds
        Serial.println("=== Bridge Status Report ===");
        Serial.printf("WiFi: %s\n", bridgeStatus.wifi_connected ? "Connected" : "Disconnected");
        Serial.printf("BLE: %s\n", bridgeStatus.ble_connected ? "Connected" : "Disconnected");
        Serial.printf("Packets forwarded: %d\n", bridgeStatus.json_packets_forwarded);
        Serial.printf("Connection failures: %d\n", bridgeStatus.connection_failures);
        Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

        if (bridgeStatus.wifi_connected)
        {
            Serial.printf("WiFi Signal: %d dBm\n", WiFi.RSSI());
            Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
        }

        lastReport = millis();
    }
}

void setup()
{
    Serial.begin(9600);
    delay(100);
    system_update_cpu_freq(160); // Set CPU frequency
    delay(500);

    // Initialize feedback pins
    pinMode(BUILT_IN_LED, OUTPUT);
    pinMode(EXTERNAL_LED, OUTPUT);
    digitalWrite(BUILT_IN_LED, LOW);
    digitalWrite(EXTERNAL_LED, HIGH);
    delay(1000);

    Serial.println("=== Bridge Node Starting ===");

    // WiFi setup (unchanged logic, same as original)
    Serial.println("Setting up WiFi (STA)...");
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

    if (!WiFi.begin(SSID, PASSWORD))
    {
        Serial.println("Failed to start WiFi STA. Restarting...");
        ESP.restart();
    }

    // Wait for WiFi connection with timeout
    Serial.println("Connecting to WiFi...");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < 30000)
    {
        delay(1000);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi connected successfully!");
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
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

    // BLE setup
    Serial.println("Setting up BLE module...");
    Serial_BLE.begin(9600);
    delay(2000); // Allow BLE to boot up

    // Configure BLE module with pre-configured settings
    Serial_BLE.print("AT+IMME1\r\n"); // System wait for command when power on
    delay(500);
    Serial_BLE.print("AT+BAUD4\r\n"); // Set UART baud to 9600
    delay(500);
    Serial_BLE.print("AT+POWE3\r\n"); // Set RF power to highest
    delay(500);
    Serial_BLE.print("AT+NAMEESP-BLE\r\n"); // Set name
    delay(500);
    Serial_BLE.print("AT+ROLE0\r\n"); // Set BLE module as peripheral
    delay(500);
    Serial_BLE.print("AT+RESET\r\n"); // Soft reboot
    delay(2000);
    Serial_BLE.print("AT+START\r\n"); // Start BLE advertising
    delay(1000);

    Serial.println("BLE module configured and ready");
    bridgeStatus.ble_connected = true;

    Serial.println("=== Bridge Node Setup Complete ===");
}

void loop()
{
    // Monitor and maintain WiFi connection
    check_wifi_connection();

    // Monitor BLE connection
    check_ble_connection();

    // Process incoming BLE data (JSON packets)
    processIncomingBLE();

    // Handle serial monitor commands (for debugging)
    if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == "status")
        {
            reportStatus();
        }
        else if (command == "reset")
        {
            Serial.println("Resetting bridge node...");
            ESP.restart();
        }
        else if (command.startsWith("AT"))
        {
            // Forward AT commands to BLE module
            Serial_BLE.println(command);
            delay(100);
            while (Serial_BLE.available())
            {
                Serial.write(Serial_BLE.read());
            }
        }
        else
        {
            Serial.println("Commands: status, reset, AT+<command>");
        }
    }
    // Report status periodically
    reportStatus();

    // Small delay to prevent watchdog issues
    delay(10);
}
