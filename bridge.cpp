#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

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

void wifiCheckConnection()
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

void setup()
{
    Serial.begin(9600);
    delay(100);
    system_update_cpu_freq(CPU_FREQUENCY_MHZ);
    delay(500);

    // Feedback pins
    pinMode(BUILT_IN_LED, OUTPUT);
    pinMode(EXTERNAL_LED, OUTPUT);
    digitalWrite(BUILT_IN_LED, LOW);
    digitalWrite(EXTERNAL_LED, HIGH);
    delay(1000);

    Serial.println("Setting up WiFi (STA)...");
    delay(300);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    // Set WiFi as Station (will connect to an AP)
    if (!WiFi.mode(WIFI_STA))
    {
        Serial.println("Cannot set WiFi as STA mode! Restarting...");
        ESP.restart();
    }
    Serial.println("Success! WiFi mode: STA");

    // Static IP configuration
    if (!WiFi.config(STA_IP, STA_GW, STA_SN, DNS))
    {
        Serial.println("Cannot set WiFi STA parameters! Restarting...");
        ESP.restart();
    }

    // Configure the STA and connect
    if (!WiFi.begin(SSID, PASSWORD))
    {
        Serial.println("Failed to start WiFi STA. Restarting...");
        ESP.restart();
    }
    Serial.println("WiFi STA configured.");

    // Wait for connection with timeout
    Serial.println("Connecting to WiFi...");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - startTime > WIFI_TIMEOUT_MS)
        {
            Serial.println("WiFi connection timeout! Continuing...");
            break;
        }
        delay(1000);
    }

    // Connection successful - show details
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("WiFi connected successfully!");
        Serial.print("  IP Address: ");
        Serial.println(WiFi.localIP());
        Serial.print("  Signal Strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println("    dBm");
    }

    // Verify static IP
    if (!(WiFi.localIP() == IPAddress(192, 168, 120, 200)))
    {
        Serial.println("Failed to set the desired IP Address (120.200/24). Restarting...");
        ESP.restart();
    }

    wifiCheckConnection();

    // BLE Module Setup
    Serial.println("Setting up BLE module...");
    Serial_BLE.begin(9600); // Allow BLE to boot up
    delay(10000);
    Serial_BLE.print("AT+IMME1\r\n"); // System wait for command when power on
    delay(200);
    printResponse();
    Serial_BLE.print("AT+BAUD4\r\n"); // Set UART baud to 9600
    delay(200);
    printResponse();
    Serial_BLE.print("AT+POWE3\r\n"); // Set RF power to highest(?notsureyet)
    delay(200);
    printResponse();
    Serial_BLE.print("AT+NAMEESP-BLE\r\n"); // Set name
    delay(200);
    printResponse();
    Serial_BLE.print("AT+ROLE0\r\n"); // Set BLE module as peripheral
    delay(200);
    printResponse();
    Serial_BLE.print("AT+RESET\r\n"); // Soft reboot
    delay(200);
    printResponse();
    delay(8000);
    Serial_BLE.print("AT+START\r\n"); // Start BLE
    Serial.println("BLE started. Setup complete. You can now enter AT commands. Use Serial Monitor and type 'AT' or 'AT+HELP'.");
}

void loop()
{
    // Bridge input from USB/Serial Monitor to BLE
    if (Serial.available())
    {
        Serial_BLE.print(Serial.readStringUntil('\n') + "\r\n");
        delay(100);
        printResponse();
    }
    // Bridge BLE responses to USB/Serial Monitor
    if (Serial_BLE.available())
    {
        Serial.write(Serial_BLE.read());
    }
    delay(1);
}