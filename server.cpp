#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <PsychicHttp.h>
#include <ArduinoJson.h>
#include <vector>
#include <unordered_map>

// WiFi configuration
IPAddress AP_IP = IPAddress(192, 168, 120, 1); // IP Address
IPAddress AP_GW = IPAddress(192, 168, 120, 1); // Gateway
IPAddress AP_SN = IPAddress(255, 255, 255, 0); // Mask
#define AP_SSID "ESP32_AP"                     // SSID
#define AP_PASSWORD "01234567890"              // Password
#define AP_CHANNEL 6                           // Channel
#define AP_MAX_CONNECTIONS 8                   // AP Clients
#define MAX_WS_CLIENTS 8                       // WebSocket clients
#define MAX_SSE_CLIENTS 8                      // SSE clients

// Server configuration
PsychicHttpServer server;
const unsigned int serverPort = 80;
const unsigned int maxURI = 20;
const unsigned int maxSockets = 12;
const unsigned int lingerTimeout = 3;
const unsigned int taskPriority = 12;
const unsigned int stackSize = 25600;

// WebSocket and SSE handlers
PsychicWebSocketHandler wsHandler;
PsychicEventSource sseHandler;
std::vector<int> wsSockets;  // Track WS Clients
std::vector<int> sseSockets; // Track SSE clients

// Task handles and semaphores
TaskHandle_t connectionAPHandler = NULL;
TaskHandle_t asyncServerHandler = NULL;
TaskHandle_t watchdogHandler = NULL;
SemaphoreHandle_t xWiFiReadySemaphore;
SemaphoreHandle_t dataMutex; // Mutex for thread-safe sensor data access

volatile bool AP_TaskSuccess = false;

// Sensor Data with thread-safe access
struct SensorData
{
  String brand_CAR = "N/A";
  String model_CAR = "N/A";
  String pltNum_CAR = "N/A";
  String driver_RFID = "N/A";
  String uid_RFID = "N/A";
  String time_GPS = "N/A";
  String long_GPS = "N/A";
  String lat_GPS = "N/A";
  String alt_GPS = "N/A";
  String speed_GPS = "N/A";
  String sat_GPS = "N/A";
  String hdop_GPS = "N/A";
} sensorData;

// Global map for UID to driver name mappings
std::unordered_map<std::string, std::string> driverMap = {
    {"5394B838", "Allan M."},
    {"63A70F02", "Paul Lino"},
    {"FB1D0F02", "Dave"},
    {"D9C5A998", "Rey"},
    {"9AC8BE24", "Grow A Garden Dela Cruz"},
    {"129791AB", "Mama mo"}};

// Function to get driver name from UID
String getDriverName(const String &uid)
{
  // Convert Arduino String to std::string for map lookup
  std::string uidStd = uid.c_str();

  // Look up the UID in the map
  auto keyValue = driverMap.find(uidStd);
  if (keyValue != driverMap.end())
  {
    // Convert std::string back to Arduino String
    return String(keyValue->second.c_str());
  }
  else
  {
    return "Unknown Driver";
  }
}

// Thread-safe data getter
SensorData getSensorData()
{
  SensorData getData;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    getData = sensorData;
    xSemaphoreGive(dataMutex);
  }
  return getData;
}

// Thread-safe data setter
void setSensorData(const JsonDocument &doc)
{
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    sensorData.brand_CAR = doc["brand"] | sensorData.brand_CAR;
    sensorData.model_CAR = doc["model"] | sensorData.model_CAR;
    sensorData.pltNum_CAR = doc["pltNum"] | sensorData.pltNum_CAR;
    sensorData.uid_RFID = doc["uid"] | sensorData.uid_RFID;
    sensorData.driver_RFID = getDriverName(sensorData.uid_RFID);
    sensorData.time_GPS = doc["time"] | sensorData.time_GPS;
    sensorData.long_GPS = doc["long"] | sensorData.long_GPS;
    sensorData.lat_GPS = doc["lat"] | sensorData.lat_GPS;
    sensorData.alt_GPS = doc["alt"] | sensorData.alt_GPS;
    sensorData.speed_GPS = doc["spd"] | sensorData.speed_GPS;
    sensorData.sat_GPS = doc["sat"] | sensorData.sat_GPS;
    sensorData.hdop_GPS = doc["hdop"] | sensorData.hdop_GPS;
    xSemaphoreGive(dataMutex);
  }
}

// Create JSON from sensor data
String createSensorJSON(const SensorData &data)
{
  JsonDocument doc;
  doc["brand"] = data.brand_CAR;
  doc["model"] = data.model_CAR;
  doc["pltNum"] = data.pltNum_CAR;
  doc["driver"] = data.driver_RFID;
  doc["uid"] = data.uid_RFID;
  doc["time"] = data.time_GPS;
  doc["long"] = data.long_GPS;
  doc["lat"] = data.lat_GPS;
  doc["alt"] = data.alt_GPS;
  doc["spd"] = data.speed_GPS;
  doc["sat"] = data.sat_GPS;
  doc["hdop"] = data.hdop_GPS;

  String output;
  serializeJson(doc, output);
  return output;
}

// Broadcast data to all clients
void broadcastSensorData(const SensorData &data)
{
  String jsonData = createSensorJSON(data);

  if (wsSockets.size() < MAX_WS_CLIENTS)
  {
    for (int sock : wsSockets)
    {
      if (auto *client = wsHandler.getClient(sock))
      {
        client->sendMessage(jsonData.c_str());
      }
    }
  }
  else
  {
    Serial.println("Max WebSocket clients reached");
  }

  if (sseSockets.size() < MAX_SSE_CLIENTS)
  {
    sseHandler.send("brand", data.brand_CAR.c_str());
    sseHandler.send("model", data.model_CAR.c_str());
    sseHandler.send("pltNum", data.pltNum_CAR.c_str());
    sseHandler.send("driver", data.driver_RFID.c_str());
    sseHandler.send("uid", data.uid_RFID.c_str());
    sseHandler.send("time", data.time_GPS.c_str());
    sseHandler.send("long", data.long_GPS.c_str());
    sseHandler.send("lat", data.lat_GPS.c_str());
    sseHandler.send("alt", data.alt_GPS.c_str());
    sseHandler.send("spd", data.speed_GPS.c_str());
    sseHandler.send("sat", data.sat_GPS.c_str());
    sseHandler.send("hdop", data.hdop_GPS.c_str());
  }
  else
  {
    Serial.println("Max SSE clients reached");
  }
}

// Async server task
void asyncServer(void *asyncServer)
{
  Serial.printf("Initializing HTTP Server on Core #: %d\n", xPortGetCoreID());

  // Small delay to ensure WiFi is fully stable
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Configure server
  Serial.println("Configuring Server Parameters...");
  server.config.max_uri_handlers = maxURI;
  server.config.max_open_sockets = maxSockets;
  server.config.server_port = serverPort;
  server.config.linger_timeout = lingerTimeout;
  server.config.task_priority = taskPriority;
  server.config.stack_size = stackSize;
  server.config.enable_so_linger = true;

  // Start server
  esp_err_t result = server.listen(serverPort);

  if (result != ESP_OK)
  {
    Serial.printf("CRITICAL: HTTP server failed to start - Error: %d\n", result);
    vTaskDelete(NULL);
    return;
  }

  Serial.printf("SUCCESS: HTTP Server is listening! Server accessible at: http://%s:%d\n",
                WiFi.softAPIP().toString().c_str(), serverPort);

  // Configure all endpoints
  Serial.println("Configuring Server Endpoints...");
  // HTTP Dashboard endpoint
  server.on("/", HTTP_GET, [](PsychicRequest *req) -> esp_err_t
            {
    String htmlPage = R"rawliteral(
        <!DOCTYPE html>
        <html lang="en">
        <head>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <title>Vehicle Tracking Dashboard</title>
          <style>
            * {
              margin: 0;
              padding: 0;
              box-sizing: border-box;
            }

            body {
              font-family: -apple-system, BlinkMacSystemFont, 'Roboto', 'Segoe UI', sans-serif;
              background-color: #ffffff0d;
              color: #020300;
              min-height: 100vh;
              padding: 1.5rem;
              display: flex;
              justify-content: center;
              align-items: flex-start;
            }

            .container {
              max-width: 1460px;
              height: 100%;
              width: 100%;
              background: #F7F9F9;
              border-radius: 8px;
              box-shadow: 0 4px 20px rgba(129, 23, 27, 0.2);
              overflow: hidden;
            }

            .header {
              background: linear-gradient(90deg, #AD2E24);
              color: #FFFFFF;
              padding: 1.5rem;
              text-align: center;
            }

            .header h1 {
              font-size: 1.8rem;
              font-weight: 600;
              margin-bottom: 0.5rem;
            }

            .status {
              display: inline-flex;
              align-items: center;
              padding: 0.4rem 1rem;
              background: rgba(247, 249, 249, 0.3);
              border-radius: 16px;
              font-size: 0.85rem;
              font-weight: 500;
            }

            .separator {
              height: 12px;
              background-color: #020300;
              width: 100%;
            }

            .grid {
              display: grid;
              grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
              gap: 1.5rem;
              padding: 1.5rem;
            }

            .card {
              background: #ffffff;
              padding: 15px;
              border-radius: 8px;
              border-left: 3px solid #C83F12;
              box-shadow: 0 2px 8px rgba(129, 23, 27, 0.1);
              transition: transform 0.2s ease, box-shadow 0.2s ease;
            }

            .card:hover {
              transform: translateY(-4px);
              box-shadow: 0 6px 16px rgba(129, 23, 27, 0.4);
            }

            .card h2 {
              color: #020300;
              font-size: 1.3rem;
              font-weight: 500;
              margin-bottom: 1rem;
              display: flex;
              align-items: center;
              gap: 0.5rem;
            }

            .data-row {
              display: flex;
              justify-content: space-between;
              align-items: center;
              padding: 0.6rem 0;
              border-bottom: 1px solid #e8ecef;
            }

            .data-row:last-child {
              border-bottom: none;
            }

            .label {
              font-size: 0.9rem;
              font-weight: 500;
              color: #666;
              flex: 1;
            }

            .value {
              font-size: 0.95rem;
              font-weight: 600;
              color: #020300;
              background: #F7F9F9;
              padding: 0.3rem 0.6rem;
              border-radius: 4px;
              flex: 1;
              text-align: right;
              transition: background-color 0.3s ease;
            }

            .icon {
              font-size: 1.2rem;
            }

            @keyframes fadeIn {
              from {
                opacity: 0;
              }

              to {
                opacity: 1;
              }
            }

            .value.updated {
              animation: fadeIn 0.5s ease;
              background-color: #fff3e6;
            }

            @media (max-width: 768px) {
              body {
                padding: 1rem;
              }

              .grid {
                grid-template-columns: 1fr;
                gap: 1rem;
              }

              .header h1 {
                font-size: 1.5rem;
              }

              .card {
                padding: 1rem;
              }
            }

            @media (max-width: 480px) {
              .header {
                padding: 1rem;
              }

              .status {
                font-size: 0.75rem;
              }

              .card h2 {
                font-size: 1.1rem;
              }

              .label,
              .value {
                font-size: 0.85rem;
              }
            }
          </style>
        </head>
        <body>
          <div class="container">
            <div class="header">
              <h1>Tracking Dashboard</h1>
              <div class="status" id="connectionStatus">Connecting...</div>
            </div>
            <div class="separator"></div> <!-- Black bar separator -->
            <div class="grid">
              <div class="card">
                <h2><span class="icon">🔖</span>Driver Details</h2>
                <div class="data-row">
                  <span class="label">Driver:</span>
                  <span class="value" id="driver">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Card UID:</span>
                  <span class="value" id="uid">N/A</span>
                </div>
              </div>
              <div class="card">
                <h2><span class="icon">🪪</span>Vehicle Data</h2>
                <div class="data-row">
                  <span class="label">Brand:</span>
                  <span class="value" id="brand">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Model:</span>
                  <span class="value" id="model">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Plate Number:</span>
                  <span class="value" id="pltNum">N/A</span>
                </div>
              </div>
              <div class="card">
                <h2><span class="icon">📍</span>GPS Location</h2>
                <div class="data-row">
                  <span class="label">Time (UTC):</span>
                  <span class="value" id="time">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Latitude:</span>
                  <span class="value" id="lat">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Longitude:</span>
                  <span class="value" id="long">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Altitude (m):</span>
                  <span class="value" id="alt">N/A</span>
                </div>
              </div>              
              <div class="card">
                <h2><span class="icon">🚗</span>Movement Data</h2>
                <div class="data-row">
                  <span class="label">Speed (km/h):</span>
                  <span class="value" id="spd">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Satellites:</span>
                  <span class="value" id="sat">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">HDOP:</span>
                  <span class="value" id="hdop">N/A</span>
                </div>
              </div>
            </div>
          </div>
          <script>
            const status = document.getElementById('connectionStatus');
            let wsConnected = false, sseConnected = false;
            function updateStatus() {
              if (wsConnected || sseConnected) {
                status.textContent = '🟢 Connected';
                status.style.background = 'rgba(40, 167, 69, 0.8)';
              } else {
                status.textContent = '🔴 Disconnected';
                status.style.background = 'rgba(220, 53, 69, 0.8)';
              }
            }
            function updateField(id, value, animate = true) {
              const element = document.getElementById(id);
              if (element && element.textContent !== value) {
                element.textContent = value;
                if (animate) {
                  element.classList.add('updated');
                  setTimeout(() => element.classList.remove('updated'), 500);
                }
              }
            }
            const ws = new WebSocket(`ws://${location.host}/ws`);
            ws.onopen = () => { wsConnected = true; updateStatus(); };
            ws.onclose = () => { wsConnected = false; updateStatus(); };
            ws.onmessage = e => {
              try {
                const d = JSON.parse(e.data);
                Object.keys(d).forEach(key => updateField(key, d[key]));
              } catch (err) { console.error('WS JSON parse:', err); }
            };
            const sse = new EventSource('/events');
            sse.onopen = () => { sseConnected = true; updateStatus(); };
            sse.onerror = () => { sseConnected = false; updateStatus(); };
            ['driver', 'uid', 'time', 'long', 'lat', 'alt', 'spd', 'sat', 'hdop'].forEach(field => {
              sse.addEventListener(field, e => updateField(field, e.data));
            });
            updateStatus();
          </script>
        </body>
        </html>
      )rawliteral";
    // Assign to a local variable to ensure the buffer stays valid
    static String htmlCopy;
    htmlCopy = htmlPage;
    return req->reply(200, "text/html", htmlCopy.c_str()); });

  // WebSocket endpoint
  server.on("/ws", HTTP_GET, &wsHandler);
  wsHandler.onOpen([](PsychicWebSocketClient *client)
                   {
    if (wsSockets.size() >= MAX_WS_CLIENTS)
    {
      client->sendMessage("Max Clients Reached");
      client->close();
      return;
    }
    int sock = client->socket();
    wsSockets.push_back(sock);
    Serial.printf("Websockets Client Connected: Socket %d\n", sock);

    // Send current data to new client
    SensorData data = getSensorData();
    String jsonData = createSensorJSON(data);
    client->sendMessage(jsonData.c_str()); });

  wsHandler.onFrame([](PsychicWebSocketRequest *request, httpd_ws_frame *frame) -> int
                    {
    String msg = String((const char *)frame->payload, frame->len);
    Serial.println("WS Received: " + msg);
    request->client()->sendMessage(msg.c_str());
    return 0; });

  wsHandler.onClose([](PsychicWebSocketClient *client)
                    {
    int sock = client->socket();
    wsSockets.erase(std::remove(wsSockets.begin(), wsSockets.end(), sock), wsSockets.end());
    Serial.printf("Websockets Client Disconnected: Socket %d\n", sock); });

  // SSE endpoint
  server.on("/events", HTTP_GET, &sseHandler);
  sseHandler.onOpen([](PsychicEventSourceClient *client)
                    {
    if (sseSockets.size() >= MAX_SSE_CLIENTS)
    {
      client->send("Max Clients Reached", "error");
      client->close();
      return;
    }
    int sock = client->socket();
    sseSockets.push_back(sock);
    Serial.println("Server-Side Client Connected");

    // Send current data to new client
    SensorData data = getSensorData();
    client->send("brand", data.brand_CAR.c_str());
    client->send("model", data.model_CAR.c_str());
    client->send("pltNum", data.pltNum_CAR.c_str());
    client->send("driver", data.driver_RFID.c_str());
    client->send("uid", data.uid_RFID.c_str());
    client->send("time", data.time_GPS.c_str());
    client->send("long", data.long_GPS.c_str());
    client->send("lat", data.lat_GPS.c_str());
    client->send("alt", data.alt_GPS.c_str());
    client->send("spd", data.speed_GPS.c_str());
    client->send("sat", data.sat_GPS.c_str());
    client->send("hdop", data.hdop_GPS.c_str()); });

  sseHandler.onClose([](PsychicEventSourceClient *client)
                     {
    int sock = client->socket();
    sseSockets.erase(std::remove(sseSockets.begin(), sseSockets.end(), sock), sseSockets.end());
    Serial.println("SSE client disconnected"); });

  // Data upload endpoint
  server.on("/upload", HTTP_POST, [](PsychicRequest *req) -> esp_err_t
            {
    String body = req->body();
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error)
    {
      Serial.printf("JSON parse error: %s\n", error.c_str());
      return req->reply(400, "text/plain", "Bad JSON");
    }

    setSensorData(doc);
    SensorData currentData = getSensorData();
    broadcastSensorData(currentData);
    Serial.println("Data updated and broadcasted");
    return req->reply(200, "text/plain", "OK"); });

  Serial.println("Server Endpoints Configured Successfully!");
  Serial.println("HTTP server is fully operational!");

  // Check Server Health
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(10000)); // Keep task alive, check every 10 seconds
    Serial.printf("Server Health: Stack Size: %d/%d (bytes) | AP Clients: %d | WS Clients: %d | SSE Clients: %d\n",
                  stackSize - uxTaskGetStackHighWaterMark(asyncServerHandler), stackSize,
                  WiFi.softAPgetStationNum(), wsSockets.size(), sseSockets.size());
  }
  // Server task continues to run asychronously
}

// Connection AP task
void connectionAP(void *connection)
{
  Serial.println("Initializing WiFi Service...");
  WiFi.disconnect(true);
  vTaskDelay(pdMS_TO_TICKS(500));
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.mode(WIFI_AP);
  vTaskDelay(pdMS_TO_TICKS(500));

  WiFi.softAPConfig(AP_IP, AP_GW, AP_SN);
  bool connectionOKAY = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTIONS);

  if (connectionOKAY)
  {
    esp_wifi_set_max_tx_power(78);

    // Wait a moment for the access point to fully initialize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Verify the access point is actually working
    IPAddress apIP = WiFi.softAPIP();
    if (apIP == IPAddress(0, 0, 0, 0))
    {
      Serial.println("Access Point IP address is invalid!");
      AP_TaskSuccess = false;
    }
    else
    {
      Serial.printf("  SSID: %s\n", WiFi.softAPSSID().c_str());
      Serial.printf("  IP Address: %s\n", apIP.toString().c_str());
      Serial.printf("  Subnet Mask: %s\n", WiFi.softAPSubnetMask().toString().c_str());
      // Serial.printf("  Access Point fully operational\n");
      AP_TaskSuccess = true;
    }
  }
  xSemaphoreGive(xWiFiReadySemaphore);
  vTaskDelete(NULL);
}

void setup()
{
  setCpuFrequencyMhz(160);
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== ESP32 VEHICLE TRACKING SYSTEM ===");
  // Create mutex and semaphore
  dataMutex = xSemaphoreCreateMutex();
  xWiFiReadySemaphore = xSemaphoreCreateBinary();

  if (dataMutex == NULL || xWiFiReadySemaphore == NULL)
  {
    Serial.println("CRITICAL: Failed to create synchronization objects");
    return;
  }
  // Create AP task
  BaseType_t taskCreated1 = xTaskCreatePinnedToCore(
      connectionAP,
      "WiFi AP Task",
      4096,
      NULL,
      1,
      &connectionAPHandler,
      CONFIG_ARDUINO_RUNNING_CORE);

  if (taskCreated1 != pdPASS)
  {
    Serial.println("CRITICAL: Failed to create Access Point task");
    return;
  }

  // Wait for WiFi to be ready
  if (xSemaphoreTake(xWiFiReadySemaphore, pdMS_TO_TICKS(15000)) == pdTRUE)
  {
    if (AP_TaskSuccess)
    {
      // Give the AP some time to fully stabilize
      vTaskDelay(pdMS_TO_TICKS(2000));

      // Create server task - no need to wait for it to signal back
      BaseType_t taskCreated2 = xTaskCreatePinnedToCore(
          asyncServer,
          "HTTP Server task",
          25600,
          NULL,
          2,
          &asyncServerHandler,
          CONFIG_ARDUINO_RUNNING_CORE);

      if (taskCreated2 == pdPASS)
      {
        Serial.println("HTTP Server task created - it will initialize independently");
      }
      else
      {
        Serial.println("Failed to create HTTP Server task");
      }
    }
    else
    {
      Serial.println("Access Point setup failed");
    }
  }
  else
  {
    Serial.println("TIMEOUT: WiFi Access Point failed to initialize");
  }

  Serial.println("=== SETUP COMPLETE ===");
}

void loop()
{
  /*
  static unsigned long lastHealthCheck = 0;
  static unsigned long lastTaskCheck = 0;

  if (millis() - lastHealthCheck > 30000)
  {
    int freeHeap = ESP.getFreeHeap();
    int connectedClients = WiFi.softAPgetStationNum();
    Serial.printf("System Health: Free Heap: %d, Connected Clients: %d\n", freeHeap, connectedClients);
    if (freeHeap < 20000)
    {
      Serial.println("WARNING: Low free heap memory!");
    }
    lastHealthCheck = millis();
  }

  // Add task status checking every 10 seconds
  if (millis() - lastTaskCheck > 10000)
  {
    printTaskStatus();
    lastTaskCheck = millis();
  }*/

  vTaskDelay(pdMS_TO_TICKS(1000));
}