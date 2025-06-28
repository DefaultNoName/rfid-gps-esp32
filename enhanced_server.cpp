#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <PsychicHttp.h>
#include <ArduinoJson.h>
#include <vector>
#include <unordered_map>
#include <map>

// WiFi configuration
IPAddress AP_IP = IPAddress(192, 168, 120, 1);
IPAddress AP_GW = IPAddress(192, 168, 120, 1);
IPAddress AP_SN = IPAddress(255, 255, 255, 0);
#define AP_SSID "ESP32_AP"
#define AP_PASSWORD "01234567890"
#define AP_CHANNEL 6
#define AP_MAX_CONNECTIONS 8
#define MAX_WS_CLIENTS 8
#define MAX_SSE_CLIENTS 8

// Server configuration
PsychicHttpServer server;
const unsigned int serverPort = 80;
const unsigned int maxURI = 30;
const unsigned int maxSockets = 16;
const unsigned int lingerTimeout = 3;
const unsigned int taskPriority = 12;
const unsigned int stackSize = 25600;

// WebSocket and SSE handlers
PsychicWebSocketHandler wsHandler;
PsychicEventSource sseHandler;
std::vector<int> wsSockets;
std::vector<int> sseSockets;

// Task handles and semaphores
TaskHandle_t connectionAPHandler = NULL;
TaskHandle_t asyncServerHandler = NULL;
TaskHandle_t watchdogHandler = NULL;
SemaphoreHandle_t xWiFiReadySemaphore;
SemaphoreHandle_t dataMutex;

volatile bool AP_TaskSuccess = false;

// Enhanced Sensor Data Structure for multiple ESP32s
struct SensorData
{
  String esp32_id = "N/A";
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
  unsigned long lastUpdate = 0;
  bool isActive = false;
};

// Multi-ESP32 data storage
std::map<String, SensorData> multiESPData;

// Global map for UID to driver name mappings
std::unordered_map<std::string, std::string> driverMap = {
    {"5394B838", "Allan M."},
    {"63A70F02", "Paulino"},
    {"FB1D0F02", "Dave"},
    {"D9C5A998", "Poging Drayber"},
    {"9AC8BE24", "Grow A Garden Dela Cruz"},
    {"129791AB", "Mama mo"}};

// Function to get driver name from UID
String getDriverName(const String &uid)
{
  std::string uidStd = uid.c_str();
  auto keyValue = driverMap.find(uidStd);
  if (keyValue != driverMap.end())
  {
    return String(keyValue->second.c_str());
  }
  else
  {
    return "Unknown Driver";
  }
}

// Thread-safe data getter for specific ESP32
SensorData getSensorData(const String &esp32_id)
{
  SensorData getData;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    auto it = multiESPData.find(esp32_id);
    if (it != multiESPData.end())
    {
      getData = it->second;
    }
    xSemaphoreGive(dataMutex);
  }
  return getData;
}

// Thread-safe data getter for all ESP32s
std::map<String, SensorData> getAllSensorData()
{
  std::map<String, SensorData> allData;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    allData = multiESPData;
    xSemaphoreGive(dataMutex);
  }
  return allData;
}

// Thread-safe data setter
void setSensorData(const String &esp32_id, const JsonDocument &doc)
{
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    SensorData &data = multiESPData[esp32_id];
    data.esp32_id = esp32_id;
    data.brand_CAR = doc["brand"] | data.brand_CAR;
    data.model_CAR = doc["model"] | data.model_CAR;
    data.pltNum_CAR = doc["pltNum"] | data.pltNum_CAR;
    data.uid_RFID = doc["uid"] | data.uid_RFID;
    data.driver_RFID = getDriverName(data.uid_RFID);
    data.time_GPS = doc["time"] | data.time_GPS;
    data.long_GPS = doc["long"] | data.long_GPS;
    data.lat_GPS = doc["lat"] | data.lat_GPS;
    data.alt_GPS = doc["alt"] | data.alt_GPS;
    data.speed_GPS = doc["spd"] | data.speed_GPS;
    data.sat_GPS = doc["sat"] | data.sat_GPS;
    data.hdop_GPS = doc["hdop"] | data.hdop_GPS;
    data.lastUpdate = millis();
    data.isActive = true;
    xSemaphoreGive(dataMutex);
  }
}

// Create JSON from sensor data
String createSensorJSON(const SensorData &data)
{
  JsonDocument doc;
  doc["esp32_id"] = data.esp32_id;
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
  doc["lastUpdate"] = data.lastUpdate;
  doc["isActive"] = data.isActive;

  String output;
  serializeJson(doc, output);
  return output;
}

// Create JSON for all ESP32s
String createAllSensorJSON()
{
  JsonDocument doc;
  auto allData = getAllSensorData();
  
  for (const auto &pair : allData)
  {
    JsonObject espObj = doc[pair.first].to<JsonObject>();
    espObj["esp32_id"] = pair.second.esp32_id;
    espObj["brand"] = pair.second.brand_CAR;
    espObj["model"] = pair.second.model_CAR;
    espObj["pltNum"] = pair.second.pltNum_CAR;
    espObj["driver"] = pair.second.driver_RFID;
    espObj["uid"] = pair.second.uid_RFID;
    espObj["time"] = pair.second.time_GPS;
    espObj["long"] = pair.second.long_GPS;
    espObj["lat"] = pair.second.lat_GPS;
    espObj["alt"] = pair.second.alt_GPS;
    espObj["spd"] = pair.second.speed_GPS;
    espObj["sat"] = pair.second.sat_GPS;
    espObj["hdop"] = pair.second.hdop_GPS;
    espObj["lastUpdate"] = pair.second.lastUpdate;
    espObj["isActive"] = pair.second.isActive;
  }

  String output;
  serializeJson(doc, output);
  return output;
}

// Broadcast data to all clients
void broadcastSensorData(const String &esp32_id)
{
  String jsonData = createAllSensorJSON();

  // WebSocket broadcast
  for (int sock : wsSockets)
  {
    if (auto *client = wsHandler.getClient(sock))
    {
      client->sendMessage(jsonData.c_str());
    }
  }

  // SSE broadcast
  sseHandler.send("data", jsonData.c_str());
}

// Async server task
void asyncServer(void *asyncServer)
{
  Serial.printf("Initializing HTTP Server on Core #: %d\n", xPortGetCoreID());
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
  
  // Enhanced Dashboard endpoint
  server.on("/", HTTP_GET, [](PsychicRequest *req) -> esp_err_t
            {
    String htmlPage = R"rawliteral(
        <!DOCTYPE html>
        <html lang="en">
        <head>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <title>Multi-ESP32 Vehicle Tracking Dashboard</title>
          <style>
            :root {
              --primary-red: #CF2E2E;
              --dark-gray: #202020;
              --medium-gray: #2A2A2B;
              --light-gray: #F9F9F9;
              --orange: #FF6900;
              --blue: #0693E3;
            }

            * {
              margin: 0;
              padding: 0;
              box-sizing: border-box;
            }

            body {
              font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
              background: linear-gradient(135deg, var(--dark-gray) 0%, var(--medium-gray) 100%);
              color: var(--light-gray);
              min-height: 100vh;
              padding: 1rem;
            }

            .header {
              background: linear-gradient(135deg, var(--primary-red) 0%, #8B1E1E 100%);
              padding: 2rem;
              text-align: center;
              margin-bottom: 2rem;
              border-radius: 12px;
              box-shadow: 0 8px 32px rgba(207, 46, 46, 0.3);
            }

            .header h1 {
              font-size: 2.5rem;
              font-weight: 700;
              margin-bottom: 0.5rem;
              text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            }

            .header p {
              font-size: 1.1rem;
              opacity: 0.9;
            }

            .stats-bar {
              display: flex;
              justify-content: center;
              gap: 2rem;
              margin-bottom: 2rem;
              flex-wrap: wrap;
            }

            .stat-item {
              background: rgba(249, 249, 249, 0.1);
              padding: 1rem 2rem;
              border-radius: 8px;
              backdrop-filter: blur(10px);
              border: 1px solid rgba(255, 255, 255, 0.1);
            }

            .stat-value {
              font-size: 2rem;
              font-weight: 700;
              color: var(--orange);
            }

            .stat-label {
              font-size: 0.9rem;
              opacity: 0.8;
            }

            .connection-status {
              display: inline-flex;
              align-items: center;
              gap: 0.5rem;
              padding: 0.5rem 1rem;
              background: rgba(255, 255, 255, 0.1);
              border-radius: 20px;
              font-size: 0.9rem;
              margin-top: 1rem;
            }

            .vehicles-grid {
              display: grid;
              grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
              gap: 2rem;
              margin-bottom: 2rem;
            }

            .vehicle-card {
              background: rgba(249, 249, 249, 0.95);
              color: var(--dark-gray);
              border-radius: 16px;
              overflow: hidden;
              box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
              transition: transform 0.3s ease, box-shadow 0.3s ease;
              position: relative;
            }

            .vehicle-card:hover {
              transform: translateY(-8px);
              box-shadow: 0 20px 60px rgba(0, 0, 0, 0.25);
            }

            .vehicle-card::before {
              content: '';
              position: absolute;
              top: 0;
              left: 0;
              right: 0;
              height: 4px;
              background: linear-gradient(90deg, var(--primary-red), var(--orange), var(--blue));
            }

            .vehicle-header {
              padding: 1.5rem;
              background: linear-gradient(135deg, var(--light-gray) 0%, #E8E8E8 100%);
              border-bottom: 2px solid var(--primary-red);
            }

            .vehicle-id {
              font-size: 1.2rem;
              font-weight: 700;
              color: var(--primary-red);
              margin-bottom: 0.5rem;
            }

            .vehicle-status {
              display: inline-flex;
              align-items: center;
              gap: 0.5rem;
              padding: 0.25rem 0.75rem;
              border-radius: 12px;
              font-size: 0.8rem;
              font-weight: 600;
            }

            .status-active {
              background: rgba(40, 167, 69, 0.2);
              color: #28a745;
            }

            .status-inactive {
              background: rgba(220, 53, 69, 0.2);
              color: #dc3545;
            }

            .vehicle-body {
              padding: 1.5rem;
            }

            .data-sections {
              display: grid;
              grid-template-columns: 1fr 1fr;
              gap: 1.5rem;
            }

            .data-section {
              background: rgba(42, 42, 43, 0.05);
              padding: 1rem;
              border-radius: 8px;
              border-left: 3px solid var(--primary-red);
            }

            .section-title {
              font-size: 1rem;
              font-weight: 600;
              margin-bottom: 1rem;
              color: var(--primary-red);
              display: flex;
              align-items: center;
              gap: 0.5rem;
            }

            .data-row {
              display: flex;
              justify-content: space-between;
              align-items: center;
              margin-bottom: 0.75rem;
              padding: 0.5rem 0;
              border-bottom: 1px solid rgba(0, 0, 0, 0.05);
            }

            .data-row:last-child {
              border-bottom: none;
              margin-bottom: 0;
            }

            .data-label {
              font-size: 0.85rem;
              color: #666;
              font-weight: 500;
            }

            .data-value {
              font-size: 0.9rem;
              font-weight: 600;
              color: var(--dark-gray);
              background: rgba(6, 147, 227, 0.1);
              padding: 0.25rem 0.5rem;
              border-radius: 4px;
              transition: all 0.3s ease;
            }

            .data-value.updated {
              background: rgba(255, 105, 0, 0.2);
              transform: scale(1.05);
            }

            .no-vehicles {
              text-align: center;
              padding: 3rem;
              background: rgba(249, 249, 249, 0.1);
              border-radius: 12px;
              backdrop-filter: blur(10px);
            }

            .no-vehicles h2 {
              font-size: 1.5rem;
              margin-bottom: 1rem;
              color: var(--orange);
            }

            .last-update {
              font-size: 0.75rem;
              color: #888;
              text-align: right;
              margin-top: 1rem;
            }

            @media (max-width: 768px) {
              .header h1 {
                font-size: 2rem;
              }
              
              .stats-bar {
                gap: 1rem;
              }
              
              .vehicles-grid {
                grid-template-columns: 1fr;
              }
              
              .data-sections {
                grid-template-columns: 1fr;
              }
            }

            @keyframes pulse {
              0% { opacity: 1; }
              50% { opacity: 0.7; }
              100% { opacity: 1; }
            }

            .pulse {
              animation: pulse 2s infinite;
            }
          </style>
        </head>
        <body>
          <div class="header">
            <h1>🚗 Multi-ESP32 Vehicle Tracking</h1>
            <p>Real-time monitoring of multiple vehicles and drivers</p>
            <div class="connection-status" id="connectionStatus">
              <span id="statusIcon">🔄</span>
              <span id="statusText">Connecting...</span>
            </div>
          </div>

          <div class="stats-bar">
            <div class="stat-item">
              <div class="stat-value" id="totalVehicles">0</div>
              <div class="stat-label">Total Vehicles</div>
            </div>
            <div class="stat-item">
              <div class="stat-value" id="activeVehicles">0</div>
              <div class="stat-label">Active Now</div>
            </div>
            <div class="stat-item">
              <div class="stat-value" id="connectedESPs">0</div>
              <div class="stat-label">ESP32 Devices</div>
            </div>
          </div>

          <div class="vehicles-grid" id="vehiclesGrid">
            <div class="no-vehicles">
              <h2>No Vehicles Connected</h2>
              <p>Waiting for ESP32 devices to send data...</p>
            </div>
          </div>

          <script>
            let wsConnected = false, sseConnected = false;
            let vehicleData = {};

            function updateConnectionStatus() {
              const statusIcon = document.getElementById('statusIcon');
              const statusText = document.getElementById('statusText');
              const status = document.getElementById('connectionStatus');
              
              if (wsConnected || sseConnected) {
                statusIcon.textContent = '🟢';
                statusText.textContent = 'Connected';
                status.style.background = 'rgba(40, 167, 69, 0.2)';
                status.style.color = '#28a745';
              } else {
                statusIcon.textContent = '🔴';
                statusText.textContent = 'Disconnected';
                status.style.background = 'rgba(220, 53, 69, 0.2)';
                status.style.color = '#dc3545';
              }
            }

            function updateStats() {
              const totalVehicles = Object.keys(vehicleData).length;
              const activeVehicles = Object.values(vehicleData).filter(v => v.isActive).length;
              
              document.getElementById('totalVehicles').textContent = totalVehicles;
              document.getElementById('activeVehicles').textContent = activeVehicles;
              document.getElementById('connectedESPs').textContent = totalVehicles;
            }

            function createVehicleCard(espId, data) {
              const isActive = data.isActive && (Date.now() - data.lastUpdate < 30000);
              const lastUpdateTime = new Date(data.lastUpdate).toLocaleString();
              
              return `
                <div class="vehicle-card ${isActive ? 'active' : 'inactive'}">
                  <div class="vehicle-header">
                    <div class="vehicle-id">ESP32: ${espId}</div>
                    <div class="vehicle-status ${isActive ? 'status-active' : 'status-inactive'}">
                      <span>${isActive ? '🟢' : '🔴'}</span>
                      ${isActive ? 'Active' : 'Inactive'}
                    </div>
                  </div>
                  <div class="vehicle-body">
                    <div class="data-sections">
                      <div class="data-section">
                        <div class="section-title">🔖 Driver Info</div>
                        <div class="data-row">
                          <span class="data-label">Driver:</span>
                          <span class="data-value">${data.driver || 'N/A'}</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">Card UID:</span>
                          <span class="data-value">${data.uid || 'N/A'}</span>
                        </div>
                      </div>
                      <div class="data-section">
                        <div class="section-title">🚗 Vehicle</div>
                        <div class="data-row">
                          <span class="data-label">Brand:</span>
                          <span class="data-value">${data.brand || 'N/A'}</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">Model:</span>
                          <span class="data-value">${data.model || 'N/A'}</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">Plate:</span>
                          <span class="data-value">${data.pltNum || 'N/A'}</span>
                        </div>
                      </div>
                      <div class="data-section">
                        <div class="section-title">📍 Location</div>
                        <div class="data-row">
                          <span class="data-label">Time:</span>
                          <span class="data-value">${data.time || 'N/A'}</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">Latitude:</span>
                          <span class="data-value">${data.lat || 'N/A'}</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">Longitude:</span>
                          <span class="data-value">${data.long || 'N/A'}</span>
                        </div>
                      </div>
                      <div class="data-section">
                        <div class="section-title">📊 Metrics</div>
                        <div class="data-row">
                          <span class="data-label">Speed:</span>
                          <span class="data-value">${data.spd || 'N/A'} km/h</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">Satellites:</span>
                          <span class="data-value">${data.sat || 'N/A'}</span>
                        </div>
                        <div class="data-row">
                          <span class="data-label">HDOP:</span>
                          <span class="data-value">${data.hdop || 'N/A'}</span>
                        </div>
                      </div>
                    </div>
                    <div class="last-update">Last update: ${lastUpdateTime}</div>
                  </div>
                </div>
              `;
            }

            function updateVehicleDisplay() {
              const grid = document.getElementById('vehiclesGrid');
              
              if (Object.keys(vehicleData).length === 0) {
                grid.innerHTML = `
                  <div class="no-vehicles">
                    <h2>No Vehicles Connected</h2>
                    <p>Waiting for ESP32 devices to send data...</p>
                  </div>
                `;
              } else {
                grid.innerHTML = Object.entries(vehicleData)
                  .map(([espId, data]) => createVehicleCard(espId, data))
                  .join('');
              }
              
              updateStats();
            }

            function processData(data) {
              if (typeof data === 'string') {
                try {
                  data = JSON.parse(data);
                } catch (e) {
                  console.error('JSON parse error:', e);
                  return;
                }
              }
              
              vehicleData = data;
              updateVehicleDisplay();
            }

            // WebSocket connection
            const ws = new WebSocket(`ws://${location.host}/ws`);
            ws.onopen = () => {
              wsConnected = true;
              updateConnectionStatus();
              console.log('WebSocket connected');
            };
            ws.onclose = () => {
              wsConnected = false;
              updateConnectionStatus();
              console.log('WebSocket disconnected');
            };
            ws.onmessage = (e) => {
              processData(e.data);
            };

            // SSE connection
            const sse = new EventSource('/events');
            sse.onopen = () => {
              sseConnected = true;
              updateConnectionStatus();
              console.log('SSE connected');
            };
            sse.onerror = () => {
              sseConnected = false;
              updateConnectionStatus();
              console.log('SSE error');
            };
            sse.addEventListener('data', (e) => {
              processData(e.data);
            });

            // Initial status update
            updateConnectionStatus();
            updateVehicleDisplay();

            // Periodic inactive check
            setInterval(() => {
              let updated = false;
              for (let espId in vehicleData) {
                const wasActive = vehicleData[espId].isActive;
                const isActive = (Date.now() - vehicleData[espId].lastUpdate) < 30000;
                if (wasActive !== isActive) {
                  vehicleData[espId].isActive = isActive;
                  updated = true;
                }
              }
              if (updated) {
                updateVehicleDisplay();
              }
            }, 5000);
          </script>
        </body>
        </html>
      )rawliteral";

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
    Serial.printf("WebSocket Client Connected: Socket %d\n", sock);

    // Send current data to new client
    String jsonData = createAllSensorJSON();
    client->sendMessage(jsonData.c_str()); });

  wsHandler.onFrame([](PsychicWebSocketRequest *request, httpd_ws_frame *frame) -> int
                    {
    String msg = String((const char *)frame->payload, frame->len);
    Serial.println("WS Received: " + msg);
    return 0; });

  wsHandler.onClose([](PsychicWebSocketClient *client)
                    {
    int sock = client->socket();
    wsSockets.erase(std::remove(wsSockets.begin(), wsSockets.end(), sock), wsSockets.end());
    Serial.printf("WebSocket Client Disconnected: Socket %d\n", sock); });

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
    Serial.println("SSE Client Connected");

    // Send current data to new client
    String jsonData = createAllSensorJSON();
    client->send("data", jsonData.c_str()); });

  sseHandler.onClose([](PsychicEventSourceClient *client)
                     {
    int sock = client->socket();
    sseSockets.erase(std::remove(sseSockets.begin(), sseSockets.end(), sock), sseSockets.end());
    Serial.println("SSE client disconnected"); });

  // Enhanced data upload endpoint
