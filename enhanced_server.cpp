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
#define AP_MAX_CONNECTIONS 12
#define MAX_WS_CLIENTS 12
#define MAX_SSE_CLIENTS 12

// Server configuration
PsychicHttpServer server;
const unsigned int serverPort = 80;
const unsigned int maxURI = 30;
const unsigned int maxSockets = 20;
const unsigned int lingerTimeout = 3;
const unsigned int taskPriority = 12;
const unsigned int stackSize = 32768;

// WebSocket and SSE handlers
PsychicWebSocketHandler wsHandler;
PsychicEventSource sseHandler;
std::vector<int> wsSockets;
std::vector<int> sseSockets;

// Task handles and semaphores
TaskHandle_t connectionAPHandler = NULL;
TaskHandle_t asyncServerHandler = NULL;
SemaphoreHandle_t xWiFiReadySemaphore;
SemaphoreHandle_t devicesMutex;

volatile bool AP_TaskSuccess = false;

// Multi-device sensor data structure
struct DeviceData {
  String deviceId;
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
  bool isActive = true;
};

// Map to store multiple devices
std::map<String, DeviceData> devices;

// Driver mapping
std::unordered_map<std::string, std::string> driverMap = {
  {"5394B838", "Allan M."},
  {"63A70F02", "Paulino"},
  {"FB1D0F02", "Dave"},
  {"D9C5A998", "Poging Drayber"},
  {"9AC8BE24", "Grow A Garden Dela Cruz"},
  {"129791AB", "Mama mo"}
};

String getDriverName(const String &uid) {
  std::string uidStd = uid.c_str();
  auto keyValue = driverMap.find(uidStd);
  if (keyValue != driverMap.end()) {
    return String(keyValue->second.c_str());
  }
  return "Unknown Driver";
}

// Thread-safe device operations
DeviceData getDeviceData(const String &deviceId) {
  DeviceData data;
  if (xSemaphoreTake(devicesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    auto it = devices.find(deviceId);
    if (it != devices.end()) {
      data = it->second;
    }
    xSemaphoreGive(devicesMutex);
  }
  return data;
}

void setDeviceData(const String &deviceId, const JsonDocument &doc) {
  if (xSemaphoreTake(devicesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    DeviceData &device = devices[deviceId];
    device.deviceId = deviceId;
    device.brand_CAR = doc["brand"] | device.brand_CAR;
    device.model_CAR = doc["model"] | device.model_CAR;
    device.pltNum_CAR = doc["pltNum"] | device.pltNum_CAR;
    device.uid_RFID = doc["uid"] | device.uid_RFID;
    device.driver_RFID = getDriverName(device.uid_RFID);
    device.time_GPS = doc["time"] | device.time_GPS;
    device.long_GPS = doc["long"] | device.long_GPS;
    device.lat_GPS = doc["lat"] | device.lat_GPS;
    device.alt_GPS = doc["alt"] | device.alt_GPS;
    device.speed_GPS = doc["spd"] | device.speed_GPS;
    device.sat_GPS = doc["sat"] | device.sat_GPS;
    device.hdop_GPS = doc["hdop"] | device.hdop_GPS;
    device.lastUpdate = millis();
    device.isActive = true;
    xSemaphoreGive(devicesMutex);
  }
}

String createDevicesJSON() {
  JsonDocument doc;
  JsonArray devicesArray = doc.createNestedArray("devices");
  
  if (xSemaphoreTake(devicesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (auto &pair : devices) {
      DeviceData &device = pair.second;
      // Mark as inactive if no update for 30 seconds
      if (millis() - device.lastUpdate > 30000) {
        device.isActive = false;
      }
      
      JsonObject deviceObj = devicesArray.createNestedObject();
      deviceObj["deviceId"] = device.deviceId;
      deviceObj["brand"] = device.brand_CAR;
      deviceObj["model"] = device.model_CAR;
      deviceObj["pltNum"] = device.pltNum_CAR;
      deviceObj["driver"] = device.driver_RFID;
      deviceObj["uid"] = device.uid_RFID;
      deviceObj["time"] = device.time_GPS;
      deviceObj["long"] = device.long_GPS;
      deviceObj["lat"] = device.lat_GPS;
      deviceObj["alt"] = device.alt_GPS;
      deviceObj["spd"] = device.speed_GPS;
      deviceObj["sat"] = device.sat_GPS;
      deviceObj["hdop"] = device.hdop_GPS;
      deviceObj["lastUpdate"] = device.lastUpdate;
      deviceObj["isActive"] = device.isActive;
    }
    xSemaphoreGive(devicesMutex);
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

void broadcastDevicesData() {
  String jsonData = createDevicesJSON();
  
  // Broadcast to WebSocket clients
  for (int sock : wsSockets) {
    if (auto *client = wsHandler.getClient(sock)) {
      client->sendMessage(jsonData.c_str());
    }
  }
  
  // Broadcast to SSE clients
  sseHandler.send("devices", jsonData.c_str());
}

void asyncServer(void *asyncServer) {
  Serial.printf("Initializing HTTP Server on Core #: %d\n", xPortGetCoreID());
  vTaskDelay(pdMS_TO_TICKS(1000));

  server.config.max_uri_handlers = maxURI;
  server.config.max_open_sockets = maxSockets;
  server.config.server_port = serverPort;
  server.config.linger_timeout = lingerTimeout;
  server.config.task_priority = taskPriority;
  server.config.stack_size = stackSize;
  server.config.enable_so_linger = true;

  esp_err_t result = server.listen(serverPort);
  if (result != ESP_OK) {
    Serial.printf("CRITICAL: HTTP server failed to start - Error: %d\n", result);
    vTaskDelete(NULL);
    return;
  }

  Serial.printf("SUCCESS: HTTP Server listening at: http://%s:%d\n",
                WiFi.softAPIP().toString().c_str(), serverPort);

  // Enhanced Dashboard with multi-device support
  server.on("/", HTTP_GET, [](PsychicRequest *req) -> esp_err_t {
    String htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Multi-Device Vehicle Tracking Dashboard</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', sans-serif;
            background: linear-gradient(135deg, #2A2A2B 0%, #202020 100%);
            color: #F9F9F9;
            min-height: 100vh;
            overflow-x: hidden;
        }

        .header {
            background: linear-gradient(135deg, #CF2E2E 0%, #FF6900 100%);
            padding: 2rem;
            text-align: center;
            box-shadow: 0 4px 20px rgba(207, 46, 46, 0.3);
            position: relative;
            overflow: hidden;
        }

        .header::before {
            content: '';
            position: absolute;
            top: 0;
            left: -100%;
            width: 100%;
            height: 100%;
            background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.1), transparent);
            animation: shine 3s infinite;
        }

        @keyframes shine {
            0% { left: -100%; }
            100% { left: 100%; }
        }

        .header h1 {
            font-size: 2.5rem;
            font-weight: 700;
            margin-bottom: 0.5rem;
            text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.3);
            position: relative;
            z-index: 1;
        }

        .header p {
            font-size: 1.1rem;
            opacity: 0.9;
            position: relative;
            z-index: 1;
        }

        .status-bar {
            background: #202020;
            padding: 1rem 2rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
            flex-wrap: wrap;
            gap: 1rem;
            border-bottom: 3px solid #0693E3;
        }

        .connection-status {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            padding: 0.5rem 1rem;
            background: rgba(249, 249, 249, 0.1);
            border-radius: 25px;
            backdrop-filter: blur(10px);
            transition: all 0.3s ease;
        }

        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }

        .status-connected { background: #0693E3; }
        .status-disconnected { background: #CF2E2E; }

        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .device-count {
            font-size: 1.1rem;
            font-weight: 600;
            text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
        }

        .main-container {
            padding: 2rem;
            max-width: 1400px;
            margin: 0 auto;
        }

        .no-devices {
            text-align: center;
            padding: 3rem;
            background: rgba(249, 249, 249, 0.05);
            border-radius: 16px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(249, 249, 249, 0.1);
        }

        .no-devices h2 {
            color: #FF6900;
            margin-bottom: 1rem;
            font-size: 1.8rem;
        }

        .devices-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(380px, 1fr));
            gap: 2rem;
            margin-top: 2rem;
        }

        .device-card {
            background: rgba(249, 249, 249, 0.05);
            backdrop-filter: blur(15px);
            border-radius: 20px;
            padding: 1.5rem;
            border: 1px solid rgba(249, 249, 249, 0.1);
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
            transition: all 0.4s ease;
            position: relative;
            overflow: hidden;
        }

        .device-card:hover {
            transform: translateY(-8px);
            box-shadow: 0 16px 48px rgba(6, 147, 227, 0.2);
            border-color: #0693E3;
        }

        .device-card.inactive {
            opacity: 0.6;
            border-color: #CF2E2E;
        }

        .device-card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 4px;
            background: linear-gradient(90deg, #0693E3, #FF6900);
        }

        .device-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1.5rem;
            padding-bottom: 1rem;
            border-bottom: 1px solid rgba(249, 249, 249, 0.1);
        }

        .device-id {
            font-size: 1.2rem;
            font-weight: 700;
            color: #0693E3;
            text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
        }

        .device-status {
            padding: 0.3rem 0.8rem;
            border-radius: 15px;
            font-size: 0.8rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .status-active {
            background: rgba(6, 147, 227, 0.2);
            color: #0693E3;
            border: 1px solid #0693E3;
        }

        .status-inactive {
            background: rgba(207, 46, 46, 0.2);
            color: #CF2E2E;
            border: 1px solid #CF2E2E;
        }

        .data-sections {
            display: grid;
            gap: 1rem;
        }

        .data-section {
            background: rgba(32, 32, 32, 0.5);
            border-radius: 12px;
            padding: 1rem;
            border-left: 3px solid #FF6900;
        }

        .section-title {
            font-size: 0.9rem;
            font-weight: 600;
            color: #FF6900;
            margin-bottom: 0.8rem;
            text-transform: uppercase;
            letter-spacing: 1px;
        }

        .data-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
            gap: 0.8rem;
        }

        .data-item {
            background: rgba(249, 249, 249, 0.05);
            border-radius: 8px;
            padding: 0.8rem;
            transition: all 0.3s ease;
        }

        .data-item:hover {
            background: rgba(6, 147, 227, 0.1);
        }

        .data-label {
            font-size: 0.75rem;
            color: rgba(249, 249, 249, 0.7);
            margin-bottom: 0.3rem;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }

        .data-value {
            font-size: 1rem;
            font-weight: 600;
            color: #F9F9F9;
            word-break: break-word;
        }

        .data-value.updated {
            animation: highlight 0.6s ease;
        }

        @keyframes highlight {
            0% { background: rgba(255, 105, 0, 0.3); }
            100% { background: transparent; }
        }

        .last-update {
            margin-top: 1rem;
            padding-top: 1rem;
            border-top: 1px solid rgba(249, 249, 249, 0.1);
            font-size: 0.8rem;
            color: rgba(249, 249, 249, 0.6);
            text-align: center;
        }

        @media (max-width: 768px) {
            .header h1 { font-size: 2rem; }
            .devices-grid { grid-template-columns: 1fr; }
            .status-bar { flex-direction: column; text-align: center; }
            .main-container { padding: 1rem; }
            .data-grid { grid-template-columns: 1fr; }
        }

        .loading {
            text-align: center;
            padding: 2rem;
            color: #0693E3;
            font-size: 1.1rem;
        }

        .spinner {
            width: 40px;
            height: 40px;
            border: 4px solid rgba(6, 147, 227, 0.3);
            border-top: 4px solid #0693E3;
            border-radius: 50%;
            animation: spin 1s linear infinite;
            margin: 0 auto 1rem;
        }

        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚗 Multi-Device Vehicle Tracking</h1>
        <p>Real-time monitoring of multiple ESP32 tracking devices</p>
    </div>

    <div class="status-bar">
        <div class="connection-status" id="connectionStatus">
            <div class="status-indicator status-disconnected" id="statusIndicator"></div>
            <span id="statusText">Connecting...</span>
        </div>
        <div class="device-count" id="deviceCount">Devices: 0</div>
    </div>

    <div class="main-container">
        <div class="loading" id="loadingIndicator">
            <div class="spinner"></div>
            <div>Loading devices...</div>
        </div>
        
        <div class="no-devices" id="noDevices" style="display: none;">
            <h2>No Active Devices</h2>
            <p>Waiting for ESP32 devices to connect and send data...</p>
        </div>

        <div class="devices-grid" id="devicesGrid"></div>
    </div>

    <script>
        let wsConnected = false, sseConnected = false;
        let devices = {};

        function updateConnectionStatus() {
            const indicator = document.getElementById('statusIndicator');
            const text = document.getElementById('statusText');
            
            if (wsConnected || sseConnected) {
                indicator.className = 'status-indicator status-connected';
                text.textContent = '🟢 Connected';
            } else {
                indicator.className = 'status-indicator status-disconnected';
                text.textContent = '🔴 Disconnected';
            }
        }

        function formatLastUpdate(timestamp) {
            if (!timestamp) return 'Never';
            const now = Date.now();
            const diff = now - timestamp;
            const seconds = Math.floor(diff / 1000);
            
            if (seconds < 60) return `${seconds}s ago`;
            if (seconds < 3600) return `${Math.floor(seconds / 60)}m ago`;
            return `${Math.floor(seconds / 3600)}h ago`;
        }

        function createDeviceCard(device) {
            const isActive = device.isActive;
            const statusClass = isActive ? 'status-active' : 'status-inactive';
            const cardClass = isActive ? '' : 'inactive';
            
            return `
                <div class="device-card ${cardClass}" id="device-${device.deviceId}">
                    <div class="device-header">
                        <div class="device-id">Device: ${device.deviceId}</div>
                        <div class="device-status ${statusClass}">
                            ${isActive ? 'Active' : 'Inactive'}
                        </div>
                    </div>
                    
                    <div class="data-sections">
                        <div class="data-section">
                            <div class="section-title">👤 Driver Information</div>
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Driver</div>
                                    <div class="data-value" id="${device.deviceId}-driver">${device.driver}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Card UID</div>
                                    <div class="data-value" id="${device.deviceId}-uid">${device.uid}</div>
                                </div>
                            </div>
                        </div>

                        <div class="data-section">
                            <div class="section-title">🚗 Vehicle Details</div>
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Brand</div>
                                    <div class="data-value" id="${device.deviceId}-brand">${device.brand}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Model</div>
                                    <div class="data-value" id="${device.deviceId}-model">${device.model}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Plate Number</div>
                                    <div class="data-value" id="${device.deviceId}-pltNum">${device.pltNum}</div>
                                </div>
                            </div>
                        </div>

                        <div class="data-section">
                            <div class="section-title">📍 Location & Movement</div>
                            <div class="data-grid">
                                <div class="data-item">
                                    <div class="data-label">Time (UTC)</div>
                                    <div class="data-value" id="${device.deviceId}-time">${device.time}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Latitude</div>
                                    <div class="data-value" id="${device.deviceId}-lat">${device.lat}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Longitude</div>
                                    <div class="data-value" id="${device.deviceId}-long">${device.long}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Altitude (m)</div>
                                    <div class="data-value" id="${device.deviceId}-alt">${device.alt}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Speed (km/h)</div>
                                    <div class="data-value" id="${device.deviceId}-spd">${device.spd}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">Satellites</div>
                                    <div class="data-value" id="${device.deviceId}-sat">${device.sat}</div>
                                </div>
                                <div class="data-item">
                                    <div class="data-label">HDOP</div>
                                    <div class="data-value" id="${device.deviceId}-hdop">${device.hdop}</div>
                                </div>
                            </div>
                        </div>
                    </div>
                    
                    <div class="last-update">
                        Last Update: <span id="${device.deviceId}-lastUpdate">${formatLastUpdate(device.lastUpdate)}</span>
                    </div>
                </div>
            `;
        }

        function updateDeviceValue(deviceId, field, value) {
            const element = document.getElementById(`${deviceId}-${field}`);
            if (element && element.textContent !== value) {
                element.textContent = value;
                element.classList.add('updated');
                setTimeout(() => element.classList.remove('updated'), 600);
            }
        }

        function updateDevices(devicesData) {
            const grid = document.getElementById('devicesGrid');
            const loading = document.getElementById('loadingIndicator');
            const noDevices = document.getElementById('noDevices');
            const deviceCount = document.getElementById('deviceCount');
            
            loading.style.display = 'none';
            
            if (!devicesData || devicesData.length === 0) {
                noDevices.style.display = 'block';
                grid.innerHTML = '';
                deviceCount.textContent = 'Devices: 0';
                return;
            }
            
            noDevices.style.display = 'none';
            deviceCount.textContent = `Devices: ${devicesData.length} (${devicesData.filter(d => d.isActive).length} active)`;
            
            // Update existing devices or create new ones
            devicesData.forEach(device => {
                const existingCard = document.getElementById(`device-${device.deviceId}`);
                
                if (!existingCard) {
                    // Create new device card
                    const cardHTML = createDeviceCard(device);
                    grid.insertAdjacentHTML('beforeend', cardHTML);
                } else {
                    // Update existing device
                    const fields = ['driver', 'uid', 'brand', 'model', 'pltNum', 'time', 'lat', 'long', 'alt', 'spd', 'sat', 'hdop'];
                    fields.forEach(field => {
                        updateDeviceValue(device.deviceId, field, device[field]);
                    });
                    
                    // Update status
                    const statusEl = existingCard.querySelector('.device-status');
                    const cardEl = existingCard;
                    
                    if (device.isActive) {
                        statusEl.className = 'device-status status-active';
                        statusEl.textContent = 'Active';
                        cardEl.classList.remove('inactive');
                    } else {
                        statusEl.className = 'device-status status-inactive';
                        statusEl.textContent = 'Inactive';
                        cardEl.classList.add('inactive');
                    }
                    
                    // Update last update time
                    updateDeviceValue(device.deviceId, 'lastUpdate', formatLastUpdate(device.lastUpdate));
                }
            });
        }

        // WebSocket connection
        const ws = new WebSocket(`ws://${location.host}/ws`);
        ws.onopen = () => { wsConnected = true; updateConnectionStatus(); };
        ws.onclose = () => { wsConnected = false; updateConnectionStatus(); };
        ws.onmessage = e => {
            try {
                const data = JSON.parse(e.data);
                if (data.devices) {
                    updateDevices(data.devices);
                }
            } catch (err) { console.error('WS JSON parse:', err); }
        };

        // SSE connection
        const sse = new EventSource('/events');
        sse.onopen = () => { sseConnected = true; updateConnectionStatus(); };
        sse.onerror = () => { sseConnected = false; updateConnectionStatus(); };
        sse.addEventListener('devices', e => {
            try {
                const data = JSON.parse(e.data);
                if (data.devices) {
                    updateDevices(data.devices);
                }
            } catch (err) { console.error('SSE JSON parse:', err); }
        });

        // Update last update times every second
        setInterval(() => {
            document.querySelectorAll('[id$="-lastUpdate"]').forEach(el => {
                const deviceId = el.id.replace('-lastUpdate', '');
                const device = devices[deviceId];
                if (device) {
                    el.textContent = formatLastUpdate(device.lastUpdate);
                }
            