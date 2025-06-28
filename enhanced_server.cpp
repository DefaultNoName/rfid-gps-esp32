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
IPAddress AP_IP = IPAddress(192, 168, 120, 1); // IP Address
IPAddress AP_GW = IPAddress(192, 168, 120, 1); // Gateway
IPAddress AP_SN = IPAddress(255, 255, 255, 0); // Mask
#define AP_SSID "ESP32_AP"                     // SSID
#define AP_PASSWORD "01234567890"              // Password
#define AP_CHANNEL 6                           // Channel
#define AP_MAX_CONNECTIONS 8                   // AP Clients
#define MAX_WS_CLIENTS 8                       // WebSocket clients
#define MAX_SSE_CLIENTS 8                      // SSE clients
#define MAX_ESP32_CLIENTS 10                   // Maximum number of ESP32 sensor nodes

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

// Enhanced sensor data structure for multiple clients
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
  unsigned long lastUpdate = 0; // Timestamp of last update
  bool isOnline = false;        // Connection status
};

// Client registration structure
struct ESP32Client 
{
  String macAddress;
  String friendlyName;
  String ipAddress;
  unsigned long lastSeen;
  unsigned long registrationTime;
  bool isActive;
};

// Multi-client data storage with MAC address as key
std::map<String, SensorData> clientSensorData;
std::map<String, ESP32Client> registeredClients;
String activeClientMAC = ""; // Currently selected client for dashboard display

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

// Function to register a new ESP32 client
bool registerClient(const String &macAddress, const String &ipAddress, const String &friendlyName = "")
{
  if (registeredClients.size() >= MAX_ESP32_CLIENTS)
  {
    Serial.println("Maximum client limit reached!");
    return false;
  }

  ESP32Client newClient;
  newClient.macAddress = macAddress;
  newClient.ipAddress = ipAddress;
  newClient.friendlyName = friendlyName.isEmpty() ? "ESP32_" + macAddress.substring(9) : friendlyName;
  newClient.lastSeen = millis();
  newClient.registrationTime = millis();
  newClient.isActive = true;

  registeredClients[macAddress] = newClient;
  
  // Initialize sensor data for this client
  clientSensorData[macAddress] = SensorData();
  
  // If this is the first client, make it the active one
  if (activeClientMAC.isEmpty())
  {
    activeClientMAC = macAddress;
  }

  Serial.printf("Client registered: MAC=%s, IP=%s, Name=%s\n", 
                macAddress.c_str(), ipAddress.c_str(), newClient.friendlyName.c_str());
  return true;
}

// Function to update client last seen timestamp
void updateClientActivity(const String &macAddress)
{
  if (registeredClients.find(macAddress) != registeredClients.end())
  {
    registeredClients[macAddress].lastSeen = millis();
    registeredClients[macAddress].isActive = true;
  }
}

// Function to check for inactive clients (timeout: 30 seconds)
void checkClientActivity()
{
  unsigned long currentTime = millis();
  const unsigned long TIMEOUT_MS = 30000; // 30 seconds

  for (auto &client : registeredClients)
  {
    if (currentTime - client.second.lastSeen > TIMEOUT_MS)
    {
      if (client.second.isActive)
      {
        client.second.isActive = false;
        clientSensorData[client.first].isOnline = false;
        Serial.printf("Client %s marked as inactive\n", client.first.c_str());
        
        // If the inactive client was the active one, switch to another active client
        if (activeClientMAC == client.first)
        {
          switchToNextActiveClient();
        }
      }
    }
  }
}

// Function to switch to the next active client
void switchToNextActiveClient()
{
  for (const auto &client : registeredClients)
  {
    if (client.second.isActive && client.first != activeClientMAC)
    {
      activeClientMAC = client.first;
      Serial.printf("Switched active client to: %s\n", activeClientMAC.c_str());
      
      // Broadcast the new active client's data
      if (clientSensorData.find(activeClientMAC) != clientSensorData.end())
      {
        broadcastSensorData(clientSensorData[activeClientMAC], activeClientMAC);
      }
      return;
    }
  }
  
  // No active clients found
  activeClientMAC = "";
  Serial.println("No active clients available");
}

// Thread-safe data getter for specific client
SensorData getSensorData(const String &macAddress = "")
{
  String targetMAC = macAddress.isEmpty() ? activeClientMAC : macAddress;
  SensorData getData;
  
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    if (!targetMAC.isEmpty() && clientSensorData.find(targetMAC) != clientSensorData.end())
    {
      getData = clientSensorData[targetMAC];
    }
    xSemaphoreGive(dataMutex);
  }
  return getData;
}

// Thread-safe data setter for specific client
void setSensorData(const JsonDocument &doc, const String &macAddress)
{
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
  {
    // Ensure client data exists
    if (clientSensorData.find(macAddress) == clientSensorData.end())
    {
      clientSensorData[macAddress] = SensorData();
    }
    
    SensorData &data = clientSensorData[macAddress];
    
    // Update data fields
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
    data.isOnline = true;
    
    xSemaphoreGive(dataMutex);
  }
}

// Create JSON from sensor data with client information
String createSensorJSON(const SensorData &data, const String &macAddress = "")
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
  doc["lastUpdate"] = data.lastUpdate;
  doc["isOnline"] = data.isOnline;
  
  // Add client information if MAC address is provided
  if (!macAddress.isEmpty() && registeredClients.find(macAddress) != registeredClients.end())
  {
    doc["clientMAC"] = macAddress;
    doc["clientName"] = registeredClients[macAddress].friendlyName;
    doc["clientIP"] = registeredClients[macAddress].ipAddress;
  }

  String output;
  serializeJson(doc, output);
  return output;
}

// Create JSON list of all clients
String createClientListJSON()
{
  JsonDocument doc;
  JsonArray clients = doc["clients"].to<JsonArray>();
  
  for (const auto &client : registeredClients)
  {
    JsonObject clientObj = clients.add<JsonObject>();
    clientObj["mac"] = client.first;
    clientObj["name"] = client.second.friendlyName;
    clientObj["ip"] = client.second.ipAddress;
    clientObj["active"] = client.second.isActive;
    clientObj["lastSeen"] = client.second.lastSeen;
    clientObj["isActive"] = (client.first == activeClientMAC);
    
    // Add sensor data status
    if (clientSensorData.find(client.first) != clientSensorData.end())
    {
      clientObj["online"] = clientSensorData[client.first].isOnline;
      clientObj["lastUpdate"] = clientSensorData[client.first].lastUpdate;
    }
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

// Broadcast data to all clients
void broadcastSensorData(const SensorData &data, const String &macAddress = "")
{
  String jsonData = createSensorJSON(data, macAddress);

  // Broadcast to WebSocket clients
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

  // Broadcast to SSE clients
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
    sseHandler.send("clientMAC", macAddress.c_str());
    if (registeredClients.find(macAddress) != registeredClients.end())
    {
      sseHandler.send("clientName", registeredClients[macAddress].friendlyName.c_str());
    }
  }
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
  
  // HTTP Dashboard endpoint
  server.on("/", HTTP_GET, [](PsychicRequest *req) -> esp_err_t
            {
    String htmlPage = R"rawliteral(
        <!DOCTYPE html>
        <html lang="en">
        <head>
          <meta charset="UTF-8">
          <meta name="viewport" content="width=device-width, initial-scale=1.0">
          <title>Multi-Vehicle Tracking Dashboard</title>
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
              margin-right: 1rem;
            }

            .client-selector {
              margin-top: 1rem;
            }

            .client-selector select {
              padding: 0.5rem 1rem;
              border: none;
              border-radius: 4px;
              background: rgba(255, 255, 255, 0.9);
              color: #020300;
              font-size: 0.9rem;
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

            .client-status {
              padding: 0.2rem 0.5rem;
              border-radius: 12px;
              font-size: 0.8rem;
              font-weight: 500;
            }

            .client-status.online {
              background: #d4edda;
              color: #155724;
            }

            .client-status.offline {
              background: #f8d7da;
              color: #721c24;
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
              <h1>Multi-Vehicle Tracking Dashboard</h1>
              <div class="status" id="connectionStatus">Connecting...</div>
              <div class="client-selector">
                <label for="clientSelect" style="color: white; margin-right: 0.5rem;">Active Client:</label>
                <select id="clientSelect">
                  <option value="">No clients available</option>
                </select>
              </div>
            </div>
            <div class="separator"></div>
            <div class="grid">
              <div class="card">
                <h2><span class="icon">🖥️</span>Client Information</h2>
                <div class="data-row">
                  <span class="label">Client Name:</span>
                  <span class="value" id="clientName">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">MAC Address:</span>
                  <span class="value" id="clientMAC">N/A</span>
                </div>
                <div class="data-row">
                  <span class="label">Status:</span>
                  <span class="value client-status" id="clientStatus">N/A</span>
                </div>
              </div>
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
            const clientSelect = document.getElementById('clientSelect');
            let wsConnected = false, sseConnected = false;
            let clients = {};
            
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

            function updateClientSelect(clientsData) {
              clientSelect.innerHTML = '';
              if (clientsData.clients && clientsData.clients.length > 0) {
                clientsData.clients.forEach(client => {
                  const option = document.createElement('option');
                  option.value = client.mac;
                  option.textContent = client.name + (client.active ? ' (Online)' : ' (Offline)');
                  if (client.isActive) option.selected = true;
                  clientSelect.appendChild(option);
                });
              } else {
                const option = document.createElement('option');
                option.value = '';
                option.textContent = 'No clients available';
                clientSelect.appendChild(option);
              }
            }

            // Client selection change handler
            clientSelect.addEventListener('change', function() {
              if (this.value) {
                fetch('/switch-client', {
                  method: 'POST',
                  headers: { 'Content-Type': 'application/json' },
                  body: JSON.stringify({ mac: this.value })
                });
              }
            });

            const ws = new WebSocket(`ws://${location.host}/ws`);
            ws.onopen = () => { wsConnected = true; updateStatus(); };
            ws.onclose = () => { wsConnected = false; updateStatus(); };
            ws.onmessage = e => {
              try {
                const d = JSON.parse(e.data);
                Object.keys(d).forEach(key => {
                  if (key === 'clientMAC') {
                    updateField('clientMAC', d[key]);
                  } else if (key === 'clientName') {
                    updateField('clientName', d[key]);
                  } else if (key === 'isOnline') {
                    const statusEl = document.getElementById('clientStatus');
                    if (statusEl) {
                      statusEl.textContent = d[key] ? 'Online' : 'Offline';
                      statusEl.className = 'value client-status ' + (d[key] ? 'online' : 'offline');
                    }
                  } else {
                    updateField(key, d[key]);
                  }
                });
              } catch (err) { console.error('WS JSON parse:', err); }
            };

            const sse = new EventSource('/events');
            sse.onopen = () => { sseConnected = true; updateStatus(); };
            sse.onerror = () => { sseConnected = false; updateStatus(); };
            
            // SSE event listeners for sensor data
            ['driver', 'uid', 'time', 'long', 'lat', 'alt', 'spd', 'sat', 'hdop', 'brand', 'model', 'pltNum'].forEach(field => {
              sse.addEventListener(field, e => updateField(field, e.data));
            });
            
            // SSE event listeners for client data
            sse.addEventListener('clientMAC', e => updateField('clientMAC', e.data));
            sse.addEventListener('clientName', e => updateField('clientName', e.data));

            // Fetch client list periodically
            setInterval(() => {
              fetch('/clients')
                .then(response => response.json())
                .then(data => updateClientSelect(data))
                .catch(err => console.error('Error fetching clients:', err));
            }, 5000);

            updateStatus();
          </script>
        </body>
        </html>
      )rawliteral";
    static String htmlCopy;
    htmlCopy = htmlPage;
    return req->reply(200, "text/html", htmlCopy.c_str()); });

  // < ------ Continue from here ------ >
  // WebSocket endpoint 
  server.on("/ws", HTTP_GET, &wsHandler);
  wsHandler.onOpen([](PsychicWebSocketClient *client)
                   {
    if (wsSockets.size() >= MAX_WS_CLIENTS)
    {
      client->sendMessage("Max
