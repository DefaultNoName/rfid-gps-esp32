#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <PsychicHttp.h>
#include <ArduinoJson.h>
#include <vector>
#include <unordered_map>

/*
// Station Mode - Static IP for Web Dashboard (dont' delete)
IPAddress sta_ip(192, 168, 120, 2); // Fixed IP for web dashboard
IPAddress sta_gw(192, 168, 120, 1); // ESP32-AP gateway
IPAddress sta_sn(255, 255, 255, 0); // Subnet mask
IPAddress dns(192, 168, 120, 1);    // DNS (same as gateway)
*/

// WiFi configuration
IPAddress ap_ip = IPAddress(192, 168, 120, 1); // IP Address
IPAddress ap_gw = IPAddress(192, 168, 120, 1); // Gateway
IPAddress ap_sn = IPAddress(255, 255, 255, 0); // Mask

// Default config
#define SSID "ESP32-Server"    // AP SSID
#define PASSWORD "01234567890" // AP Password
#define AP_CHANNEL 6           // Channel
#define AP_MAX_CONNECTIONS 8   // AP Clients
#define MAX_WS_CLIENTS 8       // WebSocket clients
#define MAX_SSE_CLIENTS 8      // SSE clients

// Server configuration
PsychicHttpServer server;
const unsigned int server_port = 80;
const unsigned int max_uri = 20;
const unsigned int max_sockets = 12;
const unsigned int linger_timeout = 3;
const unsigned int task_priority = 12;
const unsigned int stack_size = 25600;

// WebSocket and SSE handlers
PsychicWebSocketHandler ws_handler;
PsychicEventSource sse_handler;
std::vector<int> ws_sockets;
std::vector<int> sse_sockets;

// System States
enum SystemState
{
  INIT,
  WIFI_SETUP,
  WIFI_STABILIZING,
  SERVER_SETUP,
  SERVER_STARTING,
  RUNNING,
  ERROR_WIFI,
  ERROR_SERVER
};

SystemState current_state = INIT;
unsigned long state_timer = 0;
unsigned long health_check_timer = 0;
const unsigned long HEALTH_CHECK_INTERVAL = 10000; // 10 seconds
const unsigned long WIFI_STABILIZE_TIME = 2000;    // 2 seconds
const unsigned long SERVER_START_TIME = 1000;      // 1 second

// Sensor Data
struct SensorData
{
  String brand_car = "N/A";
  String model_car = "N/A";
  String plt_num_car = "N/A";
  String driver_rfid = "N/A";
  String uid_rfid = "N/A";
  String time_exit = "N/A";
  String time_entry = "N/A";
  String trip_duration = "N/A";
  String gps_time = "N/A";
  String long_gps = "N/A";
  String lat_gps = "N/A";
  String alt_gps = "N/A";
  String speed_gps = "N/A";
  String sat_gps = "N/A";
  String hdop_gps = "N/A";
} sensor_data;

// Driver mappings
std::unordered_map<std::string, std::string> driver_map = {
    {"5394B838", "Allan M."},
    {"63A70F02", "Paulino"},
    {"FB1D0F02", "Dave"},
    {"D9C5A998", "Poging Drayber"},
    {"9AC8BE24", "Grow A Garden Dela Cruz"},
    {"129791AB", "Mama mo"}};

String get_driver_name(const String &uid)
{
  std::string uid_std = uid.c_str();
  auto key_value = driver_map.find(uid_std);
  if (key_value != driver_map.end())
  {
    return String(key_value->second.c_str());
  }
  return "Unknown Driver";
}

void set_sensor_data(const JsonDocument &doc)
{
  sensor_data.brand_car = doc["brand"] | sensor_data.brand_car;
  sensor_data.model_car = doc["model"] | sensor_data.model_car;
  sensor_data.plt_num_car = doc["pltNum"] | sensor_data.plt_num_car;
  sensor_data.uid_rfid = doc["uid"] | sensor_data.uid_rfid;
  sensor_data.driver_rfid = get_driver_name(sensor_data.uid_rfid);
  sensor_data.time_exit = doc["time_EXIT"] | sensor_data.time_exit;
  sensor_data.time_entry = doc["time_ENTRY"] | sensor_data.time_entry;
  sensor_data.trip_duration = doc["trip_duration"] | sensor_data.trip_duration;
  sensor_data.gps_time = doc["GPS_time"] | sensor_data.gps_time;
  sensor_data.long_gps = doc["long"] | sensor_data.long_gps;
  sensor_data.lat_gps = doc["lat"] | sensor_data.lat_gps;
  sensor_data.alt_gps = doc["alt"] | sensor_data.alt_gps;
  sensor_data.speed_gps = doc["spd"] | sensor_data.speed_gps;
  sensor_data.sat_gps = doc["sat"] | sensor_data.sat_gps;
  sensor_data.hdop_gps = doc["hdop"] | sensor_data.hdop_gps;
}

String create_sensor_json(const SensorData &data)
{
  JsonDocument doc;
  doc["brand"] = data.brand_car;
  doc["model"] = data.model_car;
  doc["pltNum"] = data.plt_num_car;
  doc["driver"] = data.driver_rfid;
  doc["uid"] = data.uid_rfid;
  doc["time_EXIT"] = data.time_exit;
  doc["time_ENTRY"] = data.time_entry;
  doc["trip_duration"] = data.trip_duration;
  doc["GPS_time"] = data.gps_time;
  doc["long"] = data.long_gps;
  doc["lat"] = data.lat_gps;
  doc["alt"] = data.alt_gps;
  doc["spd"] = data.speed_gps;
  doc["sat"] = data.sat_gps;
  doc["hdop"] = data.hdop_gps;

  String output;
  serializeJson(doc, output);
  return output;
}

void broadcast_sensor_data(const SensorData &data)
{
  String json_data = create_sensor_json(data);

  if (ws_sockets.size() < MAX_WS_CLIENTS)
  {
    for (int sock : ws_sockets)
    {
      if (auto *client = ws_handler.getClient(sock))
      {
        client->sendMessage(json_data.c_str());
      }
    }
  }

  if (sse_sockets.size() < MAX_SSE_CLIENTS)
  {
    sse_handler.send("brand", data.brand_car.c_str());
    sse_handler.send("model", data.model_car.c_str());
    sse_handler.send("pltNum", data.plt_num_car.c_str());
    sse_handler.send("driver", data.driver_rfid.c_str());
    sse_handler.send("uid", data.uid_rfid.c_str());
    sse_handler.send("time_EXIT", data.time_exit.c_str());
    sse_handler.send("time_ENTRY", data.time_entry.c_str());
    sse_handler.send("trip_duration", data.trip_duration.c_str());
    sse_handler.send("GPS_time", data.gps_time.c_str());
    sse_handler.send("long", data.long_gps.c_str());
    sse_handler.send("lat", data.lat_gps.c_str());
    sse_handler.send("alt", data.alt_gps.c_str());
    sse_handler.send("spd", data.speed_gps.c_str());
    sse_handler.send("sat", data.sat_gps.c_str());
    sse_handler.send("hdop", data.hdop_gps.c_str());
  }
}

// Setup this ESP32 as an Access Point - Commented out for STA testing
bool setup_wifi_ap()
{
  Serial.println("Setting up WiFi Access Point...");

  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_OFF);
  delay(500);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.mode(WIFI_AP);
  delay(500);

  WiFi.softAPConfig(ap_ip, ap_gw, ap_sn);
  bool success = WiFi.softAP(SSID, PASSWORD, AP_CHANNEL, false, AP_MAX_CONNECTIONS);

  if (success)
  {
    esp_wifi_set_max_tx_power(78);
    Serial.printf("WiFi AP Created: %s\n", SSID);
    return true;
  }

  Serial.println("WiFi AP creation failed");
  return false;
}

bool verify_wifi_ap()
{
  IPAddress ap_ip = WiFi.softAPIP();
  if (ap_ip == IPAddress(0, 0, 0, 0))
  {
    Serial.println("WiFi AP IP invalid");
    return false;
  }

  Serial.printf("WiFi AP Ready - IP: %s\n", ap_ip.toString().c_str());
  return true;
}

/* Setup this ESP32 as a STATION
bool setup_wifi_ap()
{
  Serial.println("Initializing WiFi Station Mode...");
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_OFF);
  delay(500);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.mode(WIFI_STA);
  WiFi.config(sta_ip, sta_gw, sta_sn, dns);
  delay(500);
  // Connect to the ESP32_AP
  WiFi.begin(AP_SSID, AP_PASSWORD);
  Serial.print("Connecting to WiFi");

  static int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30)
  {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println();
    Serial.println("WiFi connected successfully!");
    Serial.printf("  SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("  Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("  DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
    return true;
  }
  else
  {
    Serial.println("WiFi AP creation failed");
    return false;
  }
}
*/

/* Verify STATION
bool verify_wifi_sta()
{
  IPAddress sta_ip = WiFi.localIP();
  if (sta_ip == IPAddress(0, 0, 0, 0))
  {
    Serial.println("WiFi STA IP Invalid");
    return false;
  }
  Serial.printf("WiFi STA Ready - IP: %s\n", sta_ip.toString().c_str());
  return true;
}*/

bool setup_http_server()
{
  Serial.println("Setting up HTTP Server...");

  // Configure server
  server.config.max_uri_handlers = max_uri;
  server.config.max_open_sockets = max_sockets;
  server.config.server_port = server_port;
  server.config.linger_timeout = linger_timeout;
  server.config.task_priority = task_priority;
  server.config.stack_size = stack_size;
  server.config.enable_so_linger = true;
  // Start server
  esp_err_t result = server.listen(server_port);

  if (result != ESP_OK)
  {
    Serial.printf("CRITICAL: HTTP server failed to start - Error: %d\n", result);
    return false;
  }

  /* Commented out for STA Testing
  Serial.printf("SUCCESS: HTTP Server is listening! Server accessible at: http://%s:%d\n",
                WiFi.softAPIP().toString().c_str(), server_port);
  */

  // Station Mode
  Serial.printf("SUCCESS: HTTP Server is listening! Server accessible at: http://%s:%d\n",
                WiFi.localIP().toString().c_str(), server_port);

  // Configure all endpoints
  Serial.println("Configuring Server Endpoints...");

  // Dashboard endpoint
  server.on("/", HTTP_GET, [](PsychicRequest *req) -> esp_err_t
            {
    static String html_page = R"rawliteral(
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
                  <div class="data-row">
                    <span class="label">Time Out:</span>
                    <span class="value" id="time_EXIT">N/A</span>
                  </div>
                  <div class="data-row">
                    <span class="label">Time In:</span>
                    <span class="value" id="time_ENTRY">N/A</span>
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
                    <span class="label">Time:</span>
                    <span class="value" id="GPS_time">N/A</span>
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
              ['driver', 'uid', 'time_EXIT', 'time_ENTRY', 'trip_duration', 'GPS_time', 'long', 'lat', 'alt', 'spd', 'sat', 'hdop'].forEach(field => {
                sse.addEventListener(field, e => updateField(field, e.data));
              });
              updateStatus();
            </script>
          </body>
          </html>
      )rawliteral";
    return req->reply(200, "text/html", html_page.c_str()); });

  // WebSocket endpoint
  server.on("/ws", HTTP_GET, &ws_handler);
  ws_handler.onOpen([](PsychicWebSocketClient *client)
                    {
    if (ws_sockets.size() >= MAX_WS_CLIENTS) {
      client->sendMessage("Max Clients Reached");
      client->close();
      return;
    }
    int sock = client->socket();
    ws_sockets.push_back(sock);
    Serial.printf("WebSocket client connected: %d\n", sock);

    String json_data = create_sensor_json(sensor_data);
    client->sendMessage(json_data.c_str()); });

  ws_handler.onFrame([](PsychicWebSocketRequest *request, httpd_ws_frame *frame) -> int
                     {
    String msg = String((const char *)frame->payload, frame->len);
    Serial.println("WS Received: " + msg);
    request->client()->sendMessage(msg.c_str());
    return 0; });

  ws_handler.onClose([](PsychicWebSocketClient *client)
                     {
    int sock = client->socket();
    ws_sockets.erase(std::remove(ws_sockets.begin(), ws_sockets.end(), sock), ws_sockets.end());
    Serial.printf("WebSocket client disconnected: %d\n", sock); });

  // SSE endpoint
  server.on("/events", HTTP_GET, &sse_handler);
  sse_handler.onOpen([](PsychicEventSourceClient *client)
                     {
    if (sse_sockets.size() >= MAX_SSE_CLIENTS) {
      client->send("Max Clients Reached", "error");
      client->close();
      return;
    }
    int sock = client->socket();
    sse_sockets.push_back(sock);
    Serial.println("SSE client connected");

    client->send("brand", sensor_data.brand_car.c_str());
    client->send("model", sensor_data.model_car.c_str());
    client->send("pltNum", sensor_data.plt_num_car.c_str());
    client->send("driver", sensor_data.driver_rfid.c_str());
    client->send("uid", sensor_data.uid_rfid.c_str());
    client->send("time_EXIT", sensor_data.time_exit.c_str());
    client->send("time_ENTRY", sensor_data.time_entry.c_str());
    client->send("trip_duration", sensor_data.trip_duration.c_str());
    client->send("GPS_time", sensor_data.gps_time.c_str());
    client->send("long", sensor_data.long_gps.c_str());
    client->send("lat", sensor_data.lat_gps.c_str());
    client->send("alt", sensor_data.alt_gps.c_str());
    client->send("spd", sensor_data.speed_gps.c_str());
    client->send("sat", sensor_data.sat_gps.c_str());
    client->send("hdop", sensor_data.hdop_gps.c_str()); });

  sse_handler.onClose([](PsychicEventSourceClient *client)
                      {
    int sock = client->socket();
    sse_sockets.erase(std::remove(sse_sockets.begin(), sse_sockets.end(), sock), sse_sockets.end());
    Serial.println("SSE client disconnected"); });

  // Data upload endpoint
  server.on("/upload", HTTP_POST, [](PsychicRequest *req) -> esp_err_t
            {
    String body = req->body();
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      Serial.printf("JSON parse error: %s\n", error.c_str());
      return req->reply(400, "text/plain", "Bad JSON");
    }

    set_sensor_data(doc);
    broadcast_sensor_data(sensor_data);
    Serial.println("Data updated and broadcasted");
    return req->reply(200, "text/plain", "OK"); });

  return true;
}

bool start_http_server()
{
  esp_err_t result = server.listen(server_port);
  if (result != ESP_OK)
  {
    Serial.printf("HTTP server start failed: %d\n", result);
    return false;
  }

  Serial.printf("HTTP Server started at http://%s:%d\n",
                WiFi.softAPIP().toString().c_str(), server_port);
  return true;
}

void print_health_status()
{
  Serial.printf("Health Check - AP Clients: %d | WS: %d | SSE: %d | Free Heap: %d\n",
                WiFi.softAPgetStationNum(), ws_sockets.size(), sse_sockets.size(), ESP.getFreeHeap());
}

void setup()
{
  setCpuFrequencyMhz(240);
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== RFID+GPS TRACKING SYSTEM ===");

  state_timer = millis();
  health_check_timer = millis();
}

void loop()
{
  unsigned long current_time = millis();

  switch (current_state)
  {
  case INIT:
    Serial.println("State: INIT");
    current_state = WIFI_SETUP;
    state_timer = current_time;
    break;

  case WIFI_SETUP:
    Serial.println("State: WIFI_SETUP");
    if (setup_wifi_ap())
    {
      current_state = WIFI_STABILIZING;
      state_timer = current_time;
    }
    else
    {
      current_state = ERROR_WIFI;
      state_timer = current_time;
    }
    break;

  case WIFI_STABILIZING:
    if (current_time - state_timer >= WIFI_STABILIZE_TIME)
    {
      if (verify_wifi_ap())
      {
        Serial.println("State: WIFI_STABILIZING -> SERVER_SETUP");
        current_state = SERVER_SETUP;
        state_timer = current_time;
      }
      else
      {
        current_state = ERROR_WIFI;
        state_timer = current_time;
      }
      /*
      if (verify_wifi_sta())
      {
        Serial.println("State: WIFI_STABILIZING -> SERVER_SETUP");
        current_state = SERVER_SETUP;
        state_timer = current_time;
      }
      else
      {
        current_state = ERROR_WIFI;
        state_timer = current_time;
      }*/
    }
    break;

  case SERVER_SETUP:
    Serial.println("State: SERVER_SETUP");
    if (setup_http_server())
    {
      current_state = SERVER_STARTING;
      state_timer = current_time;
    }
    else
    {
      current_state = ERROR_SERVER;
      state_timer = current_time;
    }
    break;

  case SERVER_STARTING:
    if (current_time - state_timer >= SERVER_START_TIME)
    {
      if (start_http_server())
      {
        Serial.println("State: SERVER_STARTING -> RUNNING");
        current_state = RUNNING;
        health_check_timer = current_time;
      }
      else
      {
        current_state = ERROR_SERVER;
        state_timer = current_time;
      }
    }
    break;

  case RUNNING:
    // Periodic health checks
    if (current_time - health_check_timer >= HEALTH_CHECK_INTERVAL)
    {
      print_health_status();
      health_check_timer = current_time;

      /* Verify system health
      if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0))
      {
        Serial.println("WiFi AP failed - transitioning to error state");
        current_state = ERROR_WIFI;
        state_timer = current_time;
      }*/
      if (WiFi.localIP() == IPAddress(0, 0, 0, 0))
      {
        Serial.println("WiFi STA failed - transitioning to error state");
        current_state = ERROR_WIFI;
        state_timer = current_time;
      }
    }
    break;

  case ERROR_WIFI:
    Serial.println("State: ERROR_WIFI - Attempting recovery in 5 seconds");
    delay(5000);
    current_state = WIFI_SETUP;
    state_timer = current_time;
    break;

  case ERROR_SERVER:
    Serial.println("State: ERROR_SERVER - Attempting recovery in 5 seconds");
    delay(5000);
    current_state = SERVER_SETUP;
    state_timer = current_time;
    break;
  }

  delay(10); // Small delay to prevent tight loop
}
