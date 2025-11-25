#include <Arduino.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

// MQTT Configuration - UPDATE THESE WITH YOUR HIVEMQ CREDENTIALS
const char* mqtt_server = "broker.hivemq.com";  // Or your HiveMQ cluster URL
const int mqtt_port = 1883;                       // 1883 for non-secure, 8883 for TLS
const char* mqtt_user = "";                       // Your HiveMQ username (leave empty for public broker)
const char* mqtt_password = "";                   // Your HiveMQ password (leave empty for public broker)
const char* mqtt_client_id = "ESP32CAM_Client";   // Unique client ID

// MQTT Topics
const char* topic_frame = "camera/esp32cam/frame";     // Image data
const char* topic_status = "camera/esp32cam/status";   // Status messages
const char* topic_control = "camera/esp32cam/control"; // Control commands

WiFiClient espClient;
PubSubClient mqttClient(espClient);
bool isMqttReady = false;
unsigned long lastStreamTime = 0;
const unsigned long STREAM_INTERVAL = 1000; // 1 second between frames
int consecutiveFailures = 0;
const int MAX_CONSECUTIVE_FAILURES = 5;
bool streamingActive = true; // Control via MQTT

// Connection monitoring
unsigned long lastMqttCheck = 0;
const unsigned long MQTT_CHECK_INTERVAL = 5000; // Check connection every 5 seconds
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000; // Try reconnecting every 5 seconds
int mqttReconnectAttempts = 0;

#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
int counter = 0;
int Active = 0;
int lastActive = -1; // Track Active state changes


// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

camera_fb_t* capturePhoto()
{
  camera_fb_t *fb = esp_camera_fb_get();
  
  if (!fb)
  {
    // Retry once
    delay(50);
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("[ERROR] Camera capture failed");
      return nullptr;
    }
  }
  
  // Validate image
  if (fb->len == 0)
  {
    Serial.println("[ERROR] Empty capture");
    esp_camera_fb_return(fb);
    return nullptr;
  }
  
  // Check if image is too large for MQTT buffer
  if (fb->len > 15000)
  {
    Serial.printf("[ERROR] Image too large: %d bytes (max 15KB)\n", fb->len);
    esp_camera_fb_return(fb);
    return nullptr;
  }
  
  Serial.printf("[Capture] %d bytes\n", fb->len);
  return fb;
}



void setupCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // MQTT allows better quality - no Base64 overhead!
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_HVGA; // 480x320 - fits in 16KB MQTT buffer
    config.jpeg_quality = 15; // Lower = better quality (10-63)
    config.fb_count = 2; // Double buffering
  }
  else
  {
    config.frame_size = FRAMESIZE_QVGA; // 320x240 without PSRAM
    config.jpeg_quality = 18;
    config.fb_count = 1;
  }
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("[ERROR] Camera init failed with error 0x%x\n", err);
    Serial.println("[Camera] Retrying in 2 seconds...");
    delay(2000);
    
    // Retry once before restarting
    err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
      Serial.println("[ERROR] Camera init failed twice. Restarting...");
      delay(1000);
      ESP.restart();
    }
  }
  
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL)
  {
    // Fine-tune sensor settings for optimal livestreaming
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_sharpness(s, 0);      // -2 to 2
    s->set_whitebal(s, 1);       // Enable auto white balance
    s->set_awb_gain(s, 1);       // Enable auto white balance gain
    s->set_wb_mode(s, 0);        // 0 = Auto, 1 = Sunny, 2 = Cloudy, 3 = Office, 4 = Home
    s->set_exposure_ctrl(s, 1);  // Enable auto exposure
    s->set_aec2(s, 1);           // Enable AEC sensor
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // Enable auto gain
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)0); // 0 to 6
    s->set_bpc(s, 0);            // Black pixel correction
    s->set_wpc(s, 1);            // White pixel correction
    s->set_raw_gma(s, 1);        // Enable gamma correction
    s->set_lenc(s, 1);           // Enable lens correction
    s->set_hmirror(s, 0);        // 0 = disable, 1 = enable
    s->set_vflip(s, 0);          // 0 = disable, 1 = enable
    s->set_dcw(s, 1);            // Enable downsize
    s->set_colorbar(s, 0);       // Disable color bar test pattern
    
    Serial.println("[Camera] Initialized with optimized settings for livestream");
  }
  else
  {
    Serial.println("[ERROR] Failed to get camera sensor");
  }
}
// MQTT callback for incoming messages (control commands)
void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  String message = "";
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  
  Serial.printf("[MQTT] Message on %s: %s\n", topic, message.c_str());
  
  // Handle control commands
  if (String(topic) == topic_control)
  {
    if (message == "start")
    {
      streamingActive = true;
      Serial.println("[Control] Streaming started");
    }
    else if (message == "stop")
    {
      streamingActive = false;
      Serial.println("[Control] Streaming stopped");
    }
  }
}

void connectMQTT()
{
  if (mqttClient.connected()) 
  {
    mqttReconnectAttempts = 0;
    return;
  }
  
  // Throttle reconnection attempts
  unsigned long now = millis();
  if (now - lastReconnectAttempt < RECONNECT_INTERVAL)
  {
    return;
  }
  lastReconnectAttempt = now;
  
  mqttReconnectAttempts++;
  Serial.printf("[MQTT] Connecting (attempt %d)...\n", mqttReconnectAttempts);
  
  // Generate unique client ID to avoid conflicts
  String clientId = String(mqtt_client_id) + "_" + String(ESP.getEfuseMac(), HEX);
  
  // Attempt to connect with last will
  bool connected = false;
  if (strlen(mqtt_user) > 0)
  {
    // Connect with credentials and last will
    connected = mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password,
                                   topic_status, 1, true, "ESP32-CAM Offline");
  }
  else
  {
    // Connect without credentials but with last will
    connected = mqttClient.connect(clientId.c_str(), topic_status, 1, true, "ESP32-CAM Offline");
  }
  
  if (connected)
  {
    isMqttReady = true;
    mqttReconnectAttempts = 0;
    Serial.println("[MQTT] Connected successfully!");
    Serial.printf("[MQTT] Server: %s:%d\n", mqtt_server, mqtt_port);
    Serial.printf("[MQTT] Client ID: %s\n", clientId.c_str());
    
    // Subscribe to control topic
    if (mqttClient.subscribe(topic_control))
    {
      Serial.printf("[MQTT] Subscribed to: %s\n", topic_control);
    }
    else
    {
      Serial.println("[MQTT] WARNING: Failed to subscribe");
    }
    
    // Publish status with retain flag
    mqttClient.publish(topic_status, "ESP32-CAM Online", true);
    
    consecutiveFailures = 0;
  }
  else
  {
    isMqttReady = false;
    int state = mqttClient.state();
    Serial.printf("[MQTT] Connection failed, rc=%d", state);
    
    // Decode error
    switch(state)
    {
      case -4: Serial.println(" (Connection timeout)"); break;
      case -3: Serial.println(" (Connection lost)"); break;
      case -2: Serial.println(" (Connect failed)"); break;
      case -1: Serial.println(" (Disconnected)"); break;
      case 1: Serial.println(" (Bad protocol)"); break;
      case 2: Serial.println(" (Bad client ID)"); break;
      case 3: Serial.println(" (Unavailable)"); break;
      case 4: Serial.println(" (Bad credentials)"); break;
      case 5: Serial.println(" (Unauthorized)"); break;
      default: Serial.println();
    }
    
    // Reset WiFi if too many failed attempts
    if (mqttReconnectAttempts > 10)
    {
      Serial.println("[MQTT] Too many failed attempts, restarting WiFi...");
      WiFi.disconnect();
      delay(1000);
      WiFi.reconnect();
      mqttReconnectAttempts = 0;
    }
  }
}

void publishFrame()
{
  if (!isMqttReady || !mqttClient.connected())
  {
    consecutiveFailures++;
    return;
  }
  
  // Capture photo
  camera_fb_t *fb = capturePhoto();
  
  if (!fb)
  {
    consecutiveFailures++;
    if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES)
    {
      Serial.println("[ERROR] Too many failures, restarting camera...");
      esp_camera_deinit();
      delay(1000);
      setupCamera();
      consecutiveFailures = 0;
    }
    return;
  }
  
  // Publish raw JPEG bytes directly to MQTT (no Base64!)
  // Use QoS 0 for faster streaming and less broker load
  bool published = mqttClient.publish(topic_frame, fb->buf, fb->len, false);
  
  if (published)
  {
    Serial.printf("[✓] Frame #%d | Size: %d bytes | Heap: %d\n", 
                  counter, fb->len, ESP.getFreeHeap());
    counter++;
    consecutiveFailures = 0;
  }
  else
  {
    Serial.println("[✗] MQTT publish failed - connection issue");
    consecutiveFailures++;
    isMqttReady = false;
    
    // Force reconnection on next loop
    mqttClient.disconnect();
  }
  
  // Return frame buffer to free memory
  esp_camera_fb_return(fb);
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=================================");
  Serial.println("ESP32-CAM MQTT Streamer v1.0");
  Serial.println("Raw JPEG Binary Streaming");
  Serial.println("=================================");
  Serial.printf("[System] CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("[System] Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("[System] PSRAM: %s\n", psramFound() ? "Yes" : "No");

  // WiFi Setup
  Serial.println("\n[WiFi] Connecting...");
  WiFi.setSleep(false); // Disable WiFi sleep for consistent streaming
  WiFiManager wm;
  wm.setConfigPortalTimeout(180); // 3 minute timeout for AP mode
  
  bool res = wm.autoConnect("Cam1_Access_Point", "password123");
  if (!res)
  {
    Serial.println("[WiFi] Failed to connect. Restarting...");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("[WiFi] ✓ Connected");
  Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
  Serial.println("[WiFi] Signal: " + String(WiFi.RSSI()) + " dBm");

  Serial.setDebugOutput(false); // Reduce verbose output
  
  // Camera Setup
  Serial.println("\n[Camera] Setting up...");
  setupCamera();
  
  // MQTT Setup
  Serial.println("\n[MQTT] Setting up...");
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
  // CRITICAL: Set buffer size BEFORE connecting
  bool bufferSet = mqttClient.setBufferSize(16384); // 16KB buffer
  if (bufferSet) {
    Serial.println("[MQTT] Buffer size set to 16KB");
  } else {
    Serial.println("[MQTT] WARNING: Failed to set buffer size");
  }
  
  // Set keepalive interval (default is 15 seconds)
  mqttClient.setKeepAlive(30); // Increase to 30 seconds for more stability
  Serial.println("[MQTT] Keepalive set to 30 seconds");
  
  // Set socket timeout
  mqttClient.setSocketTimeout(15); // 15 seconds timeout
  Serial.println("[MQTT] Socket timeout set to 15 seconds");
  
  connectMQTT();
  
  Serial.println("\n=================================");
  Serial.println("Setup Complete - Ready to Stream");
  Serial.println("=================================");
  Serial.printf("[Memory] Free heap: %d bytes\n\n", ESP.getFreeHeap());
}

void loop()
{
  unsigned long currentTime = millis();
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[WiFi] Connection lost! Reconnecting...");
    isMqttReady = false;
    WiFi.reconnect();
    delay(5000);
    return;
  }
  
  // Ensure MQTT is connected
  if (!mqttClient.connected())
  {
    isMqttReady = false;
    connectMQTT();
    return; // Skip frame publishing this loop
  }
  
  // Process MQTT messages (CRITICAL - must be called regularly)
  mqttClient.loop();
  
  // Periodic connection check
  if (currentTime - lastMqttCheck >= MQTT_CHECK_INTERVAL)
  {
    lastMqttCheck = currentTime;
    
    if (mqttClient.connected())
    {
      // Send keepalive ping
      if (!isMqttReady)
      {
        isMqttReady = true;
        Serial.println("[MQTT] Connection restored");
      }
    }
    else
    {
      if (isMqttReady)
      {
        isMqttReady = false;
        Serial.println("[MQTT] Connection lost detected");
      }
    }
  }
  
  // Publish frame if streaming is active and interval elapsed
  if (isMqttReady && streamingActive && (currentTime - lastStreamTime >= STREAM_INTERVAL))
  {
    publishFrame();
    lastStreamTime = currentTime;
  }
  
  yield(); // Allow background tasks
}