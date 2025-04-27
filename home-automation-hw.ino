#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "wifi-name";
const char* password = "wifi-password";

// WebSocket server details
const char* websocket_server = "server-url"; // no https://
const int websocket_port = 443; // 443 for SSL
const char* websocket_path = "/esp32";

// Device information
String deviceId = "esp32_living_room"; // Unique ID for this device

// Define available pins on the ESP32
const int pinNumbers[] = { 4, 5, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33, 34, 35, 36, 39 };
const int numPins = sizeof(pinNumbers) / sizeof(pinNumbers[0]);

// Add to your global variables
const int ldrPin = 34;  // Use an ADC pin like 34, 35, etc.
unsigned long lastLightSensorUpdate = 0;
const long lightSensorInterval = 5000;  // Read every 5 seconds

// Pin Load tracking - To keep track of connected loads
struct PinLoad {
  int pinId;
  String loadType;  // e.g., "LED", "motor", "sensor"
  String state;     // state of the load ("HIGH"/"LOW")
};

PinLoad pinLoads[numPins];

// Create WebSocket client
WebSocketsClient webSocket;

void setup() {
  Serial.begin(115200);
  delay(1000);   
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());
  
  // Setup WebSocket connection
  webSocket.beginSSL(websocket_server, websocket_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);  // Try to reconnect every 5 seconds if connection is lost
  
  // Set all pins as input or output based on usage
  for (int i = 0; i < numPins; i++) {
    pinMode(pinNumbers[i], INPUT);  // Defaulting all to input (can be changed as needed)
    
    // Initialize PinLoad data structure
    pinLoads[i].pinId = pinNumbers[i];
    pinLoads[i].state = "LOW"; // Default state is off (LOW)
    pinLoads[i].loadType = ""; // No load connected initially
  }

   // Configure LDR pin as input
  
  
  // Example: Assign loads to some pins
  assignLoadToPin(5, "LED");  // Pin 5 connected to an LED
  assignLoadToPin(23, "LED");  // Pin 23 connected to an LED
  assignLoadToPin(12, "Sensor"); // Pin 12 connected to a sensor
  // Assign the LDR to a pin
//  assignLoadToPin(ldrPin, "LightSensor");
  pinMode(ldrPin, INPUT);
}

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected!");
      break;
      
    case WStype_CONNECTED:
      Serial.println("WebSocket connected to server: " + String(websocket_server));
      
      // Send device registration after successful connection
      sendDeviceInfo();
      break;
      
    case WStype_TEXT:
      // Handle incoming text message
      handleIncomingMessage(payload, length);
      break;
      
    case WStype_BIN:
      Serial.println("Received binary data");
      break;
      
    case WStype_ERROR:
      Serial.println("WebSocket error");
      break;
      
    case WStype_PING:
      Serial.println("Received ping");
      break;
      
    case WStype_PONG:
      Serial.println("Received pong");
      break;
  }
}

void sendDeviceInfo() {
  // Create JSON message for device registration with pin information and connected loads
  StaticJsonDocument<1024> doc;
  doc["type"] = "device_info";
  doc["id"] = deviceId;
  doc["capabilities"] = "light,temperature";
  
  // Add pin information and load assignments
  JsonArray pinsArray = doc.createNestedArray("pins");
  for (int i = 0; i < numPins; i++) {
    JsonObject pin = pinsArray.createNestedObject();
    pin["id"] = pinLoads[i].pinId;
    pin["mode"] = "input";  // Can be 'input', 'output', etc.
    pin["state"] = pinLoads[i].state;  // State of the pin ("HIGH"/"LOW")
    pin["loadType"] = pinLoads[i].loadType;  // Load connected to the pin
    pin["value"] = pinLoads[i].state == "HIGH" ? true : false;  // Add value field for UI consistency
  }
  
  // Serialize JSON to string
  String message;
  serializeJson(doc, message);
  
  // Send message
  webSocket.sendTXT(message);
  Serial.println("Device info sent: " + message);
}

void handleIncomingMessage(uint8_t* payload, size_t length) {
  String message = String((char*)payload);
  Serial.println("Received: " + message);
  
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("Failed to parse message: " + String(error.c_str()));
    return;
  }
  
  String type = doc["type"];

  if (type == "command") {
    String command = doc["command"];
    
    if (command == "toggle_pin") {
      int pinId = doc["pinId"];
      String state = doc["state"]; // state will be "LOW" or "HIGH"
      
      // Find the pinLoad with the matching pinId
      int pinIndex = -1;
      for (int i = 0; i < numPins; i++) {
        if (pinLoads[i].pinId == pinId) {
          pinIndex = i;
          break;
        }
      }
      
      if (pinIndex >= 0) {
        // Set the physical pin state
        digitalWrite(pinId, state == "HIGH" ? HIGH : LOW);
        
        // Update our internal state tracking
        pinLoads[pinIndex].state = state;
        
        Serial.println("Setting pin " + String(pinId) + " to: " + state);
        
        // Send confirmation back to server
        sendPinStateUpdate(pinId, state);
      }
    }
  } 
  else if (type == "pin_state_update") {
    int pinId = doc["pinId"];
    String state = doc["state"];
      
    // Find the pinLoad with the matching pinId
    int pinIndex = -1;
    for (int i = 0; i < numPins; i++) {
      if (pinLoads[i].pinId == pinId) {
        pinIndex = i;
        break;
      }
    }
    
    if (pinIndex >= 0) {
      // Set the physical pin state
      digitalWrite(pinId, state == "HIGH" ? HIGH : LOW);
      
      // Update our internal state tracking
      pinLoads[pinIndex].state = state;
      
      Serial.println("Setting pin " + String(pinId) + " to: " + state);
    }
  }
}

void sendPinStateUpdate(int pinId, String state) {
  // Create JSON message for pin state update
  StaticJsonDocument<200> doc;
  doc["type"] = "pin_state_update";
  doc["id"] = deviceId;
  doc["pinId"] = pinId;
  doc["state"] = state;
  doc["value"] = (state == "HIGH"); // Add value field for UI consistency
  
  // Serialize JSON to string
  String message;
  serializeJson(doc, message);
  
  // Send message
  webSocket.sendTXT(message);
  Serial.println("Pin state update sent: " + message);
}

void assignLoadToPin(int pinId, String loadType) {
  for (int i = 0; i < numPins; i++) {
    if (pinLoads[i].pinId == pinId) {
      pinLoads[i].loadType = loadType;  // Assign load to pin
      pinMode(pinId, OUTPUT);  // Set pin mode as OUTPUT (or as needed for the load)
      digitalWrite(pinId, LOW); // Initialize to LOW
      pinLoads[i].state = "LOW"; // Set initial state
      break;
    }
  }
}

void loop() {
  // Handle WebSocket communication
  webSocket.loop();
  

  
  // Check WiFi connection and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    WiFi.reconnect();
    // Wait for reconnection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("WiFi reconnected");
  }

    // Send sensor data every 30 seconds (example)
  static unsigned long lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate > 30000) {
    lastSensorUpdate = millis();
    sendSensorData();
  }

   // Read light sensor periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastLightSensorUpdate >= lightSensorInterval) {
    lastLightSensorUpdate = currentMillis;
    readLightSensor();
  }
}

// Add this new function
void readLightSensor() {
  // Read analog value from LDR
  int rawValue = analogRead(ldrPin);

  Serial.println(rawValue);
  
  // ESP32 ADC has 12-bit resolution (0-4095)
  // Convert to percentage for easier understanding
  int lightPercentage = map(rawValue, 0, 4095, 0, 100);

  Serial.println(lightPercentage);
  
  // Create JSON message for sensor data
  StaticJsonDocument<200> doc;
  doc["type"] = "sensor_data";
  doc["id"] = deviceId;
  doc["sensor"] = "light";
  doc["value"] = lightPercentage;
  doc["raw"] = rawValue;
  
  // Serialize JSON to string
  String message;
  serializeJson(doc, message);
  
  // Send message
  webSocket.sendTXT(message);
  Serial.println("Light sensor data sent: " + message);
}

void sendSensorData() {
  // Example temperature reading (replace with actual sensor code)
  float temperature = 22.5; // Simulated temperature
  
  // Create JSON message for sensor data
  StaticJsonDocument<200> doc;
  doc["type"] = "sensor_data";
  doc["id"] = deviceId;
  doc["sensor"] = "temperature";
  doc["value"] = temperature;
  
  // Serialize JSON to string
  String message;
  serializeJson(doc, message);
  
  // Send message
  webSocket.sendTXT(message);
  Serial.println("Sensor data sent: " + message);
}
