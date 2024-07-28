#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32) || defined(ARDUINO_ARCH_RP2040)
#include <WiFi.h>
#endif

#include "SinricPro.h"
#include "SinricProSwitch.h"

const char* ssid = "";
const char* password = "";

unsigned long previousMillis = 0;
unsigned long interval = 30000;
int reconnectAttempts = 0;
const unsigned int maxReconnectAttempts = 10;

unsigned long lastHeartbeatMills = 0;
const unsigned long noHeartbeatResetIntervalMills = 60000 * 15;  /* If there's no heart-beat ping-pong for 15 mins. reset ESP! */

const String SWITCH_ID = "";
const String APP_KEY = "";
const String APP_SECRET = "";

void setup() {
  Serial.begin(115200);
  connectToWiFi();
  setupSinricPro();
}

bool onPowerState(const String& deviceId, bool& state) {
  return true;
}

// setup function for SinricPro
void setupSinricPro() {
  SinricProSwitch& mySwitch = SinricPro[SWITCH_ID];
  mySwitch.onPowerState(onPowerState);

  SinricPro.onPong([](uint32_t since) {
    Serial.printf("Got heartbeat!\r\n");
    lastHeartbeatMills = millis();
  });

  SinricPro.onConnected([]() {
    Serial.printf("Connected to SinricPro\r\n");
  });

  SinricPro.onDisconnected([]() {
    Serial.printf("Disconnected from SinricPro\r\n");
  });

  SinricPro.begin(APP_KEY, APP_SECRET);
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  unsigned long startAttemptTime = millis();
  const unsigned long timeout = 30000; // 30 seconds timeout
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
    delay(1000);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    reconnectAttempts = 0;
  } else {
    Serial.println("\nFailed to connect to WiFi within timeout period");
    ESP.restart();
  }
}

void handleNoHeartbeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatMills >= noHeartbeatResetIntervalMills) {
    Serial.println("No heartbeat for 15 mins. Restarting ESP32...");
    ESP.restart();
  }
}

void handleWiFiDisconnections() {
  unsigned long currentMillis = millis();

  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    reconnectAttempts++;

    if (reconnectAttempts <= maxReconnectAttempts) {
      WiFi.disconnect();
      WiFi.reconnect();
      previousMillis = currentMillis;

      // Wait for connection or timeout
      unsigned long startAttemptTime = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(100);
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Reconnected to WiFi");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        reconnectAttempts = 0;
      } else {
        Serial.print("Reconnection attempt(");
        Serial.print(reconnectAttempts);
        Serial.println(") failed");
      }
    } else {
      Serial.println("Failed to reconnect after 10 attempts. Restarting ESP32...");
      ESP.restart();
    }
  }
}

void printWiFiStatus() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrintTime >= 10000) {

    Serial.print("ESP is: ");

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("connected");
    } else {
      Serial.println("disconnected");
    }
    lastPrintTime = currentMillis;
  }
}

void loop() {
  SinricPro.handle();
  handleWiFiDisconnections();
  handleNoHeartbeat();
  printWiFiStatus();
}