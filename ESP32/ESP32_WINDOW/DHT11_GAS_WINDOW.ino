#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#define WIFI_SSID "IEEEsbKastoria"
#define WIFI_PASSWORD "ieee2020"
#define MQTT_HOST IPAddress(192, 168, 69, 3)
#define MQTT_PORT 1883

// Orizoume Topics gia to MQTT
#define MQTT_PUB_GAS  "esp32/gas"
#define MQTT_PUB_WINDOW "esp32/window_sensor"


// Orismos pin
#define door_sensor 19
#define Gas_analog 32
// Metablites Gas sensor(analog), Katastasi_Portas
int doorState;
int gasAnalog;

//EPIKOINWNIA MQTT
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Apothikeusi teleutaiou Publish gia thermokrasia
const long interval = 1500 ;        // Ruthmos ananeosis timwn

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(door_sensor, INPUT_PULLUP); // Ekinisi Door Sensor

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();

}
void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds)
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    gasAnalog = analogRead(Gas_analog);
    doorState = digitalRead(door_sensor); // read state


  if (isnan(gasAnalog)) {
      Serial.println(F("Failed to read from Gas sensor!"));
      return;
    }
  if (doorState == HIGH) {
    Serial.println("The door is open");
  } else {
    Serial.println("The door is closed");
  }

    // Publish MQTT minimatos sto topic esp32/gas
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_GAS, 1, true, String(gasAnalog).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_GAS, packetIdPub4);
    Serial.printf("Message: %.2f \n", gasAnalog);

    //Publish MQTT minimatos sto topic esp32/window_sensor
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_WINDOW, 1, true, String(doorState).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_DOOR, packetIdPub5);
    Serial.printf("Message: %.2f \n", doorState);

  }
}
