#include <Arduino.h>
#include <SimpleDHT.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
// #include <SRAM.h>

int pinDHT11 = 2;
SimpleDHT11 dht11(pinDHT11);

char tempString[8];
char humString[8];

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 69, 60);
IPAddress server(172, 105, 246, 197);

EthernetClient ethClient;
PubSubClient client(server, 1883, ethClient);

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Ethernet.begin(mac, ip);
  if (client.connect("arduinoClient")) {
    Serial.print("Doulepse to papari");
  }

  // sram.begin();
  // sram.seek(1);

  Serial.begin(9600);
  
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  float temperature = 0;
  float humidity = 0;
  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
    Serial.print("Read DHT22 failed, err="); Serial.print(SimpleDHTErrCode(err));
    Serial.print(","); Serial.println(SimpleDHTErrDuration(err)); delay(2000);
    return;
    }
  // Serial.print("Sample OK: ");
  // Serial.print((float)temperature); Serial.print(" *C, ");
  // Serial.print((float)humidity); Serial.println(" RH%");

  dtostrf(temperature, 1, 2, tempString);
  Serial.print("Temperature: ");
  Serial.println(tempString);
  client.publish("arduino/temperature", tempString);

  
  dtostrf(humidity, 1, 2, humString);
  Serial.print("Humidity: ");
  Serial.println(humString);
  client.publish("arduino/humidity", humString);
  
  // DHT11 sampling rate is 0.5HZ.
  delay(2500);
}
