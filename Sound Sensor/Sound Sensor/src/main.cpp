#include <Arduino.h>
#include <Ethernet.h>
#include <PubSubClient.h>

const int soundPin = A0;
int soundVal = 0;

//ethernet_shield
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 69, 151);
IPAddress server(172, 105, 246, 197);

EthernetClient ethClient;
PubSubClient client(server, 1883, ethClient);


void reconnect(){
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

void setup (){
  Serial.begin(9600);
//  pinMode (soundPin, INPUT);

  Ethernet.begin(mac, ip);
  if (client.connect("arduinoClient")) {
    Serial.print("Doulepse to papari");
  }

   Serial.begin(9600);  
}
 
void loop (){ 
  soundVal = analogRead(soundPin);
  Serial.print("Sound Level is:");
  Serial.println(soundVal);
  if (soundVal>500){
      if (!client.connected()) {
        reconnect();
      }
      client.loop();
      client.publish("arduino/sound_level", "1");
      delay(2500);
      client.publish("arduino/sound_level", "0");
  }
}