/* this is a simple demo made
for Build Actions for Your Community Study jam and istructions can be found here :  https://sobhy.me/AoG/

in this demo you need 
1- ESP8266 Board https://bit.ly/2scJDVE
2- Relay Module https://bit.ly/2sdIxIo
3- IFFT Account https://ifttt.com/
4- Adafruit IO Account https://io.adafruit.com/


---  Libraries used --- 
1- ESP support for Arduino https://github.com/esp8266/Arduino
2- Adafruit MQTT librray https://github.com/sobhydo/Adafruit_MQTT_Library
i'm using Ver. 0.17


*/
 
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define Relay1  D6


#define WLAN_SSID       "Jhordy"             // Your SSID
#define WLAN_PASS       "19971997"        // Your password

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME  "Jhordyb3"
#define AIO_KEY       "c8a1cfaaa2c443f68775e5598347d251"
/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/


// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe Lamp1 = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME"/feeds/Lampara"); // FeedName in my case i called it Lamp

void MQTT_connect();

void setup() {
  Serial.begin(115200);

  pinMode(Relay1, OUTPUT);
  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
 

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&Lamp1);
}

void loop() {
 
  MQTT_connect();
  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(20000))) {
    if (subscription == &Lamp1) {
      Serial.print(F("Got: "));
      Serial.println((char *)Lamp1.lastread);
      int Lamp1_State = atoi((char *)Lamp1.lastread);
      digitalWrite(Relay1, Lamp1_State);    
    }  
  } 
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
  
}