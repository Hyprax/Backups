// Libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <DHT.h>


// Wi-Fi credentials:
const char* ssid = "VIRGIN093";  // The name of the WiFi network
const char* password = "pArivash1354$";

// MQTT broker details:
const char* mqtt_server = "192.168.2.218"; // The MQTT broker's hostname or IP address
const int mqtt_port = 1883;  // MQTT broker port (1883 is default)
const char* mqtt_pub_topic_DHT = "Kitchen/Temp";  // MQTT topic to publish messages of data from DHT11
const char* mqtt_pub_topic_PhC = "Kitchen/Presence";  // MQTT topic to publish messages of data from PhotoCell
const char* mqtt_sub_topic = "Pi4/Order";  // MQTT topic to subscribe messages
// MQTT client name prefix (will add MAC address)
String name = "ESP32Client_";

// Millies
unsigned long previousMillis = 0;
unsigned long currentMillis = millis();
const long interval = 5000;

// Create an instance of the WiFiClient class
WiFiClient espClient;
// Create an instance of the PubSubClient class
PubSubClient client(espClient);

// Defining Output Pins
// Fan Pin
#define FanPin 18
// Speed Control Pin
//Flashlight Pin
#define FlashPin 25
// Warning Light Pin
#define LEDredPin 22
// Buzzer Pin
#define BuzzPin 23

// Defining Input Pins
// DHT11
#define DHTpin 4
DHT dht(DHTpin, DHT11);
// Photocel Pin
#define PhotoPin 34 
// Light switch PB Pin
#define LTswitchPin 26
// Alarm OFF PB
#define SilencePin 19

//Kitchen Lights(presence)
int LTstate;

void setup() 
{
  // Start Serial communication
  Serial.begin(115200);
  dht.begin();

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set MQTT server and port
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(Mosquitto_sub);

  // Setting the Pin Modes for the outputs
  pinMode(FanPin, OUTPUT);
  pinMode(FlashPin, OUTPUT);
  pinMode(LEDredPin, OUTPUT);
  pinMode(BuzzPin, OUTPUT);

  // Setting the Pin Modes for the Inputs
  pinMode(PhotoPin, INPUT);
  pinMode(LTswitchPin, INPUT);
  pinMode(SilencePin, INPUT);
}

void loop() 
{
  // Connect to MQTT if necessary
  if (!client.connected()) 
  {
    connect();
  }

  if(digitalRead(LTswitchPin) == 1)
  {
    digitalWrite(FlashPin, 1);
  }
  else
  {
    digitalWrite(FlashPin, 0);
  }

  if(digitalRead(SilencePin) == 1)
  {
    digitalWrite(BuzzPin, LOW);
    delay(5000);
    digitalWrite(BuzzPin, HIGH);
  }

  currentMillis = millis();
  // Publish a message every 5 seconds
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;

    int PhotoVal = analogRead(PhotoPin);
    if(PhotoVal > 3000)
    {
      LTstate = 1;
    }
    else if(PhotoVal < 3000)
    {
      LTstate = 0;
    }

    String KitchenLights = String(LTstate);

    float Temp = dht.readTemperature();
    String KitchenTemp = String(Temp);

    // Publish the message to the MQTT topic
    client.publish(mqtt_pub_topic_PhC, KitchenLights.c_str());
    Serial.println("Published LT: " + KitchenLights);

    client.publish(mqtt_pub_topic_DHT, KitchenTemp.c_str());
    Serial.println("Published Temp: " + KitchenTemp);
  }

  // Allow the PubSubClient to process incoming messages
  client.loop();
}

void connect() 
{
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.println("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(name.c_str())) 
    {
      Serial.println("Connected to MQTT broker");

     // Subscribe to the topic
     if (client.subscribe(mqtt_sub_topic)) 
     {
        Serial.println("Subscribed to topic: " + String(mqtt_sub_topic));
     }
     else 
     {
        Serial.println("Failed to subscribe");
     }

    } 
    else 
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(client.state());
      Serial.println("Try again in 5 seconds");
      delay(5000);
    }
  }
}

// Function for received messages
void Mosquitto_sub(char* topic, byte* payload, unsigned int length) 
{
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  // decoding the message
  Serial.print("Message: ");
  String Message;
  for (unsigned int i = 0; i < length; i++) 
  {
    Message += (char)payload[i];
  }
  Serial.println(Message);

  // Control FAN based on the message
  if (String(topic) == mqtt_sub_topic) 
  {
    if (Message == "Temperature HIGH") 
    {
      digitalWrite(FanPin, 1);
      digitalWrite(LEDredPin, 1);
      digitalWrite(BuzzPin, LOW);
    } 
    else if (Message == "Temperature LOW") 
    {
      digitalWrite(FanPin, 0);
      digitalWrite(LEDredPin, 0);
      digitalWrite(BuzzPin, LOW);
    }
  }

  if (String(topic) == mqtt_sub_topic) 
  {
    if (Message == "FIRE ALERT") 
    {
      digitalWrite(BuzzPin, HIGH);
      digitalWrite(FanPin, 0);
      digitalWrite(LEDredPin, 0);
    } 
    else if (Message == "Alert Dismissed") 
    {
      digitalWrite(BuzzPin, LOW); 
    }
  }
}
