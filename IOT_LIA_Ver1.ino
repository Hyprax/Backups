// Libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <dht.h>

// Wi-Fi credentials:
const char* ssid = "VIRGIN093";  // The name of the WiFi network
const char* password = "pArivash1354$";

// MQTT broker details:
const char* mqtt_server = "192.168.2.218"; // The MQTT broker's hostname or IP address
const int mqtt_port = 1883;  // MQTT broker port (1883 is default)
const char* mqtt_pub_topic = "Restaurant/Kitchen";  // MQTT topic to publish messages
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
#define FanSpeedPin 19
//Flashlight Pin
#define FlashPin 25
// Warning Light Pin
#define LEDredPin 22
// Buzzer Pin
#define BuzzPin 23

// Defining Input Pins
// DHT11 Pin
#define DHT11Pin 35
dht DHT;
// Photocel Pin
#define PhotocelPin 12 
// Light switch PB Pin
#define LTswitchPin 26
// Alarm OFF PB
#define SilencePin 21

// Memory
byte SwitchState;
byte SilenceState;

// DHT Variables
int chk = DHT.read11(DHT11Pin);
int Temp = DHT.temperature;
int Humid = DHT.humidity;

// Kitchen Light
int LTstate;

// Buzzer State
bool Quiet;

void setup() 
{
  // Start Serial communication
  Serial.begin(115200);

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
  pinMode(FanSpeedPin, OUTPUT);
  pinMode(FlashPin, OUTPUT);
  pinMode(LEDredPin, OUTPUT);
  pinMode(BuzzPin, OUTPUT);

  // Setting the Pin Modes for the Inputs
  pinMode(DHT11Pin, INPUT);
  pinMode(PhotocelPin, INPUT);
  pinMode(LTswitchPin, INPUT);
  pinMode(SilencePin, INPUT);

  // Button State Memory
  SwitchState = digitalRead(LTswitchPin);
  SilenceState = digitalRead(SilencePin);
}

void loop() 
{
  // Connect to MQTT if necessary
  if (!client.connected()) 
  {
    connect();
  }

  Mosquitto_pub();

  // Allow the PubSubClient to process incoming messages
  client.loop();

  byte INIstateLT = digitalRead(LTswitchPin);
  if (INIstateLT != SwitchState)
  {
    SwitchState = INIstateLT;
    if (INIstateLT == 1)
    {
      digitalWrite(LTswitchPin, 1);
    }
    else if (INIstateLT == 0)
    {
      digitalWrite(LTswitchPin, 0);
    }
  }

  byte INIstateSilence = digitalRead(SilencePin);
  if (INIstateSilence != SilenceState)
  {
    SilenceState = INIstateSilence;
    if (INIstateSilence == 1)
    {
      Quiet = true;
    }
    else if (INIstateSilence == 0)
    {
      Quiet = false;
    }
  }
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
    if (Message == "Temperature too High, turn fan ON") 
    {
      digitalWrite(FanPin, 1);
      digitalWrite(LEDredPin, 1);
    } 
    else if (Message == "Temperature too LOW, turn fan OFF") 
    {
      digitalWrite(FanPin, 0);
      digitalWrite(LEDredPin, 0);
    }
  }

  if (String(topic) == mqtt_sub_topic) 
  {
    if (Message == "FIRE ALERT") 
    {
      Buzzer();
    } 
    else if (Message == "Alert Dismissed") 
    {
      Quiet = true;
    }
  }
}

void Photo()
{
  int PhotoVal = analogRead(PhotocelPin);
  if (PhotoVal >= 1500)
  {
    LTstate = 1;
  }
  else if (PhotoVal <= 1500)
  {
    LTstate = 0;
  }
}

void DTHsens()
{
  Serial.print("Temperature =");
  Serial.println(DHT.temperature);

  Serial.print("Humidity =");
  Serial.println(DHT.humidity);

  delay(1000);
}

void Mosquitto_pub()
{
  // Publish a message every 5 seconds
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;

    String KitchenLights = String(LTstate);
    String KitchenTemp = String(Temp);
    String KitchenHumid = String(Humid);

    // Publish the message to the MQTT topic
    client.publish(mqtt_pub_topic, KitchenLights.c_str());
    Serial.println("Published: " + KitchenLights);
    client.publish(mqtt_pub_topic, KitchenTemp.c_str());
    Serial.println("Published: " + KitchenTemp);
    client.publish(mqtt_pub_topic, KitchenHumid.c_str());
    Serial.println("Published: " + KitchenHumid);
  }
}

void Buzzer()
{
  if(Quiet == false)
  {
    digitalWrite(BuzzPin, HIGH);
  }
  else if (Quiet == true)
  {
    digitalWrite(BuzzPin, LOW);
  }
}
