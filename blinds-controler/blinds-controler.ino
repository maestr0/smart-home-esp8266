#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <PubSubClient.h>         //MQTT
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <OneWire.h>
#include <DallasTemperature.h>


////**********START CUSTOM PARAMS******************//

//Define parameters for the http firmware update
const char* host = "LivingroomESP";
const char* update_path = "/WebFirmwareUpgrade";
const char* update_username = "admin";
const char* update_password = "secret";

// OpenHab2 items
//Switch Blinds {mqtt=">[broker:livingroom/blinds-button:command:ON:OPEN],>[broker:livingroom/blinds-button:command:OFF:CLOSED],<[broker:livingroom/blinds-status:state:ON:OPEN],<[broker:livingroom/blinds-status:state:OFF:CLOSED]"}

//Define the pins
//#define RELAY_PIN 5
//#define DOOR_PIN 4
#define POSITION_SENSOR D3
#define POSITION_LED D4


//Define MQTT Params. If you don't need to 
#define mqtt_server "192.168.0.6"
#define blinds_status_topic "livingroom/blinds-status"
#define blinds_button_topic "livingroom/blinds-button"
#define temp_topic "livingroom/temperature"
#define air_pressure_topic "livingroom/air-pressure"
const char* mqtt_user = "pi"; 
const char* mqtt_pass = "pi";

//************END CUSTOM PARAMS********************//
//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

//Setup the web server for http OTA updates. 
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiClient espClient;

//Initialize MQTT
PubSubClient client(espClient);

//Setup Variables
String switch1;
String strTopic;
String strPayload;
char* door_state = "UNDEFINED";
char* last_state = "";

//Wifi Manager will try to connect to the saved AP. If that fails, it will start up as an AP
//which you can connect to and setup the wifi
WiFiManager wifiManager;
long lastMsg = 0;

// BMP085
// Number Temperature  "Temperature [%.0f]C"   <temperature> { mqtt="<[mosquitto:livingroom/temperature:state:default]" }
// Number AirPressure  "Pressure [%.0f]Pa"   <pressure> { mqtt="<[mosquitto:livingroom/air-pressure:state:default]" }
Adafruit_BMP085 bmp;
long lastMsgTempAndHumidity = 0;

// external temperature sensor Dallas
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS D5

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;


// blinds position sensor
int blinds_position = 0;
int lastBlindsStatusUpdate = 0;
int lastBlindsState = 0;

void setup() {

  Serial.begin(115200);

  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 or BMP085 sensor at 0x77");
    while (1) {}
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");
  
  //Set Relay(output) and Door(input) pins
//  pinMode(RELAY_PIN, OUTPUT);
//  pinMode(RELAY_PIN, LOW);
//  pinMode(DOOR_PIN, INPUT);

  // blinds position sensor
  pinMode(POSITION_LED, OUTPUT);  // set onboard LED as output
  pinMode(POSITION_SENSOR, INPUT_PULLUP);      // set pin as input
  
    //Set the wifi config portal to only show for 3 minutes, then continue.
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect(host);

  //sets up the mqtt server, and sets callback() as the function that gets called
  //when a subscribed topic has data
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); //callback is the function that gets called for a topic sub

  //setup http firmware update page.
  MDNS.begin(host);
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  Serial.printf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and your password\n", host, update_path, update_username);

  setupExternalTempSensor();
}

void setupExternalTempSensor(){
 
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  // Assign address manually. The addresses below will beed to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // Note that you will need to use your specific address here
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  
  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices, 
  // or you have already retrieved all of them. It might be a good idea to 
  // check the CRC to make sure you didn't get garbage. The order is 
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();
}

void loop() {
  //If MQTT client can't connect to broker, then reconnect
  if (!client.connected()) {
    reconnect();
  }
  //checkBlindsState();
  checkTempAndPressure();
  checkBlindsPosition();
  client.loop(); //the mqtt function that processes MQTT messages
  httpServer.handleClient(); //handles requests for the firmware update page
}

void checkBlindsPosition(){
  blinds_position = digitalRead(POSITION_SENSOR);  
  if(lastBlindsState!=blinds_position) {    
    lastBlindsState = blinds_position;
    digitalWrite(POSITION_LED, blinds_position);  
    long now = millis();
    // update MQTT every 500ms in order to get stable sensor readings
    if (now - lastBlindsStatusUpdate > 500) {
      lastBlindsStatusUpdate = now;           
      if(blinds_position == 1) {
        client.publish(blinds_status_topic, "CLOSED");
      } else {
        client.publish(blinds_status_topic, "OPEN");
      }
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {  
  payload[length] = '\0';
  strTopic = String((char*)topic);
  if (strTopic == blinds_button_topic)
  {
    switch1 = String((char*)payload);
    if (switch1 == "OPEN")
    {
      //'click' the relay
      Serial.println("ON");
//      pinMode(RELAY_PIN, HIGH);
      delay(600);
//      pinMode(RELAY_PIN, LOW);
    } else {
      Serial.print("Button topic val:"); Serial.println(switch1);
      }
  }
}

//void checkBlindsState() {
//  //Checks if the door state has changed, and MQTT pub the change
//  last_state = door_state; //get previous state of door
//  if (digitalRead(DOOR_PIN) == 0) // get new state of door
//    door_state = "OPENED";
//  else if (digitalRead(DOOR_PIN) == 1)
//    door_state = "CLOSED"; 
//
//  if (last_state != door_state) { // if the state has changed then publish the change
//    client.publish(door_topic, door_state);
//    Serial.println(door_state);
//  }
//  //pub every minute, regardless of a change.
//  long now = millis();
//  if (now - lastMsg > 60000) {
//    lastMsg = now;
//    client.publish(door_topic, door_state);
//  }
//}

void checkTempAndPressure() {
  //pub every minute, regardless of a change.
  long now = millis();
  if (now - lastMsgTempAndHumidity > 10000) {
    lastMsgTempAndHumidity = now;        

    float t = bmp.readTemperature();
    if(t > 0 && t < 40) {
      String t1 = String(t);
      char tmp1[sizeof(t1)];    
      t1.toCharArray(tmp1,sizeof(tmp1));
      client.publish(temp_topic, tmp1);
    }
    
    float p = bmp.readPressure() / 100.0;
    if(p < 1032 && p > 980) {
      String t2 = String(p);
      char tmp2[sizeof(t2)];    
      t2.toCharArray(tmp2,sizeof(tmp2));
      client.publish(air_pressure_topic, tmp2);
    }

//    float extTemp = analogRead(A0)*5/1024.0;
//    extTemp = extTemp - 0.5;
//    extTemp = extTemp / 0.01;
//    Serial.println(extTemp);

 // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  
  // It responds almost immediately. Let's print out the data
  printTemperature(insideThermometer); // Use a simple function to print out the data
  }
}

void reconnect() {
  //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
  wifiManager.autoConnect(host);
  Serial.print("Attempting MQTT connection...");
  if (client.connect(host, mqtt_user, mqtt_pass)) {
    Serial.println("connected");
    client.subscribe("livingroom/#");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}
