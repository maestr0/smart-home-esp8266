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
#include <IRsend.h>
#include <Servo.h>

////**********START CUSTOM PARAMS******************//

//Define parameters for the http firmware update
const char* host = "LivingroomESP";
const char* update_path = "/WebFirmwareUpgrade";
const char* update_username = "admin";
const char* update_password = "secret";

// OpenHab2 items
//Switch Blinds {mqtt=">[broker:livingroom/blinds-button:command:ON:OPEN],>[broker:livingroom/blinds-button:command:OFF:CLOSED],<[broker:livingroom/blinds-status:state:ON:OPEN],<[broker:livingroom/blinds-status:state:OFF:CLOSED]"}

//Define the pins
#define POSITION_SENSOR D3
#define POSITION_LED D4


//Define MQTT Params. If you don't need to
#define mqtt_server "192.168.0.6"

#define blinds_status_topic "livingroom/blinds/status"
#define blinds_switch_topic "livingroom/blinds/switch"
#define temp_inside_topic "livingroom/temperature-inside"
#define air_pressure_topic "livingroom/air-pressure"
#define light_level_topic "livingroom/light-level"
#define temp_outside_topic "livingroom/temperature-outside"
#define ac_ir_topic "livingroom/ac_ir/switch"
#define main_ir_topic "livingroom/main_ir/switch"

const char* mqtt_user = "";
const char* mqtt_pass = "";

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
String blinds;
String acIr;
String mainIr;

String strTopic;
String strPayload;
//char* door_state = "UNDEFINED";
//char* last_state = "";

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
boolean blindsChanging = false;
Servo servo;


// light level
int lastMsgLightLevel = 0;

// IR
IRsend irsend(D6);
IRsend irsendAc(D8);

void setup() {

  Serial.begin(115200);

  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 or BMP085 sensor at 0x77");
    while (1) {}
  }

  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  // IR
  irsend.begin();
  irsendAc.begin();

  // blinds position sensor
  pinMode(POSITION_LED, OUTPUT);  // set onboard LED as output
  pinMode(POSITION_SENSOR, INPUT_PULLUP);      // set pin as input
  servo.attach(D7);

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

void setupExternalTempSensor() {

  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  blindsGoStop();
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
  checkLightLevel();
  client.loop(); //the mqtt function that processes MQTT messages
  httpServer.handleClient(); //handles requests for the firmware update page
}

void checkLightLevel() {
  //pub every minute, regardless of a change.
  long now = millis();
  if (now - lastMsgLightLevel > 10000) {
    lastMsgLightLevel = now;
    float ll = analogRead(A0);
    String t3 = String(ll);
    char tmp3[sizeof(t3)];
    t3.toCharArray(tmp3, sizeof(tmp3));
    client.publish(light_level_topic, tmp3);
  }
}

void checkBlindsPosition() {
  blinds_position = digitalRead(POSITION_SENSOR);
  if (lastBlindsState != blinds_position) {
    lastBlindsState = blinds_position;
    digitalWrite(POSITION_LED, blinds_position);
    long now = millis();
    // update MQTT every 500ms in order to get stable sensor readings
    if (now - lastBlindsStatusUpdate > 500 && blindsChanging) {
      lastBlindsStatusUpdate = now;
      if (blinds_position == 1) {
        blindsGoStop();
        blindsChanging = false;
        client.publish(blinds_status_topic, "CLOSED");
      } else {
        blindsGoUp(); // for another 2s TODO
        client.publish(blinds_status_topic, "OPEN");
      }
    }
  }

  
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  strTopic = String((char*)topic);
  Serial.print("Mqtt topic "); Serial.println(strTopic);
  if (strTopic == blinds_switch_topic)
  {
    if (blindsChanging) {
      Serial.println("Ignoring blinds status updates. Blinds position changing...");
    } else {
      blinds = String((char*)payload);
      blindsChanging = true;
      blindsChanging = true;
      if (blinds == "OPEN")
      {
        Serial.println("Opening blinds ...");
        blindsGoDown(); // fixme
      } else {
        Serial.print("Closing blinds...");
        blindsGoDown();
      }
    }
  } else if (strTopic == ac_ir_topic) {
    acIr = String((char*)payload);
    if (acIr == "SWITCH")
    {
      Serial.println("AC switch ...");
      irsendAc.sendNEC(0x10AF8877, 32);
    }
  } else if (strTopic == main_ir_topic) {
    acIr = String((char*)payload);
    if (acIr == "SPEAKER_BT") {
      Serial.println("SPEAKER BT");
      irsend.sendNEC(0x8E7B24D, 32);
    } else if (acIr == "SPEAKER_LINE_IN") {
      Serial.println("SPEAKER_LINE_IN");
      irsend.sendNEC(0x8E77A85, 32);
    } else if (acIr == "SPEAKER_VOLUME_UP") {
      Serial.println("SPEAKER_VOLUME_UP");
      irsend.sendNEC(0x8E7D42B, 32);
    } else if (acIr == "SPEAKER_VOLUME_DOWN") {
      Serial.println("SPEAKER_VOLUME_DOWN");
      irsend.sendNEC(0x8E73CC3, 32);
    } else if (acIr == "SPEAKER_MUTE") {
      Serial.println("SPEAKER_MUTE");
      irsend.sendNEC(0x8E758A7, 32);
    } else if (acIr == "TV_SWITCH") {
      Serial.println("TV_SWITCH");
      irsend.sendSAMSUNG(0xE0E040BF, 32);
    }
  }
}

void checkTempAndPressure() {
  //pub every minute, regardless of a change.
  long now = millis();
  if (now - lastMsgTempAndHumidity > 10000) {
    lastMsgTempAndHumidity = now;

    float t = bmp.readTemperature();
    if (t > 0 && t < 40) {
      String t1 = String(t);
      char tmp1[sizeof(t1)];
      t1.toCharArray(tmp1, sizeof(tmp1));
      client.publish(temp_inside_topic, tmp1);
    }

    float p = bmp.readPressure() / 100.0;
    if (p < 1032 && p > 980) {
      String t2 = String(p);
      char tmp2[sizeof(t2)];
      t2.toCharArray(tmp2, sizeof(tmp2));
      client.publish(air_pressure_topic, tmp2);
    }

    sensors.requestTemperatures();
    float to = sensors.getTempC(insideThermometer);
    if (to < 100 && to > -30) {
      String t3 = String(to);
      char tmp3[sizeof(t3)];
      t3.toCharArray(tmp3, sizeof(tmp3));
      client.publish(temp_outside_topic, tmp3);
    }
  }
}

void reconnect() {
  //Reconnect to Wifi and to MQTT. If Wifi is already connected, then autoconnect doesn't do anything.
  wifiManager.autoConnect(host);
  Serial.print("Attempting MQTT connection...");
  if (client.connect(host, mqtt_user, mqtt_pass)) {
    Serial.println("connected");
    client.subscribe(blinds_switch_topic);
    client.subscribe(ac_ir_topic);
    client.subscribe(main_ir_topic);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
}

void blindsGoDown() {
  servo.write(10);
}

void blindsGoUp() {
  servo.write(140);
}

void blindsGoStop() {
  servo.write(90);
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
