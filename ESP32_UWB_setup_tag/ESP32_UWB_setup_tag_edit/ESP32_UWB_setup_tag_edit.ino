// currently tag is module #5
// The purpose of this code is to set the tag address and antenna delay to default.
// this tag will be used for calibrating the anchors.

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <PubSubClient.h>
#include <WiFi.h>

//#define DEBUG_TRILAT   //debug output in trilateration code
//#define DEBUG_DISTANCES   //print collected anchor distances for algorithm
//#define DEBUG_ANCHOR_ID  // print anchor IDs and raw distances

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
// leftmost two bytes below will become the "short address"
char tag_addr[] = "8D:00:22:EA:82:60:3B:9C";
char short_tag_addr[] = "8D";

// variables for position determination
#define N_ANCHORS 4   //THIS VERSION WORKS ONLY WITH 4 ANCHORS. May be generalized to 5 or more.
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds)

// WiFi credentials
const char* ssid = "AIS 4G Hi-Speed Home WiFi_544638";
const char* password = "51544638";

// MQTT broker
const char* mqtt_server = "192.168.1.180";
const char* mqtt_topic = "tag_positions";

// WiFi and MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports

void setup()
{
  Serial.begin(115200);
  delay(1000);

  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

// start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  setupWiFi();
  setupMQTT();
}

void loop()
{
  DW1000Ranging.loop();
  loopMQTT();
  publishTagPosition();
}

void newRange()
{
  int i;

  //index of this anchor, expecting values 1 to 4
  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x07; //expect devices 1 to 7
  float range = DW1000Ranging.getDistantDevice()->getRange();
  if (index > 0 && index < 5) {
    last_anchor_update[index - 1] = millis();  //(-1) => array index
    last_anchor_distance[index-1] = range;
    if (range < 0.0 || range > 30.0)     last_anchor_update[index - 1] = 0;  //sanity check, ignore this measurement
  }

  #ifdef DEBUG_ANCHOR_ID
    Serial.print(index); //anchor ID, raw range
    Serial.print(", ");
    Serial.println(range);
  #endif
    //check for four measurements within the last interval
  int detected = 0;  //count anchors recently seen

  for (i = 0; i < N_ANCHORS; i++) {

    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }
  if ( (detected >= N_ANCHORS)) { //four recent measurements

    #ifdef DEBUG_DISTANCES
      // print distance and age of measurement
      uint32_t current_time = millis();
      for (i = 0; i < N_ANCHORS; i++) {
        Serial.print(last_anchor_distance[i]);
        Serial.print("\t");
        Serial.println(current_time - last_anchor_update[i]); //age in millis
      }
    #endif
    Serial.print("D= ");  //result
    Serial.print(last_anchor_distance[0]);
    Serial.write(',');
    Serial.print(last_anchor_distance[1]);
    Serial.write(',');
    Serial.print(last_anchor_distance[2]);
    Serial.write(',');
    Serial.println(last_anchor_distance[3]);
  }
  // Create a JSON payload with the anchor data
  //String payload = String(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX) + "," + String(DW1000Ranging.getDistantDevice()->getRange());

  // Print the data to serial monitor
  //Serial.println(payload);
  
}  //end newRange

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, 1883);
}

void loopMQTT() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void publishTagPosition() {
  String payload = String(short_tag_addr) + "," + String(last_anchor_distance[0]) + "," + String(last_anchor_distance[1]) + "," + String(last_anchor_distance[2]) + "," + String(last_anchor_distance[3]);
  mqttClient.publish(mqtt_topic, payload.c_str());
}
