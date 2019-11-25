#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <credentials.h>
#include <esp8266httpclient.h>
#include <ArduinoJson.h>

//Variables to be edited 

int servoBotPin = 5;
int servoTopPin = 4;
int laserPin = 14;
int servoBotMax = 180;
int servoTopMax = 120;
float q1Multiplier = 3;
float q2Multiplier = 1;
float q3Multiplier = 3;
float q4Multiplier = 1;


// other Variables and consts


// Functions
float bearing(float currentLat,float currentLon,float rangeBoxLat2,float rangeBoxLon2){

    float teta1 = radians(currentLat);
    float teta2 = radians(rangeBoxLat2);
    float delta1 = radians(rangeBoxLat2 - currentLat);
    float delta2 = radians(rangeBoxLon2-currentLon);

    //==================Heading Formula Calculation================//

    float y = sin(delta2) * cos(teta2);
    float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    float brng = atan2(y,x);
    brng = degrees(brng);// radians to degrees
    brng = ( ((int)brng + 360) % 360 ); 

    Serial.print("Heading GPS: ");
    Serial.println(brng);

    return brng;


  }

  float bearingtoX(float bearing){
    if(bearing>=270 && bearing < 359) {

    } else if(bearing>=0 && bearing < 89) {

    } else if(bearing>=90 && bearing < 179) {

    } else if(bearing>=180 && bearing < 269) {

    } else Serial.println("Bearing Out Of Bounds");

  } 

void setup()
{
  Serial.begin(9600);
  WiFiManager wifiManager;
  wifiManager.autoConnect("ADS-B Tracker");
  Serial.println("Connected!");
}

void loop()
{

  const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_ARRAY_SIZE(17) + JSON_OBJECT_SIZE(2) + 50;
  DynamicJsonDocument doc(capacity);

  const char* json = "{\"time\":1574661430,\"states\":[[\"7c6ae1\",\"VEB     \",\"Australia\",1574661430,1574661430,145.1089,-37.5592,2339.34,false,93.35,30.1,6.83,null,2331.72,\"3673\",false,0]]}";

  deserializeJson(doc, json);

  long time = doc["time"]; // 1574661430

  JsonArray states_0 = doc["states"][0];
  const char* states_0_0 = states_0[0]; // "7c6ae1"
  const char* states_0_1 = states_0[1]; // "VEB     "
  const char* states_0_2 = states_0[2]; // "Australia"
  long states_0_3 = states_0[3]; // 1574661430
  long states_0_4 = states_0[4]; // 1574661430
  float states_0_5 = states_0[5]; // 145.1089
  float states_0_6 = states_0[6]; // -37.5592
  float states_0_7 = states_0[7]; // 2339.34
  bool states_0_8 = states_0[8]; // false
  float states_0_9 = states_0[9]; // 93.35
  float states_0_10 = states_0[10]; // 30.1
  float states_0_11 = states_0[11]; // 6.83
  float states_0_13 = states_0[13]; // 2331.72
  const char* states_0_14 = states_0[14]; // "3673"
  bool states_0_15 = states_0[15]; // false
  int states_0_16 = states_0[16]; // 0

  Serial.println(states_0_5);
  Serial.println(states_0_6);
  Serial.println(states_0_11);
  delay(2000);
}


 

 
