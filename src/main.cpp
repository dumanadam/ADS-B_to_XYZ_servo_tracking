#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <Servo.h>
#include <esp8266httpclient.h>
#include <ArduinoJson.h>
#include <Math.h>
#include <credentials.h>

//Variables to be edited 

int servoBotPin = 5;
int servoTopPin = 4;
int laserPin = 14;
int servoBotMin = 500;
int servoTopMin = 500;
int servoBotMax = 2500;
int servoTopMax = 1900;
double q1Multiplier = 7.777777777;
double q2Multiplier = 11.1;
double q3Multiplier = 7.777777777;
double q4Multiplier = 11.1;


// other Variables and consts
Servo servoBot; Servo servoTop; 
double servoPosition[2]; 
double servoStart[] = {servoBotMin,servoTopMin};
double servoMid[] = {((servoBotMax-servoBotMin)/2) + servoBotMin,((servoTopMax-servoTopMin)/2) + servoTopMin};
double servoEnd[] = {servoBotMax,servoTopMax};

// Functions

void positionServos(String position = "NA") {
  if(position == "start") {
    Serial.println("Servo pos  = start ");
    servoPosition[0] = servoStart[0];
    servoPosition[1] = servoStart[1];
  } else if(position == "mid") {
    Serial.println("Servo pos  = mid");
    servoPosition[0] = servoMid[0];
    servoPosition[1] = servoMid[1];
  } else if(position == "end") {
    Serial.println("Servo pos  = end");
    servoPosition[0] = servoEnd[0];
    servoPosition[1] = servoEnd[1];
  } else {
    Serial.println("Servo pos  = nothing");
    servoPosition[0] = servoMid[0];
    servoPosition[1] = servoMid[1];
    }
  Serial.print("Servo position servo  = bot");
  Serial.println(servoPosition[0]);
  Serial.print("Servo position servo  = top");
  Serial.println(servoPosition[1]);
  servoBot.writeMicroseconds(servoPosition[0]);
  delay(100);
  servoTop.writeMicroseconds(servoPosition[1]);
  delay(100);

  }

void positionServosXY(int servoBotPosition = servoPosition[0], int ServoTopPosition = servoPosition[1] ){
  Serial.print("servo bot moved to:");
  Serial.println(servoBotPosition);
  Serial.print("servo top moved to:");
  Serial.println(ServoTopPosition);
  servoBot.writeMicroseconds(servoBotPosition);
  servoTop.writeMicroseconds(ServoTopPosition);
  delay(2000);
  Serial.println("servo reset");
  servoBot.writeMicroseconds(servoMid[0]);
  servoTop.writeMicroseconds(servoMid[1]);
  delay(1000);
}

void bootSequence() {
    for (int pos = 500; pos <= servoBotMax; pos += 1) { // goes from 0 degrees to 180 degrees
    Serial.print("bootSequence  servoBot pos = >");
    Serial.println(pos);
    // in steps of 1 degree
    servoBot.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  for (int pos = 500; pos<= servoTopMax; pos += 1) { // goes from 180 degrees to 0 degrees
  Serial.print("bootSequence  servoTop pos = >");
    Serial.println(pos);
    servoTop.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  

}

  float toRadians(float angle) {
  return (PI / 180) * angle;
}

  float toDegrees(float rad) {
    return (rad * 180) / PI;
}
 
  int getDirection(float latitude1, float longitude1, float latitude2, float longitude2) {
    float lat1 = toRadians(latitude1);
    float lat2 = toRadians(latitude2);
    float lng1 = toRadians(longitude1);
    float lng2 = toRadians(longitude2);
    float Y = sin(lng2 - lng1) * cos(lat2);
    float X = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1);  
    float deg = toDegrees(atan2(Y, X));  
    // note that this implementation doesn't use the module, but angles lower than 0 get augmented by 360 only
    if (deg < 0) {
      deg = 360 + deg;
    }
    float angle = deg;
    int a = (int) (abs(angle) + (1 / 7200));
    return a;
}

double bearing(double currentLat,double currentLon,double acLat,double acLon){

    double teta1 = radians(currentLat);
    double teta2 = radians(acLat);
    double delta1 = radians(acLat - currentLat);
    double delta2 = radians(acLon-currentLon);

    //==================Heading Formula Calculation================//

    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    double brng = atan2(y,x);
    brng = degrees(brng);// radians to degrees
    brng = ( ((int)brng + 360) % 360 ); 

    Serial.print("Heading GPS: ");
    Serial.println(brng);

    return brng;
  }

  double bearingtoX(double bearing){
    if(bearing>=270 && bearing < 359) {
      servoPosition[0]= 0;
      servoPosition[1]= servoStart[1] + (((int)bearing % 270) / q1Multiplier);
    } else if(bearing>=0 && bearing < 89) {
      servoPosition[0]= 0;
      servoPosition[1]= servoMid[1] + (bearing*q2Multiplier);
    } else if(bearing>=90 && bearing < 179) {
      servoPosition[0]= 180;
      servoPosition[1]= servoMid[1] + (bearing*q2Multiplier);
    } else if(bearing>=180 && bearing < 269) {
      servoPosition[0]= 180;
      servoPosition[1]= servoStart[1] + (((int)bearing % 180) / q4Multiplier);;

    } else Serial.println("Bearing Out Of Bounds");

  }









void setup()
{
  Serial.begin(115200);
  WiFiManager wifiManager;
  wifiManager.autoConnect("ADS-B Tracker");
  Serial.println("Connected!");
  pinMode(laserPin,OUTPUT); // turning the laser on 
  servoBot.attach(4); //Digital 2
  servoTop.attach(5); //Digital 1
  //servoBot.setMaximumPulse(2000);
  //servoBot.setMinimumPulse(700);
  servoBot.writeMicroseconds(servoMid[0]);
  servoTop.writeMicroseconds(servoMid[1]);
  delay(500);

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
  double states_0_5 = states_0[5]; // 145.1089
  double states_0_6 = states_0[6]; // -37.5592
  double states_0_7 = states_0[7]; // 2339.34
  bool states_0_8 = states_0[8]; // false
  double states_0_9 = states_0[9]; // 93.35
  double states_0_10 = states_0[10]; // 30.1
  double states_0_11 = states_0[11]; // 6.83
  double states_0_13 = states_0[13]; // 2331.72
  const char* states_0_14 = states_0[14]; // "3673"
  bool states_0_15 = states_0[15]; // false
  int states_0_16 = states_0[16]; // 0
   Serial.println("finish setup");

for (size_t i = 0; i < 10; i++)
{
  double flightBearing = bearing(currentLat,currentLon,states_0_6,states_0_5);
Serial.print("flightBearing :");
Serial.println(flightBearing);
delay(500);
bearingtoX(flightBearing);

positionServosXY();
delay(1000);
double direction = getDirection(currentLat,currentLon,states_0_6,states_0_5);
Serial.print("direction :");
Serial.println(direction);
delay(1000);
  states_0_5 += .2542;
  /* code */
}



/* Serial.println("start");
positionServos("start");
delay(1000);
Serial.println("mid");
positionServos("mid");
delay(1000);
Serial.println("end");
positionServos("end");
delay(1000);
bootSequence();
servoBot.writeMicroseconds(500);              // tell servo to go to position in variable 'pos'
delay(2000);                       // waits 15ms for the servo to reach the position
servoTop.writeMicroseconds(500);              // tell servo to go to position in variable 'pos'
delay(500);                       // waits 15ms for the servo to reach the position

 */
}





 
