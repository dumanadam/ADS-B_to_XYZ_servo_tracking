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

#define eaRa 6371 // Earths Radius
#define toRad (3.1415926536 / 180) // To Radian

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
double acTimePos, acLat, acLon, acAlt, acOnGround, acVelocity, acTTrack, acVRate;
//
double bearingToAc;



// Functions
float toRadians(float angle) {
  return (PI / 180) * angle;
}

float toDegrees(float rad) {
  return (rad * 180) / PI;
}

//Sets and moves the position of the servos in servoPosition[] according to start,mid,end
void setServosDef(String position = "NA") {
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

//Moves servos to currently stored positions in servoBotPosition and ServoTopPosition unless passed into function as microseconds
void setServosXY(int servoBotPosition = servoPosition[0], int ServoTopPosition = servoPosition[1] ){
  Serial.print("servo bot moved to:");
  Serial.println(servoBotPosition);
  Serial.print("servo top moved to:");
  Serial.println(ServoTopPosition);
  servoBot.writeMicroseconds(servoBotPosition);
  servoTop.writeMicroseconds(ServoTopPosition);
  delay(15);
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

 
// Calculate bearing from servo to AC
int bearingServoToAC(float latitude1, float longitude1, float latitude2, float longitude2) {
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

//Convert bearing from ADSB data to X co-ordinate for top servo
void bearingtoX(double bearing){
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
  Serial.print("calculated servoPosition[1] ======> ");
  Serial.println(servoPosition[1]);
  delay(15);
  if(servoPosition[1] >= servoTopMax) {
    servoPosition[1] = servoTopMax;
    Serial.println("Hit max");
    }
  if(servoPosition[1] <= servoTopMin) {
    servoPosition[1] = servoTopMin;
    Serial.println("Hit min");
  }

}

//Returns the distance between the servos and the Aircraft
double calcServoToAc(double currentLat,double currentLon,double acLat,double acLon)
{
	double dx, dy, dz;
	currentLon -= acLon;
	currentLon *= toRad, currentLat *= toRad, acLat *= toRad;
 
	dz = sin(currentLat) - sin(acLat);
	dx = cos(currentLon) * cos(currentLat) - cos(acLat);
	dy = sin(currentLon) * cos(currentLat);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * eaRa;
}

//returns km for m/s and seconds input
double calcDistance(double speedMsec, double timeSec ) {
  return speedMsec / timeSec; 
}


//Estimate end point in 10 secs between JSON calls so vector can be made
void trackAC(double acLat, double acLon, int bearing, double distanceKM) {
  acLat *= toRad, acLon *= toRad;

  double lat2 = asin( sin(acLat)*cos(distanceKM/eaRa) + cos(acLat)*sin(distanceKM/eaRa)*cos(bearing));
  double lon2 = acLon + atan2(sin(bearing)*sin(distanceKM/eaRa)*cos(acLat),cos(distanceKM/eaRa)-sin(acLat)*sin(lat2));

Serial.print("lat AC dest ======> ");
Serial.println(toDegrees(lat2));
Serial.print("lon AC dest ======> ");
Serial.println(toDegrees(lon2));
setServosXY

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
  acTimePos = states_0[3]; // 1574661430
  acLon = states_0[5]; // 145.1089
  acLat = states_0[6]; // -37.5592
  acAlt = states_0[7]; // 2339.34
  acOnGround = states_0[8]; // false
  acVelocity = states_0[9]; // 93.35
  acTTrack = states_0[10]; // 30.1
  acVRate = states_0[11]; // 6.83

for (size_t i = 0; i < 10; i++){
  bearingToAc = bearingServoToAC(currentLat,currentLon,acLat,acLon);

  Serial.print("bearingToAc :");
  Serial.println(bearingToAc);

  bearingtoX(bearingToAc);

  Serial.print("moving servos to bearing from servo");
  Serial.println(servoPosition[0]);
  Serial.println(servoPosition[1]);
  Serial.println("");

  setServosXY();

  delay(1000);

  trackAC(acLat,acLon, acTTrack, .9  );

  //acLon -= .01;
  
  Serial.println("");
  Serial.println("");
  double distance  = calcServoToAc(currentLat,currentLon,acLat,acLon);
  Serial.print("distance in loop :");
  Serial.println(distance);
  delay(1000);
  Serial.println("");
  Serial.println("");

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





 
