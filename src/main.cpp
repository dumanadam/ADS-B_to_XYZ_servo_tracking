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
int timeBetweenJSONCalls = 10;

// other Variables and consts
Servo servoBot;
Servo servoTop;
double servoPosition[2];
double servoStart[] = {servoBotMin, servoTopMin};
double servoMid[] = {((servoBotMax - servoBotMin) / 2) + servoBotMin, ((servoTopMax - servoTopMin) / 2) + servoTopMin};
double servoEnd[] = {servoBotMax, servoTopMax};
double acTimePos, acLat, acLon, acAlt, acOnGround, acVelocity, acTTrack, acVRate;
//
double bearingToAc;

// Functions

//Sets and moves the position of the servos in servoPosition[] according to start,mid,end
void setServosDef(String position = "NA")
{
  if (position == "start")
  {
    Serial.println("Servo pos  = start ");
    servoPosition[0] = servoStart[0];
    servoPosition[1] = servoStart[1];
  }
  else if (position == "mid")
  {
    Serial.println("Servo pos  = mid");
    servoPosition[0] = servoMid[0];
    servoPosition[1] = servoMid[1];
  }
  else if (position == "end")
  {
    Serial.println("Servo pos  = end");
    servoPosition[0] = servoEnd[0];
    servoPosition[1] = servoEnd[1];
  }
  else
  {
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
void setServosXY(int servoBotPosition = servoPosition[0], int ServoTopPosition = servoPosition[1])
{
  Serial.print("servo bot moved to:");
  Serial.println(servoBotPosition);
  Serial.print("servo top moved to:");
  Serial.println(ServoTopPosition);
  servoBot.writeMicroseconds(servoBotPosition);
  servoTop.writeMicroseconds(ServoTopPosition);
  delay(15);
}

void bootSequence()
{
  for (int pos = 500; pos <= servoBotMax; pos += 1)
  { // goes from 0 degrees to 180 degrees
    Serial.print("bootSequence  servoBot pos = >");
    Serial.println(pos);
    // in steps of 1 degree
    servoBot.writeMicroseconds(pos); // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  for (int pos = 500; pos <= servoTopMax; pos += 1)
  { // goes from 180 degrees to 0 degrees
    Serial.print("bootSequence  servoTop pos = >");
    Serial.println(pos);
    servoTop.writeMicroseconds(pos); // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
}

// Calculate bearing from servo to AC - Takes long lat as radians. Returns Bearing as int.
int calcBearing2Points(float lat1, float lon1, float lat2, float lon2)
{

  float Y = sin(lon2 - lon1) * cos(lat2);
  float X = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
  float deg = degrees(atan2(Y, X));
  // note that this implementation doesn't use the module, but angles lower than 0 get augmented by 360 only
  if (deg < 0)
  {
    deg = 360 + deg;
    Serial.print("calcBearing2points = less than zero");
  }
  float angle = deg;
  int a = (int)(abs(angle) + (1 / 7200));
  return a;
}

//Convert bearing from ADSB data to X co-ordinate for top servo
void bearingtoX(double bearing)
{
  if (bearing >= 270 && bearing < 359)
  {
    servoPosition[0] = 0;
    servoPosition[1] = servoStart[1] + (((int)bearing % 270) / q1Multiplier);
  }
  else if (bearing >= 0 && bearing < 89)
  {
    servoPosition[0] = 0;
    servoPosition[1] = servoMid[1] + (bearing * q2Multiplier);
  }
  else if (bearing >= 90 && bearing < 179)
  {
    servoPosition[0] = 180;
    servoPosition[1] = servoMid[1] + (((int)bearing % 90) / q3Multiplier);
  }
  else if (bearing >= 180 && bearing < 269)
  {
    servoPosition[0] = 180;
    servoPosition[1] = servoStart[1] + (((int)bearing % 180) / q4Multiplier);
    ;
  }
  else
    Serial.println("Bearing Out Of Bounds");
  Serial.print("calculated servoPosition[1] ======> ");
  Serial.println(servoPosition[1]);
  delay(15);
  if (servoPosition[1] >= servoTopMax)
  {
    servoPosition[1] = servoTopMax;
    Serial.println("Hit max");
  }
  if (servoPosition[1] <= servoTopMin)
  {
    servoPosition[1] = servoTopMin;
    Serial.println("Hit min");
  }
}

//Returns the distance between the servos and the Aircraft
double calcDistHomeToAc(double currentLat, double currentLon, double acLat, double acLon)
{
  double dx, dy, dz;
  currentLon -= acLon;
  radians(currentLon), radians(currentLat), radians(acLat);

  dz = sin(currentLat) - sin(acLat);
  dx = cos(currentLon) * cos(currentLat) - cos(acLat);
  dy = sin(currentLon) * cos(currentLat);
  return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * eaRa;
}

//returns km for m/s and seconds input
double calcDistance(double speedMsec, double timeSec)
{
  return speedMsec / timeSec;
}

//Estimate end point at 10 second mark (time between JSON calls) so vector can be made and tracked
void trackAC(double acLat, double acLon, double bearing, double distanceKM)
{
  double currentLat, currentLon, nextLat, nextLon;
  double nextDistanceKM = distanceKM / 10;
  double distanceInMetres = distanceKM * 1000;

  for (size_t j = 0; j <= timeBetweenJSONCalls; j++)
  {
    //set starting lat & lon as initial point then move over time

    float acLonRad = radians(acLon);
    float acLatRad = radians(acLat);
    float acBearingRad = radians(bearing);

    float nextLatRad = asin(sin(acLatRad) * cos(nextDistanceKM / eaRa) + cos(acLatRad) * sin(nextDistanceKM / eaRa) * cos(acBearingRad));
    float nextLonRad = acLonRad + atan2(sin(acBearingRad) * sin(nextDistanceKM / eaRa) * cos(acLatRad), cos(nextDistanceKM / eaRa) - sin(acLatRad) * sin(nextLatRad));
    nextLonRad = (nextLonRad + 3 * PI) / (2 * PI);
    int i = nextLonRad;
    nextLonRad = (nextLonRad - i) * (2 * PI) - PI; // normalise to -180..+180º

    //System.out.printf("starting at a Longitude of %f and a Latitude of %f ",acLon,acLat);
    Serial.print("starting at a Longitude of ");
    Serial.print(acLon, 6);
    Serial.print(" and a Latitude of ");
    Serial.println(acLat, 6);
    //System.out.printf("if we travel %f km on a bearing of %f degrees ",nextDistanceKM,bearing);
    Serial.print("if we travel ");
    Serial.print(nextDistanceKM, 6);
    Serial.print(" km on a bearing of ");
    Serial.print(bearing, 6);
    Serial.println(" degrees");
    //System.out.printf("we end up at Longitude of %f and a Latitude of %f ",degrees(nextLonRad),degrees(nextLatRad));
    Serial.print("we end up at a Longitude of ");
    Serial.print(degrees(nextLonRad), 6);
    Serial.print(" and a Latitude of ");
    Serial.println(degrees(nextLatRad), 6);

    bearingToAc = calcBearing2Points(homeLatRad, homeLonRad, nextLatRad, nextLonRad);

    Serial.print("bearingToAc :");
    Serial.println(bearingToAc);
    //Convert and store bearing to X coord for Servo
    bearingtoX(bearingToAc);

    Serial.print("moving servos to bearing from servo");
    Serial.println(servoPosition[0]);
    Serial.println(servoPosition[1]);
    Serial.println("");
    //Move servo to next point
    setServosXY();
    delay(1000);
    nextDistanceKM = nextDistanceKM + (distanceKM / 10);
  }
}

void setup()
{
  Serial.begin(115200);
  WiFiManager wifiManager;
  wifiManager.autoConnect("ADS-B Tracker");
  Serial.println("Connected!");
  pinMode(laserPin, OUTPUT); // turning the laser on
  servoBot.attach(4);        //Digital 2
  servoTop.attach(5);        //Digital 1
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

  const char *json = "{\"time\":1574661430,\"states\":[[\"7c6ae1\",\"VEB     \",\"Australia\",1574661430,1574661430,145.1089,-37.5592,2339.34,false,93.35,30.1,6.83,null,2331.72,\"3673\",false,0]]}";

  deserializeJson(doc, json);

  long time = doc["time"]; // 1574661430

  JsonArray states_0 = doc["states"][0];
  acTimePos = states_0[3];  // 1574661430
  acLon = states_0[5];      // 145.1089
  acLat = states_0[6];      // -37.5592
  acAlt = states_0[7];      // 2339.34
  acOnGround = states_0[8]; // false
  acVelocity = states_0[9]; // 93.35
  acTTrack = states_0[10];  // 30.1
  acVRate = states_0[11];   // 6.83

  trackAC(acLat, acLon, acTTrack, calcDistance(acVelocity, 10));

}
