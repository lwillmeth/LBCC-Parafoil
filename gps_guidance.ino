#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>

#define steerPin 11
#define launchPin 10
#define GPS_RX 8
#define GPS_TX 7
#define LZButton 5
#define LZHoldDelay 3000

// The serial connection to the GPS device
SoftwareSerial satData(GPS_RX, GPS_TX);
TinyGPSPlus gps;
Servo steer;
Servo launch;
  
// The default LZ is the hurdle pit near the Albany LBCC track
double LZ_LAT = 44.586337;
double LZ_LNG = -123.117409;

// Timestamp the LZ button was most recently pressed.
unsigned long LZStartHold = millis();

void setup(){
  Serial.begin(115200);
  satData.begin(9600);
  
  // Prepare servos w/ (pin, min millis, max millis)
  steer.attach(steerPin,1000,2000);
  launch.attach(launchPin,1000,2000);
  // The arduino can use a 20k built-in pullup resistor on digital I/O pins.
  pinMode(LZButton, INPUT_PULLUP);
  
  Serial.println("LBCC Spring 2014 Parafoil v1.2");
  Serial.print("By default, steering towards: ");
  Serial.print(LZ_LAT,5);
  Serial.print(", ");
  Serial.println(LZ_LNG,5);
  
  dropTimer(1000);
}

void loop(){
  // Continually interpret the incoming gps data stream
  while (satData.available() > 0){
    gps.encode(satData.read());
  }
  // Only perform these actions if the GPS is locked
  if(gps.location.isValid()){
    // If LZ button is pressed, set LZ location
    if(digitalRead(LZButton)==LOW){
      setLZ();
    }
    // Recalculate course only if GPS info has changed
    if(gps.location.isUpdated()){
      setCourse();
    }
  }
}

void dropTimer(unsigned long wait){
  // Used to trigger drop conditions based on time or altitude.
  Serial.println("Waiting to drop...");
  delay(wait);
  launch.write(180);
  Serial.println("Dropped.  Beginning navigation...");
  // Launch is a one-time event, so from now on we can ignore the servo.
  launch.detach();
}

void setLZ(){
  // I'm not entirely happy with this code and it should be rewritten.
  if( (millis()-LZStartHold) > (LZHoldDelay+1000) ){
    // If button press is older than x+1 seconds, reset timer.
    LZStartHold = millis();
  }else if( (millis()-LZStartHold) > LZHoldDelay){
    // If button press is older than x seconds, set current location as LZ and reset timer.
    LZStartHold = millis();
    LZ_LAT = gps.location.lat();
    LZ_LNG = gps.location.lng();
    // Serial.print rounds floats/doubles to ints unless we specify decimal length.
    Serial.print("Setting new target LZ to: ");
    Serial.print(LZ_LAT,5);
    Serial.print(", ");
    Serial.println(LZ_LNG,5);
  }
}

void setCourse(){
  // Find distance from current location, to LZ
  double distanceToDestination = TinyGPSPlus::distanceBetween(
    gps.location.lat(), gps.location.lng(), LZ_LAT, LZ_LNG);
  // Find heading in degrees, from current location, to LZ
  double courseToDestination = TinyGPSPlus::courseTo(
    gps.location.lat(), gps.location.lng(), LZ_LAT, LZ_LNG);
  // Calculate course correction needed using current heading and courseToDestination
  int courseChange = (int)(360+courseToDestination-gps.course.deg())%360;
  Serial.print("Course: ");
  Serial.print(gps.course.deg());
  Serial.print(" courseToDest: ");
  Serial.print(courseToDestination);
  Serial.print(" courseChange: ");
  Serial.print(courseChange);
  // Adjust courseChange because compass degrees run clockwise from 270=W, 0=N, 90=E
  // The servo uses math degrees from 180=W, 90=N, 0=E.
  if(courseChange>330 || courseChange<30)
    courseChange = 90; // close enough, go straight.
  else if(courseChange > 270)
    courseChange = 135; // slight left
  else if(courseChange > 180)
    courseChange = 180; // hard left turn
  else if(courseChange > 90)
    courseChange = 0; // hard right turn
  else
    courseChange = 45; // slight right
  Serial.print(" servo: ");
  Serial.print(courseChange);
  Serial.print(" distance: ");
  Serial.println(distanceToDestination);
  steer.write(courseChange);
}
