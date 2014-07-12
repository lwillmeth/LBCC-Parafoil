#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <PWMServo.h>

#define steerPin 10
#define launchPin 9
#define GPS_RX 8
#define GPS_TX 7
#define LZButton 5
#define LZHoldDelay 3000
#define LAUNCHDELAY 10 // launch time delay in seconds

// The serial connection to the GPS device
SoftwareSerial satData(GPS_RX, GPS_TX);
TinyGPSPlus gps;

PWMServo steer;
PWMServo launch;

// The default LZ is the hurdle pit near the Albany LBCC track
double LZ_LAT = 44.586337;
double LZ_LNG = -123.117409;

boolean flying = false;
unsigned int courseChange;
unsigned int curCourse;
// Timestamp the LZ button was most recently pressed.
unsigned long LZStartHold = millis();

void setup(){
  Serial.begin(115200);
  satData.begin(9600);
  
  steer.attach(steerPin);
  steer.write(90);
  launch.attach(launchPin);
  // Set launch servo to 'closed' position in case arduino resets while being lifted.
  launch.write(80);
  
  // The arduino can use a 20k built-in pullup resistor on digital I/O pins.
  pinMode(LZButton, INPUT_PULLUP);

  Serial.println("LBCC Spring 2014 Parafoil v1.3");
  Serial.print("By default, steering towards: ");
  Serial.print(LZ_LAT,5);
  Serial.print(", ");
  Serial.println(LZ_LNG,5);
}

void loop(){
  // Continually interpret the incoming gps data stream
  while (satData.available() > 0){
    gps.encode(satData.read());
  }
  if(flying && gps.location.isUpdated()){
    // Flying and new GPS readings? Update the course.
    setCourse();
  }else if(!flying && digitalRead(LZButton)==LOW){
    // Pre-launch, and button is pressed.
    // This code should be rewritten.
    if( (millis()-LZStartHold) > (LZHoldDelay+1000) ){
      // If button press is older than x+1 seconds, reset timer.
      LZStartHold = millis();
    }else if(gps.location.isValid() && (millis()-LZStartHold) > LZHoldDelay){
      // Set current location as the new LZ.
      setLZ();
    }else{
      // Button pressed but not long enough to set LZ, means cycle launch gate.
      launch.write(0); // open
      delay(3000); // time to leave the gate open in milliseconds
      launch.write(80); // close
    }
  }else if(!flying && (millis()-LZStartHold)>(LAUNCHDELAY*1000) ){
    // Meets condition to launch
    drop();
    flying = true;
  }
}

void drop(){
  // Used to trigger drop conditions based on time or altitude.
  launch.write(0);
  // Launch is a one-time event, so from now on we can ignore the servo.
  delay(100); // prevents servo from being detached while moving to position.
  launch.detach();
}

void setLZ(){
  // Set the target landing area to our current coordinates.
  LZStartHold = millis();
  LZ_LAT = gps.location.lat();
  LZ_LNG = gps.location.lng();
  // Serial.print() rounds floats/doubles to ints unless we specify decimal length.
  Serial.print("Setting new target LZ to: ");
  Serial.print(LZ_LAT,5);
  Serial.print(", ");
  Serial.println(LZ_LNG,5);
}

void setCourse(){
  // Find distance from current location, to LZ
  double distanceToDestination = TinyGPSPlus::distanceBetween(
    gps.location.lat(), gps.location.lng(), LZ_LAT, LZ_LNG);
  // Find heading in degrees, from current location, to LZ
  double courseToDestination = TinyGPSPlus::courseTo(
    gps.location.lat(), gps.location.lng(), LZ_LAT, LZ_LNG);
  // Calculate course correction needed using current heading and courseToDestination
  courseChange = (int)(360+courseToDestination-gps.course.deg())%360;
  Serial.print("Course: ");
  Serial.print(gps.course.deg());
  Serial.print(" courseToDest: ");
  Serial.print(courseToDestination);
  Serial.print(" courseChange: ");
  Serial.print(courseChange);
  // Adjust courseChange because compass degrees run clockwise from 270=W, 0=N, 90=E
  // The servo uses math degrees counterclockwise from 180=W, 90=N, 0=E.
  if(courseChange>330 || courseChange<30)
    courseChange = 90; // close enough, go straight.
  else if(courseChange > 270)
    courseChange = 135; // slight left
  else if(courseChange > 180)
    courseChange = 170; // hard left turn
  else if(courseChange > 90)
    courseChange = 10; // hard right turn
  else
    courseChange = 45; // slight right
  if(courseChange != curCourse){
    steer.write(courseChange);
  }
  Serial.print(" servo: ");
  Serial.print(courseChange);
  Serial.print(" distance: ");
  Serial.println(distanceToDestination);
}
