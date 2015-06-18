/* MeArm IK joysticks - York Hackspace May 2014
 * For use with MeBrain and joystick board.
 * Adapted from original joystick control code by Daniel Bailey.
 *
 * The left stick moves gripper forwards, backwards, left and right
 * The right stick moves gripper up, down, and closes and opens.
 * 
 */

#include "meArm.h"
#include <Servo.h>

#define MAXMAG(x,y) ((abs(x)>abs(y))?x:y)

int basePin = 5;
int shoulderPin = 6;
int elbowPin = 9;
int gripperPin = 10;

int tdirPin = 1;
int rdirPin = 0;//
int zdirPin = 2;
int tdirPin2 = 3;

int gripperButton = 14;
int gripperButton2 = 16;

meArm arm(
      180,21, -pi/2, pi/2,    //Base     (-90 to  90 degrees)
      99,12, pi/4, 3*pi/4,   //Shoulder ( 45 to 135 degrees)
      174,47, 0.332, -pi/2,      //Elbow    (  something to -90 degrees)
      135,168, pi/2, 0);        //Gripper  ( 90 to   0 degrees)

bool gripperClosed = false;

long lastGripperSet = millis();

void setGripper(bool closed){
  gripperClosed = closed;
  if (gripperClosed){
    arm.closeGripper();
  }
  else{
    arm.openGripper();
  }
}

void setup() {
  arm.begin(basePin, shoulderPin, elbowPin, gripperPin);
  
  //Input buttons
  pinMode(gripperButton, INPUT);
  pinMode(gripperButton2, INPUT);  
  //With pull up resistors
  digitalWrite(gripperButton, HIGH);
  digitalWrite(gripperButton2, HIGH);  

  setGripper(gripperClosed);
}

#define ANGLESCALE 0.002
#define SCALE 0.2

#define ZMIN (-40)
#define ZMAX (130)

#define TMIN (-pi/2)
#define TMAX (pi/2)

#define RMIN (100)
#define RMAX (230)

#define THRESHOLD (5.0)
#define STICKSCALE (10.0)

//#define MIN_SPHERE_RADIUS (100)

bool gripperButtonPressed(){
  return (digitalRead(gripperButton) == LOW) || (digitalRead(gripperButton2) == LOW);
}


void loop() {
  float dt = map(analogRead(tdirPin), 0, 1023, STICKSCALE, -STICKSCALE);
  float dr = map(analogRead(rdirPin), 0, 1023, -STICKSCALE, STICKSCALE);
  float dz = map(analogRead(zdirPin), 0, 1023, STICKSCALE, -STICKSCALE);
  float dt2 = map(analogRead(tdirPin2), 0, 1023, STICKSCALE, -STICKSCALE);
  if (abs(dt) < THRESHOLD) dt = 0;
  if (abs(dr) < THRESHOLD) dr = 0;
  if (abs(dz) < THRESHOLD) dz = 0;
  if (abs(dt2) < THRESHOLD) dt2 = 0;
  
  if (dt>0.001) {dt -= THRESHOLD;} if (dt<-0.001) {dt += THRESHOLD;}
  if (dr>0.001) {dr -= THRESHOLD;} if (dr<-0.001) {dr += THRESHOLD;}
  if (dz>0.001) {dz -= THRESHOLD;} if (dz<-0.001) {dz += THRESHOLD;}
  if (dt2>0.001) {dt2 -= THRESHOLD;} if (dt2<-0.001) {dt2 += THRESHOLD;}
    
  dt *= ANGLESCALE;
  dr *= SCALE;
  dz *= SCALE;
  dt2 *= ANGLESCALE;
  
  if (!(dt == 0 && dr == 0 && dz == 0 && dt2 == 0)){
    
    float newTheta = arm.getTheta() + (MAXMAG(dt, dt2));
    if (newTheta > TMAX){ newTheta = TMAX; }
    if (newTheta < TMIN){ newTheta = TMIN; }
    
    float newZ = arm.getZ() + dz;
    if (newZ > ZMAX) {newZ = ZMAX;}
    if (newZ < ZMIN) {newZ = ZMIN;}
    
    float newR = arm.getR() + dr;
    if (newR > RMAX) {newR = RMAX;}
    if (newR < RMIN) {newR = RMIN;}
    
    //if (((newR*newR) + (newZ*newZ)) > (MIN_SPHERE_RADIUS*MIN_SPHERE_RADIUS)){ //Don't get within MIN_SPHERE_RADIUS of the centre
      arm.goDirectlyToCylinder( newTheta , newR, newZ);
    //}
    
  }
 
  if (gripperButtonPressed() && ((lastGripperSet + 100) < millis())){
    setGripper(!gripperClosed);
    lastGripperSet = millis();
  }
  
  Serial.print("XYZ: ");
  Serial.print(arm.getX());
  Serial.print(", ");
  Serial.print(arm.getY());
  Serial.print(", ");
  Serial.print(arm.getZ());
  Serial.print(" - R: ");
  Serial.print(arm.getR());  
  Serial.print(", theta: ");
  Serial.print(arm.getTheta());
  Serial.print(" - distance from centre: ");
  float dist = sqrt((arm.getR()*arm.getR())+(arm.getZ()*arm.getZ()));
  Serial.println(dist);
  delay(5);
}

