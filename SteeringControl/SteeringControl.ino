#include "Wire.h"               // Include wire library
#include <Servo.h>              // Include servo library
#include <MPU6050_light.h>      // Include MPU 6050 library (light version used due to higher speed quoted)
#include <SD.h>                 // Include SD card library


const uint8_t aWheelFMap = 120;
boolean BUsenYawforaUOSteer = 1;

const uint8_t aSteeringDeadband = 50;
  
int8_t nTurnLorR = 0;                // extract the sign from the steering wheel -ve left, +ve right
uint16_t wheelbase = 380;        // integer for the wheelbase of the vehicle
uint8_t aWheelFMax = 50;         // integer for the max steering angle at the wheels

// Intergers for Independent Variable Steering
int ch1_pwm = 0;   // integer for ch1 delta
float ang,theta,phi;       // floating point number for angle, inside angle and outside angle
int theta_pwm = 1500; // integer for inside angle pwm
int phi_pwm = 1500;   // integer for outside angle pwm
int l_steer = 1500;   // integer for left steering
int r_steer = 1500;   // integer for right steering

void setup() {
}

void loop() {
// Independent Variable Steering
{
  ch1_pwm = abs(ch1-1500);
  phi = map(ch1_pwm,0,500,0,aWheelFMax);
  constrain(phi,0,aWheelFMax);
  theta = atan((wheelbase*tan(phi))/(wheelbase-(fronttrackwidth*tan(phi))));
  phi_pwm = map(phi,0,aWheelFMax,1000,2000);
  theta_pwm = map(theta,0,aWheelFMax,1000,2000);
  if (ch1 < 1500 - aSteeringDeadband) {                 //when turning right
    l_steer = 1500 - phi_pwm;
    r_steer = 1500 - theta_pwm;
  }
  else if (ch1 > 1500+ aSteeringDeadband) {             //when turning left
    l_steer = theta_pwm + 1500;
    r_steer = phi_pwm + 1500;
  }
  else
    l_steer = ch1;
    r_steer = ch1;
}
if (aSteeringWheel < -1){
  nTurnLorR = -1
}
elseif (aSteeringWheel > 1){
  nTurnLorR = 1
}
else {
  nTurnLorR = 0
}

// Calculated Channels
  aWheelFL = map(l_steer,1000,2000,0,aWheelFMax);
  awheelFR = map(r_steer,1000,2000,0,aWheelFMax);


if (BUsenYawforaUOSteer = 1){
  angMeasured = nYaw;
}
else{
  angMeasured = gLat;
}

// calc for aUOSteer
// once calc is generated, will be used to close the loop to maximise rotation and target neutral steer
  aUOSteer = (((wheelbase*angMeasured)/vCar)-((aWheelFL+aWheelFR)/2))*nTurnLorR;
  
}
