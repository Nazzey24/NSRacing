#include "Wire.h"               // Include wire library
#include <Servo.h>              // Include servo library
#include <MPU6050_light.h>      // Include MPU 6050 library (light version used due to higher speed quoted)
#include <SD.h>                 // Include SD card library


const uint8_t aWheelFMap = 120;
boolean BUsenYawforaUOSteer = 1


// Dynamic Vehicle Parameters
int8_t pitch;                    //create integer for pitch of vehicle
int8_t roll;                     //create integer for roll of vehicle
int8_t yaw;                      //create integer for yaw of vehicle
int8_t nTurnLorR;
uint16_t wheelbase = 380;   //create integer for the wheelbase of the vehicle

// Intergers for Independent Variable Steering
int ch1_pwm;   //create integer for ch1 delta
int ang;       //create integer angle
int theta;     //create integer for inside angle
int phi;       //create integer for outside angle
int theta_pwm; //create integer for inside angle pwm
int phi_pwm;   //create integer for outside angle pwm
int l_steer;   //create integer left steering
int r_steer;   //create integer right steering


void setup() {
}

void loop() {
// Independent Variable Steering
{
  ch1_pwm = abs(ch1-1500);
  aWheelFRoad = map(ch1_pwm,0,500,0,aWheelFMap/2);
  phi = ang;
  theta = atan((wheelbase*tan(phi))/(wheelbase-(fronttrackwidth*tan(phi))));
  if (ch1 < 1400) {                 //when turning right
    l_steer = 1500 - phi_pwm;
    r_steer = 1500 - theta_pwm;
  }
  else if (ch1 > 1600) {             //when turning left
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
if (BUsenYawforaUOSteer = 1){
  angMeasured = nYaw;
}
else{
  angMeasured = gLat;
}

}
