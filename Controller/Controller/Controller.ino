#include "Wire.h"               // Include wire library
#include <Servo.h>              // Include servo library
#include <MPU6050_light.h>      // Include MPU 6050 library (light version used due to higher speed quoted)

// Assign MPU 6050 to use Wire
MPU6050 mpu(Wire);

const uint8_t ch1_pin = 1;
const uint8_t ch2_pin = 2;
const uint16_t BrakingThresholdPWM = 1475;

// Variables to store data from MPU-6050  // Note: Use of int to reduce sensitivity of data causing steady state oscillation due to over sensitivity
int8_t xp;                         // Create value for Raw Pitch
int8_t yp;                         // Create value for Raw Roll
int8_t zp;                         // Create value for Raw Yaw
float xa, ya, za;                  // Create floating point variable values for longitudinal, lateral and vertical acceleration
float xpf, ypf;                    // Create floating point variable values

Servo fl_esc;    // Create servo object for front left esc
Servo fr_esc;    // Create servo object for front right esc
Servo rl_esc;    // Create servo object for rear left esc
Servo rr_esc;    // Create servo object for rear right esc

void setup() {
  // put your setup code here, to run once:

}

void loop() {
    mpu.update();

    // make a string for assembling the data to log:
    String dataString = "";

    // read both sensors on MPU-6050
    // Obtain Angle from Accelerometer and Gyroscope
    xp = mpu.getAngleX();
    yp = mpu.getAngleY();
    zp = mpu.getAngleZ();
    // Obtain Acceleration from Accelerometer
    xa = mpu.getAccX();
    ya = mpu.getAccY();
    za = mpu.getAccZ();
    // Account for Gravity
    za = za - 1;

// Active Front Differential
{
    if ch2 < BrakingThresholdPWM
      CornerState = 1; // Braking
    end
    if abs(ya) > CornerLatThresh && CornerState = 1
      CornerState = 2;  // Entry
    end
    if ch2 > Accelerating ThresholdPWM && CornerState = 2
      CornerState = 3;  // Exit
    end
    if abs(ya) < CornerLatThresh && CornerState = 3
      CornerState = 4; // Straight
    end
    rThrottlePedal = map(ch2,1500,2000,0,100);  //maps Throttle Actuation from 0 to full from channel 2
    constrain(rThrottlePedal,0,100);
    rBrakePedal = map(ch2,1500,1000,0,100);  //maps Brake Actuation from 0 to full from channel 2
    constrain(rBrakePedal,0,100);

// Got to here with the code, now to populate the COrner State and Behaviours

    if ((ch1) <= 1450) {
      r_visco_map = map(ch1,1000,1450,viscosity,0);   //if steering right, map viscosity to turn and take away power from inner (right) wheel
    }
    else if ((ch1) >= 1550) {
      l_visco_map = map(ch1,1550,2000,0,viscosity);   //if steering left, map viscosity to turn and take away power from inner (left) wheel
    }
    else {
      l_visco_map = 0;    
      r_visco_map = 0;    //else 0 viscosity so no power take off
    }
    if ((ch2)<=1500){
      l_pwr=1000;         //if statement limiting lower limit to 1000 for output
      r_pwr=1000;
    }
    if ((ch2)>=2000||(l_pwr)>=2000||(r_pwr)>=2000) {
      l_pwr=2000;         //if statement limiting upper limit to 2000 for output
      r_pwr=2000;
    }
  l_pwr = left - l_visco_map;
  r_pwr = right - r_visco_map;
}
}
