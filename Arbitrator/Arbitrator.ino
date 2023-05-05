#include "Wire.h"               // Include wire library
#include <Servo.h>              // Include servo library
#include <MPU6050_light.h>      // Include MPU 6050 library (light version used due to higher speed quoted)
#include <SD.h>                 // Include SD card library


Servo left_steer;   // Create servo object for left steer
Servo right_steer;  // Create servo object for right steer
Servo fl_esc;    // Create servo object for front left esc
Servo fr_esc;    // Create servo object for front right esc
Servo rl_esc;    // Create servo object for rear left esc
Servo rr_esc;    // Create servo object for rear right esc
Servo drs_stability;      // Create servo object for variable wheelbase
// Servo aero_regera;  // Create servo object to raise active aero wing
// Servo aero_wing;    // Create servo object to actuate wing angle
Servo edf_aero;     // Create servo object for active edf aero

unsigned long  CurrentTime;        // Create 32 bit value for relative time keeper
unsigned long startMillis;         // Create 32 bit value for some time keeper for integration
uint8_t edition;                   // Counter to ensure datalogger doesnt overlap

// Assign MPU 6050 to use Wire
MPU6050 mpu(Wire);

//Pin Allocations
const uint8_t ch1_pin = 1;    // Steering Wheel
const uint8_t ch2_pin = 2;    // Throttle and Brake Pedal
const uint8_t ch3_pin = 3;    // is the middle finger button press for drs
const uint8_t ch4_pin = 4;    // Three position switch to choose between bmig settings
const uint8_t ch5_pin = 5;    // rotary to change between pedal maps if vCar = 0 & xa<0.1 and bbal in other scenarios
const uint8_t ch6_pin = 6;    // rotary to change gyro settings

const uint8_t left_steer_pin = 6;
const uint8_t right_steer_pin = 7;
const uint8_t vary_wb_pin = 11; // can vary_wb_pin and aero_pin be integrated?
const uint8_t FL_dec_pin = 10;  // Pin to decrease FL Position
const uint8_t FL_inc_pin = 11;  // Pin to increase FL Position

const uint8_t FR_dec_pin = 4;  // Pin to decrease FR Position
const uint8_t FR_inc_pin = 5;  // Pin to increase FR Position

const uint8_t RL_dec_pin = 6;  // Pin to decrease RL Position
const uint8_t RL_inc_pin = 7;  // Pin to increase RL Position

const uint8_t RR_dec_pin = 8;  // Pin to decrease RR Position
const uint8_t RR_inc_pin = 9;  // Pin to increase RR Position
uint16_t FL_pot_pin;   // Pin for FL Rotary Potentiometer
uint16_t FR_pot_pin;   // Pin for FR Rotary Potentiometer
uint16_t RL_pot_pin;   // Pin for RL Rotary Potentiometer
uint16_t RR_pot_pin;   // Pin for RR Rotary Potentiometer

// Variables to store data from MPU-6050  // Note: Use of int to reduce sensitivity of data causing steady state oscillation due to over sensitivity
int8_t nPitch;                     // Create value for Raw Pitch
int8_t nRoll;                      // Create value for Raw Roll
int8_t nYaw;                       // Create value for Raw Yaw
float gLong, gLat, gVert;          // Create floating point variable values for longitudinal, lateral and vertical acceleration
float fnPitch, fnRoll;                    // Create floating point variable values

void setup() {
  pinMode(FL_pot_pin, INPUT);
  pinMode(FR_pot_pin, INPUT);
  pinMode(RL_pot_pin, INPUT);
  pinMode(RR_pot_pin, INPUT);

  pinMode(FL_dec_pin, OUTPUT);
  pinMode(FL_inc_pin, OUTPUT);
  digitalWrite(FL_inc_pin, HIGH);
  digitalWrite(FL_dec_pin, HIGH);

  pinMode(FR_dec_pin, OUTPUT);
  pinMode(FR_inc_pin, OUTPUT);
  digitalWrite(FR_inc_pin, HIGH);
  digitalWrite(FR_dec_pin, HIGH);

  pinMode(RL_dec_pin, OUTPUT);
  pinMode(RL_inc_pin, OUTPUT);
  digitalWrite(FL_inc_pin, HIGH);
  digitalWrite(FL_dec_pin, HIGH);

  pinMode(RR_dec_pin, OUTPUT);
  pinMode(RR_inc_pin, OUTPUT);
  digitalWrite(RR_inc_pin, HIGH);
  digitalWrite(RR_dec_pin, HIGH);
  
  // Assign Servos to pins
  left_steer.attach(left_steer_pin);                 //output left steering actuation to output 4
  left_steer.writeMicroseconds(1000);                //initialise signal to 1 millisecond
  right_steer.attach(right_steer_pin);               //output right steering actuator to output 5
  right_steer.writeMicroseconds(1000);               //initialise signal to 1 millisecond 
  vary_wb.attach(vary_wb_pin);                       //output wheelbase actuation to output 9
  vary_wb.writeMicroseconds(1000);                   //initialise signal to 1 millisecond
//  aero.attach(aero_pin);                             //output aero actuation to output 10
//  aero.writeMicroseconds(1000);                      //initialise signal to 1 millisecond

}

void loop() {
    CurrentTime = millis(); // task rate initialiser
    mpu.update();

    // make a string for assembling the data to log:
    String dataString = "";

    // read both sensors on MPU-6050
    // Obtain Angle from Accelerometer and Gyroscope
    nPitch = mpu.getAngleX();
    nRoll = mpu.getAngleY();
    nYaw = mpu.getAngleZ();
    // Obtain Acceleration from Accelerometer
    gLong = mpu.getAccX();
    gLat = mpu.getAccY();
    gVert = mpu.getAccZ();
    // Account for Gravity
    gVert = gVert - 1;
    
    ch1 = pulseIn(ch1_pin, HIGH); //channel 1 value based on pin D1 input, expecting High values with no timeout
    ch2 = pulseIn(ch2_pin, HIGH); //channel 2 value based on pin D2 input, expecting High values with no timeout
    ch3 = pulseIn(ch3_pin, HIGH); //channel 3 value based on pin D3 input, expecting High values with no timeout
    ch4 = pulseIn(ch4_pin, HIGH); //channel 4 value based on pin D4 input, expecting High values with no timeout 
    ch5 = pulseIn(ch5_pin, HIGH); //channel 5 value based on pin D5 input, expecting High values with no timeout
    ch6 = pulseIn(ch6_pin, HIGH); //channel 6 value based on pin D6 input, expecting High values with no timeout 

    rThrottlePedal = map(ch2,1500,2000,0,100);  //maps Throttle Actuation from 0 to full from channel 2
    constrain(rThrottlePedal,0,100);
    rBrakePedal = map(ch2,1500,1000,0,100);  //maps Brake Actuation from 0 to full from channel 2
    constrain(rBrakePedal,0,100);
    aSteeringWheel = map(ch1,1000,2000,100,-100); // -ve = left; +ve is right
    constrain(aSteeringWheel,-100,100);

    // Apply PWM according to reaction
    if (FL_react > 0) {
      // increase
      analogWrite(FL_inc_pin, abs(FL_react));
      digitalWrite(FL_dec_pin, LOW);
    }
    else if (FL_react < 0) {
      // decrease
      analogWrite(FL_dec_pin, abs(FL_react));
      digitalWrite(FL_inc_pin, LOW);
    }
    else {
      digitalWrite(FL_inc_pin, HIGH);
      digitalWrite(FL_dec_pin, HIGH);
    }

    if (FR_react > 0) {
      // increase
      analogWrite(FR_inc_pin, abs(FR_react));
      digitalWrite(FR_dec_pin, LOW);
    }
    else if (FR_react < 0) {
      // decrease
      analogWrite(FR_dec_pin, abs(FR_react));
      digitalWrite(FR_inc_pin, LOW);
    }
    else {
      digitalWrite(FR_inc_pin, HIGH);
      digitalWrite(FR_dec_pin, HIGH);
    }

    if (RL_react > 0) {
      // increase
      analogWrite(RL_inc_pin, abs(RL_react));
      digitalWrite(RL_dec_pin, LOW);
    }
    else if (RL_react < 0) {
      // decrease
      analogWrite(RL_dec_pin, abs(RL_react));
      digitalWrite(RL_inc_pin, LOW);
    }
    else {
      digitalWrite(RL_inc_pin, HIGH);
      digitalWrite(RL_dec_pin, HIGH);
    }

    if (RR_react > 0) {
      // increase
      analogWrite(RR_inc_pin, abs(RR_react));
      digitalWrite(RR_dec_pin, LOW);
    }
    else if (RR_react < 0) {
      // decrease
      analogWrite(RR_dec_pin, abs(RR_react));
      digitalWrite(RR_inc_pin, LOW);
    }
    else {
      digitalWrite(RR_inc_pin, HIGH);
      digitalWrite(RR_dec_pin, HIGH);
    }
  }
  left_steer.writeMicroseconds(l_steer);
  right_steer.writeMicroseconds(r_steer);
  drs_stability.writeMicroseconds(wheelbase_pwm);
  edf_aero.writeMicroseconds(aero_pwm);
}
