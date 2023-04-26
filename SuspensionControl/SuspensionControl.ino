#include "Wire.h"               // Include wire library
#include <Servo.h>              // Include servo library
#include <MPU6050_light.h>      // Include MPU 6050 library (light version used due to higher speed quoted)
#include <SD.h>                 // Include SD card library

// Assign MPU 6050 to use Wire
MPU6050 mpu(Wire);


// Variables to store data from MPU-6050  // Note: Use of int to reduce sensitivity of data causing steady state oscillation due to over sensitivity
int8_t xp;                         // Create value for Raw Pitch
int8_t yp;                         // Create value for Raw Roll
int8_t zp;                         // Create value for Raw Yaw
float xa, ya, za;                  // Create floating point variable values for longitudinal, lateral and vertical acceleration
float xpf, ypf;                    // Create floating point variable values

unsigned long  CurrentTime;        // Create 32 bit value for relative time keeper
unsigned long startMillis;         // Create 32 bit value for some time keeper for integration
uint8_t edition;                   // Counter to ensure datalogger doesnt overlap
boolean sussys;                    // Boolean value to check and ensure system is ready to run

uint16_t FL_pot_pin;   // Pin for FL Rotary Potentiometer
uint16_t FR_pot_pin;   // Pin for FR Rotary Potentiometer
uint16_t RL_pot_pin;   // Pin for RL Rotary Potentiometer
uint16_t RR_pot_pin;   // Pin for RR Rotary Potentiometer

const uint8_t FL_dec_pin = 10;  // Pin to decrease FL Position
const uint8_t FL_inc_pin = 11;  // Pin to increase FL Position

const uint8_t FR_dec_pin = 4;  // Pin to decrease FR Position
const uint8_t FR_inc_pin = 5;  // Pin to increase FR Position

const uint8_t RL_dec_pin = 6;  // Pin to decrease RL Position
const uint8_t RL_inc_pin = 7;  // Pin to increase RL Position

const uint8_t RR_dec_pin = 8;  // Pin to decrease RR Position
const uint8_t RR_inc_pin = 9;  // Pin to increase RR Position

const uint8_t roll_multi = 10; // Multiplier for roll stiffness
const uint8_t pitch_multi = 5; // Multiplier for pitch stiffness
const uint8_t heave_multi = 20;// Multiplier for heave stiffness

const uint8_t p = 1;           // Multiplier for the Spring Stiffness
const uint8_t d = 4;           // Multiplier for the Damping Effect

uint8_t oldXp = 0;
uint8_t oldXpf = 0;
uint8_t oldYp = 0;
uint8_t oldYpf = 0;

uint16_t FL_setpoint = 1500;
uint16_t FR_setpoint = 1500;
uint16_t RL_setpoint = 1500;
uint16_t RR_setpoint = 1500;

int8_t FL_prev_error = 0;
int8_t FR_prev_error = 0;
int8_t RL_prev_error = 0;
int8_t RR_prev_error = 0;

uint16_t FL_target = FL_setpoint;
uint16_t FR_target = FR_setpoint;
uint16_t RL_target = RL_setpoint;
uint16_t RR_target = RR_setpoint;

// Low Pass Filter constants
const int8_t k1 = 55;              // Create constant value for Constant for Previous Filtered Value
const int8_t k2 = 10;              // Create constant value for Constant for Current Raw Value
const int8_t k3 = 35;              // Create constant value for Constant for Previous Raw Value
const int8_t k4 = 100;             // Create constant value for Constant for Divisor to make other coefficients percentages

const int8_t kPP = 150;            // Assign Proportional constant for pitch
const int8_t kPR = 100;            // Assign Proportional constant for roll

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
}

void loop() {
  if (sussys == true) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
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

    // Filtered Values
    xpf = oldXp * (k1 / k4) + xp * (k2 / k4) + oldXpf * (k3 / k4);  //roll
    ypf = oldYp * (k1 / k4) + yp * (k2 / k4) + oldYpf * (k3 / k4);  //pitch

    // Create Target Values for all four corners
    FL_target = FL_target + (roll_multi * xpf) + (pitch_multi * ypf);
    FR_target = FR_target - (roll_multi * xpf) + (pitch_multi * ypf);
    RL_target = RL_target + (roll_multi * xpf) - (pitch_multi * ypf);
    RR_target = RR_target - (roll_multi * xpf) - (pitch_multi * ypf);

    // Compute errors, PD terms and reactions
    int FL_ang = analogRead(A0);
    int FL_error = FL_target - FL_ang;
    int FL_react = FL_error * p + (FL_error - FL_prev_error) * d;
    FL_prev_error = FL_error;

    int FR_ang = analogRead(A1);
    int FR_error = FR_target - FR_ang;
    int FR_react = FR_error * p + (FR_error - FR_prev_error) * d;
    FR_prev_error = FR_error;

    int RL_ang = analogRead(A2);
    int RL_error = RL_target - RL_ang;
    int RL_react = RL_error * p + (RL_error - RL_prev_error) * d;
    RL_prev_error = RL_error;

    int RR_ang = analogRead(A3);
    int RR_error = RR_target - RR_ang;
    int RR_react = RR_error * p + (RR_error - RR_prev_error) * d;
    RR_prev_error = RR_error;

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
  else {
    Serial.print("Error Occured");
  }
}
