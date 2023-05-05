boolean sussys;                    // Boolean value to check and ensure system is ready to run
uint8_t NSuspState = 0;            // Configurable for Suspension State
// I do want to use NCornerState to change the through corner balance and compliance

const uint8_t roll_k = 4;         // Multiplier for roll stiffness
const uint8_t pitch_k = 6;        // Multiplier for pitch stiffness
const uint8_t heave_k= 4;         // Multiplier for heave stiffness
const uint8_t roll_k_driver = 2;  // Multiplier for roll stiffness driver
const uint8_t pitch_k_driver = 2; // Multiplier for pitch stiffness driver
const uint8_t roll_d = 3;         // Multiplier for roll damping
const uint8_t pitch_d = 6;        // Multiplier for pitch damping
const uint8_t roll_d_driver = 2;  // Multiplier for roll damping driver
const uint8_t pitch_d_driver = 2; // Multiplier for pitch damping driver

const uint8_t p = 1;           // Multiplier for the Single Spring Stiffness
const uint8_t d = 2;           // Multiplier for the Single Damping Effect

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

void setup() {
}

void loop() {
  if (sussys == true) && NSuspState = 0 {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    NSuspState = 1
  }
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
  else {
    Serial.print("Error Occured");
  }
}
