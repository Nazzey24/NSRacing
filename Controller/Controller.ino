const uint8_t BrakingThresholdPWM = 5;
const uint8_t CornerLatThresh = 1;
const uint8_t AcceleratingThresholdPWM = 5;
const uint16_t BMigMapTime[6] = {0,100,400,700,900,1000};
const uint8_t BMigMap1[6] = {0,0,1,2,3,4};
const uint8_t BMigMap2[6] = {0,0,1,2,4,6};
const uint8_t BMigMap3[6] = {0,0,0,1,2,4};
uint8_t BBal = 50; // gives initial percentage of brake balance 
void setup() {
}

void loop() {
// Active Front Differential
{
    if (rBrakePedal < BrakingThresholdPWM) {
      CornerState = 1; // Braking
    }
    if (abs(ya) > CornerLatThresh && CornerState = 1) {
      CornerState = 2;  // Entry
    }
    if (ch2 > AcceleratingThresholdPWM && CornerState = 2) {
      CornerState = 3;  // Exit
    }
    if (abs(ya) < CornerLatThresh && CornerState = 3) {
      CornerState = 4; // Straight
    }

// Insert Pedal Maps Here to decide the torque target for the TDUs (Torque Delivery Unit)
// How to switch between pedal maps, when car is stationary, in any other condition the its bbal.
// Got to here with the code, now to populate the Corner State and Behaviours


// Consideration of BBal & BMig 

}
