const uint16_t BrakingThresholdPWM = 1475;
const uint8_t CornerLatThresh = 1;
const uint16_t AcceleratingThresholdPWM = 1525;
const uint16_t BMigMapTime[6] = {0,100,400,700,900,1000};
const uint8_t BMigMap1[6] = {0,0,1,2,3,4};

void setup() {
}

void loop() {
// Active Front Differential
{
    if (ch2 < BrakingThresholdPWM) {
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

// Insert Pedal Maps Here todecide the torque target for the TDUs (Torque Delivery Unit)
// Got to here with the code, now to populate the Corner State and Behaviours


// Consideration of BBal & BMig 
}
