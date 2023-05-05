// Integers for Variable Wheelbase
int drs_stability;    //create integer for the wheelbase servo pwm which will also actuate flaps on edf

// Integers for Active Aero
int aero_val1;                      //create integer for aero val from channel 1
int aero_val2;                      //create integer for aero val from channel 2
int aero_pwm;                       //create integer for aero servo pwm
const int aero_braking_pwm = 1600;  //pwm where aero goes into stalling angle      **********CHANGE THIS************* between 1500 - 2000
const int aero_down = 1000;         //pwm where the wing is flush with body
int aero_raise_pwm;                 //create integer for lifting wing up or down

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:

// Variable Wheelbase
{
   if ((ch3 >1500) && (ch2 > 1700)) {
    wheelbase = .35;
    wheelbase_pwm = 2000;
   }
   else {
    wheelbase = .3;
    wheelbase_pwm = 1000;
   }
}

// Active Aero
{
if (ch3 > 1500) {
  aero_raise_pwm = 2000;                            //raise wing
  if (ch2 <= aero_braking_pwm) {
    aero_val2 = 2100;                           //when insufficient throttle, wing goes into braking mode
  }
  else{
    aero_val2 = 1000;                           //normal drs
  }
  if ((ch1) >= 1600) {
    aero_val1 = map(ch1,1600,2000,1500,2100);   //as steering increases at top speed, wing increases
  }
  else if ((ch1) <= 1400) {
    aero_val1 = map(ch1,1000,1400,2100,1500);   //as steering increases at top speed, wing increases
  }
  else{
    aero_val1=1000;                             //normal drs
  }

  if (aero_val1 > aero_val2){
    aero_pwm = aero_val1;
  }
  else{
    aero_pwm = aero_val2;                       // use bigger value out of two as braking angle
  }
}
else
  aero_raise_pwm = 1000;                        //keep wing down
  aero_pwm = aero_down;                         //set wing to fit into chassis
}


}
