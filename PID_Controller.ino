//A library that handles PID control
//https://playground.arduino.cc/Code/PIDLibrary
#include <PID_v1.h>

//The solenoid valves that stabilize the tube
//Labled by position in degrees (unit circle looking at tube from top)
#define VALVE_0    2
#define VALVE_90   3
#define VALVE_180  4
#define VALVE_270  5

  //Defines the PID ouptut threshold beyond which a valve is activated
  //TODO: Tune threshold
  const int THRESHOLD = 10;

  //Parameters for the PID
  //TODO: Tune parameters
  const int Kp_x = 2;
  const int Ki_x = 2;
  const int Kd_x = 2;
  const int Kp_y = 2;
  const int Ki_y = 2;
  const int Kd_y = 2;

  //The variables used by the PID controller
  //Looking at the top of the tube: 
  //Rotation of the top of the tube to the right is +x
  //Rotation of the top of the tube updward is +y
  double input_x;         //Current orientation of the tube
  double output_x;        //What gets compared to the threshold to see if we open a solenoid valve
  double input_y;
  double output_y;
  double setpoint = 0;    //The orientation we want the tube to maintain, in this case zero (for both axes)

  //How many milliseconds between PID updates
  //TODO: Decide sample rate
  int sampleTime = 100;

  //Create the PID objects (from library)
  PID PID_x(&input_x, &output_x, &setpoint, Kp_x, Ki_x, Kd_x, DIRECT);  //DIRECT as opposed to INVERSE control
  PID PID_y(&input_y, &output_y, &setpoint, Kp_y, Ki_y, Kd_y, DIRECT);
  
void setup() {
  //Setup output pins
  pinMode(VALVE_0, OUTPUT);
  pinMode(VALVE_90, OUTPUT);
  pinMode(VALVE_180, OUTPUT);
  pinMode(VALVE_270, OUTPUT);

  //TODO: Bring in input from sensors to determine orientation

  //Activate the PIDs
  //TODO: Determine activation trigger?
  PID_x.SetMode(AUTOMATIC);
  PID_y.SetMode(AUTOMATIC);

  //Set the update interval
  PID_x.SetSampleTime(sampleTime);
  PID_y.SetSampleTime(sampleTime);
}

void loop() {
  //Each loop, call compute function on PIDs
  PID_x.Compute();
  PID_y.Compute();

  //Open and close solenoids as necessary
  if(output_x > THRESHOLD) {
    digitalWrite(VALVE_180, HIGH);
  } else {
    digitalWrite(VALVE_180, LOW);
  }

  if(output_x < (-1 * THRESHOLD)) {
    digitalWrite(VALVE_0, HIGH);
  } else {
    digitalWrite(VALVE_0, LOW);
  }

  if(output_y > THRESHOLD) {
    digitalWrite(VALVE_270, HIGH);
  } else {
    digitalWrite(VALVE_270, LOW);
  }

  if(output_x > (-1 * THRESHOLD)) {
    digitalWrite(VALVE_90, HIGH);
  } else {
    digitalWrite(VALVE_90, LOW);
  }
}
