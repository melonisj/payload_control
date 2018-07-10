#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h> 
#include <Adafruit_L3GD20.h>
#include <PID_v1.h>


#define GYRO_CS 4 // labeled CS
#define GYRO_DO 5 // labeled SA0
#define GYRO_DI 6  // labeled SDA
#define GYRO_CLK 7 // labeled SCL
#define BMP_SCK 7
#define BMP_MISO 5
#define BMP_MOSI 6 
#define BMP_CS 8
#define PITCHTHRESHOLD 5
#define ROLLTHRESHOLD 5

#define CLIMBING_THRESHOLD 2
#define EJECTION_DELAY 3


Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int scale = 200;
float alt_offset = 0;

const int Kp_pitch = 2;
const int Ki_pitch = 2;
const int Kd_pitch = 2;
const int Kp_roll = 2;
const int Ki_roll = 2;
const int Kd_roll = 2;

  //The variables used by the PID controller

double input_pitch;     //current orientation
double output_pitch;    //what gets compared to threshold
double input_roll;
double output_roll;
double setpoint = 0;    //The orientation we want the plane to maintain

int sampleTime = 100;
PID PID_pitch(&input_pitch, &output_pitch, &setpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);  //DIRECT as opposed to INVERSE control
PID PID_roll(&input_roll, &output_roll, &setpoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);


enum payload_lifecycle {
  prelaunch,
  prep_to_launch,
  launching,
  ejected,
  in_flight,
  landed
};

enum payload_lifecycle fstatus;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);                 // Start the serial terminal
    Serial.println("Testing Active Payload System");
    Serial.println();

  // setup gyro
   if (!gyro.begin(gyro.L3DS20_RANGE_250DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
  //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
  {
    Serial.println("Oops ... unable to initialize the L3GD20. Check your wiring!");
    while (1);
  }


  //setup pressure
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }


  
  fstatus = prelaunch;
  delay(5000);
  float sum = 0;
  float pressure_raw[2];
  for(int i = 0; i < 100; i++){
    pressure_read(pressure_raw, alt_offset);
    sum += pressure_raw[2];
    delay(100);
  }
  alt_offset = sum/100;
  fstatus = prep_to_launch;
  Serial.print("Altitude Offset"); Serial.println(alt_offset);
    
}



void loop() {
  // put your main code here, to run repeatedly:
  int gyro_raw[2];
  float pressure_raw[2], accel_raw[2];
  int climbing_count = 0;
  
  switch(fstatus){
    case prelaunch: {
      Serial.print("Initialization Error, restart device");
      break;
    }
    
    case prep_to_launch: {
      Serial.print("Prepping to Launch");
      //waiting for launch, low power mode
        accel_read(accel_raw);
        if(accel_raw[2] > CLIMBING_THRESHOLD){
          climbing_count++;
        } else {
          climbing_count = 0;
        }
        if(climbing_count > 5){
          fstatus = launching;
        }
        delay(500);

        //save data
        break;
    }
    
    case launching: {
      Serial.print("Launching");
      //save data
        //activate pid
      PID_pitch.SetMode(AUTOMATIC);
      PID_roll.SetMode(AUTOMATIC);
      PIT_pitch.SetSampleTime(sampleTime);
      PIT_roll.SetSampleTime(sampleTime);
      break;
    }
    case ejected: {
      Serial.print("Ejected");
      //wait a few seconds for wings to deply
      delay(EJECTION_DELAY);
      //pull up to level out
      fstatus = in_flight;
      while(1){
        int gyro_data[2];
        gyro_read(gyro_data);
        input_pitch = gyro_read[0];
        PID_pitch.Compute();
        if(output_pitch < -1*PITCHTHRESHOLD){
          //move motor a little bit to pitch up
          //TODO:
        } else if(output_pitch > PITCHTHRESHOLD){
          //move motor a little bit to pitch down
        } else {
          break;
        }
      }
      break;
    }
    case in_flight: {
      Serial.print("In Flight");
      //control loop
      while(1){
        int gyro_data[2];
        gyro_read(gyro_data);
        input_pitch = gyro_read[0];
        input_roll = gyro_read[1];
        PID_pitch.Compute();
        PID_roll.Compute();
        if(output_pitch < -1*PITCHTHRESHOLD){
          //move motor a little bit to pitch up
          //TODO:
        } else if(output_pitch > PITCHTHRESHOLD){
          //move motor a little bit to pitch down
          //TODO:
        }
        
        if(output_roll < -1*ROLLTHRESHOLD){
          //move motor a little bit to pitch up
          //TODO:
        } else if(output_roll > ROLLTHRESHOLD){
          //move motor a little bit to pitch down
          //TODO:
        }
      }
      }
      //did land yet?
      break;
    }
    case landed: {
      Serial.print("Landed");
      //landed, return to low power mode
      
      break;
    }
    default: {
      asm("nop");
      break;
    }
  }

  
  gyro_read(gyro_raw);
  pressure_read(pressure_raw, alt_offset);
  accel_read(accel_raw);
  delay(100);


  
 
}




void gyro_read(int gyro_arr[]){
  gyro.read();
//  Serial.print("X: "); Serial.print((int)gyro.data.x);   Serial.println(" ");
//  Serial.print("Y: "); Serial.print((int)gyro.data.y);   Serial.println(" ");
//  Serial.print("Z: "); Serial.print((int)gyro.data.z);   Serial.println(" ");
  gyro_arr[0] = (int)gyro.data.x;
  gyro_arr[1] = (int)gyro.data.y;
  gyro_arr[2] = (int)gyro.data.z;
}

void pressure_read(float pressure_arr[], float alt_offset){
//  Serial.print("Temperature = ");
//  Serial.print(bme.readTemperature());
//  Serial.println(" *C");
  
//  Serial.print("Pressure = ");
//  Serial.print(bme.readPressure());
//  Serial.println(" Pa");
//
//  Serial.print("Approx altitude = ");
//  Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
//  Serial.println(" m");
  pressure_arr[0] = bme.readTemperature();
  pressure_arr[1] = bme.readPressure();
  pressure_arr[2] = bme.readAltitude(1013.25)-alt_offset;
  

}


void accel_read(float accel_arr[]){
  int rawX = analogRead(A0);
  int rawY = analogRead(A1);
  int rawZ = analogRead(A2);
  
  // Scale accelerometer ADC readings into common units
  // Scale map depends on if using a 5V or 3.3V microcontroller
  float scaledX, scaledY, scaledZ; // Scaled values for each axis
  scaledX = mapf(rawX, 0, 1023, -scale, scale);
  scaledY = mapf(rawY, 0, 1023, -scale, scale);
  scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
  
  // Print out scaled X,Y,Z accelerometer readings
//  Serial.print("X: "); Serial.print(scaledX); Serial.println(" g");
//  Serial.print("Y: "); Serial.print(scaledY); Serial.println(" g");
//  Serial.print("Z: "); Serial.print(scaledZ); Serial.println(" g");
//  Serial.println();
  accel_arr[0] = scaledX;
  accel_arr[1] = scaledY;
  accel_arr[2] = scaledZ;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
