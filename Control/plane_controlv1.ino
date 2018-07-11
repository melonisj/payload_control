#include <Servo.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h> 
#include <Adafruit_L3GD20.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <math.h>


#define GPSACTIVE
#define GYRO_CS 4 // chip selecte labeled CS for SPI for gyro/accel
#define GYRO_DO 5 // Master in, slave out labeled SA0 for SPI for gyro/accel
#define GYRO_DI 6  // master out slave in labeled SDA for SPI for gyro/accel
#define GYRO_CLK 7 // clock labeled SCL for SPI for gyro/accel
#define BMP_SCK 7 //clock for SPI for pressure sensor
#define BMP_MISO 5 //master in, sensor out for SPI for pressure sensor
#define BMP_MOSI 6 //master out, sensor(slave) in for SPI for pressure sensor
#define BMP_CS 8 // chip select for SPI for pressure sensory
#define PITCHTHRESHOLD 5 //degrees of acceptable pitch
#define ROLLTHRESHOLD 5 //degress of accetable roll
#define LFLAP_PIN 9 //pin for left flap
#define LAIL_PIN 10 //pin for left aileron
#define RAIL_PIN 11 //pin for right aileron
#define RFLAP_PIN 12 //pin for right flap
#define SERVO_DEFAULT 90 //servo default position TODO
#define SERVO_MOVE_AMNT 2 //degrees to move servo by each iteration
#define GPSRX 13  //gps transmits, board recieves
#define GPSTX 14  //gps recieves, board transmits
#define LIGHT_SENSOR_PIN 15 //pin for light sensor
#define LIGHT_THRESH 3 //threshold for ejection
#define MOTOR_PIN 16 //pin for thrust motor
#define TARGET_THRESH 0.0014 //rougly 500 feet in degrees

#define CLIMBING_THRESHOLD 2 //how many g's must be pulled to detect motor ignition
#define EJECTION_DELAY 3 //how long after ejection until control loop starts (seconds)

//initialize communication to boards
Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
#ifdef GPSACTIVE
  SoftwareSerial gps_serial(GPSRX, GPSTX); // RX, TX (TX not used)
#endif

//setup servos for flight surface controls
Servo lflap;
Servo laileron;
Servo raileron;
Servo rflap;

//create array to keep track of servo positions
int servo_pos[] = {SERVO_DEFAULT, SERVO_DEFAULT, SERVO_DEFAULT, SERVO_DEFAULT}; //0 = lf, 1 = la, 2 = ra, 3 = rf


int scale = 200; //scale for accel
float alt_offset = 0;  //launch pad alititude, determined by software
float target_lat = 38.209658; //target lattitude
float target_long = -103.702843; //target longitude

//coefficients for PID control
const int Kp_pitch = 2;
const int Ki_pitch = 2;
const int Kd_pitch = 2;
const int Kp_roll = 2;
const int Ki_roll = 2;
const int Kd_roll = 2;

  //The variables used by the PID controller

double input_pitch;     //current pitch of aircraft
double output_pitch;    //what gets compared to threshold
double input_roll;      //current roll of aircraft
double output_roll;     //calculated expected roll
double setpoint = 0;    //The orientation we want the plane to maintain

int sampleTime = 100;   //how many ms between each data sample

//create PID controllers
PID PID_pitch(&input_pitch, &output_pitch, &setpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);  //DIRECT as opposed to INVERSE control
PID PID_roll(&input_roll, &output_roll, &setpoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);

//create finite states
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
    #ifdef GPSACTIVE
      gps_serial.begin(9600);               //connect link to gps
    #endif

    //connect servo pins to flaps
    lflap.attach(LFLAP_PIN);
    laileron.attach(LAIL_PIN);
    raileron.attach(RAIL_PIN);
    rflap.attach(RFLAP_PIN);

    lflap.write(SERVO_DEFAULT);
    laileron.write(SERVO_DEFAULT);
    raileron.write(SERVO_DEFAULT);
    rflap.write(SERVO_DEFAULT);

    
    

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


  //set status to prelaunch
  fstatus = prelaunch;
  delay(5000);

  //determine starting alititude
  float sum = 0;
  float pressure_raw[2];
  for(int i = 0; i < 100; i++){
    pressure_read(pressure_raw, alt_offset);
    sum += pressure_raw[2];
    delay(100);
  }
  alt_offset = sum/100;

  //move into prep for launch stage
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
      //read accel, need 5 measurements of CLIMBING_THRESHOLD (2 g's) or more
        accel_read(accel_raw);
        if(accel_raw[2] > CLIMBING_THRESHOLD){
          climbing_count++;
        } else {
          climbing_count = 0;
        }
        if(climbing_count > 5){
          fstatus = launching; //if sustained, move into launching stage
        }
        delay(200);

        //save data
        break;
    }
    
    case launching: {
      Serial.print("Launching");
      //save data
        //activate pid
      PID_pitch.SetMode(AUTOMATIC);
      PID_roll.SetMode(AUTOMATIC);
      PID_pitch.SetSampleTime(sampleTime);
      PID_roll.SetSampleTime(sampleTime);
      //detect changes to light sensor
      int light_read = analogRead(LIGHT_SENSOR_PIN);
      if(light_read > LIGHT_THRESH){
        fstatus = ejected;
      }
      break;
    }
    case ejected: {
      Serial.print("Ejected");
      //wait a few seconds for wings to deply
      delay(EJECTION_DELAY);
      digitalWrite(MOTOR_PIN,HIGH);
      //pull up to level out
      fstatus = in_flight;
      while(1){
        //read gyro data to determine orientation
        int gyro_data[2];
        gyro_read(gyro_data);
        input_pitch = gyro_data[0];
        //PID_pitch.Compute();  not currently using this
        if(gyro_data[0] < -1*PITCHTHRESHOLD){
          //move motor a little bit to pitch up
          //TODO make this more responsive to output_pitch
            servo_pos[0] += SERVO_MOVE_AMNT;
            servo_pos[3] += SERVO_MOVE_AMNT;
            lflap.write(servo_pos[0]);
            rflap.write(servo_pos[3]);
          //TODO:
        } else if(gyro_data[0] > PITCHTHRESHOLD){
          //move motor a little bit to pitch down
            servo_pos[0] -= SERVO_MOVE_AMNT;
            servo_pos[3] -= SERVO_MOVE_AMNT;
            lflap.write(servo_pos[0]);
            rflap.write(servo_pos[3]);
        } else {
            servo_pos[0] = SERVO_DEFAULT;
            servo_pos[3] = SERVO_DEFAULT;
            lflap.write(SERVO_DEFAULT);
            rflap.write(SERVO_DEFAULT);
          break;
        }
      }
      break;
    }
    case in_flight: {
      //WARNING, code beyond here gets just absolutely bananas
      Serial.print("In Flight");
      float bearing_mag, angle_to_target;
      bool return_to_target = false;
      //control loop
      while(1){
        #ifdef GPSACTIVE
          char incoming_chars[83];
          if(gps_serial.available()){
            gps_serial.readBytesUntil('\n', incoming_chars, 82); //read gps as chars
            incoming_chars[82] = 0x00; //terminate chars to convert to string
            String gps_string = String(incoming_chars); //convert gps to string

            //gps heading data
            if(incoming_chars[3] == 'V' && incoming_chars[4] == 'T' && incoming_chars[5] == 'G'){
              int comma1 = gps_string.indexOf(','); //find data needed in string
              int comma2 = gps_string.indexOf(',', comma1 +1);
              int comma3 = gps_string.indexOf(',', comma2 +1);
              int comma4 = gps_string.indexOf(',', comma3 +1);
              if(comma3+1 != comma4){ //if this is false it means GPS data is invalid
                String bearing_mag_str = gps_string.substring(comma3, comma4);
                bearing_mag = bearing_mag_str.toFloat(); //current heading (magnetic north)
              } // else gps not valid

              //gps position data
            } else if(incoming_chars[3] == 'G' && incoming_chars[4] == 'L' && incoming_chars[5] == 'L'){
              int comma1 = gps_string.indexOf(','); //find data needed in string
              int comma2 = gps_string.indexOf(',', comma1 +1);
              int comma3 = gps_string.indexOf(',', comma2 +1);
              int comma4 = gps_string.indexOf(',', comma3 +1);
              if(comma1+1 != comma2){
                String lat_str = gps_string.substring(comma1, comma2);
                String long_str = gps_string.substring(comma3, comma4);
                //data is in DDMM.MMMMMM, need DD.DDDDDD, so lots of unit conversions
                float lat_cur = lat_str.toFloat();
                float long_cur = long_str.toFloat();
                int lat_degrees = int((lat_cur/100)-0.5);
                int long_degrees = int((long_cur/100)-0.5);
                float lat_minutes = (lat_cur-lat_degrees)/60;
                float long_minutes = (long_cur-long_degrees)/60;
                float real_lat = float(lat_degrees)+lat_minutes;
                float real_long = float(long_degrees)+long_minutes;

                //determine angle to target
                angle_to_target = atan2(target_lat-real_lat, target_long-real_long);

                //determine if we are too far from target and need to go back there
                if(real_lat < target_lat || real_long < target_long){
                  if(real_lat + TARGET_THRESH < target_lat || real_long + TARGET_THRESH < target_long){
                    return_to_target = true;
                  }
                } else if(real_lat > target_lat || real_long > target_long){
                  if(real_lat - TARGET_THRESH > target_lat || real_long + TARGET_THRESH > target_long ){
                    return_to_target = true;
                  }
                  
                } else {
                  return_to_target = false;
                }
                
              } // else gps not valid

              //if we know current heading and desired heading
             if(angle_to_target != NULL && bearing_mag != NULL){
              if(return_to_target){
                //return to target
                if(bearing_mag - angle_to_target < 0){
                  //turn left
                  while(bearing_mag-angle_to_target < 0){
                    //set motors to set value to turn x attitude
                    //TODO: determine this angle
//                    servo_pos[1] = TURNING_ANGLE;
//                    servo_pos[2] = -1*TURNING_ANGLE;
//                    lflap.write(servo_pos[1]);
//                    rflap.write(servo_pos[2]);

                    
                    delay(1000);
                  }
                } else {
                  //turn right
                  while(bearing_mag - angle_to_target > 0){
                    //set motors to preset value to turn at x attitude
                    //TODO: determine this angle
//                    servo_pos[1] = -1*TURNING_ANGLE;
//                    servo_pos[2] = TURNING_ANGLE;
//                    lflap.write(servo_pos[1]);
//                    rflap.write(servo_pos[2]);
                    delay(1000);
                  }
                }
              }
             }

             //if we are in the target threshold zone
             int gyro_data[2];
              gyro_read(gyro_data);
              input_pitch = gyro_data[0];
              input_roll = gyro_data[1];
//              PID_pitch.Compute(); not presently using this
//              PID_roll.Compute();
              if(gyro_data[0] < -1*PITCHTHRESHOLD){
                //move motor a little bit to pitch up
                //TODO:
                  servo_pos[0] += SERVO_MOVE_AMNT;
                  servo_pos[3] += SERVO_MOVE_AMNT;
                  lflap.write(servo_pos[0]);
                  rflap.write(servo_pos[3]);
              } else if(gyro_data[0] > PITCHTHRESHOLD){
                //move motor a little bit to pitch down
                //TODO:
                  servo_pos[0] -= SERVO_MOVE_AMNT;
                  servo_pos[3] -= SERVO_MOVE_AMNT;
                  lflap.write(servo_pos[0]);
                  rflap.write(servo_pos[3]);
              }
              
              if(gyro_data[1] < -1*ROLLTHRESHOLD){
                //move motor a little bit to roll to the right
                //TODO: refine servo move amount
                  servo_pos[1] -= SERVO_MOVE_AMNT;
                  servo_pos[2] += SERVO_MOVE_AMNT;
                  laileron.write(servo_pos[1]);
                  raileron.write(servo_pos[2]);
              } else if(gyro_data[1] > ROLLTHRESHOLD){
                //move motor a little bit to roll to the left
                //TODO: refine servo move amount
                  servo_pos[1] += SERVO_MOVE_AMNT;
                  servo_pos[2] -= SERVO_MOVE_AMNT;
                  laileron.write(servo_pos[1]);
                  raileron.write(servo_pos[2]);
              }
            }
          }
        #else

        //THIS CODE IS A DUPLICATE OF THAT ABOVE, with GPS removed though
          int gyro_data[2];
          gyro_read(gyro_data);
          input_pitch = gyro_data[0];
          input_roll = gyro_data[1];
          PID_pitch.Compute();
          PID_roll.Compute();
          if(output_pitch < -1*PITCHTHRESHOLD){
            //move motor a little bit to pitch up
            //TODO:
              servo_pos[0] += SERVO_MOVE_AMNT;
              servo_pos[3] += SERVO_MOVE_AMNT;
              lflap.write(servo_pos[0]);
              rflap.write(servo_pos[3]);
          } else if(output_pitch > PITCHTHRESHOLD){
            //move motor a little bit to pitch down
            //TODO:
              servo_pos[0] -= SERVO_MOVE_AMNT;
              servo_pos[3] -= SERVO_MOVE_AMNT;
              lflap.write(servo_pos[0]);
              rflap.write(servo_pos[3]);
          }
        
            if(gyro_data[1] < -1*ROLLTHRESHOLD){
              //move motor a little bit to roll to the right
              //TODO: refine servo move amount
                servo_pos[1] -= SERVO_MOVE_AMNT;
                servo_pos[2] += SERVO_MOVE_AMNT;
                laileron.write(servo_pos[1]);
                raileron.write(servo_pos[2]);
            } else if(gyro_data[1] > ROLLTHRESHOLD){
              //move motor a little bit to roll to the left
              //TODO: refine servo move amount
                servo_pos[1] += SERVO_MOVE_AMNT;
                servo_pos[2] -= SERVO_MOVE_AMNT;
                laileron.write(servo_pos[1]);
                raileron.write(servo_pos[2]);
            }
          #endif
      }
      //did it land yet?
      float pressure_raw[2];
      pressure_read(pressure_raw, alt_offset);
      if(pressure_raw[2] < 10){
        delay(500);
        digitalWrite(MOTOR_PIN, LOW);
        fstatus = landed;
      }
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

//void collect_gps(){
//  #ifdef GPSACTIVE
//    char incoming_chars[83];
//    if(gps_serial.available()){
//      gps_serial.readBytesUntil('\n', incoming_chars, 82);
//      incoming_chars[82] = 0x00;
//      String gps_string = String(incoming_chars);
//      if(incoming_chars[3] == 'V' && incoming_chars[4] == 'T' && incoming_chars[5] == 'G'){
//        int comma1 = gps_string.indexOf(',');
//        int comma2 = gps_string.indexOf(',', comma1 +1);
//        int comma3 = gps_string.indexOf(',', comma2 +1);
//        int comma4 = gps_string.indexOf(',', comma3 +1);
//        if(comma3+1 != comma4){
//          String bearing_mag_str = gps_string.substring(comma3, comma4);
//          float bearing_mag = bearing_mag_str.toFloat();
//        } // else gps not valid
//        
//      } else if(incoming_chars[3] == 'G' && incoming_chars[4] == 'L' && incoming_chars[5] == 'L'){
//        int comma1 = gps_string.indexOf(',');
//        int comma2 = gps_string.indexOf(',', comma1 +1);
//        int comma3 = gps_string.indexOf(',', comma2 +1);
//        int comma4 = gps_string.indexOf(',', comma3 +1);
//        if(comma1+1 != comma2){
//          String lat_str = gps_string.substring(comma1, comma2);
//          String long_str = gps_string.substring(comma3, comma4);
//          float lat_cur = lat_str.toFloat();
//          float long_cur = long_str.toFloat();
//          int lat_degrees = int((lat_cur/100)-0.5);
//          int long_degrees = int((long_cur/100)-0.5);
//          float lat_minutes = (lat_cur-lat_degrees)/60;
//          float long_minutes = (long_cur-long_degrees)/60;
//          float real_lat = float(lat_degrees)+lat_minutes;
//          float real_long = float(long_degrees)+long_minutes;
//        } // else gps not valid
//        
//      }
//      
//    }
//   #endif
//}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
