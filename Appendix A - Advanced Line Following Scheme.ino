#include <Wire.h>  //Include the Wire library

#include <PID_v1.h>  //Include the PID_v1 library

#define I2C_SLAVE_ADDR  0x04 //Define the I2C address of the Arduino Nano

#define Sensor1 34 //Assign the pin numbers for all the sensors
#define Sensor2 35
#define Sensor3 32
#define Sensor4 33
#define Sensor5 25
#define Sensor6 26

int baseSpeed = 80; //Set the base motor speed for the robotic car
int leftMotor_speed = 0; //Initalises the left motor speed
int rightMotor_speed = 0; //Initalises the right motor speed
int servoAngle = 40; //Initalises the servo angle
int servo_Centre = 40; //Initalises the servo center

int val1,val2,val3,val4,val5,val6 = 0; //Initalises the sensor values
int refpoint = 0; //Set the reference point
int W_Avg = 0; //Initalises the weighted average
int error = 0; // Initalises error which is used as input for PID
float K = 0; //Initalises the scaling factor for motor speed
double Refpoint, Input, U; //Initalises the variables for PID
double Kp=192, Ki=0.97, Kd=0; //Set the PID values

PID myPID(&Input, &U, &Refpoint, Kp, Ki, Kd, DIRECT); //Define the PID object

float w1 = -32.0; //Set the weight for each sensor
float w2 = -20.0;
float w3 = -7.0;
float w4 = 7.0;
float w5 = 20.0;
float w6 = 32.0;

int min_s1, min_s2, min_s3, min_s4, min_s5, min_s6; //Initalises the calibration variables
int max_;


void setup() {
  pinMode(Sensor1, INPUT); //Set the pin mode for all sensors
  pinMode(Sensor2, INPUT);
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  pinMode(Sensor5, INPUT);
  pinMode(Sensor6, INPUT);

  Wire.begin(); //Start the I2C communication
  Serial.begin(9600); //Start the serial communication
  myPID.SetMode(AUTOMATIC); //Set the PID mode to automatic

}

void loop() {
  myPID.SetSampleTime(5); //Set the sample time for PID
  refpoint = 0; //Set the reference point

  min_s1 = 900; //Set the minimum and maximum values for calibration
  min_s2 = 1300;
  min_s3 = 1400;
  min_s4 = 1300;
  min_s5 = 1000;
  min_s6 = 900;
  max_   = 4095;

  val1 = analogRead(Sensor1); //Read the values from all sensors
  val2 = analogRead(Sensor2);
  val3 = analogRead(Sensor3);
  val4 = analogRead(Sensor4);
  val5 = analogRead(Sensor5);
  val6 = analogRead(Sensor6);

  val1 = constrain(val1, min_s1, max_); //Constrain the values to a specified range
  val2 = constrain(val2, min_s2, max_);
  val3 = constrain(val3, min_s3, max_);
  val4 = constrain(val4, min_s4, max_);
  val5 = constrain(val5, min_s5, max_);
  val6 = constrain(val6, min_s6, max_);

  //Maps the values into the 1 to 254 range
  val1 = map(val1, min_s1, max_, 1, 254);
  val2 = map(val2, min_s2, max_, 1, 254);
  val3 = map(val3, min_s3, max_, 1, 254);
  val4 = map(val4, min_s4, max_, 1, 254);
  val5 = map(val5, min_s5, max_, 1, 254);
  val6 = map(val6, min_s6, max_, 1, 254);


// Invert the values to make them proportional to the reflectance of the line
// and subtract them from 255 so that the dark line is a high number and the white background is a low number
  val1 = 255 - val1;
  val2 = 255 - val2;
  val3 = 255 - val3;
  val4 = 255 - val4;
  val5 = 255 - val5;
  val6 = 255 - val6;


  // Calculate the weighted average of the Sensors 
  float numerator = (val1 * w1 + val2 * w2 + val3 * w3 + val4 * w4 + val5 * w5 + val6 * w6);
  float total_W = (val1 + val2 + val3 + val4 + val5 + val6);

  float W_Avg = (numerator/total_W);
  float error = Refpoint - (W_Avg);
  Input = error;

  Serial.print("Weighted average: ");
  Serial.println(W_Avg);
  myPID.Compute();

  Serial.print("Output:");
  Serial.println(U);
 
  // Calculate the new speed and steering values
  leftMotor_speed = baseSpeed + (K*U);
  rightMotor_speed = baseSpeed - (K*U);
  servoAngle = servo_Centre + U;
  
  // Constrain the values to the appropriate range
  leftMotor_speed = constrain(leftMotor_speed, 0, 255);
  rightMotor_speed = constrain(rightMotor_speed, 0, 255);
  servoAngle = constrain(servoAngle, -30, 160);
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); 
/*
  // Debugging output
  Serial.print(leftMotor_speed);
  Serial.print(", ");
  Serial.print(rightMotor_speed);
  Serial.print(", ");
  Serial.println(servoAngle);
*/

}
// Send the Left/Right motor speed and servo angle to the Arduino Nano over I2C
void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle){
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
  Wire.write((byte)(leftMotor_speed & 0x000000FF));           //leftMotor_speed, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));   // first byte of rightMotor_speed, containing bits 16 to 9
  Wire.write((byte)(rightMotor_speed & 0x000000FF));          // second byte of rightMotor_speed, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         // first byte of servoAngle, containing bits 16 to 9
  Wire.write((byte)(servoAngle & 0x000000FF)); // second byte of servoAngle, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting
}