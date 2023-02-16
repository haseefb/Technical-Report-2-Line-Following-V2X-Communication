int Sensor1 = 34;
int Sensor2 = 35;
int Sensor3 = 32 ;
int Sensor4 = 33;
int Sensor5 = 25;
int Sensor6 = 26;

int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;
int val5 = 0;
int val6 = 0;

#include <Adafruit_NeoPixel.h>
#define LED_PIN 17 // Pin number for NeoPixel strip
#define LED_COUNT 15 // Number of LEDs in strip
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
int hue = 0; // variable to store the current hue value
int leftMotor_speed = 0;
int rightMotor_speed = 0;
int servoAngle = 85;

#define I2C_SLAVE_ADDR 0x04  //define I2C slave address as 4 in hexadecimal

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_SDA 21 // OLED display SDA pin
#define OLED_SCL 22 // OLED display SCL pin
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

void setup() {
    strip.begin();
  Serial.begin(9600);  // put your setup code here, to run once:
  Wire.begin();  // Initialize the I2C bus
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize the OLED display
  display.clearDisplay();  // Clear the display buffer
  pinMode(Sensor3, INPUT);
  pinMode(Sensor4, INPUT);
  // Center servo
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 88;
  Serial.println("Sensor 1 | Sensor 2 | Sensor 3 | Sensor 4 | Sensor 5 | Sensor 6");
}
 
void loop() {
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.ColorHSV(hue, 255, 255)); // set the color of the LED based on hue value
  }
  strip.show(); // update the LEDs with the new color
  hue++; // increment the hue value
  if (hue > 359) {
    hue = 0; // reset the hue value when it reaches the maximum value
  }
  delay(10); // delay for smooth transition


  val1 = analogRead(Sensor1);
  val2 = analogRead(Sensor2);
  val3 = analogRead(Sensor3);
  val4 = analogRead(Sensor4);
  val5 = analogRead(Sensor5);
  val6 = analogRead(Sensor6);

  
  // Print sensor values
  Serial.print(val1);
  Serial.print("       | ");
  Serial.print(val2);
  Serial.print("       | ");
  Serial.print(val3);
  Serial.print("       | ");
  Serial.print(val4);
  Serial.print("       | ");
  Serial.print(val5);
  Serial.print("       | ");
  Serial.println(val6);

  display.setTextColor(WHITE);
  display.setTextSize(0.4);
  display.setCursor(0, 0);
  display.print("Sensor1Sensor2Sensor3");
  display.setCursor(0, 20);
  display.print("Sensor4Sensor5Sensor6");

  // Print sensor values
  display.setCursor(10, 40);  
  display.print(val1);  
  display.setCursor(50, 40);  
  display.print(val2);  
  display.setCursor(90, 40);  
  display.print(val3);  
  display.setCursor(10, 50);  
  display.print(val4);  
  display.setCursor(50, 50);  
  display.print(val5);  
  display.setCursor(90, 50);  
  display.print(val6);  

  display.display(); 
  delay(100);
  display.clearDisplay();
  
  if (val3 < 4095 && val4 < 4095) {
    //Robot is on the line, move forward
   leftMotor_speed = 150;
   rightMotor_speed = 150;
   servoAngle = 88;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); 
  }
  else if (val3 == 4095 && val4 == 4095) {
    // Turn left
    leftMotor_speed = 0;
    rightMotor_speed = 0;
    servoAngle = 88;
    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle); 
  }

}
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

