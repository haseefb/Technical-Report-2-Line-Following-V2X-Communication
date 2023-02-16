int Sensor1 = 34;
int Sensor2 = 35;
int Sensor3 = 32;
int Sensor4 = 33;
int Sensor5 = 25;
int Sensor6 = 26;


int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;
int val5 = 0;
int val6 = 0;

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_SDA 21 // OLED display SDA pin
#define OLED_SCL 22 // OLED display SCL pin
#include <Wire.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT);

void setup() {
  Serial.begin(9600);  // put your setup code here, to run once:
  Wire.begin();  // Initialize the I2C bus
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize the OLED display
  display.clearDisplay();  // Clear the display buffer
  Serial.println("Sensor 1 | Sensor 2 | Sensor 3 | Sensor 4 | Sensor 5 | Sensor 6");
}
 
void loop() {
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
  
  // Print table header
  display.setTextSize(1);  
  display.setTextColor(WHITE);  
  display.setCursor(0, 0);  
  display.print("Sensor 3");
  display.setCursor(80, 0);  
  display.print("Sensor 4");
  
  // Print sensor values
  display.setTextSize(2);  
  display.setCursor(0, 40);  
  display.print(val3);  
  display.setCursor(80, 40);  
  display.print(val4);  
  
  
  display.display(); 
  delay(100);
  display.clearDisplay();
}








