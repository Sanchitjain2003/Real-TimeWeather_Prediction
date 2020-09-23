//#include "LowPower.h" //ignore this

// We'll use SoftwareSerial to communicate with the HC12:
#include <SoftwareSerial.h>

//SoftwareSerial MySerial(5, 4); // RX, TX for communication

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

float ff;
float addff;
float hh;
float addhh;
float addhin;
float f;
float h;
float p;
int x;
int y;
int z;

void setup() {
  Serial.begin(9600);
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  //MySerial.begin(9600);
  //Wire.begin (0x76, &Wire);
  bme.begin(0x76, &Wire);

} // end setup

void loop() {
  // location of sleep mode seems to have desired effect, must be at top of code.
  /*for (int i = 0; i < 65; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    // 8s is max for low.power lib, but with a 65 loops = 10mins
  }*/
  float cc = (bme.readTemperature());
  for (z = 0; z < 2; z++) {          // needed to set at least 2 runs, serial data does not always send on just one
    for (x = 0; x < 10; x++) {       // increased sample size to 20 to minimize spikes in data effecting average
      delay (500);                   // small delay to allow DHT to start and collect data
      //int cc = (bme.readTemperature());
      ff = (cc * 9.0 / 5.0) + 25.0; 
      addff = ff + addff;

      hh = (bme.readHumidity());
      addhh = hh + addhh;

      float hpa = (bme.readPressure() / 100.0F);
      float hin = hpa * 0.0295299830714;
      addhin = hin + addhin;

    } // end for x loop

    for (y = 0; y < 1; y ++) {
      delay (500);
      f = addff / 20;
      h = addhh / 20;
      p = addhin / 20;
      double gamma = log(h / 100) + ((17.62 * cc) / (243.5 + cc));
      double dew_pt = 243.5 * gamma / (17.62 - gamma);
      Serial.print("Humdity: ");
      Serial.print(h);
      //MySerial.write(h);
      Serial.print("");
      Serial.print("  Temp: ");
      Serial.print(cc);
      //MySerial.write(cc);
      Serial.print(" deg/C  ");
      Serial.print ("Dew point: ");
      Serial.print(dew_pt);
      //MySerial.write (dew_pt);
      Serial.print("  ");
      Serial.print("Baro: ");
      Serial.print(p);
      //MySerial.write(p);
      Serial.print(" hpa");
      Serial.println("");
    }
    addff = 0;                   // clear old added data, otherwise number just keeps getting larger...
    addhh = 0;
    addhin = 0;
  }
  if (Serial.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // clear the screen
    lcd.clear();
    // read all the available characters
    while (Serial.available() > 0) {
      // display each character to the LCD
      lcd.write(Serial.read());
    }
  }
} // end loop
