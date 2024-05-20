//#include <LCD_I2C.h>
#include <bme68x.h>
#include <bme68x_defs.h>

// v3 print temp pre only
/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

// v3 ave calc. add
double temp_act_ave = 0.0, press_act_ave = 0.0,hum_act_ave =0.0;

//竹内追加分
const int buttonPin_1 = 9;
const int buttonPin_2 = 10;
const int buttonPin_3 = 11;
const int buttonPin_4 = 12;

const int speakerPin = 8;
//竹内追加分終わり

//LCD_I2C lcd(0x27);

void setup() {

//Add by Takeuchi
pinMode(buttonPin_1, INPUT_PULLUP);
pinMode(buttonPin_2, INPUT_PULLUP);
pinMode(buttonPin_3, INPUT_PULLUP);
pinMode(buttonPin_4, INPUT_PULLUP);

pinMode(speakerPin, OUTPUT);
//Add end

  //lcd.begin();
  //lcd.setCursor(1, 0);
  
// Add v3 by Yas.
    byte i;
    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
//    signed long int temp_cal;
//    unsigned long int press_cal,hum_cal;
    pinMode(13,OUTPUT); // led check
//

Serial.begin(74880);
  while (!Serial);
  Serial.println(F("BME680 test v3 "));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  Serial.write(4);

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme.setGasHeater(320, 150); // 320*C for 150 ms
  bme.setGasHeater(320, 10); // 320*C for 150 ms
/*  
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
*/

// Sensor ave 5 times 
    for( i=0 ; i<5 ; i++){
//        readData();
    if (! bme.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }   
/*        temp_cal = bme.temperature ;
        press_cal = bme.pressure / 100.0 ;
        hum_cal = bme.humidity ;  */
        temp_act = bme.temperature ;
        press_act = bme.pressure / 100.0 ;
        hum_act = bme.humidity ;
        temp_act_ave += temp_act, press_act_ave += press_act, hum_act_ave += hum_act;
    delay(200);
    }
    temp_act_ave = temp_act_ave/5 , press_act_ave = press_act_ave/5 , hum_act_ave = hum_act_ave/5;
    Serial.print("TEMP ave : ");
    Serial.print(temp_act_ave);
    Serial.print(" DegC  PRESS ave : ");
    Serial.print(press_act_ave);
    Serial.print(" hPa  HUM ave : ");
    Serial.print(hum_act_ave);
    Serial.println(" %");    
    delay(1000);

}

void loop() {

//竹内追加分
  int buttonState_1 = digitalRead(buttonPin_1);
  int buttonState_2 = digitalRead(buttonPin_2);
  int buttonState_3 = digitalRead(buttonPin_3);
  int buttonState_4 = digitalRead(buttonPin_4);

  unsigned long time;
  delay(100);
  time = millis();
//竹内追加分
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature,6);
//  Serial.println(" *C");
  Serial.print(" *C : ");
  
/*おそらくここからが気圧を測って表示するプログラム*/

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
//  Serial.println(" hPa");
  Serial.print(" hPa : ");
  
/*気圧を測って表示する部分終わり*/

// 吹いたかを判定し、吹いた判定ならFUITAと表示
//    if ( (bme.temperature > temp_act_ave ) && ( (bme.pressure / 100.0)  <= (press_act_ave - 0.50)) && (bme.humidity > (hum_act_ave + 1.0))){

/*
  if(buttonState_1 == LOW)
  {
    if(((bme.pressure / 100.0) >= (press_act_ave + 0.10)) && ((bme.pressure / 100.0) < (press_act_ave + 2.0)))
   {  // only press hum v3
      Serial.println("吹いた!");

      tone(speakerPin, 262, 500);
      
      //Serial.print("いいね！");
      //Serial.print(time);
      //Serial.println("ms");
   }
  }

  if(buttonState_2 == LOW)
  {
    if(((bme.pressure / 100.0) >= (press_act_ave + 0.10)) && ((bme.pressure / 100.0) < (press_act_ave + 2.0)))
   {  // only press hum v3
      Serial.println("吹いた!");

      tone(speakerPin, 294, 500);
      
      //Serial.print("いいね！");
      //Serial.print(time);
      //Serial.println("ms");
   }
  }

  if(buttonState_3 == LOW)
  {
    if(((bme.pressure / 100.0) >= (press_act_ave + 0.10)) && ((bme.pressure / 100.0) < (press_act_ave + 2.0)))
   {  // only press hum v3
      Serial.println("吹いた!");

      tone(speakerPin, 329, 500);
      
      //Serial.print("いいね！");
      //Serial.print(time);
      //Serial.println("ms");
   }
  }
  */
  
//竹内追加分


   if(((bme.pressure / 100.0) >= (press_act_ave + 0.13)) && ((bme.pressure / 100.0) < (press_act_ave + 2.0)))
   {  // only press hum v3
      Serial.println("吹いた！！！");
      Serial.print("いいね！");
      Serial.print(time);
      Serial.println("ms");
      
      if(buttonState_1 == LOW)
      {
        tone(speakerPin, 262, 500);
        delay(300);
      }

      if(buttonState_2 == LOW)
      {
        tone(speakerPin, 294, 500);
        delay(300);
      }

      if(buttonState_3 == LOW)
      {
        tone(speakerPin, 329, 500);
        delay(300);
      }

      if(buttonState_4 == LOW)
      {
        tone(speakerPin, 349, 500);
        delay(300);
      }
   }

   
   if(((bme.pressure / 100.0) >= (press_act_ave + 0.5) && (bme.humidity > (hum_act_ave + 2.0)))
   {
      Serial.println("素晴らしい！");
      Serial.print(time);
      Serial.println("ms");
      delay(300);
   }
   
   else
   {
      Serial.println("もっと吹いて！");
      delay(300);
   }

//竹内追加分おわり
    
  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
//  Serial.println(" %");  
  Serial.print(" %");
/*
  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
*/

  Serial.println();
//  delay(2000);
  delay(20);
}

/*
int measureTime(int time)
{
  int a;
  a = 0;
  a = time;
  a = millis();
  time = 0;
  return(a);
  a = 0;
}
*/
