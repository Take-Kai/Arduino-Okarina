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
//使うスイッチがどのピンなのか宣言
const int buttonPin_1 = 5;
const int buttonPin_2 = 6;
const int buttonPin_3 = 7;
const int buttonPin_4 = 8;
const int buttonPin_5 = 10;
const int buttonPin_6 = 11;
const int buttonPin_7 = 12;
const int buttonPin_8 = 13;

const int speakerPin = 9;
//竹内追加分終わり


void setup() {

//竹内追加分
//スイッチ用のピンを入力に、スピーカーのピンを出力に設定
pinMode(buttonPin_1, INPUT_PULLUP);
pinMode(buttonPin_2, INPUT_PULLUP);
pinMode(buttonPin_3, INPUT_PULLUP);
pinMode(buttonPin_4, INPUT_PULLUP);
pinMode(buttonPin_5, INPUT_PULLUP);
pinMode(buttonPin_6, INPUT_PULLUP);
pinMode(buttonPin_7, INPUT_PULLUP);
pinMode(buttonPin_8, INPUT_PULLUP);

pinMode(speakerPin, OUTPUT);
//竹内追加分終わり
  
// Add v3 by Yas.
    byte i;
    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
//    signed long int temp_cal;
//    unsigned long int press_cal,hum_cal;
    pinMode(13,OUTPUT); // led check
//

Serial.begin(115200);
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

//ここから平均を求める
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
//平均を求めるの終わり

}

void loop() {

//竹内追加分
//デジタルリードで押された時の信号を取得
  int buttonState_1 = digitalRead(buttonPin_1); //5番ピン
  int buttonState_2 = digitalRead(buttonPin_2); //6番ピン
  int buttonState_3 = digitalRead(buttonPin_3); //7番ピン
  int buttonState_4 = digitalRead(buttonPin_4); //8番ピン
  int buttonState_5 = digitalRead(buttonPin_5); //10番ピン
  int buttonState_6 = digitalRead(buttonPin_6); //11番ピン
  int buttonState_7 = digitalRead(buttonPin_7); //12番ピン
  int buttonState_8 = digitalRead(buttonPin_8); //13番ピン
//竹内追加分終わり
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature,6);
//  Serial.println(" *C");
  Serial.print(" *C : ");

 
  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
//  Serial.println(" hPa");
  Serial.print(" hPa : ");
  

// 吹いたかを判定し、吹いた判定ならFUITAと表示
//    if ( (bme.temperature > temp_act_ave ) && ( (bme.pressure / 100.0)  <= (press_act_ave - 0.50)) && (bme.humidity > (hum_act_ave + 1.0))){


  
//竹内追加分
//音が出るプログラム。気圧値が範囲内の時に、押したスイッチの組み合わせで何の音が出るか設定。「素晴らしい！」とかの表示
   if(((bme.pressure / 100.0) >= (press_act_ave + 0.04)) && ((bme.pressure / 100.0) < (press_act_ave + 2.0)))
   {  // only press hum v3
      Serial.println("吹いた！！！");
      Serial.print("いいね！");

      if((buttonState_1 == LOW) && (buttonState_2 == LOW) && (buttonState_3 == LOW) && (buttonState_4 == LOW) 
         && (buttonState_5 == LOW) && (buttonState_6 == LOW) && (buttonState_7 == LOW) && (buttonState_8 == LOW))
      {
        Serial.println("ド");
        tone(speakerPin, 262, 500); //ド
        delay(300);
      }

      if((buttonState_1 == LOW) && (buttonState_2 == LOW) && (buttonState_3 == LOW) && (buttonState_4 == HIGH) 
         && (buttonState_5 == LOW) && (buttonState_6 == LOW) && (buttonState_7 == LOW) && (buttonState_8 == LOW))
      {
        Serial.println("レ");
        tone(speakerPin, 294, 500); //レ
        delay(300);
      }

      if((buttonState_1 == LOW) && (buttonState_2 == LOW) && (buttonState_3 == HIGH) && (buttonState_4 == HIGH) 
         && (buttonState_5 == LOW) && (buttonState_6 == LOW) && (buttonState_7 == LOW) && (buttonState_8 == LOW))
      {
        Serial.println("ミ");
        tone(speakerPin, 329, 500); //ミ
        delay(300);
      }

      if((buttonState_1 == LOW) && (buttonState_2 == HIGH) && (buttonState_3 == HIGH) && (buttonState_4 == HIGH) 
         && (buttonState_5 == LOW) && (buttonState_6 == LOW) && (buttonState_7 == LOW) && (buttonState_8 == LOW))
      {
        Serial.println("ファ");
        tone(speakerPin, 349, 500); //ファ
        delay(300);
      }

      if((buttonState_1 == HIGH) && (buttonState_2 == HIGH) && (buttonState_3 == HIGH) && (buttonState_4 == HIGH) 
         && (buttonState_5 == LOW) && (buttonState_6 == LOW) && (buttonState_7 == LOW) && (buttonState_8 == LOW))
      {
        Serial.println("ソ");
        tone(speakerPin, 392, 500); //ソ
        delay(300);
      }

      if((buttonState_1 == HIGH) && (buttonState_2 == HIGH) && (buttonState_3 == HIGH) && (buttonState_4 == HIGH) 
         && (buttonState_5 == LOW) && (buttonState_6 == LOW) && (buttonState_7 == HIGH) && (buttonState_8 == LOW))
      {
        Serial.println("ラ");
        tone(speakerPin, 440, 500); //ラ
        delay(300);
      }

      if((buttonState_1 == HIGH) && (buttonState_2 == HIGH) && (buttonState_3 == HIGH) && (buttonState_4 == HIGH) 
         && (buttonState_5 == LOW) && (buttonState_6 == HIGH) && (buttonState_7 == HIGH) && (buttonState_8 == LOW))
      {
        Serial.println("シ");
        tone(speakerPin, 494, 500); //シ
        delay(300);
      }

      if((buttonState_1 == HIGH) && (buttonState_2 == HIGH) && (buttonState_3 == HIGH) && (buttonState_4 == HIGH) 
         && (buttonState_5 == HIGH) && (buttonState_6 == HIGH) && (buttonState_7 == HIGH) && (buttonState_8 == LOW))
      {
        Serial.println("ド");
        tone(speakerPin, 523, 500); //ド
        delay(300);
      }
   }

   
   if((bme.pressure / 100.0) >= (press_act_ave + 0.5) && (bme.humidity > (hum_act_ave + 2.0)))
   {
      Serial.println("素晴らしい！");
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
