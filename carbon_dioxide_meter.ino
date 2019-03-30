nclude <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>;
#include <Encoder.h>
#include "Timer.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MHZ19_uart.h>


#define MHZ_RX 2
#define MHZ_TX 3
#define DHTPIN            13 
#define DHTTYPE           DHT11     // DHT 11 
//#define DHTTYPE           DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);

SoftwareSerial mySerial(7, 8); // 4 - к TX сенсора, 5 - к RX

byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
unsigned char response[9];

int tem;
int hum;

// Объект класса для дисплея
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define ENCODER_DO_NOT_USE_INTERRUPTS
Encoder myEnc(2, 3); //подключение энкодера

int ppm;
int flag=0;

Timer t;
void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  dht.begin();
  sensor_t sensor;
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
t.every(15000, get_co2);
t.every(2000, get_temp);
}
long criticalppm = -999;

void loop()
{
t.update();



  long newppm = myEnc.read();
    if (newppm != criticalppm) {
    criticalppm = newppm;
    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
    lcd.setCursor(0, 1);
    lcd.print("CO2=");lcd.print(ppm);
    lcd.setCursor(10, 1);
    lcd.print("V:");lcd.print(criticalppm);

    lcd.setCursor(0, 0);
    lcd.print("T=");lcd.print(tem);lcd.print(" C");
    lcd.setCursor(9, 0);
    lcd.print("H=");lcd.print(hum);lcd.print(" %");
    

  //if (criticalppm <= ppm)
  //  digitalWrite(7,LOW);
  //else digitalWrite(7,HIGH);
}
//=================================================
void get_co2()
{
  mySerial.write(cmd, 9);
  memset(response, 0, 9);
  mySerial.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc+=response[i];
  crc = 255 - crc;
  crc++;

  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8]));
  } else {
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    ppm = (256*responseHigh) + responseLow;
        Serial.println(ppm);

    lcd.setCursor(0, 1);
    lcd.print("                ");
  }
}
//=================================================
void get_temp(){
    sensors_event_t event;  
  dht.temperature().getEvent(&event);
tem=event.temperature;
  dht.humidity().getEvent(&event);
hum=event.relative_humidity;
    lcd.setCursor(0, 0);
    lcd.print("                ");Серийников бессрочных 4.0 меньше 900 (Текущее значение = 899) 
}

