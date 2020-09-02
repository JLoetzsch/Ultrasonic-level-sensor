# Ultrasonic-level-sensor

This code is for an Arduino based ultrasonic level sensor, which can used to determin the level of water in a river. It's a part of a master thesis from Technische Universität Dresden.

# Code

//sketch for low-budget-sensor (02.09.2020)


//necessary libaries, under the point Links you will find the download location from the libaries
#include "LowPower.h"
#include <SD.h>
#include <SPI.h>
#include <DS3231.h>
#include <OneWire.h>


File myFile;                      //erstellt die Variable Datei
DS3231 rtc(SDA, SCL);             //iniziert RTC-Kommunikation

long Messintervall = 60000;       //Zeit der Messabstände in ms, mind. 10000 ms sonst ist der Idle-Mode nicht aktiv
long x_Mess = 10;                 //Anzahl der Messwerte
float xAbstand = 0;               //Abstand Sohle - Messgerät (84.2)

int DS18S20_Pin = 5;              //DS18S20 Signal pin on digital 2
int triggerPIN = 4;               //legt den Namen und Pin für den Trigger am Sensor fest 
int echoPIN = 3;                  //legt den Namen und Pin für den Echo am Sensor fest

float messung = 0;                //Mittelwert aus den Messungen
float ergebnis = 0;               //Berechnete Füllhöhe/Abstand
float vs = 0;                     //Berechnete Schallgeschwindigkeit
float t = 0;                      //gemessene Zeit des Sensors (gesamt)
float tx = 0;                     //gemessene Zeit des Sensors (momentan)
int x_Vergleich = 0;              //Variable zum Hochzählen der Anzahl an Messwerten
int x_Power = 0;                  //Variable zum Hochzählen der Anzahl an Schlafroutinen
long Messintervall_x = 0;         //Berechnung der Anzahl an Schlafroutinen
long Delay = 0;


const int chipSelect = 10;

int pinCS = 10;                   // Pin 10 on Arduino Uno

OneWire ds(DS18S20_Pin);          // on digital pin 2

void setup() {
 
Serial.begin(115200);             // öffnet die serielle Schnittstelle und stellt die Datenrate auf 9600Bit/s ein
delay(20);
pinMode(pinCS, OUTPUT);           //pinMode definiert ob Digitale Pins gleich In oder Output sind
pinMode (triggerPIN, OUTPUT);
pinMode (echoPIN, INPUT);

Messintervall_x = ((Messintervall-(x_Mess*512))/9255); //berechnet die Anzahl der Wiederholungen des 8sekündigen Schlafmodus, um in etwa den Messintervall zu erreichen ((Messintervall-(x_Mess*512))/8000) 8255
Serial.print("Anzahl der Routinen =");
Serial.println(Messintervall_x);
delay(20);

Delay = (Messintervall-(Messintervall_x*9255)-(12*x_Mess)-(512*x_Mess)); //berechnet den Delay um auf auf das Messintervall zu kommen ... Delay = (Messintervall- (Anzahl Messintervalle * Zeit des Schlafzyklus) - (delay der Messung * Anzahl Messung) - (zusätzliches delay der Messung * Anzahl Messung))
Serial.print("Rest Delay [ms] =");
Serial.println(Delay);
delay(20);

//SD, Card Initialization (Gibt im seriellen Monitor die Funktionalität der SD Karte wieder)
if (SD.begin())
{
  Serial.println("SD card is ready to use.");
  delay(20);
  Serial.println("min:h:s ;d.m.y  ;[°C]  ;t [µs]  ;  [cm]");
  delay(20);
  
} else
 
{
delay(1000);
Serial.println("SD card initialization failed");

return; //Unterbrechung des Setups, um es von neuem zu starten

}

rtc.begin();

}

void loop() {

while (x_Power < Messintervall_x) //Routine für Idle-Mode, wird so lange wiederholt bis der Messintervall erreicht ist
{
LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
x_Power++;
Serial.println(x_Power);                 //Kontrolle für seriellen Monitor
delay(1000);                              //Delay für eine bessere serielle Kommunikation
}
   delay(Delay);                         //Ausgleichs delay
   x_Power = 0;
 
   while (x_Mess > x_Vergleich)          //Messroutine aus x Messwerten wird ein Mittelwert gebildet
  {
  digitalWrite(triggerPIN,LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPIN,LOW);

  tx = pulseIn(echoPIN, HIGH);
  
    while (tx <= 1)                      //Routine im Fall eines Nullwertes
      {
        digitalWrite(triggerPIN,LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPIN,HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPIN,LOW);

        tx = pulseIn(echoPIN, HIGH);
      }
  
  t = (t+tx);                           //Aufsummierung der Messwerte 
  x_Vergleich++;                        //Zähler für die x Durchläufe
  Serial.print(t);                      //Kontrolle der Zwischenwerte
  Serial.print(";");
  Serial.print(tx);
  Serial.print(";");
  Serial.println(x_Vergleich);
  delay(500);
  }

 float temperature = getTemp();        //Temperatur auslesen 

  x_Vergleich = 0;                      //nullen des Zählers
  messung = (t/x_Mess);                 //Berechnung des Mittelwertes 
  t = 0;                                //nullen der Summe
  vs = (331.5+(0.6*temperature));       //Berechnung der Schallgeschindigkeit in abhängikeit von der Temperatur     
  ergebnis = (xAbstand-(vs*messung/(2*10000)));    //Berechnung des Abstandes/Füllhöhe

  Serial.print(rtc.getTimeStr());       //Kontrolle durch seriellen Output der Daten
  Serial.print(";");
  Serial.print(rtc.getDateStr());
  Serial.print(";");
  Serial.print(temperature);
  Serial.print("°C");
  Serial.print(";");
  Serial.print(messung);
  Serial.print(";");
  Serial.print(ergebnis,1);
  Serial.println("cm");

  myFile = SD.open("daten.txt", FILE_WRITE);  //Schreiben der Daten auf die SD
  if (myFile) {
    myFile.print(rtc.getTimeStr());
    myFile.print(";");
    myFile.print(rtc.getDateStr());
    myFile.print(";");
    myFile.print(temperature);
    myFile.print(";");
    myFile.print(tx);
    myFile.print(";");
    myFile.print(messung);
    myFile.print(";");
    myFile.println(ergebnis,1);
   

    myFile.close(); //close the file
  }

  //if the file didn't open, print an errror:
  else {
    Serial.println("error opening daten.txt");
  }


} 

float getTemp(){    //Beiwerk zur Temperatur 
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;

}

# Links

DS3231.h - https://github.com/NorthernWidget/DS3231
OneEire.h - https://www.pjrc.com/teensy/td_libs_OneWire.html
LowPower.h - https://github.com/rocketscream/Low-Power
