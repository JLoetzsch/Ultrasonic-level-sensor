## Ultrasonic-level-sensor

This code is for an Arduino based ultrasonic level sensor, which can used to determin the level of water.
## Code
```
//sketch for low-budget-sensor (02.09.2020)


//necessary libaries, under the point links you will find the download location for the libaries
#include "LowPower.h"
#include <SD.h>
#include <SPI.h>
#include <DS3231.h>
#include <OneWire.h>


File myFile;                      
DS3231 rtc(SDA, SCL);             //initiation of RTC

long Messintervall = 60000;       //time of measure interval in ms
long x_Mess = 10;                 //amount of measured values
float xAbstand = 0;               //distance bottom - ultrasonic sensor

int DS18S20_Pin = 5;              //DS18S20 signal pin on digital 5 (temperature sensor)
int triggerPIN = 4;               //ultrasonic sensor trigger signal on digital 4
int echoPIN = 3;                  //ultrasonic sensor echo signal on digital 3

float messung = 0;                //mean value from measurements
float ergebnis = 0;               //result of caculated water level
float vs = 0;                     //caculated speed of sound
float t = 0;                      //measured time of ultrasonic sensor (total)
float tx = 0;                     //measured time of ultrasonic sensor (from single measurement)
int x_Vergleich = 0;              //variable for the amount of measured values
int x_Power = 0;                  //variable for the amount of idle mode routines
long Messintervall_x = 0;         //variable for calculation of the amount of idle mode routines
long Delay = 0;


const int chipSelect = 10;        //chip select pin on digital 10 (for SD-Card Modul)

int pinCS = 10;                   //chip select pin on digital 10 (for SD-Card Modul)

OneWire ds(DS18S20_Pin);          //


void setup() {
 
Serial.begin(115200);             // starts the serial communication

pinMode(pinCS, OUTPUT);           //pinCS = Output
pinMode (triggerPIN, OUTPUT);     //triggerPin = Output
pinMode (echoPIN, INPUT);         //echoPin = Input

Messintervall_x = ((Messintervall-(x_Mess*212))/8355);
//calculate the amount of repetitions of idle mode, formula: Amount of repetions = ((time of measure interval)-(amount of measured values)*(total delay during the measurement (212ms)))/((time of idle mode (8255ms) + (total delay during idle mode routine(100ms)))

Serial.print("Anzahl der Routinen =");  //print the result in serial monitor 
Serial.println(Messintervall_x);
delay(20);

Delay = (Messintervall-(Messintervall_x*8355)-(212*x_Mess)); 
//calculate the delay um auf auf das Messintervall zu kommen, formula: Delay = ((time of measure interval)-(amount of repetions)*(total time of idle mode)-(total time of measurment

Serial.print("Rest Delay [ms] =");     //print the result in serial monitor
Serial.println(Delay);
delay(20);

//initialisation of SD Card-Modul
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

return; //stops "void setup"

}

rtc.begin();

}


void loop() {

while (x_Power < Messintervall_x) //routine of idle mode

{
LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
                
x_Power++;                               //count plus one
Serial.println(x_Power);                 //prints value of counter in serial monitor for control
delay(100);                              //delay for a better serial communication (1000ms)

}
   delay(Delay);                         //rest delay
   x_Power = 0;                          //zeroing the counter
 
   while (x_Mess > x_Vergleich)          //start measurement rountine
  {
  digitalWrite(triggerPIN,LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPIN,LOW);

  tx = pulseIn(echoPIN, HIGH);           //save the result
  
    while (tx <= 1)                      //start: checking the result, in case of value of zero the measurement will repeat
      {
        digitalWrite(triggerPIN,LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPIN,HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPIN,LOW);

        tx = pulseIn(echoPIN, HIGH);    //overwrite the result
      }                                 //end: checking result
  
  t = (t+tx);                           //summation of the measured values
  x_Vergleich++;                        //counter plus one
  Serial.print(t);                      //prints the sum; actual value; counter in serial monitor for control
  Serial.print(";");
  Serial.print(tx);
  Serial.print(";");
  Serial.println(x_Vergleich);         
  delay(200);                           //delay for a safe power supply between the measurements (500ms)
  }                                     //end of the measurement routine

 float temperature = getTemp();         //get the temperature  

  x_Vergleich = 0;                      //zeroing the counter
  messung = (t/x_Mess);                 //calculate mean value of measurements 
  t = 0;                                //zeroing the sum
  vs = (331.5+(0.6*temperature));       //calculate approximately the sound of speed     
  ergebnis = (xAbstand-(vs*messung/(2*10000))); //calculate the water level, formula: (water level) = (distance between bottom - ultrasonic sensor)-((sound of speed)*(mean value of measurements)/(2*10000(cm/m)*(s/µs)))

  Serial.print(rtc.getTimeStr());       //prints time; date; temperature; measured time; water level in serial monitor for control
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

  myFile = SD.open("daten.txt", FILE_WRITE);  //writes time; date; temperature; measured time; water level on the SD-card
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

float getTemp(){   
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
```

## Links

DS3231.h - https://github.com/NorthernWidget/DS3231

OneEire.h - https://www.pjrc.com/teensy/td_libs_OneWire.html

LowPower.h - https://github.com/rocketscream/Low-Power
