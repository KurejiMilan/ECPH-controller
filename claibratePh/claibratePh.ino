#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define VoltPhSlope 108
#define CalTemp 112

OneWire oneWire(8);
DallasTemperature sensor(&oneWire);

uint8_t state = 0x00;
float voltPhSlope = 0.00f, calTemp = 0.00f, ph1=0.00f,ph2=0.00f,v1 = 0.00f, v2 = 0.00f, temp=0.00f;
uint16_t sample = 0; 
char c = "";

void setup() {
  Serial.begin(9600);
  sensor.begin();
  Serial.println("Calibrating the Ph probe");
  Serial.println("Inset Ph probe in the buffer solution");
  Serial.println("Enter s to start the sampling of Ph buffre solution");
  while(Serial.available()!=1);
  c = Serial.read();
  if(c=="s"){
    temp = 0.00f;
    for(int i=0; i<10; i++){
      sample = analogRead(A0);
      temp += sample*(5/1023);      
    }
    v1 = temp/10;
  }
  Serial.println("Inset Ph probe in the buffer solution");
  Serial.println("Enter s to start the sampling of Ph buffre solution");
  while(Serial.available()!=1);
  c = Serial.read();
  if(c=="s"){
    temp = 0.00f;
    for(int i=0; i<10; i++){
      sample = analogRead(A0);
      temp += sample*(5/1023);      
    }
    v2 = temp/10;
  }
  v1 = (v1/3)-float(2.5/3.0);
  v2 = (v2/3)-float(2.5/3.0);
  float voltPhslope = (v1-v2)/(ph1-ph2);
  Serial.print("volts per ph slope=");
  Serial.println(voltPhslope,6);
  EEPROM.write(VoltPhSlope, voltPhslope);
  Serial.println("Getting temperature reading");
  sensor.requestTemperatures();
  temp = sensor.getTempCByIndex(0);
  EEPROM.write(CalTemp, temp);
}

void loop() {

}
