/*
  this code is used to initialize the EEPROM with values that is used as a settings
  flash this code before flashing the interface.ino source code
*/
#include <EEPROM.h>

#define DEBUG 1
//
//#define PHAddress 0
//#define ECAddress 4
#define OffsetAddress 8
#define PHValue 12
#define ECValue 16
#define LettucePH 20
#define LettuceEC 24
#define StrawberryPH 28
#define StrawberryEC 32
#define TomatoPH 36
#define TomatoEC 40
#define DefaultPH 100
#define DefaultEC 104
#define VoltPhSlope 108
#define CalTemp 112
#define SAVE 200

float PH=4.00f, EC=2.00f, offset=0.50f, temp=0.00f; 
float lettucePH = 6.5f, lettuceEC =1.5f ,strawberryPH=6.0f, strawberryEC=2.0f, tomatoPH=6.25f, tomatoEC=3.0f; 

void setup() {
  Serial.begin(9600);
  Serial.println("initializing the setting process");
  Serial.println("Saving PH");
  EEPROM.put(DefaultPH, PH);
  Serial.println("Saving EC");
  EEPROM.put(DefaultEC, EC);
  Serial.println("Saving Offset");
  EEPROM.put(OffsetAddress, offset);
  Serial.println("Saving the preset EC and PH values in EEPROM");
  EEPROM.put(LettucePH, lettucePH);
  EEPROM.put(LettuceEC, lettuceEC);
  EEPROM.put(StrawberryPH, strawberryPH);
  EEPROM.put(StrawberryEC, strawberryEC);
  EEPROM.put(TomatoPH, tomatoPH);
  EEPROM.put(TomatoEC, tomatoEC);
  EEPROM.put(DefaultPH, float(4.00f));
  EEPROM.put(DefaultEC, float(2.00f));
  EEPROM.put(VoltPhSlope, float(0.40f));
  EEPROM.put(CalTemp,float(25.0f));
  EEPROM.write(SAVE, 0b00000000);

  if(DEBUG){
    // get operation to check the EEPROM content
    EEPROM.get(DefaultPH,temp);
    Serial.print("PH=");
    Serial.println(temp);
    EEPROM.get(DefaultEC, temp);
    Serial.print("EC=");
    Serial.println(temp);
    EEPROM.get(OffsetAddress, temp);
    Serial.print("offset=");
    Serial.println(temp);
   
    EEPROM.get(LettucePH,temp);
    Serial.print("Lettuce PH=");
    Serial.print(temp);
    EEPROM.get(LettuceEC,temp);
    Serial.print(" and EC=");
    Serial.println(temp);
    
    EEPROM.get(StrawberryPH,temp);
    Serial.print("Strawberry PH=");
    Serial.print(temp);
    EEPROM.get(StrawberryEC,temp);
    Serial.print(" and EC=");
    Serial.println(temp);

    EEPROM.get(TomatoPH,temp);
    Serial.print("Tomato PH=");
    Serial.print(temp);
    EEPROM.get(TomatoEC,temp);
    Serial.print(" and EC=");
    Serial.println(temp);
  
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
