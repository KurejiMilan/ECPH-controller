#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

#ifndef setBit
#define setBit(F,B) (F |= (0x01<<B))
#endif

#ifndef clearBit
#define clearBit(F, B) (F &= ~(0x01<<B))
#endif

#ifndef checkBit
#define checkBit(F, B) (F&(0x01<<B))
#endif

#define PHSensorPin   A4
#define ECSensorPin   A5
#define ONE_WIRE_BUS  8

#define DEBUG   1
#define RESET   A2
#define ENABLE  A3
#define D4      7
#define D5      6
#define D6      5
#define D7      4


#define OffsetAddress 8
#define PHValue       12
#define ECValue       16
#define LettucePH     20
#define LettuceEC     24
#define StrawberryPH  28
#define StrawberryEC  32
#define TomatoPH      36
#define TomatoEC      40
#define DefaultPH     100
#define DefaultEC     104
#define VoltPhSlope   108
#define CalTemp       112
#define SAVE          200

#define ENCOD1DIR 0
#define ENCOD2DIR 1
#define ENCOD1BTN 2
#define ENCOD2BTN 3
#define PRINTLCD  4
#define PHUP      5
#define PHDOWN    6
#define ECUP      7

float PH=0.00f, EC=0.00f, offset=0.00f, measuredPh=0.00f, measuredEc=0.00f;
float temperature = 0.00f, phVoltage=0.00f, ecVoltage=0.00, voltPhSlope=0.00f, calTemp = 0.00f, atcVoltPhSlope=0.00f; 
float phError = 0.00f, phPrevError = 0.00f, ecError=0.00f, ecPrevError = 0.00f, phIntegral = 0.00f, ecIntegral=0.00f;
float pwm=0.00f;                                                                                                        //for temporary operation
const float Vin = 1.00f, Rf = 1000, phPGain=0.00f, phIGain=0.00f, phDGain=0.00f, ecPGain=0.00f, ecIGain=0.00f, ecDGain=0.00f;

uint16_t phSample = 0, ecSample = 0;
uint8_t currentState=0, state = 0, state0Active = 0, state1Active = 0;                         //these State variable are used to represent state of the UI 
int8_t cursorPointer = 0;

String presetInterface[] = {"Lettuce", "StrawBerry", "Tomato", "aquatic"};

byte arrow[] = {B00000,B00100,B00110,B11111,B11111,B00110,B00100,B00000};
byte tick[] = {B00000,B00000,B00001,B00010,B10100,B01000,B00000,B00000};
byte set[] = {B01010,B01010,B01010,B01010,B01010,B01010,B01010,B01010};

// look up table stores the various states of the rotary encoder and related directional output 
int8_t lookUpTable[16] = {0, 0, 0, 0,1, 0, 0, 0,-1, 0, 0, 0,0, 0, 0, 0};                          
uint8_t PHLookUpTable[4] = {LettucePH, StrawberryPH, TomatoPH, 0};
uint8_t ECLookUpTable[4] = {LettuceEC, StrawberryEC, TomatoEC, 0};

volatile uint8_t encoder1State=0x00;                                              // stores the state of the rotary encoder 1
volatile uint8_t encoder2State=0x00;                                              // stores the state of the rotary encoder 2
volatile int8_t encoder1Direction = 0;                                            // stores the direction of the rotary encoder 1
volatile int8_t encoder2Direction = 0;                                            // stores the direction of the rotary encoder 2

/*
          bit7  |   bit6  |   bit5   |    bit4   |    bit3     |    bit2    |   bit1    |   bit0
          EC_up | PH_DOWN |   PH_UP  | print_lcd |  encod2_btn |  ecod1_btn | ecod2_dir | encod1_dir
*/

uint8_t flags = 0x00;                                                             //using bit field data type to save space
bool active = true;
uint8_t phUpPWM = 0x00, phDownPWM=0x00, ecUpPWM =0x00;
uint64_t timer = 0;
/*
  savedSetting is used to store the seleted set value of PH and EC
  bit6 and bit5 is used to store either manual or preset value
  00 = manual, 01 = preset
  bit3, bit2, bit1 and bit0 is used as index for PH and EC look up table
*/
int8_t savedSetting = 0x00;

ISR(PCINT0_vect){
  encoder1State = encoder1State<<2;
  encoder1State |= (PINB&0b00110000)>>4;
  encoder1Direction = lookUpTable[encoder1State&0x0F];
  if(encoder1Direction!=0)setBit(flags, ENCOD1DIR);
}

ISR(PCINT1_vect){
  encoder2State = encoder2State<<2;
  encoder2State |= PINC&0b00000011;
  encoder2Direction = lookUpTable[encoder2State&0x0F];
  if(encoder2Direction!=0)setBit(flags,ENCOD2DIR);
}

ISR(INT0_vect){
  setBit(flags, ENCOD1BTN);
  //Serial.println("b");
}
ISR(INT1_vect){
  setBit(flags, ENCOD2BTN);
}

//void transmitSettings(uint8_t);
void navigate();
void printScreen();

LiquidCrystal lcd(RESET, ENABLE, D4, D5, D6, D7);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);

void setup() {
  cli();
  Serial.begin(9600);
  sensor.begin();

  DDRB = (1<<DDB0)|(1<<DDB1)|(1<<DDB2);                           //PORT B pin 0 = phup, pin1 =phdown, pin2 = ec 
  TCCR1B = (1<<WGM12)|(1<<CS11);
  TCCR1A |= (1<<WGM10);
  TCCR2B |= (1<<CS21);
  TCCR2A |= (1<<WGM21)|(1<<WGM20);
  //TCCR1A |= (1<<COM1A1);
  //TCCR1A |= (1<<COM1B1);
  //TCCR2A |= (1<<COM2A1);
  OCR1A=0;
  OCR1B=0;
  OCR2A=0;
  
  lcd.createChar(0, arrow);
  lcd.createChar(1, tick);
  lcd.createChar(2, set);
  lcd.begin(16, 2);
  lcd.noCursor();
  lcd.print("AquaBalancer");
  
  /*
    PCINT22 pin is connected to the pin A of the rotary encoder 1
    PCINT21 pin is connected to the pin B of the rotary encoder 2
    INT0 is used for rotary encoder 1 push button
    INT1 is used for rotary encoder 2 push button 
  */
  PCICR |= (1<<PCIE1)|(1<<PCIE0);                                                         //pin change interrupt control register
  PCMSK0 = (1<<PCINT4)|(1<<PCINT5);                                                       //PCINT4=> Rotary encoder B, PCINT5=> Rotary encoder A                                              //pin change mask register
  PCMSK1 = (1<<PCINT8)|(1<<PCINT9);
  EIMSK = 0b00000011;
  EICRA = 0x0F;

  savedSetting = EEPROM.read(SAVE);
  if((savedSetting&0x10)==0x10){
    EEPROM.get(PHLookUpTable[savedSetting&0x0f], PH);
    EEPROM.get(ECLookUpTable[savedSetting&0x0f], EC);
  }else{
    EEPROM.get(DefaultPH, PH);  
    EEPROM.get(DefaultEC, EC);
  }
  EEPROM.get(OffsetAddress,offset);
  EEPROM.get(VoltPhSlope, voltPhSlope);
  EEPROM.get(CalTemp, calTemp);
  sei(); 
  delay(1000);
  lcd.clear();                                                                               //sets the global interrupt bit in the SREG
  flags = 0b00010000;
  timer = millis();
}

void loop() {

  sensor.requestTemperatures();
  temperature = sensor.getTempCByIndex(0);
  phSample = analogRead(PHSensorPin);
  ecSample = analogRead(ECSensorPin);
  
  phVoltage = phSample*(5/1023);
  phVoltage = (phVoltage/3)-float(2.5/3.00);
  atcVoltPhSlope = (temperature*voltPhSlope)/calTemp;
  measuredPh = atcVoltPhSlope*phVoltage;

  ecVoltage = ecSample*(5/1023); 
  measuredEc = ((ecVoltage-Vin)/(Rf*Vin))*(1+(25.00f-temperature)*0.0185f);                                                                       // k=1, this can be calibrate which can be updated later  

  if(active){
    if(millis()-timer>50){
      timer = millis();
      active = false;
      if(checkBit(flags, PHUP))TCCR1A &= ~(1<<COM1A1);
      if(checkBit(flags, PHDOWN))TCCR1A &= ~(1<<COM1B1);
      if(checkBit(flags, ECUP))TCCR2A &= ~(1<<COM2A1);
    }
  }else{
    if(millis()-timer>1000){
      timer = millis();
      active = true;
      if(checkBit(flags, PHUP))TCCR1A |= 1<<COM1A1;
      if(checkBit(flags, PHDOWN))TCCR1A |= 1<<COM1B1;
      if(checkBit(flags, ECUP))TCCR2A |= 1<<COM2A1;
    }
  }  
  if(active){
    if(measuredPh<PH-offset){
      phError = PH-measuredPh;
      phIntegral += phError*phIGain; 
      pwm = (phError*phPGain)+phIntegral+((phError-phPrevError)*phDGain);
      if(pwm>200)pwm=200;
      else if(pwm<4)pwm=4;
      phUpPWM = uint8_t(pwm);
      phPrevError = phError;
      if(!checkBit(flags, PHUP)){
        setBit(flags, PHUP);
        TCCR1A |= (1<<COM1A1);
      }
      OCR1A = phUpPWM;  
    }
    if(measuredPh>PH+offset){
      phError = PH-measuredPh;
      phIntegral -= phError*phIGain; 
      pwm = (phError*phPGain)+phIntegral+((phError-phPrevError)*phDGain);
      if(pwm>200)pwm=200;
      else if(pwm<4)pwm=4;
      phDownPWM = uint8_t(pwm);
      phPrevError = phError;
      if(!checkBit(flags, PHDOWN)){
        setBit(flags, PHDOWN);
        TCCR1A |= (1<<COM1B1);
      }
      OCR1B =phDownPWM;  
    }
    if(measuredEc<EC-offset){
      ecError = EC-measuredEc;
      ecIntegral += ecError*ecIGain;
      pwm = (ecError*ecPGain)+ecIntegral+((ecError-ecPrevError)*ecDGain);
      if(pwm>200)pwm=200;
      else if(pwm<4)pwm=4;
      ecUpPWM = uint8_t(pwm);
      ecPrevError = ecError;
      if(!checkBit(flags, ECUP)){
        setBit(flags, ECUP);
        TCCR2A |= (1<<COM2A1);
      }
      OCR2A = ecUpPWM;
    }
    if(measuredPh>=PH-offset){
      if(checkBit(flags, PHUP)){
        TCCR1A &= ~(1<<COM1A1);
        clearBit(flags, PHUP);
        phError = 0;
        phPrevError= 0;
        phIntegral =0;
        OCR1A = 0;
      }
    }
    if(measuredPh<=PH+offset){
      if(checkBit(flags, PHDOWN)){
        TCCR1A &= (~1<<COM1B1);
        clearBit(flags, PHDOWN);
        phError = 0;
        phPrevError = 0;
        phIntegral = 0;
        OCR1B = 0;
      }
    }
    if(measuredEc>=EC+offset){
      if(checkBit(flags, ECUP)){
        TCCR2A &= ~(1<<COM2A1);
        clearBit(flags, ECUP);
        ecError = 0;
        ecPrevError = 0;
        ecIntegral = 0;
        OCR2A =0;
      }
    }
  }
  
  if(flags&0x0F){
    state = currentState;
    Serial.println(flags, BIN);
    switch(state){  
      case 0:
        if(checkBit(flags,ENCOD1BTN)){
          state0Active = cursorPointer;
          currentState = 1;
          cursorPointer = 0;
          clearBit(flags,ENCOD1BTN);
          setBit(flags, PRINTLCD);
        }else if(checkBit(flags,ENCOD1DIR)){
          clearBit(flags,0);
          if(encoder1Direction==1){
            ++cursorPointer;
            if(cursorPointer>=2)cursorPointer=0;
          }else{
            --cursorPointer;
            if(cursorPointer<0)cursorPointer=1;
          }
          encoder1Direction=0;
          setBit(flags, PRINTLCD);  
        }
      clearBit(flags, ENCOD2DIR);   
      clearBit(flags, ENCOD2BTN);  
      break;
        
      case 1:
        if(state0Active == 1){
          if(checkBit(flags,ENCOD1BTN)){
            state1Active = cursorPointer;
            currentState = 2;
            cursorPointer = 0;
            clearBit(flags,2);
            setBit(flags, PRINTLCD);  
          }else if(checkBit(flags,ENCOD2BTN)){
            currentState = 0;
            cursorPointer = 0;
            clearBit(flags, 3);
            setBit(flags, PRINTLCD);
          }else if(checkBit(flags,ENCOD1DIR)){
              clearBit(flags,ENCOD1DIR);
              if(encoder1Direction==1){
                ++cursorPointer;
                if(cursorPointer>=3)cursorPointer=0;
              }else{
                --cursorPointer;
                if(cursorPointer<0)cursorPointer=2;
              }
              encoder1Direction=0;
              setBit(flags, PRINTLCD);  
          } 
        }
        else if(state0Active == 0){
          if(checkBit(flags,ENCOD2BTN)){
            currentState = 0;
            cursorPointer = 0;
            clearBit(flags,ENCOD2BTN);
            setBit(flags, PRINTLCD);
          }
        } 
      clearBit(flags, ENCOD2DIR);       
      break;
        
      case 2:
        if(state0Active==1){
          
          if(state1Active == 0){                                        // manual PH settings                     
            if(checkBit(flags, ENCOD2DIR)){
              clearBit(flags, ENCOD2DIR);
              if(cursorPointer==0){                                     // change PH on cursor 0
                if(encoder2Direction == 1){
                  PH += 0.1f;
                  if(PH>=14.00f)PH=14.00f;
                }else{
                  PH -= 0.1f;
                  if(PH<=0.00f)PH=0.00f;
                }
                setBit(flags, PRINTLCD);
              }else if(cursorPointer == 1){                             // change PH on cursor 1
                if(encoder2Direction == 1){
                  EC += 0.1f;
                }else{
                  EC -= 0.1f;
                }
                setBit(flags, PRINTLCD);
              }
              encoder2Direction = 0;
            }else if(checkBit(flags, ENCOD1BTN)){
               if(cursorPointer==2){                                  // only relevent when pointer is on save
                   EEPROM.put(DefaultPH, PH);                         // save to DEFAULT PH AND EC
                   EEPROM.put(DefaultEC, EC);
                   savedSetting &= 0x0f;
                   EEPROM.write(SAVE, savedSetting);
                   Serial.println("I'm here");
               }
               clearBit(flags, ENCOD1BTN);
            }else if(checkBit(flags, ENCOD2BTN)){
                cursorPointer = 0;
                currentState = 1;
                setBit(flags, PRINTLCD);
                clearBit(flags, ENCOD2BTN);
            }else if(checkBit(flags, ENCOD1DIR)){
              clearBit(flags,ENCOD1DIR);
              if(encoder1Direction==1){
                ++cursorPointer;
                if(cursorPointer==3)cursorPointer=0;
              }else{
                --cursorPointer;
                if(cursorPointer<0)cursorPointer=2;
              }
              encoder1Direction=0;
              setBit(flags, PRINTLCD);  
            }
          }else if(state1Active == 1){                                // PRESET SETTINGS
            if(checkBit(flags, ENCOD1DIR)){
              clearBit(flags,ENCOD1DIR);
              if(encoder1Direction==1){
                ++cursorPointer;
                if(cursorPointer>=4)cursorPointer=0;
              }else{
                --cursorPointer;
                if(cursorPointer<0)cursorPointer=3;
              }
              encoder1Direction=0;
              setBit(flags, PRINTLCD); 
            }else if(checkBit(flags, ENCOD2BTN)){
              cursorPointer = 0;
              currentState = 1;
              setBit(flags, PRINTLCD);
              clearBit(flags, ENCOD2BTN);
            }else if(checkBit(flags, ENCOD1BTN)){
              clearBit(flags, ENCOD1BTN);
              savedSetting = 0x0f&cursorPointer;
              PH = PHLookUpTable[savedSetting&0x0f];
              EC = ECLookUpTable[savedSetting&0x0f];
              savedSetting |= 0x10;
              EEPROM.write(SAVE, savedSetting);
              setBit(flags, PRINTLCD);
            }
          }else if(state1Active == 2){                                // OFFSET SETTINGS
            if(checkBit(flags, ENCOD2BTN)){
                cursorPointer = 0;
                currentState = 1;
                setBit(flags, PRINTLCD);
                clearBit(flags, ENCOD2BTN);
            }else if(checkBit(flags, ENCOD2DIR)){
              clearBit(flags, ENCOD2DIR);
              if(encoder2Direction == 1){
                offset += 0.1f;
                if(offset >= 1.5) offset = 1.5;
              }else{
                offset -= 0.1f;
                if(offset <= 0.1) offset = 0.1;
              }
              encoder2Direction = 0;
              setBit(flags, PRINTLCD);
            }else if(checkBit(flags, ENCOD1BTN)){
              clearBit(flags, ENCOD1BTN);
              if(cursorPointer==1)EEPROM.put(OffsetAddress, offset);
            }else if(checkBit(flags, ENCOD1DIR)){
              clearBit(flags,ENCOD1DIR);
              if(encoder1Direction==1){
                ++cursorPointer;
                if(cursorPointer>=2)cursorPointer=0;
              }else{
                --cursorPointer;
                if(cursorPointer<0)cursorPointer=1;
              }
              encoder1Direction=0;
              setBit(flags, PRINTLCD); 
            }
          } 
        }
      clearBit(flags, ENCOD1DIR); 
      clearBit(flags, ENCOD2DIR);  
      clearBit(flags, ENCOD1BTN); 
      clearBit(flags, ENCOD2BTN);  
      break;
    Serial.println(flags, BIN);
    }
    flags &=~0x0f;      
  }
  if(checkBit(flags,PRINTLCD))printScreen();
}




// navigate handle all the input function and corresponding actions
//
void navigate(){
  
}

// printScreen function is used to print when menuvering throught the menu
void printScreen(){
    switch(currentState){  
      case 0:
        lcd.clear();
        if(cursorPointer == 0)lcd.setCursor(0,0);
        else lcd.setCursor(0,1);
        lcd.write(uint8_t(0));
          
        lcd.setCursor(1, 0);
        lcd.print("Monitor");
        lcd.setCursor(1, 1);
        lcd.print("Set Value");
      break;
      
      case 1:
      lcd.clear();
        if(state0Active==0){
          lcd.setCursor(0, 0);
          lcd.print("PH:");
          lcd.print(measuredPh,1);
          lcd.setCursor(7,0);
          lcd.write(uint8_t(2));
          lcd.print(PH,1);
   
          lcd.setCursor(0, 1);
          lcd.print("EC:");
          lcd.print(measuredEc,1);
          lcd.setCursor(7,1);
          lcd.write(uint8_t(2));
          lcd.print(EC,1);
        }else if(state0Active==1){
          if(cursorPointer<2){
            if(cursorPointer==0)lcd.setCursor(0,0);
            else lcd.setCursor(0,1);
            lcd.write(uint8_t(0));
            if((savedSetting&0x10)==0x10)lcd.setCursor(15,1);
            else lcd.setCursor(15,0);
            lcd.write(uint8_t(1));
            lcd.setCursor(1,0);
            lcd.print("Manual");
            lcd.setCursor(1,1);
            lcd.print("Preset");
          }else{
            lcd.setCursor(0,1);
            lcd.write(uint8_t(0));
            if((savedSetting&0x10)==0x10){
              lcd.setCursor(15,0);
              lcd.write(uint8_t(1));
            }
            lcd.setCursor(1,0);
            lcd.print("Preset");
            lcd.setCursor(1,1);
            lcd.print("Offset");
          }    
        }
      break;
      
      case 2:
        lcd.clear();
        if(state0Active==1){
          if(state1Active==0){                                      //Manual PH and EC setting
            if(cursorPointer<2){
              if(cursorPointer==0)lcd.setCursor(0,0);
              else lcd.setCursor(0,1);
              lcd.write(uint8_t(0));
              lcd.setCursor(1,0);
              lcd.print("PH:");                                     //values from eeprom
              lcd.print(PH,1);
              lcd.setCursor(1,1);
              lcd.print("EC:");
              lcd.print(EC,1);
            }else{
              lcd.setCursor(0,1);
              lcd.write(uint8_t(0));
              lcd.setCursor(1,0);
              lcd.print("EC:");
              lcd.print(EC);
              lcd.setCursor(1,1);
              lcd.print("Save");
            }
          }else if(state1Active==1){
            if(cursorPointer<2){
              if(cursorPointer==0)lcd.setCursor(0,0);
              else lcd.setCursor(0,1);
              lcd.write(uint8_t(0));
              lcd.setCursor(1,0);
              lcd.print(presetInterface[0]);
              lcd.setCursor(1,1);
              lcd.print(presetInterface[1]);
            }else{
              lcd.setCursor(0,1);
              lcd.write(uint8_t(0));
              lcd.setCursor(1,0);
              lcd.print(presetInterface[cursorPointer-1]);
              lcd.setCursor(1,1);
              lcd.print(presetInterface[cursorPointer]);
            }
          }else if(state1Active==2){
            if(cursorPointer==0)lcd.setCursor(0,0);
            else lcd.setCursor(0,1);
            lcd.write(uint8_t(0));
            lcd.setCursor(1,0);
            lcd.print("Offset:");                                     //some value form eeprom
            lcd.print(offset);
            lcd.setCursor(1,1);
            lcd.print("Save");
          }  
        }
      break;
      
      default:
        lcd.clear();
        lcd.setCursor(3, 0);
        lcd.print("Error");
    }
    clearBit(flags, PRINTLCD);   
}


/*transmit Settings functiont takes a uint8_t argument which is used to send various controll parameters*/
/*
void transmitSettings(uint8_t n){
  switch(n){
    case 1:                                                                 // transmit PH
      b = (byte *)&PH; 
      Serial.print("P");
    break;
    case 2:                                                                 // transmit EC
      b = (byte *)&EC;
      Serial.print("E");
    break;
    case 3:                                                                 // transmit offset
      b = (byte *)&offset;
      Serial.print("O");
    break;
  }
  while(Serial.available()!=1);
  c = Serial.read();
  if(c=='a'){
    Serial.write(*b);
    Serial.write(*(b+1));
    Serial.write(*(b+2));
    Serial.write(*(b+3));  
  }
  while(Serial.available() != 1);
  if(Serial.read() == "s"){};
}
*/
