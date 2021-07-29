#include <OneWire.h>
#include <DallasTemperature.h>

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
#define PHUP      5
#define PHDOWN    6
#define ECUP      7

float PH=0.00f, EC=0.00f, offset=0.00f, measuredPh=0.00f, measuredEc=0.00f;
float temperature = 0.00f, phVoltage=0.00f, ecVoltage=0.00, voltPhSlope=0.00f, calTemp = 0.00f, atcVoltPhSlope=0.00f; 
float phError = 0.00f, phPrevError = 0.00f, ecError=0.00f, ecPrevError = 0.00f, phIntegral = 0.00f, ecIntegral=0.00f;
float pwm=0.00f;                                                                                                        //for temporary operation
const float Vin = 1.00f, Rf = 1000, phPGain=0.00f, phIGain=0.00f, phDGain=0.00f, ecPGain=0.00f, ecIGain=0.00f, ecDGain=0.00f;

uint16_t phSample = 0, ecSample = 0;

uint8_t flags = 0x00;                                                             //using bit field data type to save space
bool active = true, phPlot = true, ecPlot = false,phnotcomplete=true, ecnotcomplete=true;
uint8_t phUpPWM = 0x00, phDownPWM=0x00, ecUpPWM =0x00;
uint64_t timer = 0;

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
  OCR1A=0;
  OCR1B=0;
  OCR2A=0;
  sei();
}

void loop() {

  sensor.requestTemperatures();
  temperature = sensor.getTempCByIndex(0);
  if(phPlot){
    phSample = analogRead(PHSensorPin);
    phVoltage = (((phSample*5)/1023)/3)-float(2.5/3.00);
    atcVoltPhSlope = (temperature*voltPhSlope)/calTemp;
    measuredPh = atcVoltPhSlope*phVoltage;
  }
  if(ecPlot){
    ecSample = analogRead(ECSensorPin);
    ecVoltage = ecSample*(5/1023); 
    measuredEc = ((ecVoltage-Vin)/(Rf*Vin))*(1+(25.00f-temperature)*0.0185f);                                                                       // k=1, this can be calibrate which can be updated later  
  }
  if(active){
    if(millis()-timer>50){
      timer = millis();
      active = false;
      if(checkBit(flags, PHUP)&&phPlot)TCCR1A &= ~(1<<COM1A1);
      if(checkBit(flags, PHDOWN)&&phPlot)TCCR1A &= ~(1<<COM1B1);
      if(checkBit(flags, ECUP)&&ecPlot)TCCR2A &= ~(1<<COM2A1);
    }
  }else{
    if(millis()-timer>1000){
      timer = millis();
      active = true;
      if(checkBit(flags, PHUP)&&phPlot)TCCR1A |= 1<<COM1A1;
      if(checkBit(flags, PHDOWN)&&phPlot)TCCR1A |= 1<<COM1B1;
      if(checkBit(flags, ECUP)&&ecPlot)TCCR2A |= 1<<COM2A1;
    }
  }  
  if(active){
    if(phPlot){
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
    }  
    if(ecPlot){
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
    }
    if(phPlot){
      if(measuredPh>=PH-offset){
        if(checkBit(flags, PHUP)){
          TCCR1A &= ~(1<<COM1A1);
          clearBit(flags, PHUP);
          phError = 0;
          phPrevError= 0;
          phIntegral =0;
          OCR1A = 0;
          phnotcomplete =false;
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
          phnotcomplete = false;
        }
      }
    }
    if(ecPlot){
      if(measuredEc>=EC+offset){
        if(checkBit(flags, ECUP)){
          TCCR2A &= ~(1<<COM2A1);
          clearBit(flags, ECUP);
          ecError = 0;
          ecPrevError = 0;
          ecIntegral = 0;
          OCR2A =0;
          ecnotcomplete = false;
        }
      }
    }  
  }

  if(phnotcomplete&&phPlot)Serial.println(measuredPh);
  if(ecnotcomplete&&ecPlot)Serial.println(measuredEc);
}
