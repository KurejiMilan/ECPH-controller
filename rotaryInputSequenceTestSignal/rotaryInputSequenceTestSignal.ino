#define ROT1A 0
#define ROT1B 1
#define ROT2A 2
#define ROT2B 3
#define ROT1BTN 4
#define ROT2BTN 5

#ifndef setBit
#define setBit(F,B) (F |= (0x01<<B))
#endif

#ifndef clearBit
#define clearBit(F, B) (F &= ~(0x01<<B))
#endif

#ifndef checkBit
#define checkBit(F, B) (F&(0x01<<B))
#endif

void rotary1clock(){
  PORTD = 0x30|(1<<ROT1A);
  delay(2);
  PORTD = 0x30|(1<<ROT1A)|(1<<ROT1B);
  delay(2);
  PORTD = 0x30|(1<<ROT1B);
  delay(2);
  PORTD = 0x30;
  delay(2);
}
void rotary1anticlock(){
  PORTD = 0x30|(1<<ROT1B);
  delay(2);
  PORTD = 0x30|(1<<ROT1A)|(1<<ROT1B);
  delay(2);
  PORTD = 0x30|(1<<ROT1A);
  delay(2);
  PORTD = 0x30;
  delay(2);
}
void rotary2clock(){
  PORTD = 0x30|(1<<ROT2A);
  delay(2);
  PORTD = 0x30|(1<<ROT2A)|(1<<ROT2B);
  delay(2);
  PORTD = 0x30|(1<<ROT2B);
  delay(2);
  PORTD = 0x30;
  delay(2);
}
void rotary2anticlock(){
  PORTD = 0x30|(1<<ROT2B);
  delay(2);
  PORTD = 0x30|(1<<ROT2A)|(1<<ROT2B);
  delay(2);
  PORTD = 0x30|(1<<ROT2A);
  delay(2);
  PORTD = 0x30;
  delay(2);    
}
void push1(){
  PORTD &= ~(1<<ROT1BTN);
  delay(30);
  PORTD = 0x30;
}

void push2(){
  PORTD &= ~(1<<ROT2BTN);
  delay(1);
  PORTD = 0x30;
}

void setup() {
  DDRD= 0xff;
  PORTD = 0x30;
  delay(10000);
  push1();
  delay(5000);
  push2();
  delay(5000);
  rotary1clock();
  delay(1000);
  rotary1clock();
  delay(1000);
  rotary1anticlock();
  delay(1000);
  rotary1anticlock();
}

void loop() {

}
