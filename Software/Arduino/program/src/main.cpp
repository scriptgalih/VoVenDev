#include <Arduino.h>
#include "initialization.h"

// -------------------------------------------------------------------------------------------------------- init timer
void timerInit(){
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 800;            // compare match register 16MHz/256/2Hz sementara 25
  // TCCR1B |= (1 << WGM12);   // CTC mode
  // TCCR1B |= (1 << CS01);    // 64 prescaler 
  TCCR1B = 0;
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 7812; // compare match register 16MHz/256/2Hz
  // OCR1B = 31250;
  TCCR3B |= (1 << WGM12); // CTC mode
  TCCR3B |= (1 << CS12) | (1 << CS10); // 256 prescaler
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt

  sei();
}
// -------------------------------------------------------------------------------------------------------- init pin
void initPin(){
  pinMode(LIMITSWITCH_L, INPUT);
  pinMode(LIMITSWITCH_R, INPUT);
  pinMode(DIR_PIN,OUTPUT);
  pinMode(PULSE_PIN,OUTPUT);
  pinMode(EN_PIN,OUTPUT);

  digitalWrite(LIMITSWITCH_L, HIGH);
  digitalWrite(LIMITSWITCH_R, HIGH);
  digitalWrite(DIR_PIN,LOW);
  digitalWrite(PULSE_PIN,LOW);
  digitalWrite(EN_PIN,HIGH);

  pinMode(13,1);
}

void setOCR(int tim, int i, int e, int f){
  
  // OCR1A = 800;
  if (tim == 3)
  {
    /* code */
    set_OCR3 = 15625 / ( ( i + e ) * f );
    OCR3A = set_OCR3;
  }

}

void dir_out(){
digitalWrite(DIR_PIN,HIGH); // Gerak keluar
}

void dir_in(){
digitalWrite(DIR_PIN,LOW); // gerak kedalam
}

void enable(){
  digitalWrite(EN_PIN,HIGH);
}

void disable(){
  digitalWrite(EN_PIN,LOW);
}

void calibratePosition(){
  dir_out();
  OCR1A = 400;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS01);    // 64 prescaler 
  while (1)
  {
    /* code */
    if(digitalRead(LIMITSWITCH_R) == LOW){
      TCCR1B = 0;
      OCR1A = 1000;
      break;
    }
      
    delay(1);
  }

}

bool push = false;
void step_on(){
  push = true;
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS01);    // 64 prescaler 
}
// -------------------------------------------------------------------------------------------------------- setup program
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Prepairing!");
  initPin();
  Serial.println("Pin init done!");
  timerInit();
  Serial.println("Timer init done!");
  // calibratePosition();
  // delay(100);
  Serial.println("Calibrate position done!");
  // setOCR(TIMER3, inspiration , expiration, breath_frequency);

  Serial.println("Done!");

  
  // put your setup code here, to run once:
  MACHNE_ON = true;
}
// -------------------------------------------------------------------------------------------------------- loop program
void loop() {
  // put your main code here, to run repeatedly:
}

// -------------------------------------------------------------------------------------------------------- ISR TIMER 1 COMPA
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  digitalWrite(PULSE_PIN, digitalRead(PULSE_PIN) ^ 1);   // toggle LED pin
  if(push){
    if(step_count>=500){
      TCCR1A = 0;
      push = false;
    }
    step_count ++;
  }
  
}

// -------------------------------------------------------------------------------------------------------- ISR TIMER 3 COMPA
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{

  digitalWrite(13, digitalRead(13) ^ 1);
  
}

