#include <Arduino.h>
#include <Thread.h>
#include <ThreadController.h>
#include <ArduinoJson.h>
#include "initialization.h"


Thread phaseThread = Thread();
Thread sendData = Thread();


class SensorThread: public Thread
{
public:
	int current_input;
	int pin;
  float current_output, alpha, previous_output;
	// No, "run" cannot be anything...
	// Because Thread uses the method "run" to run threads,
	// we MUST overload this method here. using anything other
	// than "run" will not work properly...
	void run(){
		// Reads the analog pin, and saves it localy
		current_input = analogRead(pin);
    current_output  = alpha * current_input + (1-alpha)*previous_output;
    previous_output = current_output;
		runned();
	}
};


SensorThread flowSensor = SensorThread();
SensorThread pressureSensor = SensorThread();
 
ThreadController controller = ThreadController();

DynamicJsonDocument doc(1024);

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
      OCR1A = 800;
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

void phaseCallback(){
  if (MACHNE_ON == false){
    return;
  }
  if (phase==0)
  {
    // Serial.println("phase 0");
    dir_in();
    step_on();
    phase++;
    delay(250);
    push = true;
  }else if (phase==1)
  {
    // Serial.println("phase 1");
    dir_out(); 
    step_on();
    phase++;
    delay(50);
    push = true;
  }else if (phase==expiration)
  {
    // Serial.println("reset");
    phase=0;
  }
  
}

void sendDataCallback(){
  if(SENDDATA == false){
    return;
  }
  Serial.print("FL:");
  Serial.print(flowSensor.current_output);
  Serial.print("\tPS:");
  Serial.print(pressureSensor.current_output);
  Serial.print("\t:");
  Serial.print(millis());

  Serial.println();

}

void serialEvent() {
  while (Serial.available()) {
    serial_text = Serial.readStringUntil('\n');
    deserializeJson(doc, serial_text);

    String command = doc["c"];
    if (command == "start"){
      if(MACHNE_ON == true)
        return;
      Serial.println("Mesin Nyala");
      phaseThread.setInterval(60000.0/((inspiration + expiration) * breath_frequency));
      calibratePosition();
      phase = 0;
      MACHNE_ON = true;
    }else if(command == "stop")
    {
      MACHNE_ON = false;
      TCCR1A = 0;
      calibratePosition();
      Serial.println("Mesin Mati");
    }    
  }
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
  calibratePosition();
  // delay(100);
  Serial.println("Calibrate position done!");

  Serial.println("Done!");

  
  // put your setup code here, to run once:
  flowSensor.pin = FLOWPIN;
  pressureSensor.pin = PRESSUREPIN;
  flowSensor.setInterval(100);
  pressureSensor.setInterval(100);
  flowSensor.alpha = 0.15;
  pressureSensor.alpha = 0.15;

  controller.add(&sendData);
  controller.add(&flowSensor);
  controller.add(&pressureSensor);

  phaseThread.onRun(phaseCallback);
  phaseThread.setInterval(1500);
  sendData.onRun(sendDataCallback);
  sendData.setInterval(10);

  SENDDATA = false;
}
// -------------------------------------------------------------------------------------------------------- loop program
void loop() {
  // put your main code here, to run repeatedly:
  if(phaseThread.shouldRun()){
    phaseThread.run();
  }
  controller.run();
  // Serial.println("aaaa");
  // Serial.println("cccc");

  if(push){
    // Serial.println("bbbb");
    if((digitalRead(LIMITSWITCH_R) == LOW) || (digitalRead(LIMITSWITCH_L) == LOW)){
      push = false;
      TCCR1B = 0;
    }
  }
}

// -------------------------------------------------------------------------------------------------------- ISR TIMER 1 COMPA
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  digitalWrite(PULSE_PIN, digitalRead(PULSE_PIN) ^ 1);   // toggle LED pin
  
}