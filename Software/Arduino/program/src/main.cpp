#include <Arduino.h>
#include <Thread.h>
#include <ThreadController.h>
#include <ArduinoJson.h>
#include "RTClib.h"
#include "initialization.h"



RTC_DS1307 rtc;

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
  }else if (phase>=expiration)
  {
    // Serial.println("reset");
    phase=0;
  }
  else{
    phase++;

  }
  
}

float adc2cmh2o(float adc){
  float cmh2o;
  float gain = 100.0 /1000.0;
  const float kPa2cmH2O = 10.19716;

  cmh2o = kPa2cmH2O * ((( adc / 1023.0 * 5.0)/gain)/2.5);
  // cmh2o = (((adc/1023.0f)*5)/gain)/2.5;
  return cmh2o;

}
void sendDataCallback(){
  if(SENDDATA == false){
    return;
  }
  Serial.print("FL:");
  // Serial.print(map(flowSensor.current_output,0,88,0,2));
  Serial.print(pressureSensor.current_output);
  Serial.print("\tPS:");
  // Serial.print(map(pressureSensor.current_output, 0, 560, 0, 45));
  Serial.print(adc2cmh2o(pressureSensor.current_output),6);
  // Serial.print(pressureSensor.current_output);
  // Serial.print("\t:");
  // Serial.print(millis());

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
      phaseThread.setInterval(60000.0f/((inspiration + expiration) * breath_frequency));
      calibratePosition();
      phase = 0;
      MACHNE_ON = true;
    }
    else if(command == "stop")
    {
      MACHNE_ON = false;
      TCCR1A = 0;
      calibratePosition();
      Serial.println("Mesin Mati");
    }
    else if(command == "set")
    {
      expiration = doc["ie"];
      breath_frequency = doc["f"];
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
  delay(100);
  Serial.println("Calibrate position done!");

  Serial.println("Done!");

  #ifndef ESP8266
    while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif

  // if (! rtc.begin()) {
  //   Serial.println("Couldn't find RTC");
  //   Serial.flush();
  //   abort();
  // }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  
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

  SENDDATA = true;
}
// -------------------------------------------------------------------------------------------------------- loop program
void loop() {
  // put your main code here, to run repeatedly:
  // DateTime time = rtc.now();
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