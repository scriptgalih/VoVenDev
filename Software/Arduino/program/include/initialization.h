#define LIMITSWITCH_R A6
#define LIMITSWITCH_L A4
#define LED A2
#define RX1 19
#define TX1 18
#define SDL 21
#define SDA 20

#define PULSE_PIN 37
#define DIR_PIN   35
#define EN_PIN    33

#define TIMER1 1
#define TIMER3 3

#define FLOWPIN     A8
#define PRESSUREPIN A9
#define OXYGENPIN   A10

int inspiration = 1;
int expiration = 2;

int breath_frequency = 10;

unsigned long set_OCR1, set_OCR3;

int step_count = 0;

int phase = 0;

bool MACHNE_ON = false;
bool change_phase = false;
bool SENDDATA = false;

int mult_timer3=0;

String serial_text;

float offset_pressure;

float disp_volume=1, disp_oxygen=2, disp_flow=3;

float flow_sensor_send, pressure_sensor_send; 