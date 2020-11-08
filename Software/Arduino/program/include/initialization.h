#define LIMITSWITCH_R A6
#define LIMITSWITCH_L A4
#define LED A2
#define RX1 19
#define TX1 18
#define SDL 21
#define SDA 20

#define PULSE_PIN 37
#define DIR_PIN 35
#define EN_PIN 33

#define TIMER1 1
#define TIMER3 3

int inspiration = 1;
int expiration = 3;

int breath_frequency = 10;

unsigned long set_OCR1, set_OCR3;

int step_count = 0;

int phase = 0;

bool MACHNE_ON = false;

int mult_timer3=0;