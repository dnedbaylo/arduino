/*
    LFO via PWM, for arduino, into a modular.

    borrowed from
   http://abrammorphew.com/notes/2012/01/05/arduino-lfo-generator/

   PWM output looks OK through 220R with 100nF to gnd
   changes take this from 10secs period to 30Hz max

   09 - adding in LED outputs
   to do - arrays for LFOs. ? speed. class.
   11 all quite weird. timer just updates place & write in loop?
   12 redo to above
*/

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#include <SPI.h>
#include <TimerOne.h>
// #include <elapsedMillis.h>

//editable
const int POT_HYSTERESIS = 10; // if getting weird errors with 1 or more channels, increase this (try 8)
const int MIN_PERIOD = 50; // experiment with this - LED will cut out at high freqs, reduce this until stops
const int MAX_PERIOD = 30000;


// don't change these
const int QTR_SINE = 256;
const int WHOLE = QTR_SINE * 4; // full period is 1024 // ? 4096 gives better resolution
const int DAC_MAX = 4095; // 12 bit resolution for MCP4822 DAC (4096 loses important bit)

const int GAIN_1 = 0x1; // 1 is no gain
const int GAIN_2 = 0x0; //
const int CH_A = 0;
const int CH_B = 1;
const int SHUTDOWN = 1;

const int EG = 0;
const int SINE = 1;
const int TRI = 2;
const int PULSE = 3;
const int RAMP_UP = 4;
const int RAMP_DOWN = 5;
const int RANDOM = 6;
const int POT = 7;

// one quarter of
int sineTable[] = {
  2048, 2060, 2073, 2085, 2098, 2110, 2123, 2136, 2148, 2161, 2173, 2186, 2198, 2211, 2223, 2236,
  2248, 2261, 2273, 2286, 2298, 2311, 2323, 2336, 2348, 2361, 2373, 2385, 2398, 2410, 2423, 2435,
  2447, 2460, 2472, 2484, 2497, 2509, 2521, 2533, 2545, 2558, 2570, 2582, 2594, 2606, 2618, 2630,
  2642, 2654, 2666, 2678, 2690, 2702, 2714, 2726, 2738, 2750, 2762, 2773, 2785, 2797, 2808, 2820,
  2832, 2843, 2855, 2866, 2878, 2889, 2901, 2912, 2924, 2935, 2946, 2958, 2969, 2980, 2991, 3002,
  3014, 3025, 3036, 3047, 3058, 3069, 3079, 3090, 3101, 3112, 3123, 3133, 3144, 3154, 3165, 3176,
  3186, 3196, 3207, 3217, 3227, 3238, 3248, 3258, 3268, 3278, 3288, 3298, 3308, 3318, 3328, 3338,
  3347, 3357, 3367, 3376, 3386, 3395, 3405, 3414, 3424, 3433, 3442, 3451, 3460, 3470, 3479, 3487,
  3496, 3505, 3514, 3523, 3532, 3540, 3549, 3557, 3566, 3574, 3582, 3591, 3599, 3607, 3615, 3623,
  3631, 3639, 3647, 3655, 3663, 3670, 3678, 3686, 3693, 3701, 3708, 3715, 3723, 3730, 3737, 3744,
  3751, 3758, 3765, 3772, 3778, 3785, 3792, 3798, 3805, 3811, 3818, 3824, 3830, 3836, 3842, 3848,
  3854, 3860, 3866, 3872, 3877, 3883, 3889, 3894, 3899, 3905, 3910, 3915, 3920, 3925, 3930, 3935,
  3940, 3945, 3949, 3954, 3959, 3963, 3968, 3972, 3976, 3980, 3984, 3988, 3992, 3996, 4000, 4004,
  4008, 4011, 4015, 4018, 4022, 4025, 4028, 4031, 4034, 4037, 4040, 4043, 4046, 4049, 4051, 4054,
  4056, 4059, 4061, 4063, 4065, 4067, 4069, 4071, 4073, 4075, 4077, 4078, 4080, 4081, 4083, 4084,
  4085, 4087, 4088, 4089, 4090, 4091, 4091, 4092, 4093, 4093, 4094, 4094, 4094, 4095, 4095, 4095
};

// starting phase
uint8_t tWave = 128;
uint8_t sWave = 255;
uint8_t ruWave = 128;
uint8_t rdWave = 128;
uint8_t rWave = 128;


int waveform[4]; // declaring it to [3] (ie 0-2) gave errors, as no waveform[3]
// byte  d = HIGH;

//elapsedMillis timeElapsed;
const int greenPin[] = {3, 5, 6, 9};
const int redPin[] = {2, 4, 7, A2}; // check this // wired reversed in prototype, should be red4 A2, green4 9
const int gateIn[] = {8, A0, A1};
const int potPin[] = {A4, A5, A6, A7};
const int buttonsPin = A3; // 3 buttons via different resistors, into 1 pin
const byte DAC0_CS = 10;
const byte DAC1_CS = 12;

volatile int i = 0; // clock, updates on timer
volatile int duty;

// channel vars
volatile int place[] = {0, 0, 0, 0};
int tweak[] = {512, 512, 512, 512}; //
volatile int potValue[] = {128, 128, 128, 128}; // up to 1024, top of analogRead range
volatile int value[4]; // = {1000, 2512, 2512, 2512}; // volatile if timer may change value
volatile long period[] = {1000, 1000, 1000, 1000};
int buttons;

void setup() {
  Serial.begin(9600);

  pinMode(greenPin[0], OUTPUT);
  pinMode(greenPin[1], OUTPUT);
  pinMode(greenPin[2], OUTPUT);
  pinMode(greenPin[3], OUTPUT);
  pinMode(redPin[0], OUTPUT);
  pinMode(redPin[1], OUTPUT);
  pinMode(redPin[2], OUTPUT);
  pinMode(redPin[3], OUTPUT);
  pinMode(DAC0_CS, OUTPUT);
  pinMode(DAC1_CS, OUTPUT);

  digitalWrite(redPin[0], LOW);
  digitalWrite(redPin[1], LOW);
  digitalWrite(redPin[2], LOW);
  digitalWrite(redPin[3], LOW);


  SPI.begin();
  Timer1.initialize(150); // updates every 100 usecs - reduce to get higher Hz
  Timer1.attachInterrupt(updateClock); //

}

void loop() {

  //  waveform = map(analogRead(potPin[3]), 0, 1023, 1, 7);
  waveform[0] = PULSE;
  waveform[1] = RAMP_DOWN;
  waveform[2] = RAMP_UP;
  waveform[3] = SINE;

  // needs to be exponential - little difference in slow half of pot travel
  for (int ch = 0; ch < 4; ch ++) {
    int temp = analogRead(potPin[ch]);

    if (abs(temp - potValue[ch]) > POT_HYSTERESIS) { // only change if pot has moved a bit, removes jitter
      potValue[ch] = temp;
      period[ch] = map(potValue[ch], 0, 1023, MAX_PERIOD, MIN_PERIOD);
    }
  }
  
  int temp = analogRead(buttonsPin);
  if (abs(temp - potValue[buttonsPin]) > POT_HYSTERESIS) { 
    buttons = temp;
  }

  if (i) { // if timer has updated
    noInterrupts(); // read v quick
    int incr = i;
    i = 0;
    interrupts();
    place[0] += incr; // update place along wave
    place[1] += incr;
    place[2] += incr;
    place[3] += incr;
  }




  for (int ch = 0; ch < 4; ch ++) {

    switch (waveform[ch]) {

      case SINE: // sine
        if (place[ch] >= period[ch]) {
          place[ch] = 0; // minimum waveform cycle for sine, could potentially get extra octaves out of ramp or square wave
        }
        value[ch] = int((sine(map(place[ch], 0, period[ch], 0, 1023)))); // nearest cycle, means could get ragged sine wave at faster Hz then 256 clock periodicity
        break;

      case TRI: // triangle
        if (place[ch] >= period[ch]) {
          place[ch] = 0;
        }
        if (place[ch] < (period[ch] / 2) ) { // first half of waveform
          value[ch] = int(map(place[ch], 0, period[ch] / 2, 0, DAC_MAX));
        } else {
          value[ch] = int(map(place[ch], period[ch] / 2, period[ch], DAC_MAX, 0));
        }
        break;

      case PULSE: // srq wave
        if (place[ch] >= period[ch]) {
          place[ch] = 0;
        }
        duty = 10 + (period[ch] * tweak[ch] / 1050); // fudge so always some pulse at min & max
        if (place[ch] < duty) { // (period[0] / 2)) { // first half of waveform
          value[ch] = DAC_MAX;
        } else {
          value[ch] = 0;
        }
        break;

      case RAMP_UP:
        if (place[ch] >= period[ch]) {
          place[ch] = 0;
        }
        //     value[ch] = place[ch];
        value[ch] = int(map(place[ch], 0, period[ch], 0, DAC_MAX));
        break;

      case RAMP_DOWN:
        if (place[ch] >= period[ch]) {
          place[ch] = 0;
        }
        value[ch] = int(map(place[ch], 0, period[ch], DAC_MAX, 0));
        break;

      case RANDOM:
        if (place[ch] >= period[ch]) {
          place[ch] = 0;
          value[ch] = random(DAC_MAX);
        }
        break;

      case POT: // direct CV
        value[ch] = map(potValue[ch], 0, 1024, 0, DAC_MAX);
    }
  }

  //  Serial.println(value[0]);
  //  Serial.println(value[1]);
  //  Serial.println(value[2]);
  Serial.println(buttons);

  setOutput(0, CH_A, 0, value[0]);
  setOutput(0, CH_B, 0, value[1]);
  // setOutput(1, CH_A, 0, value[2]);
  //  setOutput(1, CH_B, 0, value[3]);

  // write to LEDs
  analogWrite(greenPin[0], value[0] / 16 );
  analogWrite(greenPin[1], value[1] / 16 );
  analogWrite(greenPin[2], value[2] / 16 );
  analogWrite(greenPin[3], value[3] / 16 );

  // digitalWrite(redPin[0], false);

  // if (timeElapsed > 1000) {
  //   redOn = !redOn;
  //  digitalWrite(redPin[0], HIGH); // redOn);
  //   timeElapsed = 0;
  //  }
  //  if (timeElapsed > 300) {
  //  digitalWrite(redPin[0], LOW);
  // }

  // read buttons

}

void updateClock() {
  i ++;
}

int sine(int i) {
  int place = int(i);
  //  Serial.print(" ** ");
  //  Serial.print(place);
  //  Serial.print(" ** ");
  if (place <= 255) {
    return sineTable[place];
  } else if ((place < 512) && (place >= 256)) {
    return sineTable[511 - place];
  } else if ((place <= 767) && (place >= 512)) {
    return (4095 - sineTable[place - 512]);
  } else if (place >= 768) {
    return (4095 - sineTable[1023 - place]);
  }
}

void setOutput(byte chip, byte channel, byte gain, unsigned int val) // bit 12 shutdown 1 is active, 0 is no output
{
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | channel << 7 | gain << 5 | SHUTDOWN << 4; // 7 is bit 15, 5 bit 13. gain 0 is 2x

  //    PORTB &= 0xfb;
  if (chip == 0) {
    PORTB &= 0xfb;
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
    //  digitalWrite(DAC, HIGH);
    PORTB |= 0x4;
  } else {
    //    PORTB &= 0xfb;
    digitalWrite(DAC1_CS, LOW);
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
    //  digitalWrite(DAC, HIGH);
    digitalWrite(DAC1_CS, HIGH);
    //    PORTB |= 0x8;
  }
}

/*

  void setOutput(byte channel, unsigned int val) // MCP4822 DAC
  {
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | channel << 7 | 0 << 5 | 1 << 4; // gain (0) means x2, shutdown bit set to 1 else DAC turns off.

  digitalWrite(DAC1_CS, LOW);
  //  PORTB &= 0xfb; // CS pin 10 low
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  //  PORTB |= 0x4; // CS pin 10 high
  digitalWrite(DAC1_CS, HIGH);
  }
*/
