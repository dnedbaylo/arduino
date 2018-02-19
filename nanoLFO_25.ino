/*
    LFO via PWM, for arduino, into a modular.

   some code borrowed from http://abrammorphew.com/notes/2012/01/05/arduino-lfo-generator/

   20 - odd, can't use MISO pin 12 as output. use as input instead
   21 - sorted out buttons
   22 - to toggle pots between edit mode
   23 - put to Mary (laptop) - no changes
   24 - more buttons, now short press scrolls between waveforms
   25 - do next - EG, check voltage on TL072.
   change to 5v rail to rail. less noise on 12v / analog rails. who cares about 0-8v CV? runnable just by USB.
  learn mode: time between incoming gate clocks becomes next cycle of LFO - really interesting when driven by random source
  alternative is retrigger, which starts new LFO each gate in

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
const int HOLD_TIME = 500; // ms button needs to be held to toggle edit mode (rather than trigger EG)
const int MAX_WAVEFORMS = 8; // overflows back to 0 once
const int MAX_ATTACK = 2000; // ms
const int MAX_RELEASE = MAX_ATTACK;


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

// cutoffs for button press analog values
const int LOW1 = 50;
const int LOW2 = 103;
const int LOW3 = 180;
const int LOW4 = 600;

// starting phase
// uint8_t tWave = 128;
// uint8_t sWave = 255;
// uint8_t ruWave = 128;
//uint8_t rdWave = 128;
// uint8_t rWave = 128;

//elapsedMillis timeElapsed;

// pins
const int greenPin[] = {3, 5, 6, 9};
const int redPin[] = {2, 4, 7, A2}; // check this // wired reversed in prototype, should be red4 A2, green4 9
const int gateIn[] = {12, A1, A0}; // change this from prototype?
const int potPin[] = {A4, A5, A6, A7};
const int buttonPin = A3; // 3 buttons via different resistors, into 1 pin
const byte DAC0_CS = 10;
const byte DAC1_CS = 8;

// central vars
volatile int i = 0; // clock, updates on timer
volatile int duty;
int editCh = 0; // current channel (1-4) edited, 0 if edit mode off


// buttons
int buttonPinValue;
byte buttonPressed = 0;
byte quicklyPressed = 0; // holds channel for button press less than HOLD_TIME
boolean newPress = false;
boolean newButtonHeld = false;
long buttonStartTime;

// channel vars
volatile int potValue[] = {128, 128, 128, 128}; // up to 1024, top of analogRead range

volatile int value[4]; // = {1000, 2512, 2512, 2512}; // output value (12 bit 0-4096) use volatile if timer may change value
int tweak1[] = {512, 512, 512, 512}; // 10 bit (0-1024) used for attack, or fine tuning LFO, portamento for noise, duty cycle for pulse,
int tweak2[] = {512, 512, 512, 512}; // 10 bit, release, or max amplitude for LFOs, (or randomness)
int waveform[4]; // 4 channels (declaring it to [3] (ie 0-2) gave errors, as no waveform[3])

volatile int place[] = {0, 0, 0, 0}; // place along cycle
volatile long period[] = {1000, 1000, 1000, 1000}; // cycle length
int attackTime[4], releaseTime[4];
boolean gate[3];
// boolean newGate[3]; // as only 3 gate ins in this version
boolean oldGate[] = {false, false, false};
boolean trigger[] = {false, false, false};


void setup() {
  Serial.begin(9600);

  SPI.begin();
  Timer1.initialize(150); // updates every 100 usecs - reduce to get higher Hz
  Timer1.attachInterrupt(updateClock); //

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

  pinMode(gateIn[0], INPUT);
  pinMode(gateIn[1], INPUT);
  pinMode(gateIn[2], INPUT);

  digitalWrite(redPin[0], LOW);
  digitalWrite(redPin[1], LOW);
  digitalWrite(redPin[2], LOW);
  digitalWrite(redPin[3], LOW);

  digitalWrite(DAC0_CS, HIGH);
  digitalWrite(DAC1_CS, HIGH);

  // for testing
  waveform[0] = EG;
  waveform[1] = EG;
  waveform[2] = SINE;
  waveform[3] = SINE;

}

void loop() {




  // ? period needs to be exponential - little difference in slow half of pot travel
  for (int ch = 0; ch < 4; ch ++) {
    int temp = analogRead(potPin[ch]);
    if (abs(temp - potValue[ch]) > POT_HYSTERESIS) { // only change if pot has moved a bit, removes jitter
      potValue[ch] = temp;
      // set latch off ie pot has been moved
      period[ch] = map(potValue[ch], 0, 1023, MAX_PERIOD, MIN_PERIOD);
    }
  }

  if (editCh != 0) { // edit mode
    // only if latch gone :     period[ch] = map(potValue[0], 0, 1023, MAX_PERIOD, MIN_PERIOD);
    //                         tweak1[editCh] = potValue[1];
    //                         tweak2[editCh] = potValue[2];
    //                          waveform[editCh] = map(analogRead(potPin[3]), 0, 1023, 1, 7);
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

  checkGates();

  updateOutputs();

  setOutput(0, CH_A, 0, value[0]);
  setOutput(0, CH_B, 0, value[1]);
  setOutput(1, CH_A, 0, value[2]);
  setOutput(1, CH_B, 0, value[3]);

  // write to LEDs
  analogWrite(greenPin[0], value[0] / 16 );
  analogWrite(greenPin[1], value[1] / 16 );
  analogWrite(greenPin[2], value[2] / 16 );
  analogWrite(greenPin[3], value[3] / 16 );

  // if (timeElapsed > 1000) {
  //   redOn = !redOn;
  //  digitalWrite(redPin[0], HIGH); // redOn);
  //   timeElapsed = 0;
  //  }
  //  if (timeElapsed > 300) {
  //  digitalWrite(redPin[0], LOW);
  // }

  checkButtons();
  processButtons();



}

void updateClock() {
  i ++;
}

void checkGates() {
  for (int i = 0; i < 3; i ++) {
    gate[i] = !(digitalRead(gateIn[i])); // inverts gate in as hardware does
 //   Serial.println(gate);
    if (gate[i] == true) {
      digitalWrite(redPin[i], HIGH); // flash LED
      if (oldGate[i] == false) { // from now, gate is 1 when signal present at jack
        Serial.print("gate on ch ");
        Serial.println(i);
        trigger[i] = true; // reset after processing
        oldGate[i] = true; // don't trigger again
      }
    } else if (gate[i] == false) {
 //     Serial.println("no gate ");
      oldGate[i] = false; // now you can
      digitalWrite(redPin[i], LOW);
    }
  }
}

void checkButtons() {
  int temp = analogRead(buttonPin);
  if (temp > LOW1) { // something is pressed
    if (buttonPressed == 0) {
      newPress = true;
      buttonStartTime = millis();
      //     Serial.print(temp);
      //     Serial.print(" : ");
    }
    if (temp < LOW2) { // which is it?
      buttonPressed = 3;
    } else if ((temp < LOW3) && (temp > LOW2)) {
      buttonPressed = 4;
    } else if ((temp < LOW4) && (temp > LOW3)) {
      buttonPressed = 2;
    } else if (temp > LOW4) {
      buttonPressed = 1;
    }
    if ((newPress) && (millis() > buttonStartTime + HOLD_TIME)) { // still pressed
      newButtonHeld = true;
      newPress = false;
    }
  } else if (temp < LOW1) { // no buttons pressed
    if ((newPress) && (millis() < buttonStartTime + HOLD_TIME)) {
      quicklyPressed = buttonPressed;
    }
    buttonPressed = 0;
    newPress = false;
    //   oldButtonPressed = false;
  }
}

void processButtons() {
  if (quicklyPressed) { // toggles next waveform if not edit mode
    Serial.print("quick press waveform was ");
    Serial.println(quicklyPressed - 1);
    if (editCh == 0) {
      waveform[quicklyPressed - 1] ++;
      if (waveform[quicklyPressed - 1] >= MAX_WAVEFORMS) {
        waveform[quicklyPressed - 1] = 0;
      }
      Serial.print("channel ");
      Serial.print(quicklyPressed - 1);
      Serial.print(" : wave: ");
      Serial.println(waveform[quicklyPressed - 1]);
    } else if (editCh >= 1) { // cancels edit mode otherwise
      Serial.print("editCh is ");
      Serial.println(editCh);
      digitalWrite(redPin[editCh - 1], LOW);
      editCh = 0;
    }
    quicklyPressed = 0;
  }

  if (newButtonHeld) { // toggle edit mode
    newButtonHeld = false;
    if (buttonPressed == editCh) { // turn off
      digitalWrite(redPin[editCh - 1], LOW);
      editCh = 0;
    } else { // new editCh
      digitalWrite(redPin[editCh - 1], LOW);
      editCh = buttonPressed;
      digitalWrite(redPin[editCh - 1], HIGH);
    }
    //   Serial.print(editCh);
    //   Serial.print(", <- edit channel, button was: ");
    //   Serial.println(buttonPressed);
  }
}

void updateOutputs() {

  // incoming gate retriggers for now
  // in future could do learn mode for some waveforms (? sine, triangle)

  for (int ch = 0; ch < 4; ch ++) {
    switch (waveform[ch]) {

      case EG:
        if (trigger[ch] == true) { // reset done by trigger in
          place[ch] = 0;
          Serial.print("triggered on channel ");
          Serial.println(ch);
          trigger[ch] = false; // trigger reset once processed
        }
        // TO DO: ? only recalculate attack & release times if tweak pots moved
        attackTime[ch] = int(map(tweak1[ch], 0, 1024, 0, MAX_ATTACK));
        releaseTime[ch] = int(map(tweak1[ch], 0, 1024, 0, MAX_RELEASE));
        period[ch] = map(potValue[ch], 0, 1023, MAX_PERIOD, MIN_PERIOD) + attackTime[ch] + releaseTime[ch];
        if (place[ch] < attackTime[ch]) { // attack phase
          value[ch] = int(map(place[ch], 0, attackTime, 0, DAC_MAX));
        } else if ((place[ch] >= attackTime[ch]) && (place[ch] < attackTime[ch] + period[ch])) { // sustain
          value[ch] = DAC_MAX; // ? inverted as in red // DAC_MAX;
        } else if ((place[ch] > attackTime[ch] + period[ch]) && (place[ch] < attackTime[ch] + period[ch] + releaseTime[ch])) { // release phase
          value[ch] = int(map(place[ch], attackTime[ch] + period[ch], attackTime[ch] + period[ch] + releaseTime[ch], DAC_MAX, 0));
        } else if (place[ch] > attackTime[ch] + period[ch] + releaseTime[ch]) { // ended
          digitalWrite(redPin[i], LOW); // back to green
          value[ch] = 0;
        }
        break;

      case SINE: // sine
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0; 
          trigger[ch] = false; 
        }
        value[ch] = int((sine(map(place[ch], 0, period[ch], 0, 1023)))); // nearest cycle, means could get ragged sine wave at faster Hz then 256 clock periodicity
        break;

      case TRI: // triangle
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0; // 
          trigger[ch] = false; 
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
        // deal with triggers
        trigger[ch] = false; // for now
        duty = 10 + (period[ch] * tweak1[ch] / 1050); // fudge so always some pulse at min & max
        if (place[ch] < duty) { // (period[0] / 2)) { // first half of waveform
          value[ch] = DAC_MAX; // change this for toggle
        } else {
          value[ch] = 0;
        }
        // and write toggle now, so can be triggered by pulse - alternates high & low
        break;

      case RAMP_UP:
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0; // minimum waveform cycle for sine, could potentially get extra octaves out of ramp or square wave
          trigger[ch] = false; 
        }
        //     value[ch] = place[ch];
        value[ch] = int(map(place[ch], 0, period[ch], 0, DAC_MAX));
        break;

      case RAMP_DOWN:
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0; // minimum waveform cycle for sine, could potentially get extra octaves out of ramp or square wave
          trigger[ch] = false; 
        }
        value[ch] = int(map(place[ch], 0, period[ch], DAC_MAX, 0));
        break;

      case RANDOM:
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0; // minimum waveform cycle for sine, could potentially get extra octaves out of ramp or square wave
          trigger[ch] = false; 
          value[ch] = random(DAC_MAX);
        }
        break;

      case POT: // direct CV
        value[ch] = map(potValue[ch], 0, 1024, 0, DAC_MAX);
          trigger[ch] = false; 
    }
  }
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

  if (chip == 0) {
    //    PORTB &= 0xfb;
    //    Serial.println("chip 0");
    digitalWrite(DAC0_CS, LOW);
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
    digitalWrite(DAC0_CS, HIGH);
    //   PORTB |= 0x4;
  } else if (chip == 1) {
    PORTB &= 0xfb;
    //     Serial.println("chip 1");
    digitalWrite(DAC1_CS, LOW);
    SPI.transfer(highByte);
    SPI.transfer(lowByte);
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
