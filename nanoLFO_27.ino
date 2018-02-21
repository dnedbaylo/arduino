/*
    4 EGs / LFOs for arduino, into a modular.
    This controls 4 12bit DAC buffered outputs. These can be set as EG, LFO or direct pot CV.
    Incoming gates (3 in prototype) trigger EGs, or restarts LFOs.
    Holding a channel button access 2ndry pot functions: attack, release, duty / phase, waveform. these only do something where waveform allows.
    // future revision could change these, so adds randomness, noise, etc.

    DAC is MCP4922.

   20 - odd, can't use MISO pin 12 as output. use as input instead
   21 - sorted out buttons
   24 - more buttons, now short press scrolls between waveforms
   27 - buttons work, EG works. 27 is first normal functionality, working version.

  to do:
  learn mode: time between incoming gate clocks becomes next cycle of LFO - really interesting when driven by random source
  alternative is retrigger, which starts new LFO each gate in - implemented currently

  next hardware revision:
   change to 5v rail to rail. less noise on 12v / analog rails. who cares about 0-8v CV? runnable just by USB.

   share and share alike creative commons licence, fuzzySynths 2018
  some code ideas borrowed from http://abrammorphew.com/notes/2012/01/05/arduino-lfo-generator/

*/

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#include <SPI.h>
#include <TimerOne.h>
// #include <elapsedMillis.h>

//editable
const int POT_HYSTERESIS = 10; // keep low, but if getting weird errors with 1 or more channels, increase this (try 8)
const int MIN_PERIOD = 200; // experiment with this - LED will cut out at high freqs, reduce this until stops
const int MAX_PERIOD = 30000; // 34000 works out at around 4.8 secs
const int HOLD_TIME = 500; // ms button needs to be held to scroll to next waveform
const int MAX_HOLD_TIME = 1500; // ms, beyond which doesn't forward waveform, as you're editing it.
const int MAX_WAVEFORMS = 8; // beyond this, overflows back to 0
const int MAX_ATTACK = 2000; // ms
const int MAX_RELEASE = MAX_ATTACK;
const int MIN_AR = 10; // minimum attack & release to prevent glitches


// don't change these
const int QTR_SINE = 256;
const int WHOLE = QTR_SINE * 4; // full period is 1024 (? 4096 gives better resolution - no, this is position along, wave table does give 12 bit values)
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
int editCh = 0; // current channel (1-4) edited, 0 if edit mode off


// buttons
int buttonPinValue;
byte buttonPressed = 0;
// byte quicklyPressed = 0; // holds channel for button press less than HOLD_TIME
// byte longPress = 0; // holds channel for press more than HOLD_TIME
boolean newPress = false; // true at start of button
// boolean newButtonHeld = false; // not used?
long buttonStartTime;

// channel vars
byte redLED_on = 0; // holds LED status on bits 0-3, turned on by HOLD in bits 4-7
volatile int potValue[] = {128, 128, 128, 128}; // up to 1024, top of analogRead range
// boolean potMoved[] = {0, 0, 0, 0}; // moved flag so can latch pots

volatile int value[4]; // = {1000, 2512, 2512, 2512}; // output value (12 bit 0-4096) use volatile if timer may change value

int tweak0[] = {512, 512, 512, 512}; // fine pitch should be percentage of period ie 5% or 10% either way
int tweak1[] = {0, 0, 0, 0}; // 10 bit (0-1024) used for attack, or fine tuning LFO, portamento for noise, duty cycle for pulse,
int tweak2[] = {100, 100, 100, 100}; // 10 bit (0-1024), release, or max amplitude for LFOs, (or randomness)
int waveform[4]; // 4 channels // doh! declaring it to [3] (ie 0-2) gave errors, as no waveform[3]
int phase[] = {512, 512, 512, 512};
volatile int place[] = {0, 0, 0, 0}; // place along cycle
volatile long period[] = {1000, 1000, 1000, 1000}; // cycle length
int attackTime[4], releaseTime[4];
int duty[] = {512, 512, 512, 512}; // only departs from this if tweaked
boolean gate[3];
boolean oldGate[] = {false, false, false};
boolean trigger[] = {false, false, false, false}; // 4 channels, 3rd duplicated into 4th


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

  for (int i = 0; i <= 3; i ++) {
    digitalWrite(redPin[i], bitRead(redLED_on, i));
  }

  digitalWrite(DAC0_CS, HIGH);
  digitalWrite(DAC1_CS, HIGH);

  // for testing
  waveform[0] = EG;
  waveform[1] = EG;
  waveform[2] = EG;
  waveform[3] = EG;

}

void loop() {

  // update timer
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

  // read interface
  readPots();
  checkGates();
  updateLEDs();   // write to LEDs
  checkButtons();

  // outputs
  updateOutputs(); // update values
  setOutput(0, CH_A, 0, value[0]);
  setOutput(0, CH_B, 0, value[1]);
  setOutput(1, CH_A, 0, value[2]);
  setOutput(1, CH_B, 0, value[3]);

  // testing
  //  Serial.println(value[2]);
}

void readPots() { // read pots
  // ? period needs to be exponential - little difference in slow half of pot travel
  for (int pot = 0; pot < 4; pot ++) {
    int temp = analogRead(potPin[pot]);
    if (abs(temp - potValue[pot]) > POT_HYSTERESIS) { // only change if pot has moved a bit, removes jitter
      potValue[pot] = temp;
      if (buttonPressed == 0) { // button not held, so pot updates frequencies according to potValue
        period[pot] = map(potValue[pot], 0, 1023, MAX_PERIOD, MIN_PERIOD);
      } else if (buttonPressed > 0) { // button held, so editing that channel
        if (pot == 0) { // FINE
          tweak0[buttonPressed - 1] = potValue[pot]; // or use map(potValue[ch], 0, 1023, 0, MAX_TWEAK?)
        } else if (pot == 1) { // Pot 2
          tweak1[buttonPressed - 1] = potValue[pot];
        } else if (pot == 2) { // Pot 2
          tweak2[buttonPressed - 1] = potValue[pot];
        } else if (pot == 3) { // Pot 2
          waveform[buttonPressed - 1] = map(potValue[pot], 0, 1023, 0, MAX_WAVEFORMS);
        }
      }
    }
  }
}

void updateClock() { // called by interrupt, so timing moves on whatever else is happening
  i ++;
}

void checkGates() {
  for (int i = 0; i < 3; i ++) {
    gate[i] = !(digitalRead(gateIn[i])); // (inverts gate in as hardware does, so) gate is true when signal received
    if (gate[i] == HIGH) {
      bitSet(redLED_on, i); // turns on LED
      if (oldGate[i] == false) { // from now, gate is 1 when signal present at jack
        trigger[i] = true; // reset after processing
        oldGate[i] = true; // don't trigger again
        if (trigger[2]) {
          trigger[3] = true; // doesn't have own trigger yet // change if go to 4 ins
          bitSet(redLED_on, 3); // turns on LED3 as well
        }
      }
    } else if (gate[i] == LOW) {
      oldGate[i] = false; // now you can
      bitClear(redLED_on, i); // turns off LED
      if (i == 2) {
        bitClear(redLED_on, 3); // turns off 3rd LED also
      }
    }
  }

}

void checkButtons() {
  // buttonPressed holds current button, 0 if nothing
  // newPress true as soon as pressed, cancelled after processing in loop - triggers EG
  // only get newPress back on next positive transition
  // trigger[0-3] true at end of short press
  // processes waveform change after long hold (length between HOLD_TIME and MAX_HOLD_TIME)

  int temp = analogRead(buttonPin);
  if (temp > LOW1) { // something is pressed
    if (buttonPressed == 0) {
      newPress = true;
      buttonStartTime = millis();
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
      bitSet(redLED_on, buttonPressed - 1); // turn on LED after HOLD_TIME
    }
  } else if (temp < LOW1) { // button released, check time
    bitClear(redLED_on, buttonPressed - 1); // turn off LED
    if ((newPress) && (millis() < buttonStartTime + HOLD_TIME)) {
      trigger[buttonPressed - 1] = true;
      newPress = false;
    } else if ((newPress) && (millis() >= buttonStartTime + HOLD_TIME) && (millis() <= buttonStartTime + MAX_HOLD_TIME) ) {
      waveform[buttonPressed - 1] ++;
      if (waveform[buttonPressed - 1] >= MAX_WAVEFORMS) {
        waveform[buttonPressed - 1] = 0;
      }
      newPress = false;
    }
    buttonPressed = 0;
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
          trigger[ch] = false; // trigger reset once processed
          // only recalculate period, attack & release times at start of envelope
          attackTime[ch] = int(map(tweak0[ch], 0, 1024, MIN_AR, MAX_ATTACK));
          releaseTime[ch] = int(map(tweak1[ch], 0, 1024, MIN_AR, MAX_RELEASE));
          period[ch] = map(potValue[ch], 0, 1023, MIN_PERIOD, MAX_PERIOD); // pot goes other way round
        }
        if (place[ch] < attackTime[ch]) { // attack phase
          value[ch] = int(map(place[ch], 0, attackTime[ch], 0, DAC_MAX));
        } else if ((place[ch] >= attackTime[ch]) && (place[ch] <= attackTime[ch] + period[ch])) { // sustain
          value[ch] = DAC_MAX; // ? inverted if EGs are in red
        } else if ((place[ch] > attackTime[ch] + period[ch]) && (place[ch] < attackTime[ch] + period[ch] + releaseTime[ch])) { // release phase
          value[ch] = int(map(place[ch], attackTime[ch] + period[ch], attackTime[ch] + period[ch] + releaseTime[ch], DAC_MAX, 0));
        } else if (place[ch] >= attackTime[ch] + period[ch] + releaseTime[ch]) { // ended
          digitalWrite(redPin[i], LOW); // back to green
          place[ch] = attackTime[ch] + period[ch] + releaseTime[ch]; // stop rollover, until retriggered
          value[ch] = 0;
        }
        break;

      case SINE: // sine
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0;
          trigger[ch] = false;
        }
        phase[ch] = tweak2[ch];
        value[ch] = int(sine(map(place[ch], 0, period[ch], 0, 1023), phase[ch])); // nearest cycle, means could get ragged sine wave at faster Hz then 256 clock periodicity
        // ? tweak0 does fine tuning, tweak1 does shape (to triangle), tweak2 does noise
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
        // ? tweak1 should do flattening of waveform
        // tweak2 randomness
        break;

      case PULSE: // srq wave
        if (place[ch] >= period[ch]) { // positive cycle triggered by gate in
          place[ch] = 0;
        }
        duty[ch] = 10 + (period[ch] * tweak0[ch] / 1050); // fudge so always some pulse at min & max
        // deal with triggers
        if (trigger[ch] == true) { // if new trigger - forwards to next transition
          if (place[ch] < duty[ch]) {
            place[ch] = duty[ch];
          } else {
            place[ch] = 0;
          }
          trigger[ch] = false; // for now
        }
        if (place[ch] < duty[ch]) { // (period[0] / 2)) { // first half of waveform
          value[ch] = DAC_MAX;
        } else {
          value[ch] = 0;
        }
        // ? write toggle instead, so can be triggered by pulse - alternates high & low
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
        // ?? tweaks 1&2 do harmonics? that would be mental
        break;

      case RANDOM:
        if ((place[ch] >= period[ch]) || (trigger[ch] == true)) {
          place[ch] = 0; // minimum waveform cycle for sine, could potentially get extra octaves out of ramp or square wave
          trigger[ch] = false;
          value[ch] = random(DAC_MAX);
        }
        break;

      case POT: // direct CV
        place[ch] = 0;
        value[ch] = map(potValue[ch], 0, 1024, 0, DAC_MAX);
        trigger[ch] = false;
        // tweak2 = randomness
    }
  }
}

void updateLEDs() {
  for (int i = 0; i <= 3; i ++) {
    digitalWrite(redPin[i], bitRead(redLED_on, i));
  }
  analogWrite(greenPin[0], value[0] / 16 ); // fo from 12 to 8 bit resolution for PWM to LEDs
  analogWrite(greenPin[1], value[1] / 16 );
  analogWrite(greenPin[2], value[2] / 16 );
  analogWrite(greenPin[3], value[3] / 16 );
}

int sine(int i, int phase) {
  int place = i - phase - 135; // magic number, to get sine to start at 0 with duty, (tweak0) at half pot, 512
  while (place < 0) {
    place += 1024;
  }
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
  void flashLED() {
  // not used
   // if (timeElapsed > 1000) {
  //   redOn = !redOn;
  //  digitalWrite(redPin[0], HIGH); // redOn);
  //   timeElapsed = 0;
  //  }
  //  if (timeElapsed > 300) {
  //  digitalWrite(redPin[0], LOW);
  // }
  }
*/
