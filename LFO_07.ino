/* 
 *  LFO via PWM, for arduino, into a modular. 
 *  
 *  borrowed from 
 * http://abrammorphew.com/notes/2012/01/05/arduino-lfo-generator/
 * 
 * PWM output looks OK through 220R with 100nF to gnd
 * changes take this from 10secs period to 30Hz max
 */

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


// one quarter of 
uint8_t sineTable[] = {
128,128,129,130,131,131,132,133,134,135,135,136,137,138,138,139,
140,141,142,142,143,144,145,145,146,147,148,149,149,150,151,152,
152,153,154,155,155,156,157,158,159,159,160,161,162,162,163,164,
165,165,166,167,168,168,169,170,170,171,172,173,173,174,175,176,
176,177,178,178,179,180,181,181,182,183,183,184,185,186,186,187,
188,188,189,190,190,191,192,192,193,194,194,195,196,196,197,198,
198,199,200,200,201,202,202,203,204,204,205,205,206,207,207,208,
208,209,210,210,211,211,212,213,213,214,214,215,215,216,217,217,
218,218,219,219,220,220,221,222,222,223,223,224,224,225,225,226,
226,227,227,228,228,229,229,230,230,230,231,231,232,232,233,233,
234,234,234,235,235,236,236,237,237,237,238,238,239,239,239,240,
240,240,241,241,241,242,242,242,243,243,243,244,244,244,245,245,
245,246,246,246,247,247,247,247,248,248,248,248,249,249,249,249,
250,250,250,250,250,251,251,251,251,251,252,252,252,252,252,252,
253,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,
254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};

// starting phase
uint8_t tWave = 128;
uint8_t sWave = 255;
uint8_t ruWave = 128;
uint8_t rdWave = 128;
uint8_t rWave = 128;

int   i = 0;
float   rate;
int   waveform;
byte  d = HIGH;

void setup() {
  pinMode(3, OUTPUT);
  setupTimer();
  OCR0A = 128;
}

void loop() {
  // waveform = map(analogRead(0),0,1023,1,7);
  waveform = 1; // testing
  rate = 230; // 0 is fastest, ~30Hz
  // needs to be exponential - little difference in slow half of pot travel
  // rate = map(analogRead(1),0,1023,255,0);
  OCR0A = rate;
}

ISR(TIMER0_COMPA_vect) {
  if(i >= 2048) i = 0;
  switch(waveform) {
    case 1:
      analogWrite(3,sine(i));
    break;
    case 2:
      analogWrite(3,triangle(i));
    break;
    case 3:
      analogWrite(3, square(i));
    break;
    case 4:
      analogWrite(3, rampUp(i));
    break;
    case 5:
      analogWrite(3, rampDown(i));
    break;
    case 6:
      analogWrite(3, rand(i));
    break;
    case 7:
      analogWrite(3, white(i));
    break;
  }
  i++;
}

void setupTimer() {
  cli();
/*--- TIMER2 CONFIG ---*/
  sbi(TCCR2A,WGM20);
  sbi(TCCR2A,WGM21);
  cbi(TCCR2A,WGM22);
 
  sbi(TCCR2B, CS20);
  cbi(TCCR2B, CS21);
  cbi(TCCR2B, CS22);

  sbi(TCCR2A,COM2B1);
  cbi(TCCR2A,COM2B0);
   
 /*--- TIMER0 CONFIG ---*/ 
  cbi(TCCR0B,CS00);
  cbi(TCCR0B,CS01);
  sbi(TCCR0B,CS02);
 
  sbi(TCCR0A, COM0A1);
  cbi(TCCR0A, COM0A0);
    
  cbi(TCCR0A, WGM00);
  sbi(TCCR0A, WGM01);
  cbi(TCCR0A, WGM02);

  cbi(TIFR0,OCF0A);
  sbi(TIMSK0,OCIE0A);
  sei(); 
}

int sine(int i) {
  int place = int(i/2);
  if (place <= 255) {
  return sineTable[place];
  } else if ((place < 512) && (place >= 256)) {
     return sineTable[511-place]; 
  } else if ((place <= 767) && (place >= 512)) {
    return (255 - sineTable[place - 512]);
  } else if (place >= 768) {
    return (255 - sineTable[1023 - place]);
  }
}

int triangle(int i) {
  if(tWave >= 255) d = LOW;
  if(tWave <= 0) d = HIGH;
  if (i%4 == 0) {
  if(d == HIGH) tWave++;
  if(d == LOW) tWave--;
  }
  return tWave; 
}

int rampUp(int i) {
  if (i%8 == 0) {
  ruWave++;
  }
  if(ruWave > 255) ruWave = 0; 
  return ruWave;
}

int rampDown(int i) {
  if (i%8 == 0) {
  rdWave--;
  }
  if(rdWave < 0) rdWave = 255;
  return rdWave;
}

int square(int i) {
  if(i >= 1024) sWave = 2048;
  if(i <= 1023) sWave = 0;
  return sWave;
}

int rand(int i) {
   if (i%1024 == 0) { // 4 times per cycle
 rWave = random(255);
   }
  return rWave;
}

int white(int i) {
  return random(255);
}
