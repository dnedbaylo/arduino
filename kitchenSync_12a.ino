/*
kitchenSync! v12
send MIDI to jamman solo XT via jamsync in

4 digital ins: MIDI, clock, reset, JM. 
5 digital outs MIDI, clock, reset, JM. LED to mark loops
? SPI outs to digipot: MIDI to CV outs. 
send to ring of 3.5mm output & 3.3V to ring of 3.5mm
2 analog in: switch - 4 way for tempo master. switch for clock divider - eg 1/2/4/8/16/24 via resistor ladder
also 3 buttons - up, down, record/stop connecting 5v to 
3 jack socket for switch lead and 2 ins to JM - from back. 
1 jack socket for input to JM. 3 way switch to send this to L, R or both channels. 

to do - lagPot which delays clock2 & midi2 up to 2 beats
v12 rewrite to consolidate
v11 put it all together. sort out clock divisions ? interpolate to get half divisions? space for up to 12 ie 1,2,3,4,5,6,8,9,12,16,24 plus ? 0.5, 0.25
v10 footswitch done via 4066. 1M pulldown resistor on arduino pins. 
v09 sorted sync to MIDI or external clock, or sync to JM. switchable. 
v08 check JM in works - run as seperate program, 
v07 functionalised the JM outs. softserial to manage MIDI and JM. JM in to clock out - recognise code & timing settings. easier. 
*/

// global variables called before setup

#include <SoftwareSerial.h>
#include <Timer.h>


// constants
const byte midiStartByte = 0xfa;
const byte midiStopByte= 0xfc;
const byte midiClockByte = 0xf8;
const byte pinTime = 200; // length of clock and reset pulses ms
const byte ClockSubdivisions = 24; // interpolate clocks from crotchet beats
const byte UnityClockDiv = 8; // for ClockDiv pot clock speed reproduced, below this subClocks extrapolated (faster than incoming)
const int DebounceTime = 1000; // ms for switch debounce

// grids takes MIDI in (post optocoupler), clock and random trigger out, clock in, reset in, reset out from front. 
// grids clock runs at 4, 8, 24ppqn (crotchet)
// input pins
const byte clockDivInPin = A0;
const byte toggleAudioInPin = A1;
const byte clockInPin = A2;
const byte resetInPin = A3;
const byte clock2InPin = A4; // ? from back panel
const byte reset2InPin = A5; // from accent 3 
const byte clockLagInPin = A6;
const byte midi2InPin = A7; // not actually used but needed for software serial
const byte fromJMPin = 0; // Rx
const byte midiInPin = 2;
const byte ringFSPin = 12;
const byte tipFSPin = 13;

// output pins
const byte toJMPin = 1; // Tx
const byte midiOutPin = 3;
const byte clockOutPin = 4;
const byte resetOutPin = 5;
const byte clock2OutPin = 6; // to grids
const byte reset2OutPin = 7;
const byte LEDPin = 8; // ?? needed
const byte midi2OutPin = 9; // straight to grids
const byte audioLeftPin = 10;
const byte audioRightPin = 11;

// LEDs can come off clock and reset signal, if spare outs could use these

SoftwareSerial midiSerial(midiInPin,midiOutPin); // RX, TX
SoftwareSerial JM(fromJMPin,toJMPin);
SoftwareSerial midi2InOut(midi2InPin,midi2OutPin); // if spare pin needed could fake midiFromGrids pin
Timer t; // may not be needed

// working variables
int MIDIclock = 0;
byte MIDIdata  = 0;
byte JMdata = 0;
byte byteNumber = 0;
byte byteEight = 0;
byte byteTwelve = 0;
byte byteThirteen = 0;
byte byteTwentyTwo = 0;
int tempo = 80;
int MIDItempo = 120; // 24 ppqn
int JMtempo = 120; // from JM bytes
int clockTempo = 120; // set via clock divider
int gridsTempo = 120; // 24ppqn
int beatsInLoop = 4;
byte beatsPerBar = 4;
int maxBar = 64; // maximum bars before time resets. not crucial
byte semiQuaversInBeat = 4;
byte semiQuaver = 0;
byte beat = 0;
byte bar = 0;
byte clockIn = 0;
unsigned long lastBeatTime = 0;
unsigned long lastBeatStart = 0;
unsigned long lastClockBeatStart = 0;
unsigned long lastClockBeatTime = 0;
unsigned long lastMIDIBeatTime = 0;
unsigned long lastMIDIBeatStart = 0;
unsigned long timeSinceLastClock = 0;
unsigned long lastClockInTime = 0;
boolean notRecentPlay = 1;
boolean tempBit = 0;
boolean newTempo = 0;
boolean oldClockIn = 0;
boolean oldResetIn = 0;
boolean playFlag = 0;
boolean stopFlag = 0;
boolean loopFlag = 0;
unsigned  long syncTime = 0;
unsigned  long loopTime = 0;
unsigned long loopDuration = 0;
unsigned long divLoopDuration = 0;
unsigned float lagTime = 0;
unsigned int lagPot = 0;
int i = 0;
int beatTime = 0;
int multOffset = 0;
int incomingByte = 0;
byte subClocks = 0;
byte oldSubClocks = 0;

// footswitch
const byte pulseTime = 50; // ms of pulse
unsigned long holdRingHigh = 0;
unsigned long holdTipHigh = 0;
boolean ringFSPinToggle = 0;
boolean tipFSPinToggle = 0;

// audio input switch
boolean audioLeftToggle = 1;
boolean recentFootSwIn = 0;
unsigned long debounceFSTime = 0;
 
byte syncByte[] = {0xF0,0x00,0x00,0x10,0x7F,0x62,0x01,0x00,0x01,0x01,0xF7}; // sync byte every 412ms
byte transportByte[] = {0xF0,0x00,0x00,0x10,0x7F,0x62,0x01,0x4A,0x01,0x00,0x00,0x70,0x42,0x02,0x04,0x40,0x03,0x00,0x00,0x00,0x3F,0x05,0x78,0xF7}; 
byte stopByte[] = {};
byte playByte[] = {};

byte syncMaster = 0; // 0 is JM, 1 is MIDI, 2 is clock, 3 is grids

void setup() 
{
Serial.begin(9600); // for testing 
JM.begin(31250); // JM interface 
midiSerial.begin(31250); // MIDI in & out
midi2InOut.begin(31250); // actually just MIDI clocks to grids
syncTime = millis();
loopTime = millis();
pinMode(clockOutPin, OUTPUT);
digitalWrite(clockOutPin, LOW);
pinMode(resetOutPin, OUTPUT);
digitalWrite(resetOutPin, LOW);
pinMode(clock2OutPin, OUTPUT);
digitalWrite(clock2OutPin, LOW);
pinMode(reset2OutPin, OUTPUT);
digitalWrite(reset2OutPin, LOW);
pinMode(LEDPin, OUTPUT);
digitalWrite(LEDPin, LOW);
pinMode(tipFSPin, INPUT); // digital pins inputs by default
pinMode(ringFSPin, INPUT);
digitalWrite(tipFSPin, LOW);
digitalWrite(ringFSPin, LOW);
pinMode(toggleAudioInPin, INPUT); 
pinMode(audioLeftPin, OUTPUT); // turns off internal pull up
pinMode(audioRightPin, OUTPUT);
digitalWrite(audioLeftPin, HIGH);
digitalWrite(audioRightPin, LOW);
}


void loop() 
{
checkClockDivPot();  
checkClockLagPot();
// checkReset();
  
// read syncMaster - analogRead 5V -> 4 way switch via resistors JM, MIDI, clock, clock2
syncMaster = (analogRead(clockDivInPin) / 256);

if (syncMaster != oldSyncMaster) {  // change tempo only if switch changed
  // load new tempo
  /*
if (syncMaster == 0) { // JM
  newTempo = JMtempo;
} else if (syncMaster == 1) {
  newTempo = MIDItempo; 
} else if (syncMaster == 2) { // clock on
  newTempo = clockTempo;
} else if (syncMaster == 3) {
  newTempo = gridsTempo;
}  
  */
oldSyncMaster = syncMaster; 

// look for change tempo & update JM chunks if so
if (newTempo != tempo) {
 setJMbytes();   // if tempo changes, send off to function to recalculate transport bytes.  
 tempo = newTempo;
  }
}



// this should be main calling function - including extrapolateSubClock() if JM, clock, clock2
if (syncMaster == 0) {
  checkJM(); // look for incoming JM tempo
  // check for loopFlag ie JM has looped. if so send clock and reset to grids, not MIDI
  // extrapolate subclock and send MIDI subclock
  // send clock at every (clockDiv) subclocks
  // send laggedMIDI and clock if needed
} else if (syncMaster == 1) {
  checkMIDI(); // pick up incoming MIDI tempo
  // send JMloop when needed - look at tempo & length of JM loop
  // if MIDI start signal, send JM loop & grids reset & start
  // ? send resets at end of loop
  // send lagged MIDI2 shifted if needed
} else if (syncMaster == 2) {
  checkClockIn(); // look for incoming clock or reset
 // extrapolate and send MIDI subclock 
 // calculate lag & send
 // send JM loop when needed
} else if (syncMaster == 3) { // grids puts out ACC / CLK / RST from 3 accent outs. RST should trigger JM loop
  checkClock2In();  // look for reset or clock
  // extrapolate and send MIDI subclock
  // calculate lag & send
  // send JM loop if ready
}

// if JM master, clock signals start at loop, and happen according to tempo
// would need to extrapolate clock from JM tempo then
// one reset in (reset 1?) should make JM replay to do a n-n-n-nineteen thing...
// if MIDI master, subclocks at 24ppqn coming in so derive clock from this
// if clock in or clock2in, extrapolate subclock

// send subclock to MIDI outs 24ppqn
// look at subClock and send Clock at every ClockDiv * subclock ie everytime subClock = 8, send Clock out


// should update subClocks at 24ppqn
// sort out subdivisions of tempo for each syncMaster

extrapolateSubClock();

checkAudioToLRToggleIn(); // look at audio in switches & toggle audio input

// deal with flags

if (resetFlag) {
  if (syncMaster != 0) { // not JM sync
    sendJMloop();
  }
  if (syncMaster != 2) {
    sendReset();
  }
  if (syncMaster != 3) { // not clock 2 (grids)
    sendMidi2Reset(); // send MIDI reset to grids
  }
  // don't send MIDI reset either way, in case running cubase
} // end resetFlag processing

// MIDI start signal retriggers grids 
  // incoming MIDI reset signal triggers loop chunk
  // incoming reset signal (eg from seq) triggers loop




/* not sure about this bit
  clockIn = 0;
  lastClockBeatTime = millis() - lastClockBeatStart; // recheck tempo from last crotchet
  lastClockBeatStart = millis();
  clockTempo = 60000 / lastClockBeatTime; 
*/  

// need visual ouput of loop - clock signal routed to yellow LED, beats to green, reset to blue?

//housekeeping
// send transport chunks
t.every(412, sendJMsync);
t.update(); // services timer 

/* 
// this should be dealt with in main loop
  if ((millis () - loopTime) >= loopDuration) {
    sendJMreset (); 
    loopTime = millis();
    notRecentPlay = 1; // reset play button
  }
*/ 

} // end loop

void extrapolateSubClock() { // this should calculate SubClock time from tempo, and return subClockFlag if a 24ppqn subclock due
// if a new clock in, subclock times start from this. doesn't deal with new clock otherwise

//   if (clockIn != oldClockIn) {
//     timeSinceLastClock = (micros() - lastClockInTime);
//     lastClockInTime = micros();
//     clockSubdivisionTime = timeSinceLastClock / ClockSubdivisions; // 24 clocks per crotchet 

  clockSubdivisionTime = 60000000 * clockDiv / (tempo * ClockSubDivisions * UnityClockDiv); // time between subclocks in us
     clockFlag = 1; // need to send clock
   oldClockIn = clockIn; 
    }
  // sendClock at subdivisions
  if (micros() > (lastSubClockTime + clockSubdivisionTime)) {
    subClockFlag = 1; // flag subClock due
    lastSubClockTime = micros();
    } 
} // end extrapolateSubClock

void sendJMsync () {
    Serial.write(syncByte,sizeof(syncByte));
  }

void sendJMreset () {
    Serial.write(transportByte,sizeof(transportByte));
}  

void sendJMplay() {
    Serial.write(playByte,sizeof(playByte));
} 

void sendMidiPlay () {
  midiSerial.write(midiStartByte,sizeof(midiStartByte));  
}

void sendMidiClock () {
  midiSerial.write(midiClockByte,sizeof(midiClockByte));  
  
void sendMidi2Reset() {
  midiSerial.write(midiResetByte,sizeof(midiResetByte));   
}

void sendMidi2Clock() { // send clock bytes on midi2OutPin to grids
  midi2InOut.write(midiClockByte,sizeof(midiClockByte));   
}

void sendMidi2Reset() { // send reset bytes on midi2OutPin
  midi2InOut.write(midiResetByte,sizeof(midiResetByte));   
}

void checkClockLagPot () {
 // look at 0-1023 value of pot. makes (float) lagTime of 2 beats, holes at 0,1,2. 
  lagPot = analogRead(clockLagInPin):
  if (lagPot < 10) {
   lagTime = 0;
  } else if (lagPot > 1013) {
   lagTime = 2;
  } else if ((lagPot <= 500) && (lagPot >=10)) {
    lagTime = ((lagPot - 10) / 490); 
    else if ((lagPot >= 523) && (lagPot <= 1013)) {
    lagTime = ((lagPot - 523) / 490);
    lagTime ++;
    else {
      lagTime = 1;
    }
}

void checkClockIn () {
  if ((digitalRead(clockInPin)) && (!oldClockIn)) {
     oldClockIn = 1;
     clockIn ++;
  }
  if (!digitalRead(clockInPin)) {
     oldClockIn = 0; 
  }
}

void checkClock2In () {
// same as above except use clock2InPin 
}

void checkResetIn() {
  if ((!oldResetIn) && (digitalRead(resetInPin))) {
    oldResetIn = 1;
    sendReset();
  }
  if (!digitalRead(resetInPin)) {
    oldResetIn = 0;
  }
}

void sendReset () { // sends resets out - only called if not synced to clock 1
  t.pulse(resetOutPin, pinTime, HIGH);
//  t.pulse(reset2OutPin, pinTime, HIGH);
}


void sendClock() { // send clock, ? subClock signals depending on division, prob separate function
  subClockCount++;
  if (subClockCount >= clockDiv) { 
    subClockCount = 0;
    t.pulse(clockOutPin, pinTime, HIGH);
    if (syncMaster != 3) { // ie not GRIDS
      sendMidi2Clock();
    }
  }   
}

// functions for footswitch and audio in to L or R toggle
void sendFSUp() {
     ringFSPinToggle = 1;
     tipFSPinToggle = 1; 
     digitalWrite(ringFSPin, HIGH);
     digitalWrite(tipFSPin, HIGH);  
     holdRingHigh = (pulseTime + millis());
     holdTipHigh = (pulseTime + millis());
}  

void sendFSDown () {
      ringFSPinToggle = 1;
      digitalWrite(ringFSPin, HIGH);
      holdRingHigh = (pulseTime + millis());
}

void sendFSTempo() {
      tipFSPinToggle = 1;
      digitalWrite(tipFSPin, HIGH);
      holdTipHigh = (pulseTime + millis());
}

void checkFS () { // turn off pins after pulseTime
  if ((ringFSPinToggle) && (millis() > holdRingHigh)) {
    digitalWrite(ringFSPin, LOW);
    ringFSPinToggle = 0;
  }
  if ((tipFSPinToggle) && (millis() > holdTipHigh)) {
    digitalWrite(tipFSPin, LOW);
    tipFSPinToggle = 0;
  }    
}

void checkAudioToLRToggleIn() { 
    if ((digitalRead(toggleAudioInPin)) && (!recentFootSwIn)) {
      recentFootSwIn = 1;
      toggleAudioPin();
      debounceFSTime = (DebounceTime + millis());
    }  
    if ((recentFootSwIn) && (millis() > debounceFSTime)) {
      recentFootSwIn = 0;
    } 
}

void toggleAudioPin () {
  if (audioLeftToggle) {
   audioLeftToggle = 0;
  } else if (!audioLeftToggle) {
   audioLeftToggle = 1; 
  }
  if (audioLeftToggle) {
    digitalWrite(audioLeftPin, HIGH);
    digitalWrite(audioRightPin, LOW);
  } else if (!audioLeftToggle) {
    digitalWrite(audioLeftPin, LOW);
    digitalWrite(audioRightPin, HIGH);
  } 
} // end toggleAudioPin  

void checkClockDivPot() { // clockDiv is * 8 ie returns 8 if clock every 1 clock, 4 if 1/2 of clock, 1 if 1/8 of clock
 // read analogIn from clockDivInPin
  int clockDivPotReading = analogRead(clockDivInPin);
  if (clockDivPotReading < 78) { // 12 readings
  byte clockDiv = 1;
  } else if ((clockDivPotReading >= 78) && (clockDivPotReading <156)) {
    clockDiv = 2;
  } else if ((clockDivPotReading >= 156) && (clockDivPotReading <234)) {
    clockDiv = 4;
  } else if ((clockDivPotReading >= 234) && (clockDivPotReading <322)) {
    clockDiv = 8;
  } else if ((clockDivPotReading >= 322) && (clockDivPotReading <400)) {
    clockDiv = 16;
  } else if ((clockDivPotReading >= 400) && (clockDivPotReading <478)) {
    clockDiv = 24;
  } else if ((clockDivPotReading >= 478) && (clockDivPotReading <556)) {
    clockDiv = 32;
  } else if ((clockDivPotReading >= 556) && (clockDivPotReading <634)) {
    clockDiv = 40;
  } else if ((clockDivPotReading >= 634) && (clockDivPotReading <712)) {
    clockDiv = 48;
  } else if ((clockDivPotReading >= 712) && (clockDivPotReading <790)) {
    clockDiv = 56;
  } else if ((clockDivPotReading >= 790) && (clockDivPotReading <868)) {
    clockDiv = 64;
  } else if (clockDivPotReading >= 868) {
    clockDiv = 128;
  }    // or
}

void checkMIDI() { // watch incoming MIDI clock
  if (midiSerial.available()) {
   MIDIdata = midiSerial.read(); 
  } 
  if (MIDIdata == midiClockByte) { // MIDI clock at 24ppqn = crotchets
    MIDIclock ++;
    subClocks ++;
  }
  if (MIDIdata == midiStartByte) {
    MIDIclock = 0;
    beat = 0;
    bar = 0;
    // send resets
  }  
  if (MIDIclock >= 24) {
  beat ++;
  MIDIclock = 0;
  lastMIDIBeatTime = micros() - lastMIDIBeatStart; // recheck tempo from last crotchet
  lastMIDIBeatStart = micros();
  MIDItempo = 60000000 / lastMIDIBeatTime; // should this be 60000001?
  MIDItempo ++; // rounds tempo down hence this fudge
  }

  if (beat >= beatsInLoop) { // beats to bars
  bar ++;
  beat = 0;
  if (bar > maxBar) {
    bar = 0;
    }
  } 
  MIDIdata = 0; // clears buffer
} // end checkMIDI


void checkJM() {
  // watch for JM inputs & look at bytes 8, 12 & 13, to get tempo - or look at how often transport byte comes around. which is more effective?
  if (JM.available()) {
   JMdata = JM.read(); 
   byteNumber ++;
  } 
  if ((JMdata == 240) || (JMdata == 247)) { // 240 is F0 start of sysex chunk, 247 is F7 end chunk
  byteNumber = 0; // start counting here
  }
  if ((byteNumber == 7) && (JMdata > 10)) { // it's a transport byte if 8th byte is not 00
  byteEight = JMdata;
  }
  if (byteNumber == 11) {
  byteTwelve = JMdata;  
  }
  if (byteNumber == 12) {
  byteThirteen = JMdata;  
  }
  if (byteNumber == 21) {
  byteTwentyTwo = JMdata;  
  }

  // check tempo when byte thirteen received
  if (byteNumber == 12) {
      // if byte 8 is 42, tempo = 32 + byte 12 / 4
      if (byteEight == 66) { // 0x42 is 66
        JMtempo = 32 + (byteTwelve / 4);
      }
    // if byte 8 is 4A, tempo = 64 + byte 12 /2 
      if (byteEight == 74) { // 0x4A is 74
        JMtempo = 64 + (byteTwelve / 2);
      }
    // if byte 13 is 43, tempo = 128 + byte 12
      if (byteThirteen == 67) {
        JMtempo = 128 + byteTwelve;
      }
  }  
  
  if (byteNumber == 21) { // byte 22 in so check if loop etc
      if (byteTwentyTwo == 2) { // 02 for play, 04 for stop, 05 for loop
        // sendPlay();
        playFlag = 1;
        byteTwentyTwo = 0;
      }
        if (byteTwentyTwo == 4) { // stop
        // sendStop();
        stopFlag = 1;
        byteTwentyTwo = 0;
      }
        if (byteTwentyTwo == 5) { // loop
        // sendLoop();
        loopFlag = 1;
        byteTwentyTwo = 0;
      }
  }
} // end checkJM

void setJMbytes () {
   
  beatTime = 60000 / tempo;
  loopDuration = beatTime * beatsInLoop;
  
  // modulus loopDuration so reduces until 1-2 secs. count times we need to do this to get in range. 
  divLoopDuration = loopDuration;
  multOffset = 0;
  
   while (divLoopDuration >= 2000) {
      divLoopDuration /= 2;
      multOffset ++;    
     }
  
    byte multFactor = bit(multOffset);   // mF should be 8 for mO 3, 4 for 2, 2 for 1 & 1 for 0
  
  // set tempo bytes
  
    // byte 8: 4A set bit 3 to 1 (01001010) 42 set bit 3 to 0 (01000010)
    if (tempo < 64)  {
      bitClear(transportByte[7],3);
      transportByte[11] = 20 + ((tempo - 40)*4); // byte 12 goes up by 4 < 64bpm, 2 if 64-127, 1 128+
    }
    else if (tempo > 127) {
      bitClear(transportByte[7],3);
      transportByte[11] = (tempo - 128);
    }
    else {
      bitSet(transportByte[7],3);
       transportByte[11] = ((tempo - 64)*2);
    }
   
    transportByte[14] = beatsInLoop;
    
    // set bit 5-8 of byte 16 to 1000 if mult is 1 or 4 or 0000 if 2 or 8 - actually if multOffset is even
    if (!(multOffset % 2)) { // multOffset is even
        transportByte[15] |= B00001000; // set bit 3 to 1
    } else if (multOffset % 2) { // multOffset is odd
        transportByte[15] &= B11110111; // set bit 3 to 0
    }
    
    // byte 20 goes from 0-127 with loops between 1-2, then 2-4, then 4-8. need to identify better which one in 
    // ie divide down loopduration until 1-2 secs
    
    // byte 21
    transportByte[20] = 0x3F; 
     if (loopDuration > 2000) {
       transportByte[20] ++;
     }
     if (loopDuration >8000) {
       transportByte[20] ++;
     } 
     
     // byte 23 (no rhyme or reason)
   
  // set up play & stop chunks
  for (i=0; i<24; i++) {
     playByte[i] = transportByte[i];
     stopByte[i] = transportByte[i]; 
  }
  
    // NOT last 3 bits of byte 23 to get play byte
  playByte[21] = 0x02; 
  for (int j=0; j<3; j++) {
     tempBit = bitRead(playByte[22],j);
     if (!tempBit) { 
     bitSet (playByte[22], j);
     } else if (tempBit) {
     bitClear (playByte[22], j);
     }    
  }
  
    // NOT last 1 bit of byte 23 to get stop byte
  stopByte[21] = 0x04;
  tempBit = bitRead(stopByte[22],0);
  if (tempBit) {
    bitClear (stopByte[22], 0);
  } else if (!tempBit) {
    bitSet (stopByte[22],0);
  }

}  // end setJMbytes


void sendJMstop () {
    Serial.write(stopByte,sizeof(stopByte)); 
}

void sendLoop () {
// send MIDI play?
  // send reset  
}

void sendStop () {
 // currently empty - better to continue clock etc 
// could set a stopFlag which could mute clock signals 
}
