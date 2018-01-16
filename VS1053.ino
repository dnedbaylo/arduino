/*
  Adapted from Adafruit VS1053 to play on the Music Shield v1.2, 
  originally built by, but not now supported by, Seeed Studio
  the shield uses pin 10 as CS for the micro SD card.
  start playing only works if interrupts sorted
use wires, connect VS1053_DREQ to 2 to use uno interrupt. 
other digital pins not needed, anything on that side (unless using 10 for sd card). A1 not used
so use power pins, SPi header, A0,2,3,4,5

*/


/*
  This is an example for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
*/



const int nextSongPin = 3;
char currentSongName[13];
int currentSongNo = 2;

#define MAX_FILES 10
String results[MAX_FILES];

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

// These are the pins used
#define VS1053_RESET   A0     // VS1053 reset pin this IS used. 

#define VS1053_CS       A3     // VS1053 chip select pin (output)    
#define VS1053_DCS     A2     // VS1053 Data/command select pin (output)
#define CARDCS          A5 // 10     // Card chip select pin    shield uses digital 10
#define VS1053_DREQ     2 // A1     // VS1053 Data request, shield uses A1 but needs 2 for interrupt, use wire instead

Adafruit_VS1053_FilePlayer musicPlayer =
  Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

void setup() {
  Serial.begin(115200);
  pinMode(nextSongPin, INPUT_PULLUP);

  Serial.println("\n\nMusic Shield v1.2 Test");

  if (! musicPlayer.begin()) { // initialise the music player
    Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    while (1);
  }

  Serial.println(F("VS1053 found"));

  //  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working
  // first is frequency (complicated), second is length of tone

  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");

  printDirectory(SD.open("/"), 0);   // list files

  for (int i = 0; i < MAX_FILES; i ++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(results[i]);
  }

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(10, 10);
/*
#if defined(__AVR_ATmega32U4__)
  // Timer interrupts are not suggested, better to use DREQ interrupt!
  // but we don't have them on the 32u4 feather...
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT); // timer int
#elif defined(ESP32)
  // no IRQ! doesn't work yet :/
#else
  // If DREQ is on an interrupt pin we can do background
  // audio playing
    Serial.println("else, no interrupt defined");
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);  // DREQ int
#endif
*/


// put DREQ (A1) onto pin 2 instead to use interrupt?


 musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);

  // Play a file in the background, REQUIRES interrupts!

playNow(currentSongNo);



  //  musicPlayer.startPlayingFile("TRACK003.MP3");
}

void loop() {

  //  musicPlayer.sineTest(0x44, 440);
  //  Serial.println("play sine");

  if (!nextSongPin) {
    currentSongNo ++;
    playNow(currentSongNo);
    Serial.print("! next song pin");
  }

  // musicPlayer.startPlayingFile("ONHOLD1.OGG");

  // makes it wait at end of file
  Serial.print(".");
  // File is playing in the background
  if (musicPlayer.stopped()) {
    Serial.println("Done playing music");
    while (1) {
      delay(10);  // we're done! do nothing...
    }
  }

  if (Serial.available()) {
    char c = Serial.read();

    // if we get an 's' on the serial console, stop!
    if (c == 's') {
      musicPlayer.stopPlaying();
    }

    // if we get an 'p' on the serial console, pause/unpause!
    if (c == 'p') {
      if (! musicPlayer.paused()) {
        Serial.println("Paused");
        musicPlayer.pausePlaying(true);
      } else {
        Serial.println("Resumed");
        musicPlayer.pausePlaying(false);
      }
    }
  }
  delay(1000000);
}

void playNow(int current){
  results[current].toCharArray(currentSongName, 13);
    Serial.print("now playing ");
  Serial.println(currentSongName);
//  musicPlayer.playFullFile(currentSongName);
  musicPlayer.startPlayingFile(currentSongName);
  
}

void printDirectory(File dir, int numTabs) {
  int count = 0;
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) { // no more files
      break;
    }
    results[count] = entry.name();
    entry.close();
    count ++;
  }
}
