#include <Tone.h>

#include "WProgram.h"
void setup();
void loop();
void displayDigit(int digit);
void countUp();
void countDown();
boolean popEvent();
void soundAlarm();
void silenceAlarm();
void serviceAlarm();
int numDisplayPins = 8;

int audioPin = 9; //has to be a PWM pin set to output
int buttonPin = 12; //has to be set to an input pin
int displayPins[] = { //have to be set to output
  7,6,4,3,2,10,11,5
}; 
//displayPins array map to a,b,c,d,e,f,g,dp segments
//of D5611 from http://www.nb-flying.com/pdf/LED-display.pdf
//inverse of map information below

//map D5611Ax 1, 2, 4, 5, 6, 7, 9,10 from http://www.nb-flying.com/pdf/LED-display.pdf
//map Arduino 2, 3, 4, 5, 6, 7,10,11
//map segment e, d, c,dp, b, a, f, g 

//correlate with a,b,c,d,e,f,g
//from http://en.wikipedia.org/wiki/Seven-segment_display
//to be expanded with letters, plus use of decimal point as 10s or 16s
byte digitSequence[] = {
  0x3F, //0
  0x06, //1
  0x5B, //2
  0x4F, //3
  0x66, //4
  0x6D, //5
  0x7D, //6
  0x07, //7
  0x7F, //8
  0x6F, //9
  0x37, //A = abcef = 11101100 = 00110111
  0x7C, //b = cdefg = 00111110 = 01111100
  0x39, //c = adef  = 10011100 = 00111001
  0x5E, //d = bcdeg = 01111010 = 01011110
  0x79, //e = adefg = 10011110 = 01111001
  0x71 //f = aefg  = 10001110 = 01110001
}; 

//only works for 0-9 for now, can use A, b, C, D, E F to display other info.
int base = 16;

//the digit to show
int number = 0;

long lastButtonChange, lastEvent;
long buttonIdle, buttonDelay = 50;

boolean buttonCircuit,lastButtonCircuit;
boolean buttonDown,lastButtonDown;

Tone tone;
boolean alarm;
long lastTone;

int OPERATION_MODE = 0;
int CONFIGURATION_MODE = 1;

int mode = OPERATION_MODE;

void setup(){
  //configure pins in correct modes
  pinMode(audioPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  for (int i = 0; i < numDisplayPins; i++) {
    pinMode(displayPins[i], OUTPUT);  
  }  
  
  //set up debugging over serial
  Serial.begin(9600); 
  
  //configure tone library for speaker pin
  tone.begin(audioPin);
}

void loop(){
  
  //read button state with debounce
  buttonCircuit = digitalRead(buttonPin) == HIGH;
  if(buttonCircuit != lastButtonCircuit){
    lastButtonChange = millis();
  }
  lastButtonCircuit = buttonCircuit;
  buttonIdle = millis() - lastButtonChange;
  if( buttonIdle > buttonDelay){
    buttonDown = buttonCircuit;
  }

  //use buttons to control countdown digit  
  if(!buttonDown && lastButtonDown){
    countUp();
  }
  lastButtonDown = buttonDown;

  int seconds = (millis() / 1000);
  if(buttonIdle > 10000){
    mode = OPERATION_MODE;
  }
  else{
    mode = CONFIGURATION_MODE;
  }

  
  if(mode == OPERATION_MODE){
    //trigger a countdown if appropriate
    if(popEvent()){
      countDown();
    }
    if(number == 1){
      soundAlarm();
    }
    else{
      silenceAlarm();
    }
  }
  
  //show the countdown
  displayDigit(number);


  serviceAlarm();
  
}

/** Displays the specified digit. */
void displayDigit(int digit){

  if(digit >= 0 && digit < (2 * base)){
    //work out remainder (unit digit) and digital point (tens digit)
    boolean dp = digit >= base;
    int hex = digit % base;
    //write the digit
    for(int bitidx=0;bitidx<numDisplayPins -1;bitidx++){
      digitalWrite(displayPins[bitidx], (digitSequence[hex] & (0x01 << bitidx)?HIGH:LOW));
    }
    //write the decimal point
    digitalWrite(5, (dp?HIGH:LOW));
  }
  else{
    Serial.print("digit = ");
    Serial.print(digit);
    Serial.println("; cannot be displayed");                         
  }
}

void countUp(){
  number = (number + 1) % (2 * base);
}

void countDown(){
  if(number > 0){
    number--;
  }
}

/** Has the thing we've been monitoring for happened at least once? */
boolean popEvent(){
  if(millis() - lastEvent > 10000){
    lastEvent = millis();
    return true;
  }
  else{
    return false;
  }
}

void soundAlarm(){
  alarm = true;
}

void silenceAlarm(){
  alarm = false;
}

void serviceAlarm(){
  if(alarm){
    if(millis() - lastTone > 1000){
      tone.play(NOTE_C4, 500);
      lastTone = millis();
    }
  }
  else{
    tone.stop();
  }
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

