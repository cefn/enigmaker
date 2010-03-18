#include <Tone.h>
#include <Wire.h>

#define DEVICE (0x53)    //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

#include "WProgram.h"
void setup();
void loop();
float sgn(float v);
int wrap(int value, int range);
void displayDigit(int digit);
boolean pushSample(struct history *history, int sample);
void countUp();
void countDown();
void soundMip();
void soundAlarm();
void silenceAlarm();
void serviceAlarm();
char *ftoa(char *a, double f, int precision);
void writeTo(int device, byte address, byte val);
void readFrom(int device, byte address, int num, byte buff[]);
byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char numstr[16];                      //string buffer to transform data before sending it to the serial port

int numDisplayPins = 8;

int audioPin = 9; //has to be a PWM pin set to output
int buttonPin = 12; //has to be set to an input pin
int switchPin = 8; //has to be set to an input pin
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
  0x39, //C = adef  = 10011100 = 00111001
  0x5E, //D = bcdeg = 01111010 = 01011110
  0x79, //E = adefg = 10011110 = 01111001
  0x71 //F = aefg  = 10001110 = 01110001
}; 

//only works for 0-9 for now, cld use A, b, C, D, E F to display other info.
int base = 10;

//Button debounce logic
boolean buttonCircuit,lastButtonCircuit;
long circuitIdle, circuitDebounce = 50;
long lastCircuitChange;
boolean buttonDown,lastButtonDown;
long lastButtonChange, buttonHoldPeriod;

int lowAcc = 10;
int highAcc = 20;

long accLowSince = -1;
long lastStopped = -1;
long lastSample = -1;

Tone tone;
boolean alarm;
long lastTone;

int OPERATION_MODE = 0;
int CONFIGURATION_MODE = 1;

int mode = CONFIGURATION_MODE;

//the digit to show
int number = -1;

#define SAMPLE_SEPARATION 100 // in milliseconds
#define SAMPLE_NUM 100 // number of samples in running average
#define RECENT_SAMPLE_NUM 25 // number of samples in RECENT running average;

struct averager {
  int range; //number of positions to use
  long total; //total sum in averager
  float mean; //average calculated
} xaverage[] = {
  {SAMPLE_NUM, 0, 0},
  {RECENT_SAMPLE_NUM, 0, 0}
};

#define TOTAL 0 // index of total averager
#define RECENT 1 // index of recent averager

int xsample[SAMPLE_NUM];

struct history {
  int position; //start position
  long betweentime; //betweentime between samples in millis
  long lasttime;
  int numsamples;
  int *sample; //series of samples
  int numaverages;
  struct averager *average; //averagers to maintain on series
} xhistory = {
  0, 100L, -1L, 
  sizeof(xsample) / sizeof(int),
  xsample, 
  sizeof(xaverage) / sizeof(struct averager),
  xaverage
};

const float xsample_period = SAMPLE_NUM * SAMPLE_SEPARATION; //sample period in milliseconds
const float xsample_recent_period = RECENT_SAMPLE_NUM * SAMPLE_SEPARATION; //recent running average period (duration for a stop to be a stop)

void setup(){
  
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  
  //configure pins in correct modes
  pinMode(buttonPin, INPUT);
  pinMode(switchPin, INPUT);

  pinMode(audioPin, OUTPUT);
  for (int i = 0; i < numDisplayPins; i++) {
    pinMode(displayPins[i], OUTPUT);  
  }  
    
  //configure tone library for speaker pin
  tone.begin(audioPin);
  

  for(int i = 0; i < SAMPLE_NUM; i++){
    xhistory.sample[i] = 0;
  }  
  
}

void loop(){
    
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  int x, y, z, a, g = 242L, gx;
  long xsq, ysq, zsq, asq, gsq, gxsq;
  
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  x = (((int)buff[1]) << 8) | buff[0];   
  y = (((int)buff[3])<< 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  xsq = (long)x*(long)x;
  ysq = (long)y*(long)y;
  zsq = (long)z*(long)z;  
  asq = xsq+ysq+zsq;
  gsq = (long)g*(long)g;
  gxsq = gsq - (ysq + zsq);
  a = sqrt(asq);
  g = sqrt(gsq);
  gx = (int)sqrt(labs(gxsq));

  //todo add delay in

  float xsample_ave, xsample_recent_ave, xdiff;

  if(pushSample(&xhistory, x)){
    
    xdiff = float(x) - xsample_recent_ave;
    xdiff = abs(xdiff);
    
    if(xdiff < lowAcc){ //check that x is not varying a lot against recent background acceleration
      if(accLowSince == -1){
        accLowSince = millis();
      }
    }
    else{
      accLowSince == -1;
    }

    boolean still = false, decelerated = false, awaitingstop = false;
    
    if(accLowSince != -1 && ((millis() - accLowSince) > xsample_recent_period)){ //has acc not varied a lot for at least recent period
      still = true;
      //Serial.println("STILL");
    }
    
    if(xsample_ave < (-highAcc)){ //note negative - check that long term sample indicates a significant deceleration
      decelerated = true;
      //Serial.println("DECELERATED");
    }
      
    if((millis() - lastStopped) > xsample_period){ //make sure that stops are registered at most once within the sample period
      awaitingstop = true;
      //Serial.println("AWAITINGSTOP");
    }
    
    if(awaitingstop && decelerated && still){
      //Serial.println("STOPPED");
      lastStopped = millis();
      countDown();
      soundMip();
    }

    sprintf(numstr, " pos:%d,%d,%d,", x, y, z);
    Serial.print(numstr);
    sprintf(numstr, " total_av:%d recent_av:%d", (int) xhistory.average[TOTAL].mean, (int) xhistory.average[RECENT].mean);
    Serial.print(numstr);
    Serial.println();

  }
        
  //read button state and record change
  buttonCircuit = digitalRead(buttonPin) == HIGH;
  if(buttonCircuit != lastButtonCircuit){
    lastCircuitChange = millis();
  }
  lastButtonCircuit = buttonCircuit;
  
  //set button debounced
  circuitIdle = millis() - lastCircuitChange;
  if( circuitIdle > circuitDebounce){
    if(buttonDown != buttonCircuit){
      buttonDown = buttonCircuit;
      buttonHoldPeriod = millis() - lastButtonChange;
      lastButtonChange = millis();
    }
  }

  if( buttonDown && (buttonDown != lastButtonDown)){ //button pushed
    Serial.println("BUTTON PUSHED");  
  }

  if( (!buttonDown) && (buttonDown != lastButtonDown)){ //button released
    if(buttonHoldPeriod > 1000){//long period switches mode 
      mode = CONFIGURATION_MODE;
      number = -1;
    }
    else{ //short period triggers increment
      countUp();      
    }
  }  
  lastButtonDown = buttonDown;

  if(circuitIdle > 1000){ //use idle time to switch into operational mode if a number was incremented
    if(number >= 0){
      mode = OPERATION_MODE;
    }
  }
  
  if(mode == OPERATION_MODE){
    //trigger a countdown if appropriate
    //used to fake events before accelerometer attached
    //if(popEvent()){
    //  countDown();
    //}
    if(number < 2){
      soundAlarm();
    }
    else{
      silenceAlarm();
    }
  }
  if(mode == CONFIGURATION_MODE){
    silenceAlarm();
  }
  
  //show the countdown
  displayDigit(number);

  serviceAlarm();
    
}

float sgn(float v){
  return v < 0? -1.0: 1.0;
}

int wrap(int value, int range){
  value = value % range;
  if(value < 0){
    value += range;
  }
  return value;
}

/** Displays the specified digit. */
void displayDigit(int digit){
  
  byte segments;
  
  if(digit >= 0 && digit < (2 * base)){
    //write the tens into the decimal point
    digitalWrite(5, (digit >= base?HIGH:LOW));
    //choose segments for the unit digit
    segments = digitSequence[digit % base];
  }
  else if(digit == -1){
    //turn off all segments
    segments = 0x0;
  }
  else{
    Serial.print("Digit was ");
    Serial.print(digit);
    Serial.println(" and cannot be written");
  }

  //write the digit
  for(int bitidx=0;bitidx<numDisplayPins -1;bitidx++){
    digitalWrite(displayPins[bitidx], (segments & (0x01 << bitidx)?HIGH:LOW));
  }

}

boolean pushSample(struct history *history, int sample){
  //if sample due
  if( millis() - history->lasttime > history->betweentime){
    for(int i = 0; i < history->numaverages; i++){
      //remove oldest sample
      history->average[i].total -= history->sample[wrap(history->position - history->average[i].range, history->numsamples)];
    }
    history->sample[history->position] = sample;
    for(int i = 0; i < history->numaverages; i++){
      //add newest sample
      history->average[i].total += history->sample[history->position];
      //recalculate average
      history->average[i].mean = ((float)history->average[i].total / (float)history->average[i].range);
    }
    //increment position with wrap
    history->position = wrap(history->position + 1, history->numsamples);
    //update latest time sampled
    history->lasttime = millis();
    return true;
  }
  else{
    return false;
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

void soundMip(){
  if(!alarm){
    tone.play(NOTE_C8,50);
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

char *ftoa(char *a, double f, int precision)
{
  long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
  
  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}

//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.send(address);        // send register address
   Wire.send(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.send(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.receive(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

