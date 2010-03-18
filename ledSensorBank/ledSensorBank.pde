#define SAMPLE_NUM 50
#define SAMPLE_SEPARATION 213
#define LED_NUM 6

const int buttonPin = 14;

boolean sampling = true;
int pos = 0;

int n[] = { // the negative legs of the leds (with 100 ohm resistor)
  2,4,6,8,10,12
};
int p[] = {
  3,5,7,9,11,13
};

int i;

long charged[LED_NUM]; 
long discharged[LED_NUM];

unsigned int samples[LED_NUM * SAMPLE_NUM];

void setup(){
  
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);

  for(i = 0; i < LED_NUM; i++){
    pinMode(p[i],OUTPUT);
  }

}

void loop(){
  
  long loopstarted = millis();
  
  if(digitalRead(buttonPin)==LOW){
    sampling = true;
    pos = 0;
    //Serial.println("Button pushed");
  }
  
  if(sampling){

    //charge all leds
    for(i = 0; i < LED_NUM; i++){
      pinMode(n[i],OUTPUT);
      digitalWrite(n[i],HIGH);
      digitalWrite(p[i],LOW);      
      // Isolate the negative leg of the led
      pinMode(n[i],INPUT);
      digitalWrite(n[i],LOW);  // turn off internal pull-up resistor
      //record timestamp
      charged[i] = micros();
      discharged[i] = -1;
    } 
 
    boolean awaitingled;
    //detect moment charge has bled away   
    while((millis() - loopstarted) < SAMPLE_SEPARATION){
      awaitingled = false;
      for(i = 0; i < LED_NUM; i++){
        if (discharged[i] == -1){
          if(digitalRead(n[i])==LOW){
            discharged[i] = micros();
          }
          else{
            awaitingled = true;
          }
        }
      }
      if(!awaitingled){
        break;
      }
    }

    long rctime;
    for(i = 0; i < LED_NUM; i++){
      rctime = (discharged[i] - charged[i]) / 4;
      if(rctime > 32767L){
        Serial.println("Long");
      }
      samples[(pos * LED_NUM) + i] = rctime;
    }
                
  }

  for(i = 0; i < LED_NUM; i++){
    pinMode(n[i], OUTPUT);
    digitalWrite(n[i], LOW);
    if(samples[(pos * LED_NUM) + i] < 800){
      digitalWrite(p[i], HIGH);
    }
    else{
      digitalWrite(p[i], LOW);
    }
  }
  
  Serial.println(samples[pos*LED_NUM]); //output value for led 1
  
  pos++;
  if(pos == SAMPLE_NUM){
    if(sampling){
      sampling = false;
    }
    pos = 0;
  }
    
  long looptaken = millis() - loopstarted;
  if(looptaken > SAMPLE_SEPARATION){
    Serial.println("Short");
  }
  else{
    delay(SAMPLE_SEPARATION - looptaken);
  }

  for(i = 0; i < LED_NUM; i++){
    digitalWrite(p[i], LOW);
  }
  
}

int wrap(int value, int range){
  value = value % range;
  if(value < 0){
    value += range;
  }
  return value;
}

