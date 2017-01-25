#include "FastLED.h"

#define NUM_LEDS 4
#define DATA_PIN 2

CRGB leds[NUM_LEDS];
int i=100;
void setup() { 
    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    pinMode(10,OUTPUT);  
    Serial.begin(9600); 
}

void loop() {
  for(int j = 0; j<NUM_LEDS; j++) leds[j] = CHSV((i/2+230)%256,255,255);
  FastLED.show();
  i++;
  if(i>255) i=100;
  analogWrite(10,i);
  delay(50);
  Serial.println(i);

}
