
#include <Wire.h>

#include "FastLED.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define NUM_LEDS 4
#define LED_PIN 9
#define BTN0_PIN 4
#define BTN1_PIN 5
#define BTN2_PIN 6
#define BTN3_PIN 7
#define MOTOR_PIN 10

CRGB leds[NUM_LEDS];
const uint8_t btt_pins[] = {BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN};
uint8_t btt_state[] = {0,0,0,0};
Adafruit_SSD1306 display(0);

void drawFrame();

void setup() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(MOTOR_PIN,OUTPUT);
    pinMode(BTN0_PIN,OUTPUT);
    pinMode(BTN1_PIN,OUTPUT);
    pinMode(BTN2_PIN,OUTPUT);
    pinMode(BTN3_PIN,OUTPUT);

    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    Serial.begin(9600);

}

void loop() {
  if (Serial.available()) {
    // handle serial
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (btt_state[i] != digitalRead(btt_pins[i])) {
      // handle buttons
    }
  }

  drawFrame();

  analogWrite(MOTOR_PIN, 128);

  delay(10);
}

void drawFrame(){

  for (uint8_t i = 0; i < 4; i++) {
    leds[i] = CHSV((millis()/100)%256, 255, 255);
  }
  FastLED.show();

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(1,0);
  display.println("OLED - Display - Test");
  display.setTextSize(2);
  display.setCursor(20,10);
  display.println(String(millis()));
  display.display();
}
