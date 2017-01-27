#include <Arduino.h>
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

// hardware variables
CRGB leds[NUM_LEDS];
const uint8_t btn_pins[] = {BTN0_PIN, BTN1_PIN, BTN2_PIN, BTN3_PIN};
enum btn_t {BTN_BACK = 0, BTN_LEFT, BTN_RIGHT, BTN_OK};
uint8_t btn_state[] = {0,0,0,0};
Adafruit_SSD1306 display(0);

CRGB ledColor;

// menu variables
enum menu_type_t { MENU_INFOSCREEN, MENU_LIST, MENU_CHOOSER, MENU_ACTION};
enum menu_action_t {ACTION_NONE, ACTION_SET_TEMP, ACTION_SET_SPEED, ACTION_SET_PROG, ACTION_GO_NOW, ACTION_GO_WHENREADY};

struct menu_option_node_t{
  uint16_t value;
  char label[10];
};

struct menu_action_node_t{
  menu_action_t action;
  char label[10];
};

struct menu_node_t {
  char label[10];
  menu_type_t type;
  menu_node_t* parent;
  uint8_t numElements;
  uint8_t choosen;
  menu_action_t action;
  //type LIST & INFO
  menu_node_t** children;
  //type CHOOSER
  menu_option_node_t** options;
  //type ACTION
  menu_action_node_t** actions;
};

menu_node_t* homeMenu;
menu_node_t* currentMenu;
uint8_t selectedMenu;

// simulation variables
enum machine_program_t { M_PROGRAM_COTTON, M_PROGRAM_WOOL, M_PROGRAM_SILK, M_PROGRAM_SPIN };
enum machine_step_t { M_STEP_IDLE, M_STEP_HEAT, M_STEP_WASH, M_STEP_SPIN };
enum machine_state_t { STATE_STOPPED, STATE_READY, STATE_PAUSED, STATE_RUNNING };

struct machine_simstate_t {
  machine_state_t state;
  machine_program_t prog;
  machine_step_t step;
  uint8_t targetTemp;
  uint8_t currentTemp;
  uint8_t targetSpeed;
  uint8_t currentSpeed;
  uint16_t powerConsumption;
};

machine_simstate_t* simState;
uint32_t timestamp;

void handleSimulation();
void handleInput();
void drawFrame();
void paintInfoScreen();
void setLEDs(CRGB color);
void initSimulationState();
void createMenu();
menu_node_t* newMenuEntry(menu_type_t type, const char* label);
void executeAction(menu_action_t action);

void setup() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(MOTOR_PIN,OUTPUT);
    pinMode(BTN0_PIN,INPUT_PULLUP);
    pinMode(BTN1_PIN,INPUT_PULLUP);
    pinMode(BTN2_PIN,INPUT_PULLUP);
    pinMode(BTN3_PIN,INPUT_PULLUP);

    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    Serial.begin(9600);

    ledColor = CRGB::Black;
    for (uint8_t i = 0; i < 4; i++) {
      btn_state[i] = 0;
    }

    initSimulationState();
    createMenu();

    analogWrite(MOTOR_PIN,0);
    setLEDs(CRGB::Orange);
}

void loop() {
  if (Serial.available()) {
    // handle serial
  }

  handleInput();
  handleSimulation();
  drawFrame();

  delay(10);
}

void handleSimulation(){
  if(simState->state == STATE_RUNNING){
    analogWrite(MOTOR_PIN, 255);
    setLEDs(CRGB::Orange);
    if(millis()-timestamp>5000) simState->state = STATE_STOPPED;
  }else{
    setLEDs(CRGB::Black);
    analogWrite(MOTOR_PIN, 0);
  }
}

void executeAction(menu_action_t action){
  if(action == ACTION_GO_NOW){
      timestamp = millis();
      simState->state = STATE_RUNNING;
  }
  if(action == ACTION_GO_WHENREADY)
    Serial.println("GO WHEN READY");
  if(action == ACTION_SET_PROG){
    simState->prog = (machine_program_t) currentMenu->options[selectedMenu]->value;
    Serial.println("SET PROG " + String(simState->prog));
  }
  if(action == ACTION_SET_TEMP){
    simState->targetTemp = currentMenu->options[selectedMenu]->value;
    Serial.println("SET TEMP " + String(simState->targetTemp));
  }
  if(action == ACTION_SET_SPEED){
    simState->targetSpeed = currentMenu->options[selectedMenu]->value;
    Serial.println("SET SPEED " + String(simState->targetSpeed));
  }
}

void handleInput(){
  for(int i = 0; i<4 ; i++){
    int btn = digitalRead(btn_pins[i]);
    if(btn != btn_state[i]){
      btn_state[i] = btn;
      if(!btn){ // now pressed
          menu_type_t t = currentMenu->type;
          switch (i) {
            case BTN_BACK:
              if(currentMenu->parent != nullptr){
                currentMenu = currentMenu->parent;
                selectedMenu = currentMenu->choosen;
              }
              break;
            case BTN_LEFT:
              if (t == MENU_LIST || t == MENU_CHOOSER ||t == MENU_ACTION)
                selectedMenu = (selectedMenu+currentMenu->numElements-1) % currentMenu->numElements;
              break;
            case BTN_RIGHT:
              if (t == MENU_LIST || t == MENU_CHOOSER ||t == MENU_ACTION)
                selectedMenu = (selectedMenu+1) % currentMenu->numElements;
              break;
            case BTN_OK:
              currentMenu->choosen = selectedMenu;
              if (t == MENU_INFOSCREEN){
                if (simState->state == STATE_STOPPED) {
                  currentMenu = currentMenu->children[0];
                }
              }else if (t == MENU_LIST){
                currentMenu = currentMenu->children[selectedMenu];
              }else if (t == MENU_CHOOSER){
                executeAction(currentMenu->action);
                currentMenu = currentMenu->parent;
              }else if (t == MENU_ACTION){
                executeAction(currentMenu->actions[selectedMenu]->action);
                currentMenu->choosen = 0;
                currentMenu = homeMenu;
              }
              selectedMenu = currentMenu->choosen;
              break;
          }
      }
    }
  }
}

void setLEDs(CRGB color){
    for (uint8_t i = 0; i < 4; i++) {
      leds[i] = color;
    }
    FastLED.show();
}

void drawFrame(){
  display.clearDisplay();
  display.setTextColor(WHITE);

  String line = "";
  switch (currentMenu->type) {
    case MENU_INFOSCREEN:
        paintInfoScreen();
      break;
    case MENU_LIST:
      display.setTextSize(1);
      for(int i=0; i<currentMenu->numElements; i++){
        menu_node_t* child = currentMenu->children[i];
        line = "";
        if(i==selectedMenu) line += ">";
        else line += " ";
        line += String(child->label);
        if(child->type == MENU_CHOOSER){
          line += " : ";
          line += String(child->options[child->choosen]->label);
        }
        display.setCursor(1,i*8);
        display.println(line);
      }
      break;
    case MENU_CHOOSER:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(String(currentMenu->label));
      display.setTextSize(2);
      display.setCursor(0, 12);
      line = "<";
      line += String(currentMenu->options[selectedMenu]->label);
      line += ">";
      display.println(line);
      break;
    case MENU_ACTION:
      display.setTextSize(1);
      for(int i=0; i<currentMenu->numElements; i++){
        menu_action_node_t* c = currentMenu->actions[i];
        line = "";
        if(i==selectedMenu) line += ">";
        else line += " ";
        line += String(c->label);
        display.setCursor(1,i*8);
        display.println(line);
      }

  }
  display.display();
}

void paintInfoScreen(){
  if(simState->state == STATE_STOPPED){
    display.setTextSize(1);
    display.setCursor(1,0);
    display.println("Symcon Demo");
  }else{
    display.setTextSize(1);
    display.setCursor(1,0);
    display.println("Running");
    display.setTextSize(2);
    display.setCursor(1,8);
    display.println(String(5000-(millis()-timestamp)));
  }
}

void initSimulationState(){
  simState = (machine_simstate_t*) malloc(sizeof(machine_simstate_t));

  simState->state = STATE_STOPPED;
  simState->prog = M_PROGRAM_COTTON;
  simState->step = M_STEP_IDLE;
  simState->targetTemp = 90;
  simState->currentTemp = 0;
  simState->targetSpeed = 255;
  simState->currentSpeed = 0;
  simState->powerConsumption = 0;
}

void createMenu(){

    /*
    infoScreen
      configScreen
        progChooser
        tempChooser
        speedChooser
        startScreen
          goNow
          goWhenReady
    */

    menu_node_t* infoScreen   = newMenuEntry(MENU_INFOSCREEN,"info");
    menu_node_t* configScreen = newMenuEntry(MENU_LIST,"menu");
    menu_node_t* progChooser  = newMenuEntry(MENU_CHOOSER,"Program");
    menu_node_t* tempChooser  = newMenuEntry(MENU_CHOOSER,"Temp   ");
    menu_node_t* speedChooser = newMenuEntry(MENU_CHOOSER,"Speed  ");
    menu_node_t* startScreen  = newMenuEntry(MENU_ACTION,"Start");

    // options
    menu_option_node_t* progCotton = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    progCotton->value = M_PROGRAM_COTTON;
    strcpy(progCotton->label, "Cotton");
    menu_option_node_t* progWool = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    progWool->value = M_PROGRAM_WOOL;
    strcpy(progWool->label, "Wool");
    menu_option_node_t* progSilk = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    progSilk->value = M_PROGRAM_SILK;
    strcpy(progSilk->label, "Silk");
    menu_option_node_t* progSpin = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    progSpin->value = M_PROGRAM_SPIN;
    strcpy(progSpin->label, "Spin");

    menu_option_node_t* temp90 = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    temp90->value = 90;
    strcpy(temp90->label, "90");
    menu_option_node_t* temp60 = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    temp60->value = 60;
    strcpy(temp60->label, "60");
    menu_option_node_t* temp40 = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    temp40->value = 40;
    strcpy(temp40->label, "40");

    menu_option_node_t* speed1400 = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    speed1400->value = 255;
    strcpy(speed1400->label, "1400");
    menu_option_node_t* speed900 = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    speed900->value = 160;
    strcpy(speed900->label, "900");
    menu_option_node_t* speed0 = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    speed0->value = 0;
    strcpy(speed0->label, "off");

    //actions
    menu_action_node_t* goNow = (menu_action_node_t*) malloc(sizeof(menu_action_node_t));
    goNow->action = ACTION_GO_NOW;
    strcpy(goNow->label, "start now");
    menu_action_node_t* goReady = (menu_action_node_t*) malloc(sizeof(menu_action_node_t));
    goReady->action = ACTION_GO_WHENREADY;
    strcpy(goReady->label, "autostart");

    // structure
    infoScreen->parent = nullptr;
    infoScreen->numElements = 1;
    infoScreen->children = (menu_node_t**) malloc(sizeof(menu_node_t*));
    infoScreen->children[0] = configScreen;

    configScreen->parent = infoScreen;
    configScreen->numElements = 4;
    configScreen->children = (menu_node_t**) malloc(4 * sizeof(menu_node_t*));
    configScreen->children[0] = progChooser;
    configScreen->children[1] = tempChooser;
    configScreen->children[2] = speedChooser;
    configScreen->children[3] = startScreen;

    progChooser->parent = configScreen;
    progChooser->numElements = 4;
    progChooser->action = ACTION_SET_PROG;
    progChooser->options = (menu_option_node_t**) malloc(4*sizeof(menu_option_node_t*));
    progChooser->options[0] = progCotton;
    progChooser->options[1] = progWool;
    progChooser->options[2] = progSilk;
    progChooser->options[3] = progSpin;

    tempChooser->parent = configScreen;
    tempChooser->numElements = 3;
    tempChooser->action = ACTION_SET_TEMP;
    tempChooser->options = (menu_option_node_t**) malloc(3*sizeof(menu_option_node_t*));
    tempChooser->options[0] = temp90;
    tempChooser->options[1] = temp60;
    tempChooser->options[2] = temp40;

    speedChooser->parent = configScreen;
    speedChooser->numElements = 3;
    speedChooser->action = ACTION_SET_SPEED;
    speedChooser->options = (menu_option_node_t**) malloc(3*sizeof(menu_option_node_t*));
    speedChooser->options[0] = speed1400;
    speedChooser->options[1] = speed900;
    speedChooser->options[2] = speed0;

    startScreen->parent = configScreen;
    startScreen->numElements = 2;
    startScreen->actions = (menu_action_node_t**) malloc(2*sizeof(menu_action_node_t*));
    startScreen->actions[0] = goNow;
    startScreen->actions[1] = goReady;

    selectedMenu = 0;
    homeMenu = infoScreen;
    currentMenu = infoScreen;
  }

  menu_node_t* newMenuEntry(menu_type_t type, const char* label){
      menu_node_t* node = (menu_node_t*) malloc(sizeof(menu_node_t));

      strcpy(node->label,label);
      node->type = type;
      node->parent = nullptr;
      node->numElements = 0;
      node->choosen = 0;
      node->action = ACTION_NONE;
      node->children = nullptr;
      node->options = nullptr;
      node->actions = nullptr;

      return node;
  }
