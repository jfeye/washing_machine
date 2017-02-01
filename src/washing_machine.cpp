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

#define TIMESCALE 40.0
// ms
#define SIM_STEP_TIME 200
// sec real time
#define PROG_TIME_SPIN 1000
#define PROG_TIME_WOOL 1500
#define PROG_TIME_COTTON 3000
#define PROG_TIME_SILK 1000

#define POWER_HEAT 1500
#define POWER_MOTOR 500

#define AMBIENT_TEMP 18

#define SERIAL_TIMEOUT 1000

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
  //type CHOOSER & ACTION
  menu_option_node_t** options;
};

menu_node_t* homeMenu;
menu_node_t* currentMenu;
uint8_t selectedMenu;

// simulation variables
enum machine_program_t { M_PROGRAM_COTTON = 0x01, M_PROGRAM_WOOL = 0x02, M_PROGRAM_SILK = 0x03, M_PROGRAM_SPIN = 0x04 };
enum machine_step_t { M_STEP_IDLE = 0x01, M_STEP_HEAT = 0x02, M_STEP_WASH = 0x03, M_STEP_SPIN = 0x04 };
enum machine_state_t { STATE_STOPPED = 0x01, STATE_READY = 0x02, STATE_PAUSED = 0x03, STATE_RUNNING = 0x04 };

struct machine_simstate_t {
  menu_node_t* progChooser;
  menu_node_t* tempChooser;
  menu_node_t* speedChooser;
  machine_state_t state;
  machine_step_t step;
  uint8_t heaterOn;
  double currentTemp;
  double currentSpeed;
  uint16_t powerConsumption;
};

machine_simstate_t* simState;
uint32_t lastSim = 0;
uint32_t timestamp = 0;

// communication variables
const uint8_t SERIAL_START = 0xFF;
enum serial_method_t { METHOD_GET = 0x01, METHOD_POST = 0x02, METHOD_RESPONSE = 0x03 };
enum serial_code_t { S_FAIL = 0x00 , S_PROGRAM = 0x01, S_TEMP = 0x02, S_SPEED = 0x03, S_STATE = 0x04, S_STEP = 0x05, S_POWER = 0x06, S_CTEMP = 0x07 };

uint8_t inBuff[5];
uint8_t outBuff[5];


// programm structure :

// setup()
// loop()

// communication
void handleSerial();
void sendError(uint16_t code);
uint8_t receiveBytes(uint8_t bytes);
void sendMessage(uint8_t method, uint8_t code, uint16_t payload);
// simulation
void handleSimulation();
void initSimulationState();
uint32_t secSince(uint32_t timestamp);
// button input
void handleInput();
void executeAction(menu_action_t action);
// output display
void drawFrame();
void paintInfoScreen();
void setLEDs(CRGB color);
// menu
void createMenu();
menu_option_node_t* newMenuOption(const char* label, uint16_t value);
menu_node_t* newMenuEntry(menu_type_t type, const char* label);
uint16_t getChoosenValue(menu_node_t* chooser);
uint16_t setChoosen(menu_node_t* chooser, uint16_t value);


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
    //while (!Serial) ;

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
  while (Serial.available()) {
    if(Serial.read() == SERIAL_START) handleSerial();
    else simState->currentTemp += 10;
  }

  handleInput();
  handleSimulation();
  drawFrame();

  delay(10);
}

// communication --------------------------------------------------------------

void handleSerial(){
  if(!receiveBytes(4)) return;

  uint8_t method = inBuff[0];
  uint8_t code = inBuff[1];
  uint16_t payload = (inBuff[2] << 8) + inBuff[3];

  switch (method) {
    case METHOD_GET:
      switch (code) {
        case S_PROGRAM:
          sendMessage(METHOD_RESPONSE, S_PROGRAM, getChoosenValue(simState->progChooser));
          break;
        case S_TEMP:
          sendMessage(METHOD_RESPONSE, S_TEMP, getChoosenValue(simState->tempChooser));
          break;
        case S_SPEED:
          sendMessage(METHOD_RESPONSE, S_SPEED, getChoosenValue(simState->speedChooser));
          break;
        case S_STATE:
          sendMessage(METHOD_RESPONSE, S_STATE, simState->state);
          break;
        case S_STEP:
          sendMessage(METHOD_RESPONSE, S_STEP, simState->step);
          break;
        case S_POWER:
          sendMessage(METHOD_RESPONSE, S_POWER, simState->powerConsumption);
          break;
        case S_CTEMP:
          sendMessage(METHOD_RESPONSE, S_CTEMP, simState->currentTemp);
          break;
        default:
          // TODO some error
          break;
      }
      break;
    case METHOD_POST:
      switch (code) {
        case S_PROGRAM:
          sendMessage(METHOD_RESPONSE, S_PROGRAM, setChoosen(simState->progChooser, payload));
          break;
        case S_TEMP:
          sendMessage(METHOD_RESPONSE, S_TEMP, setChoosen(simState->tempChooser, payload));
          break;
        case S_SPEED:
          sendMessage(METHOD_RESPONSE, S_SPEED, setChoosen(simState->speedChooser, payload));
          break;
        case S_STATE:
          simState->state = (machine_state_t) payload;
          sendMessage(METHOD_RESPONSE, S_STATE, simState->state);
          break;
        default:
          // TODO some error
          break;
      }
      break;
  }
}

void sendError(uint16_t code){
  /* codes:
  0000 generic
  */
  outBuff[0] = 0xFF;
  outBuff[1] = 0x03;
  outBuff[2] = 0x00;
  outBuff[3] = code >> 8;
  outBuff[4] = code & 0xFF;
  Serial.write(outBuff, 5);
  Serial.flush();
}

uint8_t receiveBytes(uint8_t bytes){
  uint32_t ts = millis();
  while(Serial.available() < bytes && millis()-ts < SERIAL_TIMEOUT) delay(1);
  uint8_t n = Serial.readBytes(inBuff, bytes);
  if(n != bytes){
    sendError(0);
    return 0;
  }
  return 1;
}

void sendMessage(uint8_t method, uint8_t code, uint16_t payload){
  outBuff[0] = SERIAL_START;
  outBuff[1] = method;
  outBuff[2] = code;
  outBuff[3] = payload >> 8;
  outBuff[4] = payload & 0xFF;
  Serial.write(outBuff, 5);
  Serial.flush();
}

// simulation -----------------------------------------------------------------

void handleSimulation(){
  if(millis()-lastSim > SIM_STEP_TIME){
    lastSim = millis();

    simState->powerConsumption = 10;

    machine_state_t state = simState->state;
    machine_step_t step = simState->step;
    machine_program_t prog = (machine_program_t) getChoosenValue(simState->progChooser);
    uint16_t targetTemp = getChoosenValue(simState->tempChooser);
    uint16_t targetSpeed = getChoosenValue(simState->speedChooser);

    // state machine (steps)
    if(state == STATE_RUNNING){

      uint16_t progTime = 0;
      if(prog == M_PROGRAM_COTTON) progTime = PROG_TIME_COTTON;
      else if(prog == M_PROGRAM_WOOL) progTime = PROG_TIME_WOOL;
      else if(prog == M_PROGRAM_SILK) progTime = PROG_TIME_SILK;
      else if(prog == M_PROGRAM_SPIN) progTime = PROG_TIME_SPIN;

      currentMenu = homeMenu;


      if (step == M_STEP_IDLE) {
        timestamp = millis();
        if(prog == M_PROGRAM_SPIN)  simState->step = M_STEP_SPIN;
        else                        simState->step = M_STEP_HEAT;

      } else if (step == M_STEP_HEAT) {
        if (simState->currentTemp >= targetTemp) {
          simState->step = M_STEP_WASH;
          timestamp = millis();
        }

      } else if (step == M_STEP_WASH) {
        if (secSince(timestamp) > progTime){
          simState->step = M_STEP_SPIN;
          timestamp = millis();
        }

      } else if (step == M_STEP_SPIN) {
        if (secSince(timestamp) > PROG_TIME_SPIN){
          simState->step = M_STEP_IDLE;
          simState->state = STATE_STOPPED;
        }

      } else simState->step = M_STEP_IDLE;

    } else if (state == STATE_STOPPED) {
      simState->step = M_STEP_IDLE;
    }
    state = simState->state;
    step = simState->step;

    // simulate temperature
    // ambient cooling/heating
    simState->currentTemp -= (simState->currentTemp - AMBIENT_TEMP) * 0.00007 * TIMESCALE;

    // heater
    if(simState->heaterOn != 0){
      simState->currentTemp += 0.04 * TIMESCALE;
      simState->powerConsumption += POWER_HEAT;
    }

    // two point temp regulation
    if((step == M_STEP_HEAT || step == M_STEP_WASH) && state == STATE_RUNNING){
      if (simState->currentTemp <= 0.95*targetTemp) {
         simState->heaterOn = 1;
      }else if (simState->currentTemp >= 1.05*targetTemp) {
         simState->heaterOn = 0;
      }
    }else{
      simState->heaterOn = 0;
    }

    //  Motor
    if(state == STATE_RUNNING){
        if (step == M_STEP_SPIN){
            simState->currentSpeed = targetSpeed;
        }else if ((step == M_STEP_WASH || M_STEP_HEAT) && secSince(timestamp)%(int)(4*TIMESCALE)>TIMESCALE) {
          simState->currentSpeed = 128;
        }else{
          simState->currentSpeed = 0;
        }
    }else{
        simState->currentSpeed = 0;
    }

    simState->powerConsumption += (simState->currentSpeed*POWER_MOTOR)/255;

    analogWrite(MOTOR_PIN, simState->currentSpeed);

    // leds
    setLEDs(CHSV((int)(180-2*simState->currentTemp)%256,255,255));

    Serial.println(String(simState->powerConsumption/10) + " " + String(simState->currentTemp));
  }
}

void initSimulationState(){
  simState = (machine_simstate_t*) malloc(sizeof(machine_simstate_t));

  simState->progChooser = nullptr;
  simState->tempChooser = nullptr;
  simState->speedChooser = nullptr;
  simState->state = STATE_STOPPED;
  simState->step = M_STEP_IDLE;
  simState->heaterOn = 0;
  simState->currentTemp = 0;
  simState->currentSpeed = 0;
  simState->powerConsumption = 0;
}

uint32_t secSince(uint32_t timestamp){
  return TIMESCALE*(millis()-timestamp) / (1000);
}

// button input ---------------------------------------------------------------

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
                }else if(simState->state == STATE_READY){
                  simState->state = STATE_STOPPED;
                  currentMenu = currentMenu->children[0];
                }
              }else if (t == MENU_LIST){
                currentMenu = currentMenu->children[selectedMenu];
              }else if (t == MENU_CHOOSER){
                currentMenu = currentMenu->parent;
              }else if (t == MENU_ACTION){
                executeAction((menu_action_t)currentMenu->options[selectedMenu]->value);
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

void executeAction(menu_action_t action){
  if(action == ACTION_GO_NOW){
      timestamp = millis();
      simState->state = STATE_RUNNING;
  }
  if(action == ACTION_GO_WHENREADY)
    simState->state = STATE_READY;
}

// output display -------------------------------------------------------------

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
        menu_option_node_t* c = currentMenu->options[i];
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
  String top = "";
  String center = "";
  String bottom = "";

  switch (simState->state) {
    case STATE_RUNNING:
      top += "running";
      break;
    case STATE_PAUSED:
      top += "paused";
      break;
    case STATE_STOPPED:
      top += "stopped";
      break;
    case STATE_READY:
      top += "ready";
      break;
    default:
      top += "undefinded";
  }
  switch (simState->step) {
    case M_STEP_IDLE:
      center += "IDLE";
      break;
    case M_STEP_HEAT:
      center += "HEATING";
      break;
    case M_STEP_WASH:
      center += "WASHING";
      break;
    case M_STEP_SPIN:
      center += "SPINNING";
      break;
    default:
      center += "undef";
  }

  if(simState->state == STATE_RUNNING){
    bottom += String(secSince(timestamp));
  }

  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println(top);
  display.setCursor(0, 8);
  display.setTextSize(2);
  display.println(center);
  display.setCursor(0, 24);
  display.setTextSize(1);
  display.println(bottom);
}

void setLEDs(CRGB color){
    for (uint8_t i = 0; i < 4; i++) {
      leds[i] = color;
    }
    FastLED.show();
}

// menu -----------------------------------------------------------------------

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
    menu_option_node_t* progCotton = newMenuOption("Cotton", M_PROGRAM_COTTON);
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
    menu_option_node_t* goNow   = newMenuOption("start now", ACTION_GO_NOW);
    menu_option_node_t* goReady = newMenuOption("autostart", ACTION_GO_WHENREADY);

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
    startScreen->options = (menu_option_node_t**) malloc(2*sizeof(menu_option_node_t*));
    startScreen->options[0] = goNow;
    startScreen->options[1] = goReady;

    selectedMenu = 0;
    homeMenu = infoScreen;
    currentMenu = infoScreen;

    simState->progChooser = progChooser;
    simState->tempChooser = tempChooser;
    simState->speedChooser = speedChooser;
  }


  menu_option_node_t* newMenuOption(const char* label, uint16_t value){
    menu_option_node_t* node = (menu_option_node_t*) malloc(sizeof(menu_option_node_t));
    node->value = value;
    strcpy(node->label, label);
    return node;
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

      return node;
  }

  uint16_t getChoosenValue(menu_node_t* chooser){
    if(chooser == nullptr || chooser->type != MENU_CHOOSER || chooser->choosen > chooser->numElements) return 0;
    uint8_t i = chooser->choosen;
    return chooser->options[i]->value;
  }

  uint16_t setChoosen(menu_node_t* chooser, uint16_t value){
        int c = chooser->choosen;

        for(int i=0; i<chooser->numElements; i++){
          if (chooser->options[i]->value == value){
            c = i;
            break;
          }
        }
        chooser->choosen = c;

        return chooser->options[c]->value;
  }
