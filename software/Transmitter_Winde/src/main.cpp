/*
Erweiterung der Funktionen: 
 - Rotary-Encoder um in Setup-Menü die max. Zugkraft einstellen zu können
  -> nur möglich wenn im Idle-Mode

 - Taster für Aktivierung Kapp-System
    -> Longpress (>1s) um Fehlbedienung zu vermeiden
*/



/*
 * 
 * transmitter
 * sends current state (pull value)
 * receives acknowlegement with current parameters
 * 
 */
// communication is locked to a specific transmitter for 5 seconds after his last message
// admin ID 0 can allays take over communication
static int myID = 1;    // set to your desired transmitter id!!! [unique number from 1 - 15]
static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings

#include <Arduino.h>
#include "common.h"
#include "Button2.h"
#include <SPI.h>
#include <Wire.h> 
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <SSD1306.h>  
#include "Pangodream_18650_CL.h"
#include <LoRa.h>
#include <rom/rtc.h>

// Setup Lora alt
// #define SCK     5    // GPIO5  -- SX1278's SCK
// #define MISO    19   // GPIO19 -- SX1278's MISnO
// #define MOSI    27   // GPIO27 -- SX1278's MOSI
// #define SS      18   // GPIO18 -- SX1278's CS
// #define RST     14   // GPIO14 -- SX1278's RESET
// #define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

// Setup Lora neu
#define SCK     18    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    23   // GPIO27 -- SX1278's MOSI
#define SS       5   // GPIO18 -- SX1278's CS
#define RST     17   // GPIO14 -- SX1278's RESET
#define DI0     16   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6

// Pins Rotary Encoder
#define ROTARY_SW 14
#define ROTARY_A  32
#define ROTARY_B  33


// Buttons for state machine control
#define BUTTON_UP   26 // up
#define BUTTON_DOWN 27 // down
//#define BUTTON_KAPP ROTARY_SW

Button2 btnUp = Button2(BUTTON_UP);
Button2 btnDown = Button2(BUTTON_DOWN);
// battery measurement
//#define CONV_FACTOR 1.7
//#define READS 20
Pangodream_18650_CL BL(35); // pin 34 old / 35 new v2.1 hw


SSD1306 display(0x3c, 21, 22); // SPI an G21 und G22

int rssi = 0;
float snr = 0;
String packSize = "--";
String packet ;

static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;   // pull value range from -127 to 127
int currentPull = 0;          // current active pull on vesc
bool stateChanged = false;
int currentState = -1;   // -2 = hard brake, -1 = soft brake, 0 = no pull/no brake, 1 = default pull (~3kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
int hardBrake = -20;  //in kg
int softBrake = -7;  //in kg
int defaultPull = 7;  //in kg
int prePullScale = 18;      //in %
int takeOffPullScale = 55;  //in %
int fullPullScale = 80;     //in %
int strongPullScale = 100;  //in %
unsigned long lastStateSwitchMillis = 0;

unsigned char encIn, encInAlt, enc0, enc1, enc2;
char txtOut[40];
unsigned char setupActive;

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

struct LoraTxMessage loraTxMessage;
struct LoraRxMessage loraRxMessage;

unsigned long lastTxLoraMessageMillis = 0;    //last message send
unsigned long lastRxLoraMessageMillis = 0;    //last message received
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;

uint8_t buttons_alt;
uint8_t buttons = 0;
void checkButtons(void){
  //Serial.print("Check Buttons:");

  pinMode(ROTARY_SW, INPUT_PULLUP);
  pinMode(ROTARY_A, INPUT_PULLUP);
  pinMode(ROTARY_B, INPUT_PULLUP);

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  buttons = 0;
  buttons |= digitalRead(BUTTON_UP);
  buttons |= digitalRead(BUTTON_DOWN)*2;
  buttons |= digitalRead(ROTARY_SW)*4;
  buttons |= digitalRead(ROTARY_A)*8;
  buttons |= digitalRead(ROTARY_B)*16;
  if(buttons_alt != buttons){
    buttons_alt = buttons;
    
    sprintf(txtOut, "Buttons: 0x%X", buttons);
    Serial.println(txtOut);
  }
}

void btnPressed(Button2& btn) {
    if (btn == btnUp) {
        Serial.println("btnUP pressed");
        //do not switch up to fast
        if (millis() > lastStateSwitchMillis + 1000 && currentState < 5) {
          currentState = currentState + 1;
          // skip neutral state to prevent line mess up
          if (currentState == 0 ) {
              currentState = currentState + 1;
          }
          lastStateSwitchMillis = millis();
          stateChanged = true;
        }
    } else if (btn == btnDown) {
        Serial.println("btnDown pressed");
        if (currentState > 1) {
          currentState = 1;   //default pull
          lastStateSwitchMillis = millis();
          stateChanged = true;
        } else if (currentState > -2 && currentState < 1){
          currentState = currentState - 1;
          lastStateSwitchMillis = millis();
          stateChanged = true;
        }
    }
}
void btnDownLongClickDetected(Button2& btn) {
    currentState = -1;    //brake
    lastStateSwitchMillis = millis();
    stateChanged = true;
}
void btnDownDoubleClick(Button2& btn) {
  // only get to neutral state from brake
  if (currentState <= -1) {
    currentState = 0;    // neutral
    lastStateSwitchMillis = millis();
    stateChanged = true;
  }
}


void display_write_setup_pull(int pull){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Setup Menu");
  sprintf(txtOut, "%d kg max pull", pull);
  display.drawString(0, 18, txtOut);
  display.display();
}

void setupMenue(void){
  setupActive = 1;
  Serial.printf("Enter Setup Menu \n");
  display_write_setup_pull(myMaxPull);
  while(digitalRead(ROTARY_SW)==0);
  delay(400);
  while(setupActive){
    encIn = digitalRead(ROTARY_A)+digitalRead(ROTARY_B)*2;
    if(encInAlt != encIn){
      encInAlt = encIn;
      enc0 = enc1;
      enc1 = enc2;
      enc2 = encIn;
      if((enc0 == 0)&&(enc1 == 1)&&(enc2 == 3)){
        myMaxPull --;
      }
      if((enc0 == 3)&&(enc1 == 2)&&(enc2 == 0)){
        myMaxPull --;
      }
      if((enc0 == 0)&&(enc1 == 2)&&(enc2 == 3)){
        myMaxPull ++;
      }
      if((enc0 == 3)&&(enc1 == 1)&&(enc2 == 0)){
        myMaxPull ++;
      }
      if(myMaxPull > 127){myMaxPull = 127;}
      if(myMaxPull < 20){myMaxPull = 20;}
      display_write_setup_pull(myMaxPull);
    }
    delay(1);
    if(digitalRead(ROTARY_SW)==0){
      sprintf(txtOut, "Max Pull set to: %d", myMaxPull);
      Serial.println(txtOut);
      setupActive = 0;
    } 
  }
}

void setup() {
  Serial.begin(115200);
  
  // display init
  display.init();
  // Pin-Definition Rotary
  pinMode(ROTARY_SW, INPUT_PULLUP);
  pinMode(ROTARY_A, INPUT_PULLUP);
  pinMode(ROTARY_B, INPUT_PULLUP);

  //pinMode(BUTTON_UP, INPUT_PULLUP);
  //pinMode(BUTTON_DOWN, INPUT_PULLUP);




  if(digitalRead(ROTARY_SW)==0){
    setupMenue();
  }
/*

  while(1){
    checkButtons();
    delay(10);
  }
*/

  //lora init
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(868E6)) {   //EU: 868E6 US: 915E6
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  //LoRa.setSpreadingFactor(10);   // default is 7, 6 - 12
  LoRa.enableCrc();
  //LoRa.setSignalBandwidth(500E3);   //signalBandwidth - signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.

  //display.flipScreenVertically();  

  //Serial.println(" Longpress Time: " + String(btnUp.getLongClickTime()) + "ms");
  //Serial.println(" DoubleClick Time: " + String(btnUp.getDoubleClickTime()) + "ms");

  
  btnUp.setPressedHandler(btnPressed);
  btnDown.setPressedHandler(btnPressed);
  btnDown.setLongClickTime(500);
  btnDown.setLongClickDetectedHandler(btnDownLongClickDetected);
  btnDown.setDoubleClickTime(400);
  btnDown.setDoubleClickHandler(btnDownDoubleClick);
  

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Transmitter \n");
  display.drawString(0, 0, "Starting Transmitter");

  // admin --> scan for existing transmitter for a few seconds --> start up with his current pull state
  if (myID == 0 ) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "Searching 4s for");
      display.drawString(0, 14, "existing transmitter...");
      display.display();
      lastTxLoraMessageMillis = millis();
      int counter_ms = 0;
      while (millis() < lastTxLoraMessageMillis + 4000) {
          // packet from transmitter
          if (LoRa.parsePacket() >= sizeof(loraTxMessage) ) {
            LoRa.readBytes((uint8_t *)&loraTxMessage, sizeof(loraTxMessage));
            if (loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
                //found --> read state and exit
                currentState = loraTxMessage.currentState;
                targetPull = loraTxMessage.pullValue;
                Serial.printf("Found existing transmitter, starting up with state: %d: %d \n", currentState, targetPull);
                //exit search loop
                lastTxLoraMessageMillis = millis() - 4000;
            }
          } 
          delay(10);
          counter_ms += 10;
          if(counter_ms == 1000){
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Searching 3s for");
            display.drawString(0, 14, "existing transmitter...");
            display.display();
          }
          else if(counter_ms == 2000){
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Searching 2s for");
            display.drawString(0, 14, "existing transmitter...");
            display.display();
          }
          else if(counter_ms == 3000){
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Searching 1s for");
            display.drawString(0, 14, "existing transmitter...");
            display.display();
          }
       }
   }

   // reset to my transmitter id
   loraTxMessage.id = myID;
  
}


void loop() {

    loopStep++;
  
    // screen
    if (loopStep % 100 == 0) {
      toogleSlow = !toogleSlow;
    }
    if (loopStep % 100 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);  //10, 16, 24
      if (toogleSlow) {
          display.drawString(0, 0, loraTxMessage.id + String("-B: ") + vescBattery + "%, T: " + vescTempMotor + " C");        
      } else {
          display.drawString(0, 0, loraTxMessage.id + String("-T: ") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");        
      }
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));
      display.drawString(0, 36, String(loraRxMessage.tachometer * 10) + "m| " + String(loraRxMessage.dutyCycleNow) + "%" );
      display.display();
    }
    
    // LoRa data available?
    //==acknowledgement from receiver?
    if (LoRa.parsePacket() == sizeof(loraRxMessage) ) {
        LoRa.readBytes((uint8_t *)&loraRxMessage, sizeof(loraRxMessage));
        currentPull = loraRxMessage.pullValue;
        // vescBatteryPercentage and vescTempMotor are alternated on lora link to reduce packet size
          if (loraRxMessage.vescBatteryOrTempMotor == 1){
            vescBattery = loraRxMessage.vescBatteryOrTempMotorValue;
          } else {
            vescTempMotor = loraRxMessage.vescBatteryOrTempMotorValue;
          }
        previousRxLoraMessageMillis = lastRxLoraMessageMillis;  // remember time of previous paket
        lastRxLoraMessageMillis = millis();
        rssi = LoRa.packetRssi();
        snr = LoRa.packetSnr();
        Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraRxMessage.pullValue, rssi, snr);
        Serial.printf("tacho: %d, dutty: %d: \n", loraRxMessage.tachometer * 10, loraRxMessage.dutyCycleNow);
   }

  // if no lora message for more then 1,5s --> show error on screen + acustic
  if (millis() > lastRxLoraMessageMillis + 1500 ) {
        //TODO acustic information
        //TODO  red disply
        // display.clear(); // wieder einkommentieren, damit das Display flackert wenn die Verbindung verloren geht
        // display.display(); // auch wieder einkommentieren
        // log connection error
       if (millis() > loraErrorMillis + 5000) {
            loraErrorMillis = millis();
            loraErrorCount = loraErrorCount + 1;
       }
  }
  
        // state machine
        // -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
        switch(currentState) {
            case -2:
              targetPull = hardBrake; // -> hard brake
              break;
            case -1:
              targetPull = softBrake; // -> soft brake
              break;
            case 0:
              targetPull = 0; // -> neutral, no pull / no brake
              break;
            case 1: 
              targetPull = defaultPull;   //independent of max pull
              break;
            case 2: 
              targetPull = myMaxPull * prePullScale / 100;
              break;
            case 3:
              targetPull = myMaxPull * takeOffPullScale / 100;
              break;
            case 4:
              targetPull = myMaxPull * fullPullScale / 100;
              break;
            case 5:
              targetPull = myMaxPull * strongPullScale / 100;
              break;
            default: 
              targetPull = softBrake;
              Serial.println("no valid state");
              break;
          }

        delay(10);

        // send Lora message every 400ms  --> three lost packages lead to failsafe on receiver (>1,5s)
        // send immediatly if state has changed
        if (millis() > lastTxLoraMessageMillis + 400 || stateChanged) {
            stateChanged = false;
            loraTxMessage.currentState = currentState;
            loraTxMessage.pullValue = targetPull;
            loraTxMessage.pullValueBackup = targetPull;
            if (LoRa.beginPacket()) {
                LoRa.write((uint8_t*)&loraTxMessage, sizeof(loraTxMessage));
                LoRa.endPacket();
                Serial.printf("sending value %d: \n", targetPull);
                lastTxLoraMessageMillis = millis();  
            } else {
                Serial.println("Lora send busy");
            }
        }
 //      checkButtons();
        btnUp.loop();
        btnDown.loop();
        delay(10);
}
