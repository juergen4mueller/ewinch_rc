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
// ID and max pull can be adjustet in setup, push rotary when switching on to enter setup
static int myID = 1;    // set to your desired transmitter id!!! [unique number from 1 - 15] 
static int myMaxPull = 85;  // 0 - 127 [kg], must be scaled with VESC ppm settings
// with Setup Encoder you can set myId and myMaxPull by rotary encoder

#include <Arduino.h>
#include <RadioLib.h>
#include "common.h"
#include "Button2.h"
#include <SPI.h>
#include <Wire.h> 
#include <SSD1306.h>  
#include "Battery18650Stats.h"
#include <rom/rtc.h>
#include <EEPROM.h>
#include <WiFi.h>

#define USE_ESPNOW
#ifdef USE_ESPNOW
#include <esp_now.h>

uint8_t espnowTarget[]={0x7C, 0xDF, 0xA1, 0xED, 0x63, 0x18};
typedef struct struct_message {
  int state;
  int kgSoll;
  int kgIst;
  int lineLength;
  int windDirection;
  bool cut;
  bool timeout;
} t_RC_Winch;

t_RC_Winch rcWinch;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
#endif


#define EEPROM_SIZE 20
#define EEPROM_DEVICE_ID 0
#define EEPROM_MAX_PULL  1
#define EEPROM_DEF_PULL  2 // Default-Pull einstellbar machen

int rssi = 0;
float snr = 0;
String packSize = "--";
String packet;

static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;   // pull value range from -127 to 127
int currentPull = 0;          // current active pull on vesc
bool stateChanged = false;
int currentState = -1;   // -2 = hard brake, -1 = soft brake, 0 = no pull/no brake, 1 = default pull (~3kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
int hardBrake = -20;  //-20 kg
int softBrake = -7;  //-7 kg
int defaultPull = 7;  //7 kg
int prePullScale = 18;      //18 %
int takeOffPullScale = 55;  //55 %
int fullPullScale = 80;     //80 %
int strongPullScale = 100;  //100 %
unsigned long lastStateSwitchMillis = 0;
unsigned long nextMainTaskMillis = 0;

char txtOut[40];
unsigned char setupActive;

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

LoraTxMessage loraTxMsg;
LoraRxMessage loraRxMsg;

unsigned long lastTxLoraMessageMillis = 0;    //last message send
unsigned long lastRxLoraMessageMillis = 0;    //last message received
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;


  #define LED_ONBOARD 35
  // Setup Lora neu
  #define LORA_SS       8   // GPIO18 -- SX1262's CS
  #define LORA_RST     12   // GPIO14 -- SX1262's RESET
  #define LORA_BUSY    13  // only on Heltec Module connected
  #define LORA_DI1     14   // GPIO26 -- SX1262's IRQ(Interrupt Request)

  SX1262 radio = new Module(LORA_SS, LORA_DI1, LORA_RST, LORA_BUSY);

bool loraRxFlag;
void radioInterrupt(void){
  int irReason = radio.getIrqStatus();
//  Serial.printf("Lora INT: %d\r\n", irReason);
  if(irReason == RADIOLIB_SX126X_IRQ_RX_DONE){
    Serial.println("Set Flag");
    loraRxFlag = true;
  }
}

void lora_init(void){
  int state;
  Serial.println("SX1262 Sender startet...");
  state = radio.begin(866.5, 125.0, 8, 5, 0x12, 14);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Init fehlgeschlagen: ");
    Serial.println(state);
    while (true);
  }

  radio.setCRC(true);
  radio.setDio1Action(radioInterrupt);
  radio.startReceive();
  Serial.println("Radio bereit.");
}

bool lora_send_packet(void){ // on SX126x 

  int state = radio.transmit(loraTxMsg.byte, sizeof(loraTxMsg.byte));

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("  ! Senden fehlgeschlagen: ");
    Serial.println(state);
    delay(500);
    radio.startReceive();
    return 0;
  }
  else{
    Serial.println(" Pack sent ");
    radio.startReceive();
    return 1;
  }
}

bool lora_read_packet(void){
  int state = radio.receive(loraRxMsg.byte, sizeof(loraRxMsg.byte));
  if (state == RADIOLIB_ERR_NONE && radio.getPacketLength() == sizeof(loraRxMsg.byte)) {
    digitalWrite(LED_ONBOARD, 1);
    delay(2);
    rssi = radio.getRSSI();
    snr = radio.getSNR();
    digitalWrite(LED_ONBOARD, 0);
    radio.startReceive();
    return 1;
  }
  else{
    Serial.printf("Lora rec state: %d\r\n", state);
    radio.startReceive();
    return 0;
  }
}
  // Pins Rotary Encoder
  #define ROTARY_SW  2
  #define ROTARY_A   3
  #define ROTARY_B   4

  // Buttons for state machine control
  #define BUTTON_UP    5 // up
  #define BUTTON_DOWN  6 // down

  // Pins OLED oisplay
  #define OLED_SDA  17
  #define OLED_SCL  18
  #define OLED_RST  21 // only in Heltec mode used
  #define VEXT_CTL  36

  #define BAT_AN_IN  1
  #define BAT_EN_AN 37 // Enable ADC divider for meassure BAT voltage
  Battery18650Stats BL(BAT_AN_IN, 4.36); //Pin1 on heltec lora v3

  #define LED_OUT     35 // 


  Button2 btnUp = Button2(BUTTON_UP);
  Button2 btnDown = Button2(BUTTON_DOWN);
  // battery measurement
  //#define CONV_FACTOR 1.7
  //#define READS 20

SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL); // SPI an G21 und G22
int encValue = 0;
unsigned char encIn, encInAlt, enc0, enc1, enc2;
void rotaryInterrupt(void){
  encIn = digitalRead(ROTARY_A)+digitalRead(ROTARY_B)*2;
  if(encInAlt != encIn){
    encInAlt = encIn;
    enc0 = enc1;
    enc1 = enc2;
    enc2 = encIn;
    if((enc0 == 0)&&(enc1 == 1)&&(enc2 == 3)){
      encValue -= 1;
    }
    if((enc0 == 3)&&(enc1 == 2)&&(enc2 == 0)){
      encValue -= 1;
    }
    if((enc0 == 0)&&(enc1 == 2)&&(enc2 == 3)){
      encValue += 1;
    }
    if((enc0 == 3)&&(enc1 == 1)&&(enc2 == 0)){
      encValue += 1;
    }
  }
}

void encoderInit(void){
  pinMode(ROTARY_A, INPUT_PULLUP);
  pinMode(ROTARY_B, INPUT_PULLUP);
  attachInterrupt(ROTARY_A, rotaryInterrupt, CHANGE);
  attachInterrupt(ROTARY_B, rotaryInterrupt, CHANGE);
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

uint8_t buttonFlags = 0;
void btnLongClickDetected(Button2& btn) {
    if (btn == btnDown) {
      currentState = -1;    //brake
      lastStateSwitchMillis = millis();
      stateChanged = true;
      buttonFlags |= 0x01;
    }
    else if (btn == btnUp){
      buttonFlags |= 0x02;
    }
}

void btnDownDoubleClick(Button2& btn) {
  // only get to neutral state from brake
  if (currentState <= -1) {
    currentState = 0;    // neutral
    lastStateSwitchMillis = millis();
    stateChanged = true;
  }
}
void btnReleased(Button2& btn){
  if (btn == btnDown) {
      Serial.println("btnDown released");
      buttonFlags &= ~0x01;
    }
    else if (btn == btnUp){
      Serial.println("btnUp released");
      buttonFlags &= ~0x02;
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

void display_write_setup_id(int devId){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Setup Menu");
  sprintf(txtOut, "Device ID: %d", devId);
  display.drawString(0, 18, txtOut);
  display.display();
}

void display_write_setup_defaultPull(int defPull){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Setup Menu");
  sprintf(txtOut, "Def. Pull: %d", defPull);
  display.drawString(0, 18, txtOut);
  display.display();
}


void setupMenue(void){
  myMaxPull = EEPROM.read(EEPROM_MAX_PULL);
  int varSet;
  myID = EEPROM.read(EEPROM_DEVICE_ID);
  uint8_t setupStep = 0;
  int8_t encRotation = 0;
  setupActive = 1;
  Serial.printf("Enter Setup Menu \n");
  display_write_setup_pull(myMaxPull);
  while(digitalRead(ROTARY_SW)==0);
  delay(400);
  while(setupActive){

    if(setupStep == 0){
      varSet = myMaxPull + encValue;
      sprintf(txtOut, "myMaxPull: %d encValue: %d", varSet, encValue);
      Serial.println(txtOut);
      if(varSet > 127){varSet = 127;}
      if(varSet < 20){varSet = 20;}
      display_write_setup_pull(varSet);
    }
    else if(setupStep == 1){
      varSet =  defaultPull + encValue;
      if(varSet > 15){varSet = 15;}
      if(varSet < 7){varSet = 7;}
      display_write_setup_defaultPull(varSet);
    }
    else if(setupStep == 2){
      varSet =  myID + encValue;
      if(varSet > 15){varSet = 15;}
      if(varSet < 0){varSet = 0;}
      display_write_setup_id(varSet);
    }
      
    
    delay(1);
    if(digitalRead(ROTARY_SW)==0){
      if(setupStep == 0){
        Serial.println("Max Pull set to: " + varSet);
        myMaxPull = varSet;
        encValue = 0;
        display_write_setup_defaultPull(defaultPull);
        setupStep = 1;
        delay(800);
      }
      else if(setupStep == 1){
        Serial.println("Def Pull set to: " + varSet);
        defaultPull = varSet;
        encValue = 0;
        display_write_setup_id(myID);
        setupStep = 2;
        delay(800);
      }
      else if(setupStep == 2){
        Serial.println("Device ID set to: " + myID);
        setupActive = 0;
        myID = varSet;
        EEPROM.write(EEPROM_MAX_PULL, myMaxPull);
        EEPROM.write(EEPROM_DEF_PULL, defaultPull);
        EEPROM.write(EEPROM_DEVICE_ID, myID);
        EEPROM.commit();
      }
    } 
  }
}

bool led_shutdownState;
#define POWER_DOWN_DELAY 120000
uint32_t powerDownTime;

void power_manager(bool resetOffTime){
  // Shutdown eingebaut damit beim Laden der Transmitter nicht die ganze Zeit Lora Band belegt wird.
  // Hilft dem Laderegler das Ende des Ladevorgangs zuverlässig zu erkennen
  uint32_t now = millis();
  if(resetOffTime){
    powerDownTime = now + POWER_DOWN_DELAY;
  }
  uint32_t remainingTime = (powerDownTime - now)/1000;
  if(remainingTime < 100){
    if(remainingTime % 2){
      //digitalWrite(LED_ONBOARD, 1);
    }
    else{
      digitalWrite(LED_ONBOARD, 0);
    }
  }
  if(now > powerDownTime){
    display.clear();
    digitalWrite(LED_ONBOARD, 0);
    delay(100);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_deep_sleep_start();
  }
}


void setup() {
  power_manager(true);
  powerDownTime = POWER_DOWN_DELAY;
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, 0);
  #ifdef VEXT_CTL
    pinMode(VEXT_CTL, OUTPUT);
    digitalWrite(VEXT_CTL, 0);
  #endif

  #ifdef OLED_RST
    delay(10);
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, 1);
    delay(40);
    digitalWrite(OLED_RST, 0);
    delay(40);
    digitalWrite(OLED_RST, 1);
    pinMode(LED_ONBOARD, INPUT);
  #endif
  

  delay(10);
  Serial.begin(115200);
  delay(100);
  
#ifdef USE_ESPNOW
  // Init ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  //Register peer
  memcpy(peerInfo.peer_addr, espnowTarget, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
#endif

  if(BAT_EN_AN >= 0){
    pinMode(BAT_EN_AN, INPUT);
  }
  
    digitalWrite(BAT_EN_AN, 1);
    int BattLevel = BL.getBatteryChargeLevel();
    digitalWrite(BAT_EN_AN, 0);
  // display init
  display.init();
  delay(10);
  display.flipScreenVertically();  
  Serial.println("Init Display");
  // Pin-Definition Rotary
  pinMode(ROTARY_SW, INPUT_PULLUP);
  encoderInit();

  Serial.println("Init EEPROM");
  EEPROM.begin(EEPROM_SIZE);

  defaultPull = EEPROM.read(EEPROM_DEF_PULL);
  if(defaultPull < 7) defaultPull = 7;
  if(defaultPull > 15) defaultPull = 15;
  myMaxPull = EEPROM.read(EEPROM_MAX_PULL);
  myID = EEPROM.read(EEPROM_DEVICE_ID);

  if(myID > 0x0F) myID = 0x0F;
  Serial.print("Max Pull: ");
  Serial.println(myMaxPull);


  if(digitalRead(ROTARY_SW)==0){
    setupMenue();
  }

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Starting ...");
  display.setFont(ArialMT_Plain_16);  //10, 16, 24
  display.drawString(64, 0,"B:"+ String(BattLevel)+"%");
  display.setFont(ArialMT_Plain_24);  //10, 16, 24
  display.drawString(0, 18, "Dev ID: " + String(myID));
  display.setFont(ArialMT_Plain_16);  //10, 16, 24
  display.drawString(0, 44, "pull: " + String(defaultPull)+" - "+ String(myMaxPull));
  display.display();

  delay(3000);
  display.setBrightness(255);

  //lora init
  lora_init();
  
  btnUp.setPressedHandler(btnPressed);
  btnUp.setLongClickTime(1000);
  btnUp.setLongClickDetectedHandler(btnLongClickDetected);
  btnUp.setReleasedHandler(btnReleased);
  btnDown.setPressedHandler(btnPressed);
  btnDown.setLongClickTime(500);
  btnDown.setLongClickDetectedHandler(btnLongClickDetected);
  btnDown.setDoubleClickTime(400);
  btnDown.setDoubleClickHandler(btnDownDoubleClick);
  btnDown.setReleasedHandler(btnReleased);

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
          Serial.println("Read Lora packet "+String(millis()));
          // if(loraRxFlag){
          //   loraRxFlag = false;
          //   if (lora_read_packet()) {
          //     loraTxMsg.byte[0]= loraRxMsg.byte[0];
          //     loraTxMsg.byte[1]= loraRxMsg.byte[1];
          //     loraTxMsg.byte[2]= loraRxMsg.byte[2];
          //     if (loraTxMsg.startframe == 0xCB) {
          //         //found --> read state and exit
          //         currentState = loraTxMsg.currentState;
          //         targetPull = loraTxMsg.pullValue;
          //         Serial.printf("Found existing transmitter, starting up with state: %d: %d \n", currentState, targetPull);
          //         //exit search loop
          //         lastTxLoraMessageMillis = millis() - 4000;
          //     }
          //   } 
          // }
          Serial.println("After read Lora packet "+String(millis()));
          if(millis()-lastTxLoraMessageMillis >= 3000){
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Searching 1s for");
            display.drawString(0, 14, "existing transmitter...");
            display.display();
          }
          else if(millis()-lastTxLoraMessageMillis >= 2000){
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Searching 2s for");
            display.drawString(0, 14, "existing transmitter...");
            display.display();
          }
          else if(millis()-lastTxLoraMessageMillis >= 1000){
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.setFont(ArialMT_Plain_16);
            display.drawString(0, 0, "Searching 3s for");
            display.drawString(0, 14, "existing transmitter...");
            display.display();
          }
       }

  digitalWrite(LED_ONBOARD, 0);
   }

   // reset to my transmitter id
   loraTxMsg.id = myID;
   Serial.print("Lora TX ID: --> ");
   Serial.println(myID);
}

unsigned int now;
uint8_t dutyCycle;
uint8_t windDirection;
uint8_t pValue=0;
void simpleLoraTest(void){
  loraTxMsg.id = myID;
  loraTxMsg.currentState = 1;
  Serial.printf("Send Lora ID: 0x%02X\r\n", pValue);
  loraTxMsg.pullValue = pValue;
  loraTxMsg.pullValueBackup = pValue++;
  loraTxMsg.startframe = 0xCB;
  int state = radio.transmit(loraTxMsg.byte, sizeof(loraTxMsg.byte));
  if(state != RADIOLIB_ERR_NONE){
    Serial.printf("%5d Radiolib Error: %d\r\n", millis(), state);
  }
  radio.startReceive();
}

uint32_t nextLoraTx = 0;
uint8_t loraRx[5];
void loop() {
  while(true){
    now = millis();
    if(now >= nextLoraTx){
      nextLoraTx = now + 400;
      simpleLoraTest();
    }
    if(loraRxFlag==true){
      loraRxFlag = false;

      int state = radio.readData(loraRx, 5);
      Serial.printf("Rx State: %d\r\n", state);
      if (state == RADIOLIB_ERR_NONE) {
        Serial.printf("Empf: ");
        for (int i = 0; i < 5; i++) {
          Serial.printf("0x%02X ", loraRx[i]);
        }
        Serial.println();

        Serial.print("RSSI: ");
        Serial.println(radio.getRSSI());
        Serial.print("SNR: ");
        Serial.println(radio.getSNR());
      }
    }
    
  }
    if(now > nextMainTaskMillis){

      nextMainTaskMillis = now+100;
      loopStep++;
      rcWinch.timeout = false;
      if (millis() > lastRxLoraMessageMillis + 1500 ) {
        rcWinch.timeout = true;
            //TODO acustic information
            //TODO  red display
            display.clear(); // wieder einkommentieren, damit das Display flackert wenn die Verbindung verloren geht
            display.display(); // auch wieder einkommentieren
            // log connection error
          if (millis() > loraErrorMillis + 5000) {
                loraErrorMillis = millis();
                loraErrorCount = loraErrorCount + 1;
          }
      }
      // screen
      if (loopStep % 10 == 0) {
        toogleSlow = !toogleSlow;
      }
      if (loopStep % 5 == 0) {
        // if no lora message for more then 1,5s --> show error on screen + acustic
        
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_16);  //10, 16, 24
        if (toogleSlow) {
            display.drawString(0, 0, loraTxMsg.id + String("-B: ") + vescBattery + "%, T: " + vescTempMotor + " C");        
        } 
        else {
          digitalWrite(BAT_EN_AN, 1);
            display.drawString(0, 0, loraTxMsg.id + String("-T: ") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
          digitalWrite(BAT_EN_AN, 0);
        }
        display.setFont(ArialMT_Plain_24);  //10, 16, 24
        display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));

        display.drawString(0, 36, String(loraRxMsg.tachometer * 10) + "m| " + String(dutyCycle) + "%" );
        display.display();

        rcWinch.state = currentState;
        rcWinch.kgSoll = targetPull;
        rcWinch.kgIst = currentPull;
        rcWinch.lineLength = loraRxMsg.tachometer*10;
        rcWinch.windDirection = windDirection * 10;
        #ifdef USE_ESPNOW
        esp_err_t result = esp_now_send(espnowTarget, (uint8_t *) &rcWinch, sizeof(rcWinch));
        #endif
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
              if(targetPull < defaultPull){targetPull = defaultPull;}
              break;
            case 3:
              targetPull = myMaxPull * takeOffPullScale / 100;
              if(targetPull < defaultPull){targetPull = defaultPull;}
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
          // send Lora message every 400ms  --> three lost packages lead to failsafe on receiver (>1,5s)
          // send immediatly if state has changed
          if (millis() > lastTxLoraMessageMillis + 500 ) {
            
              stateChanged = false;
              loraTxMsg.startframe = 0xCB;
              if(buttonFlags == 0x03){
                loraTxMsg.currentState = -7;
                Serial.println("Cut the line !!!");
                rcWinch.cut = true;
              }
              else{
                loraTxMsg.currentState = currentState;
                rcWinch.cut = false;
              }
              loraTxMsg.pullValue = targetPull;
              loraTxMsg.pullValueBackup = targetPull;
              
              lora_send_packet();
              lastTxLoraMessageMillis = millis();  
              
          }
          // if (loraRxFlag) {
          //   loraRxFlag = false;

          //   //  if(lora_read_packet()){
          //   //     // rssi = LoRa.getRSSI();
          //   //     // snr = Lora.getSNR();
          //   //     Serial.println("Got LoRa msg");
          //   //     if(loraRxMsg.startframe == 0xBC){
          //   //       currentPull = loraRxMsg.pullValue;
          //   //       // vescBatteryPercentage and vescTempMotor are alternated on lora link to reduce packet size
          //   //       if(loraRxMsg.dutyCycleOrWindDirection == 1){
          //   //         windDirection = loraRxMsg.dutyCycleOrWindDirektionValue;
          //   //       }
          //   //       else{
          //   //         dutyCycle = loraRxMsg.dutyCycleOrWindDirektionValue;
          //   //       }

          //   //       if (loraRxMsg.vescBatteryOrTempMotor == 1){
          //   //         vescBattery = loraRxMsg.vescBatteryOrTempMotorValue;
          //   //       } else {
          //   //         vescTempMotor = loraRxMsg.vescBatteryOrTempMotorValue;
          //   //       }
          //   //       previousRxLoraMessageMillis = lastRxLoraMessageMillis;  // remember time of previous paket
          //   //       lastRxLoraMessageMillis = millis();
          //   //       //power_manager(true);

          //   //       // Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraRxMsg.pullValue, rssi, snr);
          //   //     //  Serial.printf("tacho: %d, dutty: %d: \n", loraRxMsg.tachometer * 10, loraRxMsg.dutyCycleNow);
          //   //     }
          //   //     digitalWrite(LED_ONBOARD, 0);
          //   //   }
          
          //   }
    }
  //      checkButtons();
  btnUp.loop();
  btnDown.loop();
  //power_manager(false);
}

