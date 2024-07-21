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

bool lora_rx_flag = 0;

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


#define BUILD_TYPE_MH_ET_ESP32 1 
#define BUILD_TYPE_HELTEC_RL   3

#ifndef BUILD_TYPE
#error "Must define BUILD_TYPE..."
#endif

#define EEPROM_SIZE 20
#define EEPROM_DEVICE_ID 0
#define EEPROM_MAX_PULL  1
#define EEPROM_DEF_PULL  2 // Default-Pull einstellbar machen

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

LoraTxMessage loraTxMessage;
LoraRxMessage loraRxMessage;

unsigned long lastTxLoraMessageMillis = 0;    //last message send
unsigned long lastRxLoraMessageMillis = 0;    //last message received
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;

#define LORA_BAND  868E6

#if BUILD_TYPE == BUILD_TYPE_MH_ET_ESP32
  #warning "Now compiling for MH-ET-ESP32"
  #include <LoRa.h>
  // Setup Lora neu
  #define LORA_SCK     18    // GPIO5  -- SX1278's SCK
  #define LORA_MISO    19   // GPIO19 -- SX1278's MISnO
  #define LORA_MOSI    23   // GPIO27 -- SX1278's MOSI
  #define LORA_SS       5   // GPIO18 -- SX1278's CS
  #define LORA_RST     17   // GPIO14 -- SX1278's RESET
  #define LORA_DI0     16   // GPIO26 -- SX1278's IRQ(Interrupt Request)

  void loar_init(void){
    Serial.println("Start SPI");
    SPI.begin(SCK,MISO,MOSI,SS);
    LoRa.setPins(LORA_SS,LORA_RST,LORA_DI0);
    if (!LoRa.begin(LORA_BAND)) {   //EU: 868E6 US: 915E6
      Serial.println("Starting LoRa failed!");
      while (1);
    }
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
    //LoRa.setSpreadingFactor(10);   // default is 7, 6 - 12
    LoRa.enableCrc();
    //LoRa.setSignalBandwidth(500E3);   //signalBandwidth - signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
  }

  bool lora_send_packet(void){
    if (LoRa.beginPacket()) { 
      // returns 1 wenn sender nicht belegt ist
      // mit beginPacket(1) wird der impilcitHandler ausgewählt, hier also explicit
      // REG_FIFO_ADDR_PTR und REG_PAYLOAD_LENGTH werden auf 0 gesetzt
      Serial.print("Send Lora pack");
      LoRa.write((uint8_t*)&loraTxMessage, sizeof(loraTxMessage.byte));
      //returns size of packet
      // Payload length ermitteln

      LoRa.endPacket();
      // wenn 1 übergeben wird gewartet 
      // auf TX-;Mode umschalten
      // warten bis TX beendet 

      return 1;
    } 
    else{
      return 0;
    }
  }
  bool lora_read_packet(void){
    if (LoRa.parsePacket() >= sizeof(loraRxMessage.byte) ) {
      // int LoRaClass::parsePacket(int size) size > 0 ? implicit Header : explicitHeader
      // adjust for explHeader
      // set FIFO addr to current RX addr
      // put module to standby
      // put module in single rx and long range mode
      LoRa.readBytes((uint8_t *)&loraRxMessage, sizeof(loraRxMessage.byte));
      // liest die daten aus dem FIFO
      rssi = LoRa.packetRssi();
      snr = LoRa.packetSnr();
      sprintf(txtOut, "RecLoraPacket: 0x%X 0x%X 0x%X 0x%X", loraRxMessage.byte[0], loraRxMessage.byte[1], loraRxMessage.byte[2], loraRxMessage.byte[3]);
      Serial.println(txtOut);
      return 1;

    }
    else{
      return 0;
    }
  }
  // Pins Rotary Encoder
  #define ROTARY_SW 14
  #define ROTARY_A  32
  #define ROTARY_B  33

  #define OLED_SDA  21
  #define OLED_SCL  22

  #define BAT_AN_IN 35
  #define BAT_EN_AN -1
  Battery18650Stats BL(BAT_AN_IN, 1.7); // pin 34 old / 35 new v2.1 hw

  // Buttons for state machine control
  #define BUTTON_UP   26 // up
  #define BUTTON_DOWN 27 // down


#elif BUILD_TYPE == BUILD_TYPE_HELTEC_RL
  #warning "Now compiling for Heltec with RadioLib"
  #include <RadioLib.h>

  #define LED_ONBOARD 35
  // Setup Lora neu
  #define LORA_SCK      9    // GPIO5  -- SX1278's SCK
  #define LORA_MISO    11   // GPIO19 -- SX1278's MISnO
  #define LORA_MOSI    10   // GPIO27 -- SX1278's MOSI
  #define LORA_SS       8   // GPIO18 -- SX1278's CS
  #define LORA_RST     12   // GPIO14 -- SX1278's RESET
  #define LORA_BUSY    13  // only on Heltec Module connected
  #define LORA_DI0     14   // GPIO26 -- SX1278's IRQ(Interrupt Request)

  SX1262 Lora = new Module(LORA_SS, LORA_DI0, LORA_RST, LORA_BUSY);
  int radioTransmissionState = RADIOLIB_ERR_NONE;
  void loar_init(void){
    Serial.println("Start SPI");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    int state = Lora.begin( 866.5, 125.0, 9, 7, 0x12, 20, 8, 1.6, false);
    /*
    
int16_t SX1262::begin(
      float freq,               -> 868 MHz
      float bw,                 -> 125 kHz
      uint8_t sf,               -> 7
      uint8_t cr,               -> 7
      uint8_t syncWord,         -> 0x12 
      int8_t power,             -> 20
      uint16_t preambleLength,  -> 8 
      float tcxoVoltage,        -> 1.6
      bool useRegulatorLDO      -> false
      ) 
    */
    
    if (state != RADIOLIB_ERR_NONE){
      Serial.println("Something wrong, can't begin LoRa radio");
      return;
    }
    else{
      Serial.println("Radio init success ");
    }
  }

  bool lora_send_packet(void){ // on SX126x 
    radioTransmissionState = Lora.transmit(loraTxMessage.byte, sizeof(loraTxMessage.byte));
    if(radioTransmissionState == RADIOLIB_ERR_NONE){
      return 1;
    }
    else{
      return 0;
    }
  }

  bool lora_read_packet(void){
    int recState = Lora.receive(loraRxMessage.byte, 5);
    if(recState == RADIOLIB_ERR_NONE){
      digitalWrite(LED_ONBOARD, 1);
      delay(2);
      rssi = Lora.getRSSI();
      snr = Lora.getSNR();
      digitalWrite(LED_ONBOARD, 0);
      return 1;
    }
    else{
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

  #define BAT_AN_IN  1
  #define BAT_EN_AN 37 // Enable ADC divider for meassure BAT voltage
  Battery18650Stats BL(BAT_AN_IN, 4.36); //Pin1 on heltec lora v3

  #define LED_OUT     35 // 

#endif

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


void set_lora_rx_flag(void){
  lora_rx_flag = 1;
}

void setup() {
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, 1);
  #ifdef OLED_RST
    delay(10);
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, 1);
    delay(40);
    digitalWrite(OLED_RST, 0);
    delay(40);
    digitalWrite(OLED_RST, 1);
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
  if(digitalRead(BAT_EN_AN)){
    Serial.println("Bat EN High");
  }
  else{
    Serial.println("Bat EN Low");
  }
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
  display.setFont(ArialMT_Plain_24);  //10, 16, 24
  display.drawString(0, 18, "Dev ID: " + String(myID));
  display.setFont(ArialMT_Plain_16);  //10, 16, 24
  display.drawString(0, 44, "pull: " + String(defaultPull)+" - "+ String(myMaxPull));
  display.display();
  delay(3000);

  //lora init
  loar_init();
  
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
          if (lora_read_packet()) {
            loraTxMessage.byte[0]= loraRxMessage.byte[0];
            loraTxMessage.byte[1]= loraRxMessage.byte[1];
            loraTxMessage.byte[2]= loraRxMessage.byte[2];
            if (loraTxMessage.startframe == 0xCB) {
                //found --> read state and exit
                currentState = loraTxMessage.currentState;
                targetPull = loraTxMessage.pullValue;
                Serial.printf("Found existing transmitter, starting up with state: %d: %d \n", currentState, targetPull);
                //exit search loop
                lastTxLoraMessageMillis = millis() - 4000;
            }
          } 
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
   loraTxMessage.id = myID;
   Serial.print("Lora TX ID: --> ");
   Serial.println(myID);

   Lora.setPacketReceivedAction(set_lora_rx_flag);
}

unsigned int now;
uint8_t dutyCycle;
uint8_t windDirection;
void loop() {

    now = millis();
    if(now > nextMainTaskMillis){
      nextMainTaskMillis = now+100;
      loopStep++;
      rcWinch.timeout = false;
      if (millis() > lastRxLoraMessageMillis + 1500 ) {
        rcWinch.timeout = true;
            //TODO acustic information
            //TODO  red disply
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
            display.drawString(0, 0, loraTxMessage.id + String("-B: ") + vescBattery + "%, T: " + vescTempMotor + " C");        
        } 
        else {
            display.drawString(0, 0, loraTxMessage.id + String("-T: ") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
        }
        display.setFont(ArialMT_Plain_24);  //10, 16, 24
        display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));

        display.drawString(0, 36, String(loraRxMessage.tachometer * 10) + "m| " + String(dutyCycle) + "%" );
        display.display();

        rcWinch.state = currentState;
        rcWinch.kgSoll = targetPull;
        rcWinch.kgIst = currentPull;
        rcWinch.lineLength = loraRxMessage.tachometer*10;
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
          if (millis() > lastTxLoraMessageMillis + 400 || stateChanged) {
            
              stateChanged = false;
              loraTxMessage.startframe = 0xCB;
              if(buttonFlags == 0x03){
                loraTxMessage.currentState = -7;
                Serial.println("Cut the line !!!");
                rcWinch.cut = true;
              }
              else{
                loraTxMessage.currentState = currentState;
                rcWinch.cut = false;
              }
              loraTxMessage.pullValue = targetPull;
              loraTxMessage.pullValueBackup = targetPull;
              if(lora_rx_flag){
                lora_rx_flag = 0;
                Lora.readData(loraRxMessage.byte, sizeof(loraRxMessage.byte));
                rssi = Lora.getRSSI();
                snr = Lora.getSNR();
                if(loraRxMessage.startframe == 0xBC){
                  digitalWrite(LED_ONBOARD, 1);
                  currentPull = loraRxMessage.pullValue;
                  // vescBatteryPercentage and vescTempMotor are alternated on lora link to reduce packet size
                  if(loraRxMessage.dutyCycleOrWindDirection == 1){
                    windDirection = loraRxMessage.dutyCycleOrWindDirektionValue;
                  }
                  else{
                    dutyCycle = loraRxMessage.dutyCycleOrWindDirektionValue;
                  }

                  if (loraRxMessage.vescBatteryOrTempMotor == 1){
                    vescBattery = loraRxMessage.vescBatteryOrTempMotorValue;
                  } else {
                    vescTempMotor = loraRxMessage.vescBatteryOrTempMotorValue;
                  }
                  previousRxLoraMessageMillis = lastRxLoraMessageMillis;  // remember time of previous paket
                  lastRxLoraMessageMillis = millis();

                  // Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraRxMessage.pullValue, rssi, snr);
                //  Serial.printf("tacho: %d, dutty: %d: \n", loraRxMessage.tachometer * 10, loraRxMessage.dutyCycleNow);
                }
                digitalWrite(LED_ONBOARD, 0);
              }
              if (lora_send_packet()) {
                Lora.startReceive();
                // Serial.printf("sending value %d: \n", targetPull);
                lastTxLoraMessageMillis = millis();  
              } 
              else {
                Serial.println("Lora send busy");
              }
            
          }

    }
  //      checkButtons();
  btnUp.loop();
  btnDown.loop();
}
