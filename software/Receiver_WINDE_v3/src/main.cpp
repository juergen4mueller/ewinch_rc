#include <Arduino.h>
/*
 * receiver
 * receives target pull value from lora link
 * sends acknowlegement on lora with current parameters
 * writes target pull with PWM signal to vesc
 * reads current parameters (tachometer, battery %, motor temp) with UART from vesc, based on (https://github.com/SolidGeek/VescUart/)
 * 
 */


#include "LiPoCheck.h"    //to calculate battery % based on cell Voltage

#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h"
#include <rom/rtc.h>
#include <Button2.h>
#include <VescUart.h>
#include "common.h"

#define LORA_FREQ  866500000
#define LORA_BW  125000 // kHz
// #define LORA_SPREADFACTOR 7  // 36ms trmt
// #define LORA_SPREADFACTOR 8  // 72ms trmt
#define LORA_SPREADFACTOR 9  // 125ms trmt
// #define LORA_SPREADFACTOR 10 // 250ms trmt

#define BUILD_TYPE_MH_ET_ESP32 1 
#define BUILD_TYPE_SX1272_RL   2

#ifndef BUILD_TYPE
#error "Must define BUILD_TYPE..."
#endif

#define EEPROM_SIZE 20
#define EEPROM_DEVICE_ID 0
#define EEPROM_MAX_PULL  1

int rssi = 0;
float snr = 0;

char txtOut[40];

LoraTxMessage loraTxMsg;
LoraRxMessage loraRxMsg;

#if BUILD_TYPE == BUILD_TYPE_MH_ET_ESP32
  #warning "Now compiling for MH-ET-ESP32"
  #include <LoRa.h>
  // Setup Lora neu
  // #define LORA_SCK     18    // GPIO5  -- SX1278's SCK
  // #define LORA_MISO    19   // GPIO19 -- SX1278's MISnO
  // #define LORA_MOSI    23   // GPIO27 -- SX1278's MOSI
  // #define LORA_SS       5   // GPIO18 -- SX1278's CS
  // #define LORA_RST     17   // GPIO14 -- SX1278's RESET
  // #define LORA_DI0     16   // GPIO26 -- SX1278's IRQ(Interrupt Request)
    // Setup Lora neu
    #define LORA_SCK     18    // GPIO5  -- SX1278's SCK
    #define LORA_MISO    19   // GPIO19 -- SX1278's MISnO
    #define LORA_MOSI    23   // GPIO27 -- SX1278's MOSI
    #define LORA_SS       5   // GPIO18 -- SX1278's CS
    #define LORA_RST     0   // GPIO14 -- SX1278's RESET
    #define LORA_DI0     2   // GPIO26 -- SX1278's IRQ(Interrupt Request)



  void lora_init(void){
    Serial.println("Start SPI");
    SPI.begin(SCK,MISO,MOSI,SS);
    LoRa.setPins(LORA_SS,LORA_RST,LORA_DI0);
    if (!LoRa.begin(LORA_FREQ)) {   //EU: 868E6 US: 915E6
      Serial.println("Starting LoRa failed!");
      while (1);
    }
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
    LoRa.enableCrc();
    LoRa.setSignalBandwidth(LORA_BW);   //signalBandwidth - signal bandwidth in Hz, defaults to 125E3. Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
    LoRa.setSpreadingFactor(LORA_SPREADFACTOR);   // default is 7, 6 - 12
    }

  bool lora_send_packet(void){
    if (LoRa.beginPacket()) { 
      // returns 1 wenn sender nicht belegt ist
      // mit beginPacket(1) wird der impilcitHandler ausgewählt, hier also explicit
      // REG_FIFO_ADDR_PTR und REG_PAYLOAD_LENGTH werden auf 0 gesetzt
      //Serial.print("Send Lora pack");
      LoRa.write((uint8_t*)&loraTxMsg, sizeof(loraTxMsg.byte));
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
    if (LoRa.parsePacket() >= sizeof(loraRxMsg.byte) ) {
      // int LoRaClass::parsePacket(int size) size > 0 ? implicit Header : explicitHeader
      // adjust for explHeader
      // set FIFO addr to current RX addr
      // put module to standby
      // put module in single rx and long range mode
      LoRa.readBytes((uint8_t *)&loraRxMsg, sizeof(loraRxMsg.byte));
      // liest die daten aus dem FIFO
      rssi = LoRa.packetRssi();
      snr = LoRa.packetSnr();
   //   sprintf(txtOut, "RecLoraPacket: 0x%X 0x%X 0x%X 0x%X", loraRxMsg.byte[0], loraRxMsg.byte[1], loraRxMsg.byte[2], loraRxMsg.byte[3]);
    //  Serial.println(txtOut);
      return 1;

    }
    else{
      return 0;
    }
  }


  // Define sonstiger Pins
  // LED's zur Statusausgabe, grün leuchtet wenn an, rot blitzt bei LoraRX auf
    #define LED_GN 26 // up
    #define LED_RT 27 // down
    // Pins Rotary Encoder -> Funktionen:
    // - Seil manuell einziehen (Zug mehr / weniger über drehen, bei Push SoftBreak)
    #define ROTARY_SW 14
    #define ROTARY_A  32
    #define ROTARY_B  33

    #define WARN_LIGHT_OUT 25
    #define FAN_OUT   12

    #define OLED_SDA  21
    #define OLED_SCL  22
    #define BAT_V_PIN  35

    #define VESC_RX  17    //connect to TX on Vesc
    #define VESC_TX  16    //connect to RX on Vesc

    #define PWM_PIN_OUT  13 //Define Digital PIN
  
#elif BUILD_TYPE == BUILD_TYPE_SX1272_RL
  #warning "Now compiling for Heltec with RadioLib"
  #include <RadioLib.h>


  // Setup Lora neu
  #define LORA_SCK      18    // GPIO5  -- SX1278's SCK
  #define LORA_MISO    19   // GPIO19 -- SX1278's MISnOv
  #define LORA_MOSI    23   // GPIO27 -- SX1278's MOSI
  #define LORA_SS       5  // GPIO18 -- SX1278's CS
  #define LORA_RST     0   // GPIO14 -- SX1278's RESET
  // #define LORA_BUSY    13  // only on Heltec Module connected
  #define LORA_DI0     2   // GPIO26 -- SX1278's IRQ(Interrupt Request)

  SX1278 LoRa = new Module(LORA_SS, LORA_DI0, LORA_RST, )
  int radioTransmissionState = RADIOLIB_ERR_NONE;
  void lora_init(void){
    Serial.println("Start SPI");
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    int state = LoRa.begin( LORA_FREQ/1000000.0, 125.0, 7, 7, 0x12, 20, 8, 1.6, false);
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
    radioTransmissionState = LoRa.transmit(loraTxMsg.byte, sizeof(loraTxMsg.byte));
    if(radioTransmissionState == RADIOLIB_ERR_NONE){
      return 1;
    }
    else{
      return 0;
    }
  }

  bool lora_read_packet(void){
    int recState = LoRa.receive(loraRxMsg.byte, sizeof(loraTxMsg.byte));
    if(recState == RADIOLIB_ERR_NONE){
      delay(2);
      rssi = LoRa.getRSSI();
      snr = LoRa.getSNR();
      return 1;
    }
    else{
      return 0;
    }
    
  }

  // Define sonstiger Pins
  // LED's zur Statusausgabe, grün leuchtet wenn an, rot blitzt bei LoraRX auf
    #define LED_GN 26 // up
    #define LED_RT 27 // down
    // Pins Rotary Encoder -> Funktionen:
    // - Seil manuell einziehen (Zug mehr / weniger über drehen, bei Push SoftBreak)
    #define ROTARY_SW 14
    #define ROTARY_A  32
    #define ROTARY_B  33

    #define WARN_LIGHT_OUT 25
    #define FAN_OUT   12

    #define OLED_SDA  21
    #define OLED_SCL  22
    #define BAT_V_PIN  35

    #define VESC_RX  17    //connect to TX on Vesc
    #define VESC_TX  16    //connect to RX on Vesc

    #define PWM_PIN_OUT  13 //Define Digital PIN

#endif




//vesc battery number of cells
static int numberOfCells = 16;
static int myMaxPull = 75;  // 0 - 127 [kg], must be scaled with VESC ppm settings



SSD1306 display(0x3c, OLED_SDA, OLED_SCL);
// battery measurement
//#define CONV_FACTOR 1.7
//#define READS 20
Pangodream_18650_CL BL(BAT_V_PIN); // pin 34 old / 35 new v2.1 hw


//Using VescUart librarie to read from Vesc (https://github.com/SolidGeek/VescUart/)
// Uart old
// #define VESC_RX  14    //connect to TX on Vesc
// #define VESC_TX  2    //connect to RX on Vesc
// Uart new

VescUart vescUART;

// PWM signal to vesc
#define PWM_TIME_0      950.0    //PWM time in ms for 0% , PWM below will be ignored!! need XXX.0!!!
#define PWM_TIME_100    2000.0   //PWM time in ms for 100%, PWM above will be ignored!!

static int loopStep = 0;
static uint8_t activeTxId = 0;

int smoothStep = 0;    // used to smooth pull changes
int hardBrake = -20;  //-20 kg
int softBrake = -7;  //-7 kg
int defaultPull = 7;  //7 kg
int prePullScale = 20;      //20 % of myMaxPull
int takeOffPullScale = 50;  //50 % of myMaxPull
int fullPullScale = 80;     //80 % of myMaxPull
int strongPullScale = 100;  //100 % of myMaxPull

int currentId = 0;
int currentState = -1;
// pull value send to VESC --> default soft brake
// defined as int to allow smooth changes without overrun
int currentPull = softBrake;     // active range -127 to 127
int8_t targetPullValue = 0;    // received from lora transmitter or rewinding winch mode
int rotaryPull = 0; // set by rotary and reset to 0 if lora link != -1

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;
unsigned long lastTxLoraMessageMillis = 0;
unsigned long previousTxLoraMessageMillis = 0;
unsigned long lastRxLoraMessageMillis = 0;
unsigned long previousRxLoraMessageMillis = 0;
uint32_t  pwmReadTimeValue = 0;
uint32_t  pwmWriteTimeValue = 0;
unsigned long lastWritePWMMillis = 0;
unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;


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

void pulseOut(int pin, int us)
{
   digitalWrite(pin, HIGH);
   us = max(us - 20, 1);  //biase caused by digital write/read
   delayMicroseconds(us);
   digitalWrite(pin, LOW);
}

void pullByUart(int current){
  if(current > 0){
    vescUART.setCurrent(current);
  }
  else{
    vescUART.setBrakeCurrent(current);
  }
}

// void pullManual(void){
//   manPull = EEPROM.read(EEPROM_MAX_PULL);
//   setupActive = 1;
//   Serial.printf("Enter Setup Menu \n");
//   display_write_setup_pull(myMaxPull);
//   while(digitalRead(ROTARY_SW)==0);
//   delay(400);
//   while(setupActive){
//     encIn = digitalRead(ROTARY_A)+digitalRead(ROTARY_B)*2;
//     if(encInAlt != encIn){
//       encInAlt = encIn;
//       enc0 = enc1;
//       enc1 = enc2;
//       enc2 = encIn;
//       if((enc0 == 0)&&(enc1 == 1)&&(enc2 == 3)){
//         myMaxPull --;
//       }
//       if((enc0 == 3)&&(enc1 == 2)&&(enc2 == 0)){
//         myMaxPull --;
//       }
//       if((enc0 == 0)&&(enc1 == 2)&&(enc2 == 3)){
//         myMaxPull ++;
//       }
//       if((enc0 == 3)&&(enc1 == 1)&&(enc2 == 0)){
//         myMaxPull ++;
//       }
//       if(myMaxPull > 127){myMaxPull = 127;}
//       if(myMaxPull < 20){myMaxPull = 20;}
//       display_write_setup_pull(myMaxPull);
//     }
//     delay(1);
//     if(digitalRead(ROTARY_SW)==0){
//       sprintf(txtOut, "Max Pull set to: %d", myMaxPull);
//       Serial.println(txtOut);
//       setupActive = 0;
//       EEPROM.write(EEPROM_MAX_PULL, myMaxPull);
//       EEPROM.commit();
//     } 
//   }
// }


uint32_t nextSendTime;
void setup() {

  pinMode(LED_GN, OUTPUT);
  digitalWrite(LED_GN, 1);
  pinMode(LED_RT, OUTPUT);
  digitalWrite(LED_RT, 1);
  Serial.begin(115200);
  encoderInit();
  //Setup UART port for Vesc communication
  Serial1.begin(115200, SERIAL_8N1, VESC_RX, VESC_TX);
  vescUART.setSerialPort(&Serial1);
  //vescUART.setDebugPort(&Serial);
  
  // //OLED display // nur über I2C, keine Reset-Leitung vorhanden
  // pinMode(16,OUTPUT);
  // pinMode(2,OUTPUT);
  // digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  // delay(50); 
  // digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high

  //lora init
  lora_init();

  // display init
  display.init();
  display.flipScreenVertically();  

  //PWM Pins
  //pinMode(PWM_PIN_IN, INPUT);
  pinMode(PWM_PIN_OUT, OUTPUT);
  
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Receiver \n");
  display.drawString(0, 0, "Starting Receiver");
  digitalWrite(LED_RT, 0);
  digitalWrite(LED_GN, 0);
  nextSendTime = millis();
}

uint32_t sendCycle =0;
uint32_t now, lastSend, lastRead;
uint16_t counter_send;


void testLora(void){

  display.clear();
  display.display();
  now = millis();
  lastRead = now;

  while(1){

    now = millis();
    
    // Serial.printf("%8d:last read\r\n", now - lastRead, counter_send++);
    
    if(lora_read_packet()){

      for(int i=0;i<5;i++){
        loraTxMsg.byte[i]= loraRxMsg.byte[i];
      }
      loraTxMsg.byte[5]=0xBC;

      Serial.printf("%8d:Read Radio %5d ", now, now - lastSend);
      for(int i = 0;i < sizeof(loraRxMsg.byte); i++){
        Serial.printf("0x%02X ", loraRxMsg.byte[i]);
      }
      Serial.println();
      if(0){ // send answer
        if(lora_send_packet()){
          Serial.printf("%8d:Sent Radio packet nr: %5d ", now, counter_send++);
          for(int i = 0;i < sizeof(loraTxMsg.byte); i++){
            Serial.printf("0x%02X ", loraTxMsg.byte[i]);
          }
          Serial.println();
        }
        lastSend = now;
      }
    }
    // if(now > nextSendTime){
    //   // Serial.println("Not active");
    //   // continue;


    //   nextSendTime += SEND_CYCLE;
    //   loraTxMsg.startframe = 0xBC;
    //   loraTxMsg.tachometer = counter_send;
    //   loraTxMsg.dutyCycleNow = counter_send>>8;
    //   loraTxMsg.pullValue = counter_send%16; // als BZ
    //   loraTxMsg.vescBatteryOrTempMotor = 0;
    //   loraTxMsg.vescBatteryOrTempMotorValue = 0;
    //   Serial.printf("%8d:Sent Radio packet nr: %5d\r\n", millis(), counter_send++);
    //   lora_send_packet();
    // }
  }

}

void loop() {
 // testLora();
 loopStep++;
 // TODO activate rewinding winch mode here
 if (true) {
    // screen
    
    if (loopStep % 10 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      display.drawString(0, 0, currentId + String("-RX: (") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      if (currentState == -7){
          display.drawString(0, 11, "Cutting");  
      }
      else if (currentState > 0){
          display.drawString(0, 11, String("P ") + currentState + ": (" + currentPull + "kg)");  
      } else {
          display.drawString(0, 11, String("B ") + currentState + ": (" + currentPull + "kg)");    
      }
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      //display.drawString(0, 36, String("Error / Uptime{min}: ") + loraErrorCount + " / " + millis()/60000);
      display.drawString(0, 36, String("B: ") + vescBattery + "%, M: " + vescTempMotor + "C" +" R"+encValue); 
      display.drawString(0, 48, String("Last TX / RX: ") + lastTxLoraMessageMillis/100 + " / " + lastRxLoraMessageMillis/100);
      display.display();
    }
    
    if(lora_read_packet()){
          if(loraRxMsg.startframe == 0xCB){          
            Serial.print("Data from Lora: ");
            Serial.printf("ID: %d State: %d PullValue: %d", loraRxMsg.id, loraRxMsg.currentState, loraRxMsg.pullValue);    
            Serial.println();
            digitalWrite(LED_RT, 1);
              if (millis() > lastTxLoraMessageMillis + 5000){
            activeTxId = loraRxMsg.id;
          }
          // The admin id 0 can allways take over
          if (loraRxMsg.id == 0){
            activeTxId = loraRxMsg.id;
          }
          if (loraRxMsg.id == activeTxId && loraRxMsg.pullValue == loraRxMsg.pullValueBackup) {
              targetPullValue = loraRxMsg.pullValue;
              currentId = loraRxMsg.id;
              currentState = loraRxMsg.currentState;
              previousTxLoraMessageMillis = lastTxLoraMessageMillis;  // remember time of previous paket
              lastTxLoraMessageMillis = millis();
              //Serial.printf("Value received: %d, RSSI: %d: , SNR: %d\n", loraRxMsg.pullValue, rssi, snr);
              
              // send ackn after receiving a value
              // delay(10);
              loraTxMsg.startframe = 0xBC;
              loraTxMsg.pullValue = currentPull;
              loraTxMsg.tachometer = abs(vescUART.data.tachometer)/725;     // %100 --> in m, %10 --> to use only one byte for up to 2550m line lenght
              loraTxMsg.dutyCycleNow = abs(vescUART.data.dutyCycleNow * 100);     //in %
              //Serial.printf("Tacho: %d\r\n", vescUART.data.tachometer);
              // alternate vescBatteryPercentage and vescTempMotor value on lora link to reduce packet size
              if (loraTxMsg.vescBatteryOrTempMotor == 0){
                loraTxMsg.vescBatteryOrTempMotor = 1;
                loraTxMsg.vescBatteryOrTempMotorValue = vescBattery;
              } else {
                loraTxMsg.vescBatteryOrTempMotor = 0;
                loraTxMsg.vescBatteryOrTempMotorValue = vescTempMotor;
              }
              digitalWrite(LED_RT, 0);
              
              // Sende Paket nach empfang von Remote  
              digitalWrite(LED_GN, 1);
              // Hier noch die Daten reinpacken
              if (lora_send_packet()) {
                Serial.printf("sending pull value %d: \r\n", targetPullValue);
                lastTxLoraMessageMillis = millis();  
              } 
              else {
                Serial.println("Lora send busy");
              }
              digitalWrite(LED_GN, 0);              
        }
      }
    }
      // if no lora message for more then 1,5s --> show error on screen + acustic
      if (millis() > lastTxLoraMessageMillis + 1500 ) {
            //TODO acustic information
            //TODO  red disply
            display.clear();
            display.display();
            // log connection error
           if (millis() > loraErrorMillis + 5000) {
                loraErrorMillis = millis();
                loraErrorCount = loraErrorCount + 1;
           }
      }
      // Failsafe only when pull was active
      if (currentState >= 1) {
            // no packet for 1,5s --> failsave
            if (millis() > lastTxLoraMessageMillis + 1500 ) {
                 // A) keep default pull if connection issue during pull for up to 10 seconds
                 if (millis() < lastTxLoraMessageMillis + 20000) {
                    targetPullValue = defaultPull;   // default pull
                    currentState = 1;
                 } else {
                 // B) go to soft brake afterwards
                    targetPullValue = softBrake;     // soft brake
                    currentState = -1;
                 }
            }
      }
      if (currentState == -7){
        Serial.println("CUT line!!!");
      }
 } 
 else {
      // rewinding winch mode
      // screen
      if (loopStep % 10 == 0) {
        display.clear();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_10);  //10, 16, 24
        display.drawString(0, 0, "rewinding winch mode");
        display.setFont(ArialMT_Plain_24);  //10, 16, 24
        display.drawString(0, 14, String(targetPullValue) + "/" + currentPull + "kg");
        display.display();
      }

      // small pull value on pull out
      // higher pull value on pull in
      if (vescUART.data.dutyCycleNow > 0.02){
        targetPullValue = 10;
      } else if (vescUART.data.dutyCycleNow < -0.02){
        targetPullValue = 17;
      } else {
        targetPullValue = -5; // no line movement --> soft brake
      }
      // ??? TODO higher pull value on fast pull out to avoid drum overshoot on line disconection ???
      
 }  // end rewind winch mode


      // auto line stop
      // (smouth to avoid line issues on main winch with rewinding winch)
      // tachometer > 2 --> avoid autostop when no tachometer values are read from uart (--> 0)
      if (vescUART.data.tachometer > 2 && vescUART.data.tachometer < 40) {
          if (targetPullValue > defaultPull){
              targetPullValue = defaultPull;
          }
          if (vescUART.data.tachometer < 20) {
              targetPullValue = softBrake;
          }
          if (vescUART.data.tachometer < 10) {
              targetPullValue = hardBrake;
          }
          // Serial.println("Autostop active, target pull value:");
          // Serial.println(targetPullValue);
      }
 
      // smooth changes
      // if brake --> immediately active
      if (targetPullValue < 0 ){
          currentPull = targetPullValue;
      } else {   
          // change rate e.g. max. 50 kg / second
          //reduce pull
          if (currentPull > targetPullValue) {
              smoothStep = 90 * (millis() - lastWritePWMMillis) / 1000;
              if ((currentPull - smoothStep) > targetPullValue)   //avoid overshooting
                  currentPull = currentPull - smoothStep;
              else
                  currentPull = targetPullValue;
          //increase pull
          } else if (currentPull < targetPullValue) {
              smoothStep = 65 * (millis() - lastWritePWMMillis) / 1000;
              if ((currentPull + smoothStep) < targetPullValue)   //avoid overshooting
                  currentPull = currentPull + smoothStep;
              else
                  currentPull = targetPullValue;
          }
          //Serial.println(currentPull);
          //avoid overrun
          if (currentPull < -127)
            currentPull = -127;
          if (currentPull > 127)
            currentPull = 127;
      }
      
      delay(10);
      //calculate PWM time for VESC
      // write PWM signal to VESC
      // pwmWriteTimeVal = (-127 ... 127 + 127)*(2000 - 950) / 254 + 950
                          // -20 = 1392 
                          // -7  = 1446
                          //  0  = 1475
                          //  7  = 1503
                          //  20 = 1557
                          // 90  = 1847 // max. für Solo
                          // 127 = 2000 // max. für Tandem
      pwmWriteTimeValue = (currentPull + 127) * (PWM_TIME_100 - PWM_TIME_0) / 254 + PWM_TIME_0;   
      // pulseOut ist eine Soft-PWM, HardPWM auch möglich?   
      pulseOut(PWM_PIN_OUT, pwmWriteTimeValue);
      lastWritePWMMillis = millis();

      if (currentState != -1){
        encValue = 0;
      } else{
        pullByUart(encValue);
        // Serial.printf("Pull value: %d\r\n", encValue);
      }
      // Turn on warning light if in pull mode
      if(currentState > 0){
        digitalWrite(WARN_LIGHT_OUT, 1);
      } else{
        digitalWrite(WARN_LIGHT_OUT, 0);
      }

      delay(10);    //RC PWM usually has a signal every 20ms (50 Hz)

      
      //read actual Vesc values from uart
      if (loopStep % 20 == 0) {
        if (vescUART.getVescValues()) {
          vescBattery = CapCheckPerc(vescUART.data.inpVoltage, numberOfCells);    // vesc battery in %
          vescTempMotor = vescUART.data.tempMotor;                                // motor temp in C   

          // Turn on fan if Mosfet temp rises above 50 °C
          if(vescUART.data.tempMosfet > 50){
            digitalWrite(FAN_OUT, 1);
          } else{
            digitalWrite(FAN_OUT, 0);
          }

            //SerialPrint(measuredVescVal, &DEBUGSERIAL);
            /*
            Serial.println(vescUART.data.tachometer);
            Serial.println(vescUART.data.inpVoltage);
            Serial.println(vescUART.data.dutyCycleNow);            
            Serial.println(vescUART.data.tempMotor);
            Serial.println(vescUART.data.tempMosfet);
            vescUART.printVescValues();
            */
        } else
        {
          //TODO send notification to lora
          //measuredVescVal.tachometer = 0;
          Serial.println("Failed to get data from VESC!");
        }
         
     if(digitalRead(ROTARY_SW)==0){
      encValue = 0;
     }

        // sprintf(txtOut,"Rotary pull: %d", encValue);
        Serial.println(txtOut);
      }
}
