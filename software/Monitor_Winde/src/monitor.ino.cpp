# 1 "/var/folders/0_/nhjxj_054s379kl5nlsrfmpr0000gp/T/tmpyde5wdtx"
#include <Arduino.h>
# 1 "/Users/juergenmueller/Library/CloudStorage/OneDrive-Persönlich/_Programmieren/PlatformIO/Projects/Monitor_Winde/src/monitor.ino"
# 9 "/Users/juergenmueller/Library/CloudStorage/OneDrive-Persönlich/_Programmieren/PlatformIO/Projects/Monitor_Winde/src/monitor.ino"
#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306.h"
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DI0 26
#define BAND 868E6

SSD1306 display(0x3c, 21, 22);
int rssi = 0;
float snr = 0;
#include <rom/rtc.h>

#include "Arduino.h"




Pangodream_18650_CL BL(35);


static int loopStep = 0;
bool toogleSlow = true;
int8_t targetPull = 0;
int currentPull = 0;
bool stateChanged = false;
int currentState = 0;
unsigned long lastStateSwitchMillis = 0;

uint8_t vescBattery = 0;
uint8_t vescTempMotor = 0;

#include "common.h"
struct LoraTxMessage loraTxMessage;
struct LoraRxMessage loraRxMessage;

unsigned long lastTxLoraMessageMillis = 0;
unsigned long lastRxLoraMessageMillis = 0;
unsigned long previousTxLoraMessageMillis = 0;
unsigned long previousRxLoraMessageMillis = 0;

unsigned int loraErrorCount = 0;
unsigned long loraErrorMillis = 0;
void setup();
void loop();
#line 59 "/Users/juergenmueller/Library/CloudStorage/OneDrive-Persönlich/_Programmieren/PlatformIO/Projects/Monitor_Winde/src/monitor.ino"
void setup() {
  Serial.begin(115200);


  pinMode(16,OUTPUT);
  pinMode(2,OUTPUT);
  digitalWrite(16, LOW);
  delay(50);
  digitalWrite(16, HIGH);


  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);

  LoRa.enableCrc();



  display.init();


  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Monitor \n");
  display.drawString(0, 0, "Starting Monitor");
}


void loop() {

    loopStep++;


    if (loopStep % 100 == 0) {
      toogleSlow = !toogleSlow;
    }
    if (loopStep % 10 == 0) {
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_16);
      if (toogleSlow) {
          display.drawString(0, 0, loraTxMessage.id + String("-B: ") + vescBattery + "%, T: " + vescTempMotor + " C");
      } else {
          display.drawString(0, 0, loraTxMessage.id + String("-T:") + ": " + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
      }
      display.setFont(ArialMT_Plain_24);
      display.drawString(0, 14, String(currentState) + String(" (") + targetPull + "/" + currentPull + String("kg)"));
      display.drawString(0, 36, String(loraRxMessage.tachometer) + " m| " + String(loraRxMessage.dutyCycleNow) + "%");
      display.display();
    }


    int loraPacketSize = 0;
    loraPacketSize = LoRa.parsePacket();

    if (loraPacketSize == sizeof(loraTxMessage) ) {
      LoRa.readBytes((uint8_t *)&loraTxMessage, sizeof(loraTxMessage));
      if ( loraTxMessage.pullValue == loraTxMessage.pullValueBackup) {
          targetPull = loraTxMessage.pullValue;
          currentState = loraTxMessage.currentState;
          previousTxLoraMessageMillis = lastTxLoraMessageMillis;
          lastTxLoraMessageMillis = millis();
          rssi = LoRa.packetRssi();
          snr = LoRa.packetSnr();
          Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraTxMessage.pullValue, rssi, snr);
      }
   }

    if (loraPacketSize == sizeof(loraRxMessage) ) {
        LoRa.readBytes((uint8_t *)&loraRxMessage, sizeof(loraRxMessage));
        currentPull = loraRxMessage.pullValue;

          if (loraRxMessage.vescBatteryOrTempMotor == 1){
            vescBattery = loraRxMessage.vescBatteryOrTempMotorValue;
          } else {
            vescTempMotor = loraRxMessage.vescBatteryOrTempMotorValue;
          }
        previousRxLoraMessageMillis = lastRxLoraMessageMillis;
        lastRxLoraMessageMillis = millis();
        rssi = LoRa.packetRssi();
        snr = LoRa.packetSnr();
        Serial.printf("Value received: %d, RSSI: %d: , SNR: %d \n", loraRxMessage.pullValue, rssi, snr);
        Serial.printf("tacho: %d, dutty: %d: \n", loraRxMessage.tachometer, loraRxMessage.dutyCycleNow);
   }


  if (millis() > lastRxLoraMessageMillis + 1500 ) {


        display.clear();
        display.display();

       if (millis() > loraErrorMillis + 5000) {
            loraErrorMillis = millis();
            loraErrorCount = loraErrorCount + 1;
       }
  }



        delay(10);
}