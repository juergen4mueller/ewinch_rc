#include <Arduino.h>
#include <RadioLib.h>
#include "LiPoCheck.h"    //to calculate battery % based on cell Voltage
#include <Pangodream_18650_CL.h>
#include <SPI.h>
#include <Wire.h>  
#include "SSD1306.h"
#include <rom/rtc.h>
#include <Button2.h>
#include <VescUart.h>
#include "common.h"
#include <EEPROM.h>

// /*
//  * receiver
//  * receives target pull value from lora link
//  * sends acknowlegement on lora with current parameters
//  * writes target pull with PWM signal to vesc
//  * reads current parameters (tachometer, battery %, motor temp) with UART from vesc, based on (https://github.com/SolidGeek/VescUart/)
//  * 
//  */


#define EEPROM_SIZE 20
#define EEPROM_STARTUP_COUNTER 0
#define EEPROM_DEVICE_ID 0
#define EEPROM_MAX_PULL  1

int rssi = 0;
float snr = 0;
bool debug = false;
char txtOut[40];
char serialIn[20];
int serialInBufferPos;
int serialReceiverWaiting;
LoraTxMessage loraTxMsg;
LoraRxMessage loraRxMsg;

#define LED_ONBOARD 0
//SX1262 radio = new Module(8, 14, 12, 13);  
SX1276 radio = new Module(18, 26, 14, RADIOLIB_NC);  
 
bool loraRxFlag;


void getLoraState(void){
  uint16_t irq = radio.getIrqFlags();
  Serial.printf("RL IRQ Status: 0x%X\r\n", irq);
}

void radioInterrupt(void){
  int irFlags = radio.getIrqFlags();
  if(debug){
    Serial.printf("Lora Int fired %d\r\n", irFlags);
  }
  if(irFlags & 0x40){
    loraRxFlag = true;
  }
  // int irFlags = radio.getIRQFlags();
  // if(irFlags & RADIOLIB_SX127X_MASK_IRQ_FLAG_RX_DONE){
  // }
}

void lora_init(void){
  Serial.println("Lora Receiver startet...");
  //int state = radio.begin(866.5, 125.0, 8, 5, 0x12, 14);
  int state = radio.begin(866.5, 125.0, 8, 5, 0x12, 17);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Init fehlgeschlagen: ");
    Serial.println(state);
  }

  radio.setCRC(true);
  radio.setDio0Action(radioInterrupt, RISING);

  // radio.setDio1Action(radioInterrupt);
  radio.startReceive();
  Serial.println("Empfänger bereit.");
}

//   // // Pins Rotary Encoder

//   // #define ROTARY_SW  2
//   // #define ROTARY_A   3
//   // #define ROTARY_B   4

//   // // Buttons for state machine control
//   // #define BUTTON_UP    5 // up
//   // #define BUTTON_DOWN  6 // down

//   // Pins OLED Display

  #define OLED_SDA  4
  #define OLED_SCL  15
  #define OLED_RST  16 // only in Heltec mode used

  // No battery used in receiver
  #define BAT_AN_IN  13
  //#define BAT_EN_AN 37 // Enable ADC divider for meassure BAT voltage

  // Define sonstiger Pins
  // LED's zur Statusausgabe, grün leuchtet wenn an, rot blitzt bei LoraRX auf
  //Battery Voltage IO 7 -> Wind direction
  // #define WindInAnalog  7

  #define VESC_RX  22    //connect to TX on Vesc
  #define VESC_TX  23    //connect to RX on Vesc

  // #define Cutter_Out 40 // up
  // #define LED_RT 41 // down
  // Pins Rotary Encoder -> Funktionen:
  // - Seil manuell einziehen (Zug mehr / weniger über drehen, bei Push SoftBreak)

  // #define WARN_LIGHT_OUT 42 
  // #define FAN_OUT   45

  #define PWM_PIN_OUT  25 //Define Digital PIN





// //vesc battery number of cells
static int numberOfCells = 16;
// static int myMaxPull = 75;  // 0 - 127 [kg], must be scaled with VESC ppm settings

SSD1306 display(0x3C, OLED_SDA, OLED_SCL);

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
  // encIn = digitalRead(ROTARY_A)+digitalRead(ROTARY_B)*2;
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
  // pinMode(ROTARY_A, INPUT_PULLUP);
  // pinMode(ROTARY_B, INPUT_PULLUP);
  // attachInterrupt(ROTARY_A, rotaryInterrupt, CHANGE);
  // attachInterrupt(ROTARY_B, rotaryInterrupt, CHANGE);
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

#define WIND_VALUE_OFFSET 112
uint8_t get_wind_direction(void){
  // Wind direction from potentiometer value 0 ... 4096
  // output 7 Bit max, value 0 ... 36 is direction with factor 10 degree
  uint16_t anValue = 0;//analogRead(WindInAnalog);
  // if(anValue < WIND_VALUE_OFFSET){
  //   anValue = 4095 - WIND_VALUE_OFFSET; 
  // }
  // else{
  //   anValue -= WIND_VALUE_OFFSET;
  // }
  // anValue /= 112;
  // Serial.print("AN Wind: ");
  // Serial.println(anValue);
  return anValue;
}
uint8_t startupCounter = 0;

uint32_t nextSendTime;
#define BOOT_PIN 0
uint8_t bootPinIn, bootPinInAlt;
#define VEXT_CTL 21
void setup() {
  EEPROM.begin(EEPROM_SIZE);
  startupCounter = EEPROM.read(EEPROM_STARTUP_COUNTER)+1;
  EEPROM.write(EEPROM_STARTUP_COUNTER, startupCounter);
  EEPROM.commit();
  if(LED_ONBOARD > 0){
    pinMode(LED_ONBOARD, OUTPUT);
    digitalWrite(LED_ONBOARD, 0);
  }
  

  #ifdef VEXT_CTL
    pinMode(VEXT_CTL, OUTPUT);
    digitalWrite(VEXT_CTL, 0);
  #endif

  #ifdef OLED_RST
    delay(10);
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, 1);
    delay(100);
    digitalWrite(OLED_RST, 0);
    delay(100);
    digitalWrite(OLED_RST, 1);
  #endif

  pinMode(BOOT_PIN, INPUT_PULLUP);
  // pinMode(Cutter_Out, OUTPUT);
  // digitalWrite(Cutter_Out, 0);
  // pinMode(LED_RT, OUTPUT);
  // digitalWrite(LED_RT, 1);
  // pinMode(WindInAnalog, ANALOG);
  Serial.begin(115200);

  encoderInit();
  //Setup UART port for Vesc communication
  Serial1.begin(115200, SERIAL_8N1, VESC_RX, VESC_TX);
  vescUART.setSerialPort(&Serial1);
  
  //lora init
  lora_init();

  // display init
  delay(500);
  display.init();
  display.setBrightness(200);
  display.flipScreenVertically();  

  //PWM Pins
  //pinMode(PWM_PIN_IN, INPUT);
  pinMode(PWM_PIN_OUT, OUTPUT);
  
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  Serial.printf("Starting Receiver \n");
  display.drawString(0, 0, "Starting Receiver");
  display.display();
  // digitalWrite(LED_RT, 0);
  nextSendTime = millis();

  Serial.print("Startup number ");
  Serial.print(startupCounter);
  Serial.println(" done.");
}

uint32_t sendCycle =0;
uint32_t now, lastSend, lastRead;
uint16_t counter_send;


uint32_t loraRxCnt = 0, loraTxCnt = 0;
uint32_t nextTime = 0;
uint8_t rxData[10];
void loop() {
 if (true) {
    // screen
    loopStep++;
    if (loopStep % 10 == 0) { // Display aktualisieren
      display.clear();
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      //display.drawString(0, 0, currentId + String("-RX: (") + BL.getBatteryChargeLevel() + "%, " + rssi + "dBm, " + snr + ")");
      display.drawString(0, 0, currentId + String("-RX: (")+ rssi + "dBm, " + snr + ")");
      display.setFont(ArialMT_Plain_24);  //10, 16, 24
      if (currentState == -7){
          display.drawString(0, 11, "Cutting");  
          // digitalWrite(Cutter_Out, 1);
      }
      else if (currentState > 0){
          // digitalWrite(Cutter_Out, 0);
          display.drawString(0, 11, String("P ") + currentState + ": (" + currentPull + "kg)");  
      } else {
          // digitalWrite(Cutter_Out, 0);
          display.drawString(0, 11, String("B ") + currentState + ": (" + currentPull + "kg)");    
      }
      display.setFont(ArialMT_Plain_10);  //10, 16, 24
      //display.drawString(0, 36, String("Error / Uptime{min}: ") + loraErrorCount + " / " + millis()/60000);
      display.drawString(0, 36, String("B: ") + vescBattery + "%, M: " + vescTempMotor + "C" +" R"+encValue + " UP "+startupCounter); 
      display.drawString(0, 48, String("TX / RX: ") + loraRxCnt + " / " + loraTxCnt);
      display.display();
    }
    if(loraRxFlag){
      loraRxFlag = false;
      int state = radio.readData(rxData, sizeof(loraRxMsg.byte));
      if (state != RADIOLIB_ERR_NONE) {
        Serial.println("Lora Rx Error");
      }
      else{
        if(rxData[0]== 0xCB){
          loraRxCnt ++;
          if(LED_ONBOARD)
            digitalWrite(LED_ONBOARD,1);
          rssi = radio.getRSSI();
          snr = radio.getSNR();
          memcpy(loraRxMsg.byte, rxData, sizeof(loraRxMsg.byte));
          if(debug){
            Serial.print("Lora Rx: ");
            for(int z=0;z<sizeof(loraRxMsg.byte);z++){
              Serial.printf("0x%02X ", loraRxMsg.byte[z]);
            }
            Serial.println("");
          }
          if (millis() > lastTxLoraMessageMillis + 5000){
            activeTxId = loraRxMsg.id;
          }
          // The admin id 0 can allways take over 
          if (loraRxMsg.id == 0){
            activeTxId = loraRxMsg.id;
          }
          if ((loraRxMsg.id == activeTxId) && (loraRxMsg.pullValue == loraRxMsg.pullValueBackup)) {
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
              //Serial.printf("Tacho: %d\r\n", vescUART.data.tachometer);
              // alternate vescBatteryPercentage and vescTempMotor value on lora link to reduce packet size
              if(loraTxMsg.dutyCycleOrWindDirection == 0){
                loraTxMsg.dutyCycleOrWindDirection = 1;
                loraTxMsg.dutyCycleOrWindDirektionValue = abs(vescUART.data.dutyCycleNow * 100);     //in %
              }
              else{ // Wind direction max value = 127, real 360 degree
                    // direction will be transmitted by faktor 10, 0...36 
                loraTxMsg.dutyCycleOrWindDirection = 0;
                loraTxMsg.dutyCycleOrWindDirektionValue = get_wind_direction();
              }
             
              if (loraTxMsg.vescBatteryOrTempMotor == 0){
                loraTxMsg.vescBatteryOrTempMotor = 1;
                loraTxMsg.vescBatteryOrTempMotorValue = vescBattery;
              } else {
                loraTxMsg.vescBatteryOrTempMotor = 0;
                loraTxMsg.vescBatteryOrTempMotorValue = vescTempMotor;
              }
              // digitalWrite(LED_RT, 0);
              delay(10);
              if(LED_ONBOARD) 
                digitalWrite(LED_ONBOARD, 0);
              int txState = radio.transmit(loraTxMsg.byte, sizeof(loraTxMsg.byte));
              radio.startReceive();
              
              if (txState == RADIOLIB_ERR_NONE) {
                loraTxCnt ++;
                if(debug){  
                  Serial.print("Lora Tx: ");
                  for(int z=0;z<sizeof(loraTxMsg.byte);z++){
                    Serial.printf("0x%02X ", loraTxMsg.byte[z]);
                  }
                  Serial.println("");
                  Serial.printf("sending pull value %d: \r\n", targetPullValue);
                }
                  lastTxLoraMessageMillis = millis();  
              } 
              else {
                Serial.println("Lora send error");
              }       
        }
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
      // PWM to current:
      // PWM range from 1.480 to 2.000
      // 520 mV for pull range 0 ... 310 A
      // 90 kg -> 0x372 V -> 222 A 
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
      // if(currentState > 0){
      //   digitalWrite(WARN_LIGHT_OUT, 1);
      // } else{
      //   digitalWrite(WARN_LIGHT_OUT, 0);
      // }

      delay(10);    //RC PWM usually has a signal every 20ms (50 Hz)

      
      //read actual Vesc values from uart
      if (loopStep % 10 == 0) {
        if (vescUART.getVescValues()) {
          vescBattery = CapCheckPerc(vescUART.data.inpVoltage, numberOfCells);    // vesc battery in %
          vescTempMotor = vescUART.data.tempMotor;                                // motor temp in C   

          // Turn on fan if Mosfet temp rises above 50 °C
          // if(vescUART.data.tempMosfet > 50){
          //   digitalWrite(FAN_OUT, 1);
          // } else{
          //   digitalWrite(FAN_OUT, 0);
          // }

        } else
        {
          //TODO send notification to lora
          //measuredVescVal.tachometer = 0;
          // Serial.println("Failed to get data from VESC!");
        }
         
        //  if(digitalRead(ROTARY_SW)==0){
        //   encValue = 0;
        //  }

        // sprintf(txtOut,"Rotary pull: %d", encValue);
        //Serial.println(txtOut);
      }

      bootPinIn = digitalRead(BOOT_PIN);
      if((bootPinIn==0)&&(bootPinInAlt != 0)){
        Serial.println("Boot Switch !!!");
      }
      bootPinInAlt = bootPinIn;

      char rcChar; 
      if(Serial.available()){
        
        Serial.read(&rcChar, 1);
        Serial.printf("Got char %c ", rcChar);

        if(rcChar == 'i'){ // info request
          getLoraState();
        }
        if(rcChar == 'd'){
          if(debug){
            debug = false;
            Serial.println("DEBUG off");
          }
          else{
            debug = true;
            Serial.println("DEBUG on");
          }
        }
        if(rcChar == 'r'){
          serialInBufferPos = 0;
          serialIn[serialInBufferPos++] =rcChar; 
          serialReceiverWaiting = 1;
        }
        else if(serialReceiverWaiting){
          serialIn[serialInBufferPos++] =rcChar; 
        }
        if(serialInBufferPos >= 3){
          serialReceiverWaiting = 0;
          serialIn[3]= 0;
          if(strcmp("rst", serialIn)==0){
            Serial.println("got reset command!");
            EEPROM.write(EEPROM_STARTUP_COUNTER, 0);
            EEPROM.commit();
          }
        }
      }
    }