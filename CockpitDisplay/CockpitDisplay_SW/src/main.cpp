/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/get-started/platforms/arduino.html  */
// Using Display from Waveshare: https://www.waveshare.com/wiki/ESP32-S3-LCD-1.28
// https://www.waveshare.com/esp32-s3-lcd-1.28.htm

#include <Arduino.h>
#include <lvgl.h>
#include <ui.h>
#include <WiFi.h>
#include <esp_now.h>

#include <TFT_eSPI.h>
#include <Battery18650Stats.h>

#include <Wire.h>
#include <SensorQMI8658.hpp>

#define ADC_PIN 1

// QMI8658 Pins
#define SENSOR_SDA 6
#define SENSOR_SCL 7
#define USE_WIRE
#define SENSOR_IRQ  -1

SensorQMI8658 qmi;


IMUdata acc[128];
IMUdata gyr[128];

void gyro_sensor_start(void){
  //Using WIRE !!
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
      Serial.println("Failed to find QMI8658 - check your wiring!");
      while (1) {
          delay(1000);
      }
  }
    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);


    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_0,
        true);


    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true);

    qmi.configFIFO(
        SensorQMI8658::FIFO_MODE_FIFO,
        SensorQMI8658::FIFO_SAMPLES_16,
        SensorQMI8658::IntPin1,
        8);

    // In 6DOF mode (accelerometer and gyroscope are both enabled),
    // the output data rate is derived from the nature frequency of gyroscope
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    // Print register configuration information
    qmi.dumpCtrlRegister();

}

void gyro_sensor_read(void){
   if (!qmi.readFromFifo(acc, 128, gyr, 128)) {
        return;
    }
    for (int i = 0; i < 16; ++i) {
        Serial.print("ACCEL: ");
        Serial.print("X:");
        Serial.print(acc[i].x);
        Serial.print(" Y:");
        Serial.print(acc[i].y);
        Serial.print(" Z:");
        Serial.println(acc[i].z);
        Serial.print("GYRO: ");
        Serial.print(" X:");
        Serial.print(gyr[i].x);
        Serial.print(" Y:");
        Serial.print(gyr[i].y );
        Serial.print(" Z:");
        Serial.println(gyr[i].z);
    }
}

Battery18650Stats battery(ADC_PIN, 2.4875);

// MAC ADDR: 7C:DF:A1:ED:63:18
/*
included espnow in this example
*/

typedef struct struct_message {
  int state;
  int kgSoll;
  int kgIst;
  int lineLength;
  bool cut;
  bool timeout;
} t_RC_Winch;

t_RC_Winch rcWinch;
esp_now_peer_info_t peerInfo;

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{
    uint16_t touchX, touchY;

    bool touched = false;//tft.getTouch( &touchX, &touchY, 600 );

    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        Serial.print( "Data x " );
        Serial.println( touchX );

        Serial.print( "Data y " );
        Serial.println( touchY );
    }
}


typedef struct {
  int state;
  int lineMeters;
} t_recData;
t_recData recData;
char text[20];
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rcWinch, incomingData, sizeof(rcWinch));
  sprintf(text, "%d ", rcWinch.state);
  lv_label_set_text(ui_lblStufe,text);
  if(rcWinch.cut){
    lv_label_set_text(ui_lblForceSoll, "CUT");
  }
  else{
    sprintf(text, "%d", rcWinch.kgSoll);
    lv_label_set_text(ui_lblForceSoll,text);
  }
  if(rcWinch.timeout){ 
    lv_obj_set_style_bg_color(ui_arcBatt, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT );
  }
  else{
    lv_obj_set_style_bg_color(ui_arcBatt, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
  }
  sprintf(text, "%d", rcWinch.kgIst);
  lv_label_set_text(ui_lblForceIst,text);
  sprintf(text, "%d ", rcWinch.lineLength);
  lv_label_set_text(ui_lblLine,text);
  if(rcWinch.lineLength > 900){
    lv_obj_set_style_text_color(ui_lblLine, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT );
  }
  else if(rcWinch.lineLength > 800){
    lv_obj_set_style_text_color(ui_lblLine, lv_color_hex(0xFFFF00), LV_PART_MAIN | LV_STATE_DEFAULT );
  }
  else{
    lv_obj_set_style_text_color(ui_lblLine, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
  }
}


#define LCD_BL_PIN 40
uint32_t nextSendEvent;

uint32_t ticks;

void setup(){
  

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, 1);

  Serial.begin(115200);

  //PSRAM Initialize
  if(psramInit()){
    Serial.println("\nPSRAM is correctly initialized");
  }else{
    Serial.println("PSRAM not available");
  }


    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

    lv_init();
    
#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( 2 ); /* Landscape orientation, flipped */


    /*Set the touchscreen calibration data,
     the actual data for your display can be acquired using
     the Generic -> Touch_calibrate example from the TFT_eSPI library*/
    uint16_t calData[5] = { 275, 3620, 264, 3532, 1 };
   // tft.setTouch( calData );

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );
    
    // /* Create simple label */
    // lv_obj_t *label = lv_label_create( lv_scr_act() );
    // lv_label_set_text( label, "Hello Ardino and LVGL!");
    // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
 
    ui_init();




  Serial.println( "Setup done" );

  nextSendEvent = millis();
  ticks = millis();

  gyro_sensor_start();


  WiFi.mode(WIFI_STA);
  delay(1000);


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

esp_now_register_recv_cb(OnDataRecv);
}
uint8_t screenNumber = 0;
void loop()
{
  if(millis() > nextSendEvent){
    nextSendEvent += 1000;
    gyro_sensor_read();
    // Serial.print("Volts: ");
    // Serial.println(battery.getBatteryVolts());

    // Serial.print("Charge level: ");
    // Serial.println(battery.getBatteryChargeLevel());

    // Serial.print("Charge level (using the reference table): ");
    // Serial.println(battery.getBatteryChargeLevel(true));
    lv_arc_set_value(ui_arcBatt, battery.getBatteryChargeLevel());
  }

  lv_task_handler(); /* let the GUI do its work */
  delay(5); /* let this time pass */
  uint32_t now = millis();
  lv_tick_inc(now - ticks);
  ticks = now;
}
