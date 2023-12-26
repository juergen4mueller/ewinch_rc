# ewinch_remote_controller
 transmitter and receiver code for remote controlling a paragliding winch
 forked from robertzach/ewinch_remote_controller

receiver uses PPM for driving the winch and (optinal) UART to read additional information (line length, battery %, dutycycle)
VESC UART communication depends on https://github.com/SolidGeek/VescUart/

Line auto stop can be implemented within VESC with vesc_ppm_auto_stop.patch
Default VESC app config is vesc_app_config.xml

## Changes / Updates
 Changed ESP32-Board to custom made with MH-ET-ESP32-Devkit and Heltec ESP32-Lora(v3).
 chandrawi/LoRaRF-lib for usage of SX126x modul on Heltec v3.
 Made new case for Heltec module, same size but different mount points.
 Moved software project to PLATFORMIO using VSCode for convenience work.

## Software improvements:
 - rotary encoder in transmitter to adjust "max. weight" (push while switch on, then select and push to confirm)
 - EEPROM function in transmitter to hold last "max. weight" value.
 - rotary encoder in receiver used to pull manually (adjust force by turning, push stops immediately)

