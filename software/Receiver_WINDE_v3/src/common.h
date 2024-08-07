/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

*/


//send by transmitter
typedef union{
   uint8_t byte[6];
   struct{
   uint8_t startframe;           // 0xBC --> from Base to ControlUnit
   int8_t pullValue;           // currently active pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   uint8_t tachometer;          // *10 --> in meter
   uint8_t dutyCycleOrWindDirection:1;
   uint8_t dutyCycleOrWindDirektionValue:7;
   uint8_t vescBatteryOrTempMotor : 1 ;  // 0 ==> vescTempMotor , 1 ==> vescBatteryPercentage
   uint8_t vescBatteryOrTempMotorValue  : 7 ;   //0 - 127
   };
}LoraTxMessage;

typedef union{
   uint8_t byte[5];
   struct {  
   uint8_t startframe;        // 0xCB --> from ControlUnit to Base
   uint8_t id : 4;              // unique id 1 - 15, id 0 is admin!
   int8_t currentState : 4;    // -2 --> -2 = hard brake -1 = soft brake, 0 = no pull / no brake, 1 = default pull (2kg), 2 = pre pull, 3 = take off pull, 4 = full pull, 5 = extra strong pull
   int8_t pullValue;           // target pull value,  -127 - 0 --> 5 brake, 0 - 127 --> pull
   int8_t pullValueBackup;     // to avoid transmission issues, TODO remove, CRC is enough??
   };
}LoraRxMessage;
