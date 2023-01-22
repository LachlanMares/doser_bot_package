#include <Wire.h>
#include <EEPROM.h>
#include "MotorInterface.h"
#include "AtSerial.h"
#include "definitions.h"
#include "HX711.h"

#define m0_step_pin         4
#define m1_step_pin         5
#define m2_step_pin         6
#define m3_step_pin         7
#define m4_step_pin         8
#define m5_step_pin         9
#define m6_step_pin         10
#define m7_step_pin         11
#define m8_step_pin         12
#define m9_step_pin         13

#define load_cell_data_pin  2
#define load_cell_clock_pin 3

MotorInterface m0, m1, m2, m3, m4, m5, m6, m7, m8, m9;

AtSerial serialport;

HX711 scale;

bool enable_status_message = false;
uint8_t serial_buffer[SERIAL_BUFFER_LENGTH];
long last_motor_status_update, last_motor_pulses_update, last_load_cell_update, last_load_cell_sample; // milliseconds
long load_cell_raw = 0;

void setup() 
{
  Wire.begin();
  Wire.setClock(400000);
  //Serial.begin(115200);
  //while (!Serial);
  initialiseSerial();
  initialiseLoadcell();
  initialiseMotors();
}

void loop() 
{
  updateLoadcell(millis());
  updateMotors();
  updateSerial();
}
