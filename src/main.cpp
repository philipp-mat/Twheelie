#include <Arduino.h>
#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include <ODriveTeensyCAN.h>
#include "ODriveFlexCAN.hpp"

ODriveTeensyCAN odriveCAN(250000);

void setup() {
  Serial.begin(115200);  //start Serial monitor

  odriveCAN.SetLimits(0, 5, 36);
  odriveCAN.RunState(0, 8);
  odriveCAN.RunState(1, 8);
}
void loop() {
  Serial.println("Hello World");
}  