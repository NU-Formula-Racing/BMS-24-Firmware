#include <Arduino.h>

#include <chrono>

#include "ELCON-Charger.h"
//#include "bms.h"
//#include "bms_telemetry.h"
#include "bq_comm.h"
#include "teensy_can.h"
#include "thermistor.h"
#include "virtualTimer.h"

#define serialdebug 1

TeensyCAN<1> hp_can{};
TeensyCAN<2> lp_can{};
TeensyCAN<3> vb_can{};

ElconCharger charger{vb_can, 120 * 15, 14};

VirtualTimerGroup timer_group{};

const uint8_t kNumSegmentsConfig = 1;

//ShutdownInput shutdown_input{A14, 1.0f / 7.0f, 15.0f, 10.0f};

NXFT15XH103FA2B050 thermistor{};
BQ79656 bq = {Serial8, 35, thermistor, 20 * kNumSegmentsConfig, 16 * kNumSegmentsConfig, 2 * kNumSegmentsConfig};

void setup() {
  // put your setup code here, to run once:
  // init BMS
}

void loop() {
  // put your main code here, to run repeatedly:
}