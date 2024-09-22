#include <Arduino.h>

#include <chrono>

#include "ELCON-Charger.h"
#include "bms.h"
#include "bms_telemetry.h"
#include "bq_comm.h"
#include "teensy_can.h"
#include "thermistor.h"
#include "virtualTimer.h"
#include "teensy_pin_defs.h"

#define serialdebug true

// #define TEENSY40_HACK

TeensyCAN<1> hp_can{};
TeensyCAN<2> vb_can{};

#ifdef TEENSY40_HACK
#include "dummy_can.hpp"
DummyCAN lp_can{};
#else
TeensyCAN<3> lp_can{};
#endif // TEENSY40_HACK

ElconCharger charger{vb_can, 120 * 15, 14};

VirtualTimerGroup    timer_group{};

const uint8_t kNumSegmentsConfig = 8;

ShutdownInput shutdown_input{A14, 1.0f / 8.0f, 15.0f, 10.0f};

NXFT15XH103FA2B050 thermistor{};
BMS bms{BQ79656{Serial8, 35, thermistor, 18 * kNumSegmentsConfig, 16 * kNumSegmentsConfig, 2 * kNumSegmentsConfig},
        18 * kNumSegmentsConfig,
        16 * kNumSegmentsConfig,
        charger,
        timer_group,
        hp_can,
        lp_can,
        vb_can,
        shutdown_input};

void setup()
{
    Serial.begin(9600);
    delay(2000);

    if (CrashReport)
    {
        Serial.print(CrashReport);
    }

#if serialdebug
    Serial.println("Starting...");
#endif
    // put your setup code here, to run once:
    bms.Initialize();
    // Serial.println("BMS Inited");
    hp_can.Initialize(ICAN::BaudRate::kBaud1M);
    lp_can.Initialize(ICAN::BaudRate::kBaud1M);
    vb_can.Initialize(ICAN::BaudRate::kBaud500K);
    charger.Initialize();
    timer_group.AddTimer(100, []()
                         { bms.Tick(); });
    // delay(1000);
}

void loop() { 
    // Serial.println("Looping...");
    timer_group.Tick(millis()); 
}