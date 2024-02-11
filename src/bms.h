#pragma once
#include <Arduino.h>

#include <algorithm>

#include "bq_comm.h"


/*
#include "I-Charger.h"
#include "Watchdog_t4.h"
#include "bms_interface.h"
#include "bms_telemetry.h"
#include "bq_comm.h"
#include "can_interface.h"
#include "cellinfo.h"
#include "coulomb_counting.h"
*/


// WTF I need to do
/* 
    1. Get voltages
    2. Get the temps
    3. Set the state accordingly
        - check for faults
            - voltage over/under
            - temp fault
            - overcurrent fault
            - shutdown / external kill
            - openwire fualt
        - 
    4. Set the values for:
        - max, min, avg voltage
        - get SOC   // TODO later
        - max, min, avg temp
        - pack voltage
        - max current_discharge allowed
    5. Tick timer and feed watchdog
    6. 
*/

// What I need to initalize
/*
    - num segments
    - num (series) cells per segment
    - num thermistors per segment
    - CAN lines / timer group
        - HP
        - LP
        - Verbose
    - teensy pins
    - shutdown circuit imput
*/

// States
/*
    0 => shutdown
    1 => fault
    2 => precharge
    3 => active
    4 => charging
*/

enum class BMSState{
    shutdown = 0,
    fault = 1,
    precharge = 2,
    active = 3,
    charging = 4
};

class BMS
{
public:
    // 
    BMS(BQ79656 bq);

    // init


private:

    // 
    BQ79656 bq_;

    // 

};