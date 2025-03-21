#include "bms.h"

#include <algorithm>
#include <numeric>

#include "I-Charger.h"
#include "can_interface.h"

int BMS::fault_pin_{-1};

void BMS::CheckFaults()
{
    overvoltage_fault_ = static_cast<BMSFault>(max_cell_voltage_ >= kCellOvervoltage);
    undervoltage_fault_ = static_cast<BMSFault>(min_cell_voltage_ <= kCellUndervoltage);
    overcurrent_fault_ = static_cast<BMSFault>(current_[0] >= kOvercurrent);
    overtemperature_fault_ = static_cast<BMSFault>(max_cell_temperature_ >= kOvertemp);
    undertemperature_fault_ = static_cast<BMSFault>(min_cell_temperature_ <= kUndertemp);
    external_kill_fault_ = static_cast<BMSFault>(digitalRead(shutdown_signal_LED) == LOW); // this is determined based on the input of
                                                                                           // contactor shutdown line.

    internal_fault_ = // static_cast<BMSFault>(false);
        static_cast<BMSFault>(static_cast<bool>(overvoltage_fault_) || static_cast<bool>(undervoltage_fault_) || static_cast<bool>(overcurrent_fault_) || static_cast<bool>(overtemperature_fault_) || /*static_cast<bool>(undertemperature_fault_) || */ static_cast<bool>(open_wire_fault_));
    external_fault_ =
        static_cast<BMSFault>(static_cast<bool>(external_kill_fault_));
}

void BMS::Tick()
{
    watchdog_timer_.feed(); // so we don't reboot
    // check fault status

    if (internal_fault_ != BMSFault::kNotFaulted && current_state_ != BMSState::kFault)
    {
#if 1
        Serial.println("Faults:");
        if (static_cast<bool>(overvoltage_fault_))
        {
            Serial.println("  Overvoltage");
        }
        else if (static_cast<bool>(undervoltage_fault_))
        {
            Serial.println("  Undervoltage");
        }
        if (static_cast<bool>(overtemperature_fault_))
        {
            Serial.println("  Overtemperature");
        }
        else if (static_cast<bool>(undertemperature_fault_))
        {
            Serial.println("  Undertemperature");
        }
        if (static_cast<bool>(overcurrent_fault_))
        {
            Serial.println("  Overcurrent");
        }
        if ((static_cast<bool>(external_kill_fault_) && current_state_ != BMSState::kShutdown))
        {
            Serial.println(" external error");
        }
        if (static_cast<bool>(open_wire_fault_))
        {
            Serial.println("  open wire");
        }
        Serial.println("");
#endif

        ChangeState(BMSState::kFault);
    }

    ProcessState();
    if (!static_cast<bool>(internal_fault_))
    {
        digitalWrite(bms_status, HIGH);
    }
    if(static_cast<bool>(external_fault_))
    {
        Serial.println("this is external faulting?");
        ChangeState(BMSState::kShutdown);
    }
    // log to SD, send to ESP, send to CAN
    // todo
}

// Find maximum discharge and regen current
void BMS::CalculateSOE()
{
    float current_per_cell = current_[0] / kNumCellsParallel;
    float internal_resistance_per_series_element = kInternalResistance / kNumCellsParallel;

    // Find highest and lowest open circuit voltage
    float min_open_circuit_voltage = min_cell_voltage_ + (current_per_cell * kInternalResistance);
    float max_open_circuit_voltage = max_cell_voltage_ + (current_per_cell * kInternalResistance);

    // Limit at V_open + I * (R_internal / numCellsParallel) = V_boundary
    //  => I = (numCellsParallel / R_internal) * (V_boundary - V_open)
    float max_discharge_voltage_delta = min_open_circuit_voltage - kCellUndervoltage;
    float max_regen_voltage_delta = kCellOvervoltage - max_open_circuit_voltage;
    float uncapped_discharge_current = max_discharge_voltage_delta / internal_resistance_per_series_element;
    float uncapped_regen_current = max_regen_voltage_delta / internal_resistance_per_series_element;

    // I = P / V
    pack_voltage_ = std::accumulate(voltages_.begin(), voltages_.end(), 0.0);
    float power_capped_current = kMaxPowerOutput / pack_voltage_;

    max_allowed_discharge_current_ = std::min({uncapped_discharge_current, power_capped_current, kDischargeCurrent});
    // Serial.println(max_allowed_discharge_current_);
    max_allowed_regen_current_ = std::min(uncapped_regen_current, kRegenCurrent); // TODO: See if regen counts towards power current
}

void BMS::ProcessCooling()
{
    // check temperatures
    bq_.GetTemps(temperatures_);
    max_cell_temperature_ = *std::max_element(temperatures_.begin(), temperatures_.end());
    min_cell_temperature_ = *std::min_element(temperatures_.begin(), temperatures_.end());
    average_cell_temperature_ = std::accumulate(temperatures_.begin(), temperatures_.end(), 0) / temperatures_.size();
    Serial.printf("%d\n", average_cell_temperature_);
}

void BMS::UpdateValues()
{
    Serial.println("Start of UpdatedValues");
    ProcessCooling();
    // bq_.GetCurrent(current_); / not used /
    current = digitalRead(current_sense);
    Serial.printf("%d \n", current);

    bq_.GetVoltages(voltages_);
    Serial.println("Got voltage ");
    max_cell_voltage_ = *std::max_element(voltages_.begin(), voltages_.end());
    min_cell_voltage_ = *std::min_element(voltages_.begin(), voltages_.end());

    // log imd status and send to can
    Serial.printf("IMD State Pre: %d, IMD Status Read: %d \n", imd_state_, digitalRead(imd_status));
    if (digitalRead(imd_status) == HIGH)
    {
        imd_state_ = BMSState::kPrecharge;
    }
    else
    {
        imd_state_ = BMSState::kShutdown;
        latch_state = true;
    }
    Serial.printf("IMD State Pre: %d, IMD Status Read: %d \n", imd_state_, digitalRead(imd_status));

    timeSinceLastCANRX += 1;

    // debug cell_v print
    u_int32_t i = 0;
    for (auto voltage : voltages_)
    {
        i++;
        Serial.printf("%d Voltages:", i);
        Serial.println(voltage);
    }
    i = 0;

    CalculateSOE();
    if (!coulomb_count_.Initialized())
    {
        coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), millis());
    }
    else
    {
        coulomb_count_.CountCoulombs(current_[0], millis());
    }
#if serialdebug
    Serial.print("Current: ");
    Serial.print(current_[0]);
    Serial.print("\tMax voltage: ");
    Serial.println(max_cell_voltage_);
#endif
}

void BMS::ProcessState()
{
    // get new values
    UpdateValues();
    // check faults
    CheckFaults();

    // prints
    // Serial.print("fault_: ");
    // Serial.println(static_cast<int>(fault_));
    // Serial.print("current_state_: ");
    // Serial.println(static_cast<int>(current_state_));

    switch (current_state_)
    {
    case BMSState::kShutdown:

        static constexpr float kMaxChargeVoltage{4.0f};
        digitalWrite(bms_sd_ctrl, HIGH);
        digitalWrite(bms_status, HIGH);
        Serial.println("Cell Balancing:");
        bq_.ProcessBalancing(voltages_, kMaxChargeVoltage);

        // check for command to go to active
        Serial.println("Shutdown");
        if (command_signal_ == Command::kPrechargeAndCloseContactors && !static_cast<bool>(external_fault_))
        {
            Serial.println("inside precharge");
            ChangeState(BMSState::kPrecharge);
        }
        else if (charger_.IsConnected() && !(command_signal_ == Command::kShutdown))
        {
            Serial.println("Detected charger");
            ChangeState(BMSState::kPrecharge);
        }
        break;
    case BMSState::kPrecharge:
        Serial.println("Precharge");
        // do a time-based precharge for charger
        if (command_signal_ == Command::kShutdown || timeSinceLastCANRX >= 1000)
        {
            ChangeState(BMSState::kShutdown);
        }
        if (millis() >= state_entry_time_ + kPrechargeTime)
        {
            if (charger_.IsConnected())
            {
                ChangeState(BMSState::kCharging);
            }
        }
        if (inverter_voltage >= 230) // if inverter voltage is greater than 58V delta of our max voltage (588 V)
        {
            if (command_signal_ == Command::kPrechargeAndCloseContactors)
            {
                ChangeState(BMSState::kActive);
            }
        }
        break;
    case BMSState::kActive:
        Serial.println("Active");

        if (command_signal_ == Command::kShutdown || timeSinceLastCANRX >= 1000)
        {
            ChangeState(BMSState::kShutdown);
        }
        else if (charger_.IsConnected())
        {
            ChangeState(BMSState::kCharging);
        }
        break;
    case BMSState::kCharging:
        Serial.println("Charging");

        charger_.Tick(millis());

        if (!charger_.IsConnected() || command_signal_ == Command::kShutdown)
        {
            charger_.Disable();
            ChangeState(BMSState::kShutdown);
            break;
        }
        // cell balancing if charging
        bq_.ProcessBalancing(voltages_, kMaxChargeVoltage);
        // pause charging if danger of overvoltage
        if (max_cell_voltage_ >= kMaxChargeVoltage)
        {
            charger_.SetVoltageCurrent(kMaxChargeVoltage * kNumCellsSeries, 0);
            // pause charging, set current to 0
        }
        else
        {
            /*if (high_current_charging_)
            {
                charger_.SetMaxCurrent(14);
                charger_.SetMaxPower(240 * 20);
            }
            else
            {
                charger_.SetMaxCurrent(0.1);
                charger_.SetMaxPower(120 * 15);
            }*/
            charger_.SetVoltageCurrent(kMaxChargeVoltage * kNumCellsSeries, max_allowed_regen_current_);
        }
        // todo

        break;
    case BMSState::kFault:
        Serial.println("Fault State");
        Serial.println("Faults:");

        if (static_cast<bool>(overvoltage_fault_))
        {
            Serial.println("  Overvoltage");
        }
        else if (static_cast<bool>(undervoltage_fault_))
        {
            Serial.println("  Undervoltage");
        }
        if (static_cast<bool>(overtemperature_fault_))
        {
            Serial.println("  Overtemperature");
        }
        else if (static_cast<bool>(undertemperature_fault_))
        {
            Serial.println("  Undertemperature");
        }
        if (static_cast<bool>(overcurrent_fault_))
        {
            Serial.println("  Overcurrent");
        }
        if ((static_cast<bool>(external_kill_fault_) && current_state_ != BMSState::kShutdown))
        {
            Serial.println(" external error");
        }
        if (static_cast<bool>(open_wire_fault_))
        {
            Serial.println(" open wire");
        }
        Serial.println("");
        // check for clear faults command
        /* if (command_signal_ == Command::kClearFaults)     // only used for testing and debugging,
         {                                                   // for the love of god please do not leave this uncommented
             external_kill_fault_ = BMSFault::kNotFaulted;   // or else i will personally come and kiss your mother
             ChangeState(BMSState::kShutdown);
         }*/
        break;
    }
}

void BMS::ChangeState(BMSState new_state)
{
#if 1
    Serial.print("Changing state: ");
    Serial.println(new_state == BMSState::kShutdown    ? "Shutdown"
                   : new_state == BMSState::kPrecharge ? "Precharge"
                   : new_state == BMSState::kActive    ? "Active"
                   : new_state == BMSState::kFault     ? "Fault"
                                                       : "Other");
#endif
    state_entry_time_ = millis();
    switch (new_state)
    {
    case BMSState::kShutdown:
        digitalWrite(shutdown_LED, HIGH);
        digitalWrite(charging_LED, LOW);
        digitalWrite(precharge_LED, LOW);
        digitalWrite(active_LED, LOW);
        digitalWrite(internal_fault_LED, LOW);
        ShutdownCar();
        current_state_ = BMSState::kShutdown;
        break;
    case BMSState::kPrecharge:
        // LED CONTROL
        digitalWrite(shutdown_LED, LOW);
        digitalWrite(charging_LED, LOW);
        digitalWrite(precharge_LED, HIGH);
        digitalWrite(active_LED, LOW);
        digitalWrite(internal_fault_LED, LOW);

        digitalWrite(contactorp_ctrl, HIGH);
        delay(1);
        digitalWrite(contactorprecharge_ctrl, HIGH); // precharge, but don't turn on car yet
        current_state_ = BMSState::kPrecharge;
        break;
    case BMSState::kActive:
        digitalWrite(shutdown_LED, LOW);
        digitalWrite(charging_LED, LOW);
        digitalWrite(precharge_LED, LOW);
        digitalWrite(active_LED, HIGH);
        digitalWrite(internal_fault_LED, LOW);

        digitalWrite(contactorn_ctrl, HIGH);        // turn on car
        digitalWrite(contactorprecharge_ctrl, LOW); // disable precharge when car is running
        coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), state_entry_time_);
        current_state_ = BMSState::kActive;
        break;
    case BMSState::kCharging:
        // enable charger?
        {
            // LED CONTROL
            digitalWrite(shutdown_LED, LOW);
            digitalWrite(charging_LED, HIGH);
            digitalWrite(precharge_LED, LOW);
            digitalWrite(active_LED, LOW);
            digitalWrite(internal_fault_LED, LOW);

            digitalWrite(contactorp_ctrl, HIGH);        // turn on car
            digitalWrite(contactorprecharge_ctrl, LOW); // disable precharge when car is running
            coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), state_entry_time_);
            charger_.Enable();
            current_state_ = BMSState::kCharging;

            break;
        }
    case BMSState::kFault:
        // open contactors
        // LED CONTROL
        digitalWrite(shutdown_LED, LOW);
        digitalWrite(charging_LED, LOW);
        digitalWrite(precharge_LED, LOW);
        digitalWrite(active_LED, LOW);
        digitalWrite(internal_fault_LED, HIGH);

        ShutdownCar();
        digitalWrite(bms_sd_ctrl, LOW);
        digitalWrite(bms_status, LOW);
        current_state_ = BMSState::kFault;
        break;
    }
}

