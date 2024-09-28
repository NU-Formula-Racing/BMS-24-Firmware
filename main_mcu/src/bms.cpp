#include "bms.h"

#include <algorithm>
#include <numeric>

#include "I-Charger.h"
#include "can_interface.h"

int BMS::fault_pin_{-1};

void BMS::CheckFaults()
{
    // check for all the faults
    // latch the faults tho
    overvoltage_fault_ = static_cast<BMSFault>(max_cell_voltage_ >= kCellOvervoltage || overvoltage_fault_ == BMSFault::kFaulted);
    undervoltage_fault_ = static_cast<BMSFault>(min_cell_voltage_ <= kCellUndervoltage || undervoltage_fault_ == BMSFault::kFaulted);  
    overcurrent_fault_ = static_cast<BMSFault>(current_[0] >= kOvercurrent || overcurrent_fault_ == BMSFault::kFaulted);
    overtemperature_fault_ = static_cast<BMSFault>(max_cell_temperature_ >= kOvertemp || overtemperature_fault_ == BMSFault::kFaulted);
    undertemperature_fault_ = static_cast<BMSFault>(min_cell_temperature_ <= kUndertemp || undertemperature_fault_ == BMSFault::kFaulted);
    external_kill_fault_ = static_cast<BMSFault>(shutdown_input_.GetStatus() == ShutdownInput::InputState::kShutdown || external_kill_fault_ == BMSFault::kFaulted);

    fault_ =
        static_cast<BMSFault>(static_cast<bool>(overvoltage_fault_) || static_cast<bool>(undervoltage_fault_) || static_cast<bool>(overcurrent_fault_) || static_cast<bool>(overtemperature_fault_) || static_cast<bool>(undertemperature_fault_) || static_cast<bool>(open_wire_fault_) || (static_cast<bool>(external_kill_fault_) && current_state_ != BMSState::kShutdown));

    // // very very danagerous, but we are only listening for the external kill switch
    // external_kill_fault_ = static_cast<BMSFault>(shutdown_input_.GetStatus() == ShutdownInput::InputState::kShutdown || external_kill_fault_ == BMSFault::kFaulted);
    // fault_ = static_cast<BMSFault>(static_cast<bool>(external_kill_fault_) && current_state_ != BMSState::kShutdown);
}

static uint32_t number_of_ticks = 0;

void BMS::Tick()
{
    watchdog_timer.feed(); // so we don't reboot
    number_of_ticks++;
    Serial.println(number_of_ticks);
    Serial.println("Probe BQ");
    // check fault status
    if (fault_ != BMSFault::kNotFaulted && current_state_ != BMSState::kFault)
    {
#if serialdebug
        Serial.println("Faults:");
        if (static_cast<bool>(overvoltage_fault_))
        {
            Serial.println("  Overvoltage");
        }
        else if (static_cast<bool>(undervoltage_fault_))
        {
            Serial.println("  Undervoltage");
        }
        if (overtemperature_fault_)
        {
            Serial.println("  Overtemperature");
        }
        else if (undertemperature_fault_)
        {
            Serial.println("  Undertemperature");
        }
        if (overcurrent_fault_)
        {
            Serial.println("  Overcurrent");
        }
        Serial.println("");
#endif

        Serial.println("Changing state to fault");
        ChangeState(BMSState::kFault);
    }

    Serial.println("About to process state");
    ProcessState();

    // // update the signals in the hp_status_message_
    // Serial.println("Updating signals");
    // Serial.printf("State: %d\n", static_cast<int>(current_state_));
    // this->state_signal_ = (uint8_t)current_state_;
    // this->max_cell_temperature_signal_ = max_cell_temperature_;
    // this->min_cell_temperature_signal_ = min_cell_temperature_;
    // this->max_cell_voltage_signal_ = max_cell_voltage_;
    // this->min_cell_voltage_signal_ = min_cell_voltage_;
    // this->soc_signal_ = GetSOC();
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
    // return;
    // check temperatures
    Serial.println("Processing cooling");
    bq_.GetTemps(temperatures_);
    Serial.println("Got temperatures");
    max_cell_temperature_ = *std::max_element(temperatures_.begin(), temperatures_.end());
    min_cell_temperature_ = *std::min_element(temperatures_.begin(), temperatures_.end());
    average_cell_temperature_ = std::accumulate(temperatures_.begin(), temperatures_.end(), 0) / temperatures_.size();
}

void BMS::UpdateValues()
{
    // return;
    Serial.println("Start of UpdatedValues");
    // ProcessCooling();
    Serial.println("Processed Cooling");

    watchdog_timer.feed(); // so we don't reboot

    Serial.println("Getting Current");
    bq_.GetCurrent(current_);
    Serial.println("Got Current");

    watchdog_timer.feed(); // so we don't reboot

    Serial.println("Getting Voltages");
    bq_.GetVoltages(voltages_);
    Serial.println("Got Voltages");

    watchdog_timer.feed(); // so we don't reboot

    Serial.println("Getting Temperatures");
    max_cell_voltage_ = *std::max_element(voltages_.begin(), voltages_.end());
    min_cell_voltage_ = *std::min_element(voltages_.begin(), voltages_.end());
    Serial.println("End of UpdatedValues");

    // debug cell_v print
    // for (auto voltage : voltages_){ Serial.println(voltage); }

    CalculateSOE();

    watchdog_timer.feed(); // so we don't reboot

    Serial.println("Calculated SOE");
    if (!coulomb_count_.Initialized())
    {
        Serial.println("Initializing coulomb count");
        coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), millis());
    }
    else
    {
        Serial.println("Counting coulombs");
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
        // check for command to go to active
        if (command_signal_ == Command::kPrechargeAndCloseContactors)
        {
            ChangeState(BMSState::kPrecharge);
        }
        else if (charger_.IsConnected() && !(command_signal_ == Command::kShutdown))
        {
            Serial.println("Detected charger");
            ChangeState(BMSState::kPrecharge);
        }
        break;
    case BMSState::kPrecharge:
        // do a time-based precharge
        if (command_signal_ == Command::kShutdown)
        {
            ChangeState(BMSState::kShutdown);
        }
        if (millis() >= state_entry_time_ + kPrechargeTime)
        {
            if (charger_.IsConnected())
            {
                ChangeState(BMSState::kCharging);
            }
            else if (command_signal_ == Command::kPrechargeAndCloseContactors)
            {
                ChangeState(BMSState::kActive);
            }
            else
            {
                ChangeState(BMSState::kShutdown);
            }
        }

        break;
    case BMSState::kActive:
        if (command_signal_ == Command::kShutdown)
        {
            ChangeState(BMSState::kShutdown);
        }
        else if (charger_.IsConnected())
        {
            ChangeState(BMSState::kCharging);
        }
        break;
    case BMSState::kCharging:
        static constexpr float kMaxChargeVoltage{4.19f};

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
        // check for clear faults command
        if (command_signal_ == Command::kClearFaults)
        {
            external_kill_fault_ = BMSFault::kNotFaulted;
            ChangeState(BMSState::kShutdown);
        }
        break;
    }
}

void BMS::ChangeState(BMSState new_state)
{

#if serialdebug
    Serial.print("Changing state: ");
    Serial.println(new_state == BMSState::kShutdown    ? "Shutdown"
                   : new_state == BMSState::kPrecharge ? "Precharge"
                   : new_state == BMSState::kFault     ? "Fault"
                                                       : "Other");
#endif
    state_entry_time_ = millis();
    switch (new_state)
    {
    case BMSState::kShutdown:
        ShutdownCar();
        current_state_ = BMSState::kShutdown;
        break;
    case BMSState::kPrecharge:
        digitalWrite(contactorn_ctrl, HIGH);
        delay(1);
        digitalWrite(contactorprecharge_ctrl, HIGH); // precharge, but don't turn on car yet
        current_state_ = BMSState::kPrecharge;
        break;
    case BMSState::kActive:
        digitalWrite(contactorp_ctrl, HIGH);        // turn on car
        digitalWrite(contactorprecharge_ctrl, LOW); // disable precharge when car is running
        coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), state_entry_time_);
        current_state_ = BMSState::kActive;
        break;
    case BMSState::kCharging:
        // enable charger?
        {

            digitalWrite(contactorp_ctrl, HIGH);        // turn on car
            digitalWrite(contactorprecharge_ctrl, LOW); // disable precharge when car is running
            coulomb_count_.Initialize(cell.VoltageToSOC(min_cell_voltage_), state_entry_time_);
            charger_.Enable();
            current_state_ = BMSState::kCharging;

            break;
        }
    case BMSState::kFault:
        // open contactors
        Serial.println("Fault state");
        ShutdownCar();
        current_state_ = BMSState::kFault;
        break;
    }
}
