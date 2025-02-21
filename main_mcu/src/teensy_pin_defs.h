//good to go
#define contactorprecharge_ctrl 25 // contactor Precharge
#define contactorp_ctrl 26 // contactor Positive
#define contactorn_ctrl 20 // contactor Negative
// not used
#define cs_esp 39  // not used in NFR25
#define miso_esp 41 // not used in NFR25
#define mosi_esp 26 // not used in NFR25
#define sck_esp 27 // not used in NFR25
// needs to be developed/worked on
#define rx_esp 0 // uart to bq
#define tx_esp 1 // uart to bq
#define drive_crx 7 // drive can // can library handles this
#define drive_ctx 8 // drive can // can library handles this
#define data_crx 30 // data can // can library handles this
#define data_ctx 31 // data can // can library handles this
#define bms_sd_ctrl 21 // bms shutdown control
#define imd_status 27 // status of the imd, input to CAN
#define bms_status 38 // status of the bms, output to IMD
#define current_sense 22 // taking the differential from TSCB
#define charger_status 17 // charger status, usually pulled down
#define charger_sense 14

#define charging_LED 39 // sends signal to charging LED
#define shutdown_LED 37 // sends signal to shutdown LED
#define precharge_LED 36 // /sends singal to precharge LED 
#define active_LED 33 //sends signal if any external fault arises

#define internal_fault_LED 40 // sends signal if any internal fault arises
#define shutdown_signal_LED 19 // if bms shutdown is inputted, it lights up
#define bms_sd_LED 21 // this is the same thing as bms_sd_ctrl but named differently so that its matches LED(when u run into a fault cus of this, you're welcome, ill kiss u)
// const int kill_pins[]{14, 15, 18, 19, 24, 25, 40, 32, 9, 6, 2, 33};
// #define num_kill_pins 12






