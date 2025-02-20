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
// const int kill_pins[]{14, 15, 18, 19, 24, 25, 40, 32, 9, 6, 2, 33};
// #define num_kill_pins 12






