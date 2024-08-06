// Device Password (will be written to flash)
#define DEVICE_PASSWORD "12345678"
// Device software version number
#define SOFTWARE_VERSION "V0.11.7"
// production date of device (yy-mm-dd)
#define PRODUCTION_DATE "24-03-13"
// Debugs
#define DEBUG 1
// bootloader select
#define WITHOUT_BOOTLOADER 0
// vrms multiplier value
#define VRMS_MULTIPLICATION_VALUE 150

// DEBUG MACRO
#if DEBUG
#define PRINTF(x, ...) printf(x, ##__VA_ARGS__)
#else
#define PRINTF(x, ...)
#endif
