#include<Arduino.h>

#define TINY_GSM_MODEM_SIM7000

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define DUMP_AT_COMMANDS
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60       /* ESP32 should sleep more seconds  (note SIM7000 needs ~20sec to turn off if sleep is activated) */

#define SerialAT serialGsm
#define SERIAL_DEBUG_BAUD 115200
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#define isNBIOT false

#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_DTR 25

#define MODEM_TX 26
#define MODEM_RX 27

#define I2C_SDA 21
#define I2C_SCL 22

#define PIN_ADC_BAT 35
#define PIN_ADC_SOLAR 36
#define ADC_BATTERY_LEVEL_SAMPLES 10
#define LED_PIN 12


const char user[] = "";
const char pass[] = "";

const char apn[] = "";
const char nbiot_apn[] = "";

const char USERNAME[] = "";
const char PASSWORD[] = "";
const char SERVER[] = "";