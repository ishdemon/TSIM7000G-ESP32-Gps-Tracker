#include <config.h>
#include <TinyGsmClient.h>
#include <MQTT.h>
#include <ArduinoJson.h>
#include <Esp.h>
#include <SimpleTimer.h>

bool modemConnected = false;
bool modemReset = false;
RTC_DATA_ATTR int bootCount = 0;
char payload[256];
uint16_t v_bat, v_solar = 0;
float lat, lon, speed;
int alt, count = 0;
int16_t signal = 0;

HardwareSerial serialGsm(1);
#ifdef DUMP_AT_COMMANDS
#include "StreamDebugger.h"
StreamDebugger debugger(serialGsm, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(serialGsm);
#endif

// Initialize GSM client
TinyGsmClient client(modem);

MQTTClient mqttclient(256);
SimpleTimer timer;

void read_adc_bat(uint16_t *voltage);
void shutdown();
void modem_off();
void enableGPS();
void disableGPS();
void sendPayload();
void connectMqtt();
void connectModem();
void checkUSB();
void getGPS();

void read_adc_bat(uint16_t *voltage)
{
  uint32_t in = 0;
  for (int i = 0; i < ADC_BATTERY_LEVEL_SAMPLES; i++)
  {
    in += (uint32_t)analogRead(PIN_ADC_BAT);
  }
  in = (int)in / ADC_BATTERY_LEVEL_SAMPLES;
  uint16_t bat_mv = (((float)in / 4096) * 3600 * 2);
  *voltage = bat_mv;
}

void enableGPS(void)
{
  // Set SIM7000G GPIO4 LOW ,turn on GPS power
  // CMD:AT+SGPIO=0,4,1,1
  // Only in version 20200415 is there a function to control GPS power
  Serial.println("Turning on GPS  power");
  modem.sendAT("+SGPIO=0,4,1,1"); // V20200415 set GPIO4 HIGH to turn on GPS power
  modem.enableGPS();
  delay(1000);
}

void disableGPS(void)
{
  // Set SIM7000G GPIO4 LOW ,turn off GPS power
  // CMD:AT+SGPIO=0,4,1,0
  // Only in version 20200415 is there a function to control GPS power
  modem.disableGPS();
  Serial.println("Turning off GPS power");
  modem.sendAT("+SGPIO=0,4,1,0"); // V20200415 set GPIO4 LOW to turn off GPS power
  delay(1000);
}

void modem_reset()
{
  Serial.println("Modem hardware reset");
  modemReset = true;
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, LOW);
  delay(260); //Treset 252ms
  digitalWrite(MODEM_RST, HIGH);
  delay(5000); //Modem takes longer to get ready and reply after this kind of reset vs power on

  //modem.factoryDefault();
  //modem.restart(); //this results in +CGREG: 0,0
}

void modem_on()
{
  // Set-up modem  power pin
  pinMode(MODEM_PWKEY, OUTPUT);
  digitalWrite(MODEM_PWKEY, HIGH);
  delay(10);
  digitalWrite(MODEM_PWKEY, LOW);
  delay(1010); //Ton 1sec
  digitalWrite(MODEM_PWKEY, HIGH);

  //wait_till_ready();
  //Serial.println("Waiting till modem ready...");
  //delay(5000); //Ton uart 4.5sec but seems to need ~7sec after hard (button) reset
  //On soft-reset serial replies immediately.
}

void modem_off()
{
  //if you turn modem off while activating the fancy sleep modes it takes ~20sec, else its immediate
  Serial.println("Going to sleep now with modem turned off");
  //modem.gprsDisconnect();
  //modem.radioOff();
  modem.sleepEnable(false); // required in case sleep was activated and will apply after reboot
  modem.poweroff();
}

// fancy low power mode - while connected
void modem_sleep() // will have an effect after reboot and will replace normal power down
{
  Serial.println("Going to sleep now with modem in power save mode");
  // needs reboot to activa and takes ~20sec to sleep
  modem.PSM_mode();    //if network supports will enter a low power sleep PCM (9uA)
  modem.eDRX_mode14(); // https://github.com/botletics/SIM7000-LTE-Shield/wiki/Current-Consumption#e-drx-mode
  modem.sleepEnable(); //will sleep (1.7mA), needs DTR or PWRKEY to wake
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, HIGH);
}

void modem_wake()
{
  Serial.println("Wake up modem from sleep");
  // DTR low to wake serial
  pinMode(MODEM_DTR, OUTPUT);
  digitalWrite(MODEM_DTR, LOW);
  delay(50);
  //wait_till_ready();
}

void sleep()
{

  if (!modemReset)
    modem_sleep();
  //modem_off();

  delay(1000);
  Serial.flush();
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  delay(1000);
  Serial.flush();
  esp_deep_sleep_start();
}

void shutdown()
{
  sleep();
}

void sendPayload()
{
  if (lat != 0)
  {
    StaticJsonDocument<256> gpsdata;
    gpsdata["latitude"] = lat;
    gpsdata["longitude"] = lon;
    gpsdata["speed"] = speed;
    gpsdata["altitude"] = alt;
    read_adc_bat(&v_bat);
    gpsdata["battery"] = v_bat;
    gpsdata["rssi"] = signal;
    serializeJson(gpsdata, payload);
    Serial.println("Sending data...");
    mqttclient.publish("/track/gpsdevice1/gps", payload, true, 1);
    Serial.println("Data Sent");
  }
}

void checkUSB()
{
  if (analogRead(PIN_ADC_BAT) != 0)
  {
    ESP.restart();
  }
}

void connectMqtt()
{
  if (!mqttclient.connected() && modemConnected)
  {
    Serial.print("Connecting to MQTT: ");
    if (!mqttclient.connect("", USERNAME, PASSWORD))
    {
      delay(1000);
      Serial.println("Failed to connect");
      modem_reset();
      shutdown();
    }
  }
}

void connectModem()
{
  modem.setPreferredMode(13); //2 Auto // 13 GSM only // 38 LTE only
  Serial.print(F("Waiting for network..."));
  if (!modem.waitForNetwork(30000L))
  {
    Serial.println(" fail");
    modem_reset();
    shutdown();
  }
  Serial.println(" OK");

  Serial.print("Signal quality:");
  Serial.println(modem.getSignalQuality());
  signal = modem.getSignalQuality();

  Serial.print(F("Connecting to "));
  Serial.print(apn);
  modem.gprsConnect(apn, user, pass);
  if (!modem.isGprsConnected())
  {
    Serial.println(" failed");
    modem_reset();
    shutdown();
  }
  modemReset = false;
  modemConnected = true;
  Serial.println(" GSM OK");
}

void modemRestart()
{
  modem_off();
  delay(1000);
  modem_on();
}

void getGPS()
{
  lat = 0;
  lon = 0;
  if (!modem.testAT())
  {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    //shutdown();
    ESP.restart();
    //modemRestart();
    return;
  }

  enableGPS();
  long startTime = millis();
  Serial.println("Scanning for location for 1 mins");
  while (millis() - startTime <= 30000)
  { // scans for 1 min
    if (modem.getGPS(&lat, &lon, &speed, &alt))
    {
      Serial.printf("lat:%f lon:%f\n", lat, lon);
      break;
    }
    delay(1000);
  }
  if (lat == 0)
  {
    Serial.println("Failed to get location");
    //modem_reset();
    //shutdown();
    //modem_reset();
    ESP.restart();
  }
  disableGPS();
}

void setup()
{
  esp_sleep_enable_timer_wakeup(10 * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                 " Seconds");

  modemReset = false;
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(PIN_ADC_BAT, INPUT);
  pinMode(PIN_ADC_SOLAR, INPUT);
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
  {
    modem_wake();
  }
  else
  {
    //modem_reset();
    modem_on();
    // modem_wake();
    //modem_reset();
  }
  SerialAT.begin(115200, SERIAL_8N1, MODEM_TX, MODEM_RX, false);
  //delay(10000);
  getGPS();
  if (!modemConnected)
    connectModem();
  mqttclient.begin(SERVER, client);
  mqttclient.setWill("/gpsdevice1/status", "offline");
  mqttclient.setKeepAlive(5);
  connectMqtt();
  if (mqttclient.connected())
  {
    sendPayload();
  }
  shutdown();
}

void loop()
{
}