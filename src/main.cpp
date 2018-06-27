#include <Arduino.h>
// Basic Config
#include "globals.h"
#include "esp_log.h"

// ESP32 lib Functions
#include <esp32-hal-log.h>  // needed for ESP_LOGx on arduino framework
#include <esp_event_loop.h> // needed for Wifi event handler
#include <esp_spi_flash.h>  // needed for reading ESP32 chip attributes

// Initialize global variables
configData_t cfg; // struct holds current device configuration
uint8_t DisplayState = 0;   // globals for state machine
uint16_t macs_total = 0, macs_wifi = 0,
         macs_ble = 0; // MAC counters globals for display
uint8_t wifiChannel = 0;   // wifi channel rotation counter global for display

std::set<uint16_t> macs; // associative container holds total of unique MAC
                         // adress hashes (Wifi + BLE)

portMUX_TYPE timerMux =
    portMUX_INITIALIZER_UNLOCKED; // sync main loop and ISR when modifying IRQ
                                  // handler shared variables

led_states LEDState = LED_OFF; // LED state global for state machine
led_states previousLEDState =
    LED_ON;                        // This will force LED to be off at boot since State is OFF
unsigned long LEDBlinkStarted = 0; // When (in millis() led blink started)
uint16_t LEDBlinkDuration = 0;     // How long the blink need to be
uint16_t LEDColor = COLOR_NONE;    // state machine variable to set RGB LED color

hw_timer_t *displaytimer =
    NULL; // configure hardware timer used for cyclic display refresh

static volatile int DisplayTimerIRQ = 0;

void blink_LED(uint16_t set_color, uint16_t set_blinkduration)
{
    LEDColor = set_color;                 // set color for RGB LED
    LEDBlinkDuration = set_blinkduration; // duration
    LEDBlinkStarted = millis();           // Time Start here
    LEDState = LED_ON;                    // Let main set LED on
}

void led_loop()
{
    // Custom blink running always have priority other LoRaWAN led management
    if (LEDBlinkStarted && LEDBlinkDuration)
    {
        // ESP_LOGI(TAG, "Start=%ld for %g",LEDBlinkStarted, LEDBlinkDuration );

        // Custom blink is finished, let this order, avoid millis() overflow
        if ((millis() - LEDBlinkStarted) >= LEDBlinkDuration)
        {
            // Led becomes off, and stop blink
            LEDState = LED_OFF;
            LEDBlinkStarted = 0;
            LEDBlinkDuration = 0;
            LEDColor = COLOR_NONE;
        }
        else
        {
            // In case of LoRaWAN led management blinked off
            LEDState = LED_ON;
        }

        // No custom blink, check LoRaWAN state
    }

    // ESP_LOGI(TAG, "state=%d previous=%d Color=%d",LEDState, previousLEDState,LEDColor );

    // led need to change state? avoid digitalWrite() for nothing
    if (LEDState != previousLEDState) {
        if (LEDState == LED_ON)
        {
            //   rgb_set_color(LEDColor);
#ifdef LED_ACTIVE_LOW
            digitalWrite(HAS_LED, LOW);
#else
            digitalWrite(HAS_LED, HIGH);
#endif
        }
        else
        {
            //   rgb_set_color(COLOR_NONE);
#ifdef LED_ACTIVE_LOW
            digitalWrite(HAS_LED, HIGH);
#else
            digitalWrite(HAS_LED, LOW);
#endif
        }
        previousLEDState = LEDState;
  }
}; // led_loop()

// populate cfg vars with factory settings
void defaultConfig()
{
    //cfg.lorasf = LORASFDEFAULT; // 7-12, initial lora sf, see pacounter.conf
    cfg.txpower = 15;    // 2-15, lora tx power
    cfg.adrmode = 1;     // 0=disabled, 1=enabled
    cfg.screensaver = 0; // 0=disabled, 1=enabled
    cfg.screenon = 1;    // 0=disabled, 1=enabled
    cfg.countermode = 0; // 0=cyclic, 1=cumulative, 2=cyclic confirmed
    cfg.rssilimit = 0;   // threshold for rssilimiter, negative value!
    //cfg.sendcycle = SEND_SECS;  // payload send cycle [seconds/2]
    cfg.wifichancycle =
        WIFI_CHANNEL_SWITCH_INTERVAL; // wifi channel switch cycle [seconds/100]
    cfg.blescantime =
        BLESCANINTERVAL /
        10;               // BT channel scan cycle [seconds/100], default 1 (= 10ms)
    cfg.blescan = 1;      // 0=disabled, 1=enabled
    cfg.wifiant = 0;      // 0=internal, 1=external (for LoPy/LoPy4)
    cfg.vendorfilter = 1; // 0=disabled, 1=enabled
    //cfg.rgblum = RGBLUMINOSITY; // RGB Led luminosity (0..100%)
    cfg.gpsmode = 1; // 0=disabled, 1=enabled

    strncpy(cfg.version, PROGVERSION, sizeof(cfg.version) - 1);
}

#ifdef HAS_DISPLAY
HAS_DISPLAY u8x8(OLED_RST, OLED_SCL, OLED_SDA);
// Display Refresh IRQ
void IRAM_ATTR DisplayIRQ() {
  portENTER_CRITICAL_ISR(&timerMux);
  DisplayTimerIRQ++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
#endif

#ifdef HAS_DISPLAY

/*
// Print a key on display
void DisplayKey(const uint8_t *key, uint8_t len, bool lsb) {
  const uint8_t *p;
  for (uint8_t i = 0; i < len; i++) {
    p = lsb ? key + len - i - 1 : key + i;
    u8x8.printf("%02X", *p);
  }
  u8x8.printf("\n");
}
*/

void init_display(const char *Productname, const char *Version) {
  uint8_t buf[32];
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.setFlipMode(0);
  u8x8.setInverseFont(1);
  u8x8.draw2x2String(0, 0, Productname);
  u8x8.setInverseFont(0);
  u8x8.draw2x2String(0, 2, Productname);
  delay(1500);
  u8x8.clear();
  u8x8.setFlipMode(1);
  u8x8.setInverseFont(1);
  u8x8.draw2x2String(0, 0, Productname);
  u8x8.setInverseFont(0);
  u8x8.draw2x2String(0, 2, Productname);
  delay(1500);

  u8x8.setFlipMode(0);
  u8x8.clear();

// #ifdef DISPLAY_FLIP
//   u8x8.setFlipMode(1);
// #endif

// Display chip information
#ifdef VERBOSE
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  u8x8.printf("ESP32 %d cores\nWiFi%s%s\n", chip_info.cores,
              (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
              (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
  u8x8.printf("ESP Rev.%d\n", chip_info.revision);
  u8x8.printf("%dMB %s Flash\n", spi_flash_get_chip_size() / (1024 * 1024),
              (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "int." : "ext.");
#endif // VERBOSE

  u8x8.print(Productname);
  u8x8.print(" v");
  u8x8.println(PROGVERSION);
//   u8x8.println("DEVEUI:");
//   os_getDevEui((u1_t *)buf);
//   DisplayKey(buf, 8, true);
  delay(5000);
  u8x8.clear();
}

void refreshDisplay() {
  // update counter (lines 0-1)
  char buff[16];
  snprintf(
      buff, sizeof(buff), "HERD:%-4d",
      (int)macs.size()); // convert 16-bit MAC counter to decimal counter value
  u8x8.draw2x2String(0, 0,
                     buff); // display number on unique macs total Wifi + BLE

  // update GPS status (line 2)
#ifdef HAS_GPS
  u8x8.setCursor(7, 2);
  if (!gps.location.isValid()) // if no fix then display Sats value inverse
  {
    u8x8.setInverseFont(1);
    u8x8.printf("Sats: %.3d", gps.satellites.value());
    u8x8.setInverseFont(0);
  } else
    u8x8.printf("Sats: %.3d", gps.satellites.value());
#endif

    // update bluetooth counter + LoRa SF (line 3)
#ifdef BLECOUNTER
  u8x8.setCursor(0, 3);
  if (cfg.blescan)
    u8x8.printf("BLTH:%-4d", macs_ble);
  else
    u8x8.printf("%s", "BLTH:off");
#endif
//   u8x8.setCursor(11, 3);
//   u8x8.printf("SF:");
//   if (cfg.adrmode) // if ADR=on then display SF value inverse
//     u8x8.setInverseFont(1);
//   u8x8.printf("%c%c", lora_datarate[LMIC.datarate * 2],
//               lora_datarate[LMIC.datarate * 2 + 1]);
//   if (cfg.adrmode) // switch off inverse if it was turned on
//     u8x8.setInverseFont(0);

  // update wifi counter + channel display (line 4)
  u8x8.setCursor(0, 4);
  u8x8.printf("WIFI:%-4d", macs_wifi);
  u8x8.setCursor(11, 4);
  u8x8.printf("ch:%02d", wifiChannel);

  // update RSSI limiter status & free memory display (line 5)
  u8x8.setCursor(0, 5);
  u8x8.printf(!cfg.rssilimit ? "RLIM:off " : "RLIM:%-4d", cfg.rssilimit);
  u8x8.setCursor(10, 5);
  u8x8.printf("%4dKB", ESP.getFreeHeap() / 1024);

//   // update LoRa status display (line 6)
//   u8x8.setCursor(0, 6);
//   u8x8.printf("%-16s", display_lora);

//   // update LMiC event display (line 7)
//   u8x8.setCursor(0, 7);
//   u8x8.printf("%-16s", display_lmic);
}

void updateDisplay() {
  // refresh display according to refresh cycle setting
  if (DisplayTimerIRQ) {
    portENTER_CRITICAL(&timerMux);
    DisplayTimerIRQ--;
    portEXIT_CRITICAL(&timerMux);

    refreshDisplay();

    // set display on/off according to current device configuration
    if (DisplayState != cfg.screenon) {
      DisplayState = cfg.screenon;
      u8x8.setPowerSave(!cfg.screenon);
    }
  }
} // updateDisplay()
#endif // HAS_DISPLAY

void setup()
{
    // put your setup code here, to run once:
    char features[64] = "";
    
// setup debug output or silence device
#ifdef VERBOSE
    Serial.begin(115200);
    esp_log_level_set("*", ESP_LOG_VERBOSE);
#else
    // mute logs completely by redirecting them to silence function
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_set_vprintf(redirect_log);
#endif

    ESP_LOGI(TAG, "Starting %s %s", PROGNAME, PROGVERSION);

    // initialize system event handler for wifi task, needed for
    // wifi_sniffer_init()
    esp_event_loop_init(NULL, NULL);

    // print chip information on startup if in verbose mode
#ifdef VERBOSE
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG,
             "This is ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision "
             "%d, %dMB %s Flash",
             chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             chip_info.revision, spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                           : "external");
    ESP_LOGI(TAG, "ESP32 SDK: %s", ESP.getSdkVersion());
#endif

    defaultConfig();

#if (HAS_LED != NOT_A_PIN)
    pinMode(HAS_LED, OUTPUT);
#endif

#ifdef HAS_DISPLAY
    strcat_P(features, " OLED");
    // initialize display
    init_display(PROGNAME, PROGVERSION);
    DisplayState = cfg.screenon;
    u8x8.setPowerSave(!cfg.screenon); // set display off if disabled
    u8x8.draw2x2String(0, 0, "HERD:0");
#ifdef BLECOUNTER
    u8x8.setCursor(0, 3);
    u8x8.printf("BLTH:0");
#endif
    u8x8.setCursor(0, 4);
    u8x8.printf("WIFI:0");
    u8x8.setCursor(0, 5);
    u8x8.printf(!cfg.rssilimit ? "RLIM:off " : "RLIM:%d", cfg.rssilimit);

    // sprintf(display_lora, "Join wait");

    // setup display refresh trigger IRQ using esp32 hardware timer 0
    // for explanation see
    // https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
    displaytimer = timerBegin(0, 80, true); // prescaler 80 -> divides 80 MHz CPU
                                            // freq to 1 MHz, timer 0, count up
    timerAttachInterrupt(displaytimer, &DisplayIRQ,
                         true); // interrupt handler DisplayIRQ, triggered by edge
    timerAlarmWrite(
        displaytimer, DISPLAYREFRESH_MS * 1000,
        true);                      // reload interrupt after each trigger of display refresh cycle
    timerAlarmEnable(displaytimer); // enable display interrupt
#endif

    if (cfg.blescan)
    {
        start_BLEscan();
    }
}

void loop()
{

#if (HAS_LED != NOT_A_PIN) || defined(HAS_RGB_LED)
    led_loop();
#endif

#ifdef HAS_DISPLAY
    updateDisplay();
#endif

    // put your main code here, to run repeatedly:
    vTaskDelay(1 / portTICK_PERIOD_MS); // reset watchdog
}