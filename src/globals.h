// The mother of all embedded development...
#include <Arduino.h>

// std::set for unified array functions
#include <set>
#include <array>
#include <algorithm>

#define VERBOSE                         1       // comment out to silence the device, for mute use build option

#include "macsniff.h"
#include "main.h"

#define HAS_DISPLAY U8X8_SSD1306_128X64_NONAME_HW_I2C
#define DISPLAY_FLIP  0//1 // rotated display

// OLED Display
#ifdef HAS_DISPLAY
#include <U8x8lib.h>
#endif

// GPS
#ifdef HAS_GPS
#include <TinyGPS++.h>
#endif

// set this to include BLE counting and vendor filter functions
#define VENDORFILTER                    1       // comment out if you want to count things, not people
#define BLECOUNTER                      1       // comment out if you don't want BLE count, saves power & memory

// BLE scan parameters
#define BLESTACKSIZE                    8192    // stack size for esp_bt_controller
#define BLESCANTIME                     0       // [seconds] scan duration, 0 means infinite [default], see note below
#define BLESCANWINDOW                   80      // [milliseconds] scan window, see below, 3 .. 10240, default 80ms
#define BLESCANINTERVAL                 80      // [illiseconds] scan interval, see below, 3 .. 10240, default 80ms = 100% duty cycle


#define WIFI_CHANNEL_MIN                1       // start channel number where scan begings
#define	WIFI_CHANNEL_MAX                13      // total channel number to scan
#define WIFI_MY_COUNTRY                 "ID"    // select locale for Wifi RF settings
#define	WIFI_CHANNEL_SWITCH_INTERVAL    50      // [seconds/100] -> 0,5 sec.


#define HAS_LED GPIO_NUM_15 // green on board LED (new board  ONLY)
#define LED_ACTIVE_LOW 1  // Onboard LED is active when pin is LOW

// Hardware pin definitions for TTGO V2 Board with OLED SSD1306 0,96" I2C Display
#define OLED_RST U8X8_PIN_NONE // connected to CPU RST/EN
#define OLED_SDA GPIO_NUM_21 // ESP32 GPIO21 -- SD1306 D1+D2
#define OLED_SCL GPIO_NUM_22 // ESP32 GPIO22 -- SD1306 D0

// OLED Display refresh cycle (in Milliseconds)
#define DISPLAYREFRESH_MS               40      // e.g. 40ms -> 1000/40 = 25 frames per second

// value for HSL color
// see http://www.workwithcolor.com/blue-color-hue-range-01.htm
#define COLOR_RED 0
#define COLOR_ORANGE 30
#define COLOR_ORANGE_YELLOW 45
#define COLOR_YELLOW 60
#define COLOR_YELLOW_GREEN 90
#define COLOR_GREEN 120
#define COLOR_GREEN_CYAN 165
#define COLOR_CYAN 180
#define COLOR_CYAN_BLUE 210
#define COLOR_BLUE 240
#define COLOR_BLUE_MAGENTA 275
#define COLOR_MAGENTA 300
#define COLOR_PINK 350
#define COLOR_WHITE 360
#define COLOR_NONE 999

extern configData_t cfg;
extern uint16_t macs_total, macs_wifi, macs_ble; // MAC counters
extern std::set<uint16_t> macs;
