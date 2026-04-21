/*
  Zigbee LIDAR Distance Sensor using VL53L1X + ESP32-C6 (End Device Mode)

=================================================================================
───────────────────────────────── Board Selection ───────────────────────────────
=================================================================================
        Combined sketch: ESP32-C6 Qwiic Pocket (EQP) and Arduino Nesso N1

Uncomment exactly *ONE* of the following lines before compiling.
Ensure Arduino IDE Tools settings match the selected board (see header below).
*/
          #define BOARD_EQP           // ESP32-C6 Qwiic Pocket
        //#define BOARD_NESSO_N1      // Arduino Nesso N1
/*
=================================================================================
=================================================================================
SET_LOOP_TASK_STACK_SIZE MUST be the very first executable statement —
before any global constructors (M5GFX display, Zigbee endpoints, etc.).
The #ifdef guard is evaluated at compile time, so this is safe to leave here.
*/
#ifdef BOARD_NESSO_N1
  SET_LOOP_TASK_STACK_SIZE(24 * 1024);   //Required for M5GFX on Nesso N1
#endif

#ifdef BOARD_EQP
  #define VERSION             "1.9 EQP OTA"
  #define MODEL_VERSION_TOKEN "1.9 EQP_OTA"
#endif
#ifdef BOARD_NESSO_N1
  #define VERSION             "1.9 N1 OTA"
  #define MODEL_VERSION_TOKEN "1.9 N1_OTA"
#endif

/* Author:  John Land
 * Date:    2026-04-08
 *
 * Designed for Zigbee integration with Hubitat.
 *
 * This sketch supports two hardware targets selected at compile time
 * via the BOARD_EQP / BOARD_NESSO_N1 define above:
 *
 *  ── BOARD_EQP ────────────────────────────────────────────────────────────
 *  Hardware:
 *   - ESP32-C6 Qwiic Pocket [EQP] from SparkFun, 4 MB Flash, no display
 *   - M5Stack Time-of-Flight Distance Unit (VL53L1X) via Qwiic connector
 *   - Grove to Qwiic Cable Adapter (100mm) from SparkFun
 *  Hardware connections for VL53L1X:
 *   - Connected via Qwiic connector (Wire1)
 *   - SDA -> GPIO 6, SCL -> GPIO 7, VCC -> 3.3V, GND -> GND
 *  Arduino IDE Tool settings:
 *   - Board:              ESP32-C6 Qwiic Pocket
 *   - Erase All Flash:    ENABLED (until Zigbee pairing, then DISABLED for Zigbee stability)
 *   - Partition Scheme:   "Custom"[uses partitions.csv]
 *   - Zigbee Mode:        "Zigbee ED (end device)"
 *  Special features:
 *   - Hold BOOT button for BUTTON_HOLD_TIME seconds: factory reset
 *   - Power-cycle OTA entry: plug in, watch for 6s-cycle LED blink,
 *     unplug within OTA_WINDOW_MS seconds (initially 15s), plug in again
 *
 *  ── BOARD_NESSO_N1 ───────────────────────────────────────────────────────
 *  Hardware:
 *   - Arduino Nesso N1 (ESP32-C6 @ 160 MHz, 16 MB Flash,
 *     co-developed with M5Stack) with Qwiic/Grove connector
 *   - M5Stack Time-of-Flight Distance Unit (VL53L1X) with Qwiic/Grove connector
 *  Hardware connections for VL53L1X:
 *   - Connected via Qwiic connector (Wire / I2C0)
 *   - SDA -> GPIO 10, SCL -> GPIO 8, VCC -> 3.3V, GND -> GND
 *     (per TPX00227 pinout p.3; same bus used by M5GFX, FT6336 touch, BMI270)
 *  Arduino IDE Tool settings:
 *   - Board:              "Arduino Nesso N1"
 *   - Erase All Flash:    ENABLED (until Zigbee pairing, then DISABLED for Zigbee stability)
 *   - Partition Scheme:   "Nesso N1 with OTA (4MB Custom)"
 *   - Zigbee Mode:        "Zigbee ED (end device)"
 *  Special features:
 *   - Short press Front Button: cycle display mode (0→1→2→3→0)
 *   - Long press Front Button:  enter OTA mode
 *   - Short press Side Button:  reboot (returns to normal Zigbee operation)
 *   - Long press Side Button:   factory reset & re-pairing (wipes Zigbee NVS)
 *   - Live distance readings on built-in 1.14" LCD
 *
 *  ── Common ───────────────────────────────────────────────────────────────
 *  Required third-party libraries (both boards):
 *   - "SparkFun VL53L1X 4m Laser Distance Sensor" by SparkFun Electronics
 *   - "Zigbee.h"          included with the ESP32 Arduino core
 *   - "esp_zigbee_core.h" included with the ESP32 Arduino core
 *   - "ElegantOTA"        for Over-The-Air firmware updates
 *
 *  Required third-party libraries (BOARD_NESSO_N1 only):
 *   - "M5GFX"             display/touch — autodetects Nesso N1 hardware
 *   - "Arduino_Nesso_N1"  Front Button/Side Button, LED_BUILTIN, POWEROFF pins
 *
 *  OTA firmware update:
 *   - Copy "secrets.h.example" to "secrets.h" and fill in WiFi credentials
 *   - Normal mode is Zigbee-only for reliability on the ESP32-C6
 *   - Use the rapid power-cycle OTA entry method (or Front Button on Nesso N1)
 *
 *  VL53L1X Specifications:
 *   - Range:    40mm to 4000mm (4 meters)
 *   - Accuracy: ±5% (±200mm at 4 meters) at best
 *   - FOV:      27° (~2m circle at 4m, ~1.44m circle at 3m)
 *
 *  Pairing with Hubitat seems to ***REQUIRE*** the "Double Luck Voodoo" process
 *  (search the Hubitat forums).
 *
 *  Dedicated to the public domain in 2025–2026.
 *
 *  Unless required by applicable law or agreed to in writing, this software is
 *  distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 *  either express or implied.
 */

#ifndef ZIGBEE_MODE_ED
  #error "Zigbee End Device mode is not selected! Go to Tools → Zigbee Mode → Zigbee ED (end device)"
#endif

//***********************************************
//INCLUDES
//***********************************************

#ifdef BOARD_NESSO_N1
  #include <M5GFX.h>                      //Display/touch — autodetects Nesso N1 hardware
  #include <Arduino_Nesso_N1.h>           //Front Button/Side Button, LED_BUILTIN, POWEROFF expander pins
#endif
#include <WebServer.h>                    //Used for ElegantOTA web server
#include <ElegantOTA.h>                   //Used for Over-The-Air (OTA) firmware updates
#include <WiFi.h>                         //Enables WiFi communication
#include "SparkFun_VL53L1X.h"             //SparkFun VL53L1X 4m Laser Distance Sensor library
#include <Wire.h>                         //I2C access for the VL53L1X sensor
#include "Zigbee.h"                       //Zigbee communications to Hubitat
#include "esp_zigbee_core.h"              //Lower-level Zigbee commands; improves pairing to Hubitat
#include "esp_coexist.h"                  //Radio coexistence preference (Zigbee vs WiFi)
#include "secrets.h"                      //WiFi SSID/password and optional OTA password
#include <Preferences.h>                  //Non-volatile storage for OTA mode flag and settings

//***********************************************
//PIN DEFINITIONS
//***********************************************
#ifdef BOARD_EQP
  #define OUTPUT_PIN        8             //Optional: EQP digital output pin to signal another device
  #define LED_PIN           LED_BUILTIN   //Status LED (onboard LED)
  #define SENSOR_SDA        6             //EQP Qwiic SDA pin
  #define SENSOR_SCL        7             //EQP Qwiic SCL pin
  #define BUTTON_PIN        BOOT_PIN      //EQP pairing/reset button
  #define BUTTON_HOLD_TIME  3000          //Hold BOOT button 3 seconds for factory reset & pairing
  #define QWIIC_CONNECTED   true          //CRITICAL: true = use Wire1 (Qwiic), false = use Wire
#endif

#ifdef BOARD_NESSO_N1
  //GPIO 8 is the Qwiic SCL pin (I2C0, shared with M5GFX); GPIO 9 is free; set to -1 to disable
  #define OUTPUT_PIN        9             //Optional: free GPIO for external signal; -1 to disable
  #define LED_PIN           LED_BUILTIN   //Green side LED (PI4IOE E1.P7 via Arduino_Nesso_N1)
  #define FRONT_BUTTON      KEY1          //Front button (PI4IOE E0.P0, active LOW)
  #define SIDE_BUTTON       KEY2          //Side button  (PI4IOE E0.P1, active LOW)
  #define BUTTON_HOLD_TIME  3000          //Hold time in ms for long-press actions on either button
  #define BUZZER_PIN        11            //G11 passive buzzer
  #define SENSOR_SDA        10            //Nesso N1 Qwiic SDA (GPIO10) — per TPX00227 pinout p.3
  #define SENSOR_SCL        8             //Nesso N1 Qwiic SCL (GPIO8)  — per TPX00227 pinout p.3

  //── Display Dimensions (landscape after setRotation(1)) ─────────────────────
  #define DISPLAY_W  240
  #define DISPLAY_H  135
  #define OTA_WINDOW_FLASH_MS  1000       //Display flash interval during OTA_WINDOW_MS power-cycle window (ms per red/black phase)

  //── Display Output Mode ───────────────────────────────────────────────────────
  //  0 = Normal : header + avg (green) + raw (yellow) + footer  [default layout]
  //  1 = Large  : Avg only, full screen, TFT_GREEN, largest built-in font
  //  2 = Large  : Raw only, full screen, TFT_YELLOW, largest built-in font
  //  3 = Large  : Avg (green) alternating with Raw (yellow) every LARGE_DISPLAY_ALT_S seconds
  #define LARGE_DISPLAY_ALT_S  10   //Mode 3 only: seconds between alternating avg/raw distance values
#endif

//***********************************************
//SENSOR, ZIGBEE, AND OTA DEFINES (common)
//***********************************************
#define OTA_WINDOW_MS              15000        //15s power-cycle OTA detection window (ms); also used for
                                                //the display flash duration and the LED SOS pattern.
#define SENSOR_READ_INTERVAL_MS 10000 //Distance sensor read interval; 10s keeps Zigbee traffic moderate
#define DISTANCE_ENDPOINT_NUMBER  10  //Zigbee endpoint for smoothed distance (Illuminance cluster)
#define RAW_ENDPOINT_NUMBER       11  //Zigbee endpoint for raw distance (Illuminance cluster)
#define WIFI_MODE_ENDPOINT_NUMBER 12  //Zigbee endpoint for the WiFi mode switch (On/Off cluster)
#define EMA_ALPHA 0.6                 //EMA smoothing fraction for distance readings: 0 < alpha < 1
   /*
    * Examples of when to use SMOOTHED data:
    * - Monitoring parking spaces (slow-changing)
    * - Tank level monitoring (slow-changing)
    * - Dashboard display (want clean values)
    * - Don't need instant response
    * - Want to avoid false triggers from noise
    * - Prefer fewer Zigbee messages
    *
    * Examples of when to use RAW data:
    * - Detecting hand gestures or quick movements
    * - Triggering immediate actions (e.g., turn on light when approaching)
    * - Need instant response
    * - Can tolerate some value jitter
    * - Using short presence threshold timeouts
    */


#define DISTANCE_LONG_MODE        true //true = Long range (4 m); false = Short range (1.3 m)
#define INTERMEASUREMENT_PERIOD   1000 //Time between measurements in ms; must be >= TIMING_BUDGET
#define TIMING_BUDGET             200  //Measurement integration time in ms (15/20/33/50/100/200/500)

/*
    The VL53L1X timing budget can be set from 20 ms up to 1000 ms. Increasing the timing budget increases
    the maximum distance the device can range and improves repeatability error; average power increases.
      * 20 ms is the minimum timing budget, ONLY usable in Short distance mode.
      * 33 ms is the minimum timing budget that works for all distance modes.
      * 140 ms allows the maximum distance of 4m to be reached in Long distance mode.
      NOTE: the SparkFun library only accepts these values: 15, 20, 33, 50, 100 (default), 200, 500.
      The max value for the intermeasurement period *may be* 1000ms; larger numbers give whacky values.
  */

#ifdef BOARD_EQP
  static const char ZB_MANUFACTURER[] = "Espressif";
#endif
#ifdef BOARD_NESSO_N1
  static const char ZB_MANUFACTURER[] = "Arduino";
#endif
static String zbModelString = String("LIDAR_Sensor_v") + VERSION;  //Persistent string so Basic cluster model reads remain valid
  //The total length of zbModelString must be <= 32 characters to fit in the Zigbee Basic cluster model attribute,
  //which is used by Hubitat during pairing to identify the device type and assign the correct driver.
  //Exceeding 32 silently prevents setManufacturerAndModel() from working on ALL endpoints, breaking firmware reporting.
  //Truncate or simplify as needed while keeping it unique across your devices.

//***********************************************
//PREFERENCES & OBJECTS (common)
//***********************************************
Preferences preferences;                                         //Non-volatile storage for OTA mode and settings

ZigbeeIlluminanceSensor zbDistance(DISTANCE_ENDPOINT_NUMBER);    //Endpoint 10: smoothed distance
ZigbeeIlluminanceSensor zbRaw(RAW_ENDPOINT_NUMBER);              //Endpoint 11: raw (unsmoothed) distance
ZigbeeLight             zbWiFiSwitch(WIFI_MODE_ENDPOINT_NUMBER); //Endpoint 12: On/Off — On triggers WiFi-only mode

SFEVL53L1X distanceSensor;                                       //VL53L1X TOF sensor instance

#ifdef BOARD_NESSO_N1
  M5GFX display;                                                 //Autodetects Nesso N1 hardware
#endif

WebServer server(80);                                            //Web server for ElegantOTA on port 80

using ulong = unsigned long;                                     //Alias for unsigned long

//***********************************************
//GLOBAL STATE VARIABLES (common)
//***********************************************
float averageDistance          = 0.0;                           //For EMA smoothing algorithm
bool emaInitialized            = false;                         //For EMA smoothing algorithm
bool measurementInProgress     = false;
uint32_t lastRead              = 0;                             //For determining sensor read interval
static uint32_t goodReadings   = 0;                             //Good range-status reading counter
static uint32_t badReadings    = 0;                             //Bad range-status reading counter

//I2C bus handle — set once in setup(); shared by reinitSensor()
TwoWire *sensorBus             = nullptr;

//── Stuck-reading watchdog ────────────────────────────────────────────────────
//The VL53L1X is known to lock up and repeatedly return a stale short distance
//(commonly 255 = 0xFF) with rangeStatus == 0 while the actual target is much
//farther away.  Because this sensor monitors a garage bay, legitimate readings
//will always be stable for long periods (parked car or empty bay), so simple
//repetition cannot be used as the trigger — that would cause continuous resets
//while a car is parked.
//
//Instead the watchdog only counts readings that are BOTH repeated AND below
//MIN_PLAUSIBLE_DISTANCE_MM — the shortest distance this sensor could ever
//legitimately see given its mounting position.  Stable readings at real garage
//distances are completely invisible to the watchdog.
//
//Set MIN_PLAUSIBLE_DISTANCE_MM to a value safely below the closest real target
//in your installation (e.g. if the nearest a bumper can get is ~600 mm, use 400).
#define MIN_PLAUSIBLE_DISTANCE_MM  300                           //Adjust to suit your installation
static uint16_t lastValidDistance   = 0;                         //Most recent accepted distance (mm)
static int      stuckReadingCount   = 0;                         //Consecutive implausible-distance counter
const  int      STUCK_READING_LIMIT = 3;                         //Readings before a reset is triggered
const int OTA_MAX_TIME              = 15;                        //Maximum OTA mode duration, in minutes

//Zigbee disconnect watchdog
static ulong zigbeeDisconnectedSince = 0;
const ulong ZIGBEE_RECONNECT_TIMEOUT = 300000UL;                 //Reboot to rejoin if disconnected > 5 minutes

//Zigbee LED status handler
enum LedState {
  LED_OFF,                                                       //LED completely off
  LED_BLINKING,                                                  //Blink while connecting
  LED_SOLID,                                                     //Solid on when connected
  LED_SOS,                                                       //SOS pattern: OTA WiFi connect, OTA wait, power-cycle window
};

LedState currentLedState        = LED_BLINKING;                  //Default: blink while trying to connect to Zigbee
ulong lastLedChange             = 0;
bool ledOn                      = false;

//SOS pattern (... --- ...) — timing pairs interleaved: on_ms, off_ms per symbol element
static const uint16_t SOS_PATTERN[] = {
  100,100, 100,100, 100,300,   //S: ...
  300,100, 300,100, 300,300,   //O: ---
  100,100, 100,100, 100,600    //S: ...  (with long gap before repeat)
};
static const int SOS_STEPS      = sizeof(SOS_PATTERN) / sizeof(SOS_PATTERN[0]);
static int   sosStep            = 0;
static ulong sosLastChange      = 0;
static bool  sosLedOn           = false;

//── Display state globals (BOARD_NESSO_N1 only) ──────────────────────────────
#ifdef BOARD_NESSO_N1
  uint16_t  lastRaw             = 0;
  uint16_t  lastSmoothed        = 0;
  bool      lastReadValid       = false;
  bool      newReadingAvailable = false;  //Set true by handleDistanceMeasurement(); cleared after display redraw
  bool      altShowAvg          = true;   //Mode 3: true = showing avg (green), false = showing raw (yellow)
  int       displayMode         = 3;      //Active display mode; cycles 0→1→2→3→0 via Front Button short press
  ulong     lastAltSwitch       = 0;      //Mode 3: timestamp of last display alternation
#endif

//***********************************************
//"AUTOMATIC" DEVICE NAMING
//***********************************************
struct DeviceMapping { uint64_t chipId; const char* name; };
const DeviceMapping deviceMappings[] = {   //Use the chipId to look up a device name
  {0x70A95AFEFFCA4C40ULL, "South Car Bay LIDAR"},
  {0x6C355AFEFFCA4C40ULL, "North Car Bay LIDAR"},
  {0x141850FEFF818C58ULL, "Nesso N1 Air Pressure"},
  {0x08A25AFEFFCA4C40ULL, "ESP32-C6 Qwiic Pocket"},
  {0xCCA7C4FEFFEEF648ULL, "Nesso N1 LIDAR"},
  //Add more as needed; prepend "0x" and append "ULL" to the chipID
};

//=============================
//FORWARD FUNCTION DECLARATIONS
//=============================
void   printVersion(int lineLength, const char* prefix, const char* printText, const char* suffix);
String getDeviceName();
String getChipIdString();                                                //Returns chip ID as "0x%016llX"
void   reinitSensor();                                                   //Soft-reset + re-configure VL53L1X
#ifdef BOARD_NESSO_N1
  void drawDistanceScreen();
  void drawLargeNumberScreen(uint16_t value, uint16_t color, bool isAvg); //Modes 1/2/3: large value + "millimeters (avg/raw)" label
  void drawOTAWindowScreen(bool redPhase);
  void drawStatusScreen(const char* line1, const char* line2 = "", const char* line3 = "", uint16_t color = TFT_WHITE);
  void beepOnce();
  void beepTwice();
#endif

//==========================================================================
//DISPLAY HELPER FUNCTIONS (BOARD_NESSO_N1 only)
//==========================================================================
#ifdef BOARD_NESSO_N1

//── drawOTAWindowScreen ─────────────────────────────────────────────────────
//Alternates display between red and black backgrounds during the OTA power-cycle window.
//Called from loop() only on phase transitions, not every iteration.
void drawOTAWindowScreen(bool redPhase) {
  uint16_t bg = redPhase ? TFT_RED : TFT_BLACK;
  uint16_t fg = redPhase ? TFT_WHITE : TFT_RED;
  display.fillScreen(bg);
  display.setFont(&fonts::Font2);
  display.setTextColor(fg);
  String msg1 = "Unplug now to enter OTA mode";
  int32_t tw  = display.textWidth(msg1);
  display.setCursor((DISPLAY_W - tw) / 2, 45);
  display.print(msg1);
  ulong remaining = millis() < OTA_WINDOW_MS ? (OTA_WINDOW_MS - millis()) / 1000 : 0;
  String msg2 = "(" + String(remaining) + "s remaining)";
  tw = display.textWidth(msg2);
  display.setCursor((DISPLAY_W - tw) / 2, 70);
  display.print(msg2);
}

//── drawStatusScreen ─────────────────────────────────────────────────────────
//General-purpose status screen: coloured header bar + up to 3 centred lines.
//Used during startup, WiFi connect, OTA, and error states.
void drawStatusScreen(const char* line1, const char* line2, const char* line3, uint16_t color) {
  display.fillScreen(TFT_BLACK);

  //Header bar
  display.fillRect(0, 0, DISPLAY_W, 22, display.color565(0, 50, 100));
  display.setFont(&fonts::Font2);
  display.setTextColor(TFT_WHITE);
  display.setCursor(4, 3);
  display.printf("%s v. " VERSION, getDeviceName().c_str());

  //Up to three centred lines of status text
  const char* lines[] = {line1, line2, line3};
  int yPos[] = {35, 65, 95};
  for (int i = 0; i < 3; i++) {
    if (lines[i] && lines[i][0] != '\0') {
      display.setFont(&fonts::Font2);
      display.setTextColor(i == 0 ? color : display.color565(180, 180, 180));
      int32_t tw = display.textWidth(lines[i]);
      display.setCursor((DISPLAY_W - tw) / 2, yPos[i]);
      display.print(lines[i]);
    }
  }

  //WiFi footer (where relevant)
  display.fillRect(0, 114, DISPLAY_W, 21, display.color565(20, 20, 20));
  display.setFont(&fonts::Font2);
  display.setTextColor(TFT_WHITE);
  display.setCursor(4, 117);
  if (WiFi.status() == WL_CONNECTED) {
    display.printf("%s  %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    display.print("Not connected");
  }
}

//── drawLargeNumberScreen ────────────────────────────────────────────────────
//Fills the entire 240×135 display with a single large distance value.
//Used by DISPLAY_MODE 1, 2, and 3.  Color encodes data type:
//  TFT_GREEN  = smoothed/avg    TFT_YELLOW = raw
//fonts::Font8 is the largest M5GFX built-in font (~75 px tall).
//The number and label are vertically centred as a group so the number sits
//slightly above screen-centre with the label beneath it.
//  isAvg = true  → label reads "millimeters (avg)"
//  isAvg = false → label reads "millimeters (raw)"
void drawLargeNumberScreen(uint16_t value, uint16_t color, bool isAvg) {
  display.fillScreen(TFT_BLACK);
  if (!lastReadValid) {
    display.setFont(&fonts::Font4);
    display.setTextColor(TFT_WHITE);
    const char* msg = "Waiting...";
    int32_t tw = display.textWidth(msg);
    display.setCursor((DISPLAY_W - tw) / 2, (DISPLAY_H - display.fontHeight()) / 2);
    display.print(msg);
    return;
  }

  //── Measure the number (Font8) ───────────────────────────────────────────
  display.setFont(&fonts::Font8);
  int32_t th     = display.fontHeight();
  String  valStr = String(value);
  int32_t tw     = display.textWidth(valStr);

  //── Measure the label (Font4) ────────────────────────────────────────────
  const char* label = isAvg ? "millimeters (avg)" : "millimeters (raw)";
  display.setFont(&fonts::Font4);
  int32_t labelH = display.fontHeight();
  int32_t ltw    = display.textWidth(label);

  //── Centre the number+label group vertically ────────────────────────────
  const int32_t gap    = 4;                          //px between number bottom and label top
  int32_t       groupH = th + gap + labelH;
  int32_t       numY   = (DISPLAY_H - groupH) / 2;  //number sits above screen-centre
  int32_t       lblY   = numY + th + gap;

  //── Draw number ──────────────────────────────────────────────────────────
  display.setFont(&fonts::Font8);
  display.setTextColor(color);
  display.setCursor((DISPLAY_W - tw) / 2, numY);
  display.print(valStr);

  //── Draw "millimeters" label in muted white ──────────────────────────────
  display.setFont(&fonts::Font4);
  display.setTextColor(display.color565(160, 160, 160));
  display.setCursor((DISPLAY_W - ltw) / 2, lblY);
  display.print(label);
}

//── drawDistanceScreen ───────────────────────────────────────────────────────
//Dispatches to the layout selected by DISPLAY_MODE.
//  0 = Normal (header + both values + footer)
//  1 = Large avg only (green, Font8, full screen)
//  2 = Large raw only (yellow, Font8, full screen)
//  3 = Large alternating avg/raw — driven by altShowAvg, toggled in loop()
void drawDistanceScreen() {
  if (displayMode == 1) {
    drawLargeNumberScreen(lastSmoothed, TFT_GREEN, true);
    return;
  } else if (displayMode == 2) {
    drawLargeNumberScreen(lastRaw, TFT_YELLOW, false);
    return;
  } else if (displayMode == 3) {
    if (altShowAvg)
      drawLargeNumberScreen(lastSmoothed, TFT_GREEN, true);
  else
    drawLargeNumberScreen(lastRaw, TFT_YELLOW, false);
    return;
  }

  //── displayMode == 0: Normal layout ──────────────────────────────────────
  display.fillScreen(TFT_BLACK);

  //── Header bar ───────────────────────────────────────────────────────────
  display.fillRect(0, 0, DISPLAY_W, 22, display.color565(0, 50, 100));
  display.setFont(&fonts::Font2);
  display.setTextColor(TFT_WHITE);
  display.setCursor(4, 3);
  display.print(getDeviceName());                          //Device name left-aligned

  String ver = "v" VERSION;                                //Version right-aligned
  int32_t vw = display.textWidth(ver);
  display.setCursor(DISPLAY_W - vw - 4, 3);
  display.print(ver);

  if (lastReadValid) {
    display.setFont(&fonts::Font4);                        //~26 px

    display.setTextColor(TFT_GREEN);
    String valStr = String(lastSmoothed) + " mm avg";      //Smoothed value
    int32_t tw    = display.textWidth(valStr);
    display.setCursor((DISPLAY_W - tw) / 2, 37);
    display.print(valStr);

    display.setTextColor(TFT_YELLOW);
    valStr = String(lastRaw) + " mm raw";                  //Raw value
    tw     = display.textWidth(valStr);
    display.setCursor((DISPLAY_W - tw) / 2, 77);
    display.print(valStr);

  } else {
    display.setFont(&fonts::Font4);
    display.setTextColor(TFT_YELLOW);
    String waiting = "Waiting for reading...";
    int32_t tw = display.textWidth(waiting);
    display.setCursor((DISPLAY_W - tw) / 2, 60);
    display.print(waiting);
  }

  //── Footer bar ───────────────────────────────────────────────────────────
  bool zbConnected = Zigbee.connected();
  display.fillRect(0, 114, DISPLAY_W, 21, display.color565(20, 20, 20));
  display.setFont(&fonts::Font2);
  display.setTextColor(TFT_WHITE);
  display.setCursor(4, 117);
  if (WiFi.status() == WL_CONNECTED) {
    display.print(WiFi.localIP().toString());              //WiFi connected
  } else if (zbConnected) {
    display.print("Zigbee");                               //Zigbee connected, WiFi off
  } else {
    display.print("No connection");                        //Neither connected
  }
  //Show Good / Bad counts on right side
  display.setTextColor(goodReadings > 0 ? display.color565(0, 200, 0) : display.color565(140, 140, 140));
  display.setCursor(165, 117);
  display.printf("G:%lu  B:%lu", goodReadings, badReadings);
}

//=============================================================
//BUZZER HELPERS (BOARD_NESSO_N1 only)
//=============================================================
//Passive buzzer responds loudest around 2–4 kHz.
//Adjust BEEP_FREQ to tune loudness for your specific buzzer:
//  1000 Hz = lower pitch, often quieter
//  2000 Hz = good middle ground
//  2730 Hz = typical piezo resonant frequency — usually loudest
//  4000 Hz = higher pitch, can be louder but more shrill
#define BEEP_FREQ      2730   //Hz — adjust for loudest output
#define BEEP_DURATION   150   //ms — longer = more audible

void beepOnce() {
  tone(BUZZER_PIN, BEEP_FREQ, BEEP_DURATION);
}

void beepTwice() {
  tone(BUZZER_PIN, BEEP_FREQ, BEEP_DURATION);
  delay(300);
  tone(BUZZER_PIN, BEEP_FREQ, BEEP_DURATION);
}

#endif  //BOARD_NESSO_N1 display and buzzer functions

//***********************************************
//CHECK FOR RAPID REBOOT (OTA MODE TRIGGER)
//***********************************************
bool checkForOTAMode() {
  //How this works:
  // Boot 1: Zigbee starts → setOTAFlag() → 3 LED flashes → "safe to unplug"
  // Boot 2 (genuine power-on): flag found + ESP_RST_POWERON → OTA mode
  //
  // CRITICAL: only ESP_RST_POWERON counts as a valid rapid reboot trigger.
  // Software resets (USB upload, ESP.restart(), watchdog) all produce other reset
  // reasons and must NOT trigger OTA mode even if the flag is set — those are
  // handled by the callers which clear the flag before calling ESP.restart().

  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("[OTA] Reset reason: %d (%s)\n", resetReason,
    resetReason == ESP_RST_POWERON  ? "Power-on"      :
    resetReason == ESP_RST_SW       ? "Software reset" :
    resetReason == ESP_RST_PANIC    ? "Crash/panic"    :
    resetReason == ESP_RST_WDT      ? "Watchdog"       :
    resetReason == ESP_RST_BROWNOUT ? "Brownout"       :
    resetReason == ESP_RST_DEEPSLEEP? "Deep sleep"     :
    resetReason == 11               ? "USB reset"      : "Other");

  preferences.begin("ota-mode", false);
  ulong lastBootFlag = preferences.getULong("lastBoot", 0);
  bool quickReboot   = false;

  if (lastBootFlag > 0) {
    if (resetReason == ESP_RST_POWERON) {
      //Genuine power cycle with flag set — this is the intended rapid reboot
      Serial.println("\n*** RAPID REBOOT DETECTED! ***");
      quickReboot = true;
      preferences.putULong("lastBoot", 0);      //Clear flag; entering OTA mode
    } else {
      //Flag was set but this is a software/crash reset, not a power cycle.
      //Clear the stale flag and boot normally — handles USB uploads,
      //ESP.restart() calls that forgot to clear the flag, etc.
      Serial.printf("[OTA] Flag found but reset was not a power-on (%d) — clearing and booting normally.\n", resetReason);
      preferences.putULong("lastBoot", 0);
    }
  }
  //If lastBootFlag == 0: normal boot — flag set later by setOTAFlag() after Zigbee starts
  preferences.end();
  return quickReboot;
}

void setOTAFlag() {
  //Called ONLY after Zigbee.begin() succeeds — guarantees a crash can never set this flag.
  //Sets lastBoot = 1; loop() drives the 6s-cycle LED blink for the OTA window.
  preferences.begin("ota-mode", false);
  preferences.putULong("lastBoot", 1);
  preferences.end();                            //Force NVS commit
  delay(100);

  Serial.println("\n[OTA] OTA flag set — LED will blink (6s cycle) for 15s. Unplug during that window for OTA mode.");
}

//*****************************************
//WEB PAGE BUILDER (common)
//*****************************************
//Shared HTML page builder used by all three web server modes (WiFi-only, OTA, normal).
// modeLabel  : bold mode description shown in the Mode row
// extraRows  : zero or more <tr>...</tr> strings inserted after the Mode row
// buttons    : HTML for the Actions section (links, forms, buttons)
String buildRootPage(const String& modeLabel, const String& extraRows, const String& buttons) {
  String p = "<html><head><meta charset='utf-8'>";
  p += "<style>";
  p += "body{font-family:sans-serif;max-width:480px;margin:2em auto;padding:0 1em;}";
  p += "table{border-collapse:collapse;width:100%;}";
  p += "td{padding:4px 8px;border-bottom:1px solid #eee;}";
  p += "td:first-child{color:#666;width:40%;}";
  p += "h2{border-bottom:2px solid #333;padding-bottom:.3em;}";
  p += ".btn{display:inline-flex;align-items:center;margin:.3em .2em;padding:.5em 1em;background:#0066cc;";
  p += "color:#fff;text-decoration:none;border:none;border-radius:4px;cursor:pointer;font-size:1em;line-height:1;}";
  p += ".btn-red{background:#cc2200;}";
  p += ".banner{padding:.6em 1em;border-radius:4px;margin:1em 0;font-weight:bold;}";
  p += "</style></head><body>";
  p += "<h2>&#128225; LIDAR Distance Sensor " VERSION "</h2>";
  p += "<h3>Device Status</h3><table>";
  p += "<tr><td>Device Name</td><td>" + getDeviceName() + "</td></tr>";
  p += "<tr><td>Mode</td><td>" + modeLabel + "</td></tr>";
  p += extraRows;
  p += "<tr><td>IP Address</td><td>" + WiFi.localIP().toString() + "</td></tr>";
  {
    int rssi = WiFi.RSSI();
    String rssiColor =
      rssi >= -70 ? "#2a7a2a" :   //Green  — Good/Excellent
      rssi >= -80 ? "#b35c00" :   //Amber  — Fair
                    "#cc2200";    //Red    — Poor
    p += "<tr><td>WiFi RSSI</td><td style='color:" + rssiColor + ";font-weight:bold;'>"
         + String(rssi) + " dBm</td></tr>";
    p += "<tr><td>WiFi Channel</td><td>" + String(WiFi.channel()) + "</td></tr>";
    p += "<tr><td>WiFi MAC</td><td>" + WiFi.macAddress() + "</td></tr>";
    p += "</table>";
    if (rssi < -80) {
      p += "<div class='banner banner-err'>&#10060; Weak signal (" + String(rssi)
           + " dBm) &mdash; OTA upload will very likely fail."
           " Move the device closer to the access point before uploading.</div>";
    } else if (rssi < -70) {
      p += "<div class='banner banner-warn'>&#9888; Fair signal (" + String(rssi)
           + " dBm) &mdash; OTA upload may be unreliable."
           " If the upload fails, try again or move closer to the access point.</div>";
    }
  }
  p += "<h3>Actions</h3>";
  p += buttons;
  p += "</body></html>";
  return p;
}

//*****************************************
//HELPER FUNCTIONS (common)
//*****************************************

//── reinitSensor ─────────────────────────────────────────────────────────────
//Soft-resets the VL53L1X via register 0x0000 and fully re-applies all settings.
//Called from handleDistanceMeasurement() when the stuck-reading watchdog fires,
//and mirrors the sensor-configuration sequence in setup().
//
//After the reset the EMA is discarded so a fresh run of good readings is needed
//before the smoothed value is considered reliable again.  The raw Zigbee endpoint
//will still report the first valid post-reset reading immediately.
void reinitSensor() {
  Serial.println("[SENSOR] reinitSensor() — stopping ranging...");
  distanceSensor.stopRanging();

  //Soft-reset: write 0x00 to SOFT_RESET (register 0x0000), then 0x01 to release.
  //SFEVL53L1X does not expose softReset(), so we write directly to the register.
  //The VL53L1X uses 16-bit register addresses (MSB first) at I2C address 0x29.
  sensorBus->beginTransmission(0x29);
  sensorBus->write(0x00);   //SOFT_RESET register MSB
  sensorBus->write(0x00);   //SOFT_RESET register LSB
  sensorBus->write(0x00);   //value: assert reset
  sensorBus->endTransmission();
  delay(1);                 //Hold reset for at least 100 µs (1 ms is safe)

  // Release reset (write 0x01 to same register)
  sensorBus->beginTransmission(0x29);
  sensorBus->write(0x00);   //SOFT_RESET register MSB
  sensorBus->write(0x00);   //SOFT_RESET register LSB
  sensorBus->write(0x01);   //value: release reset
  sensorBus->endTransmission();
  delay(100);               //Allow sensor boot sequence to complete before begin()

  //Re-initialise on the same I2C bus that was selected at startup.
  if (distanceSensor.begin(*sensorBus) != 0) {
    //If begin() fails the I2C bus or sensor may have a hard fault.
    //Log the error and leave measurementInProgress = false so loop() will
    //attempt startRanging() again on the next SENSOR_READ_INTERVAL_MS tick.
    Serial.println("[SENSOR] reinitSensor() ERROR: begin() failed after soft-reset. Will retry next cycle.");
    measurementInProgress = false;
    return;
  }

  //Re-apply all settings exactly as in setup()
  if (DISTANCE_LONG_MODE) distanceSensor.setDistanceModeLong();
  else                    distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(TIMING_BUDGET);
  distanceSensor.setIntermeasurementPeriod(INTERMEASUREMENT_PERIOD);

  //Discard poisoned EMA — next good reading will seed it from scratch
  emaInitialized   = false;
  averageDistance  = 0.0;

  //Reset watchdog counters
  stuckReadingCount  = 0;
  lastValidDistance  = 0;

  distanceSensor.startRanging();
  measurementInProgress = true;
  lastRead              = millis();  //Restart the inter-read timer

  Serial.println("[SENSOR] reinitSensor() complete — ranging restarted.");
}

//── handleDistanceMeasurement ────────────────────────────────────────────────
//Distance measurement and EMA smoothing. Nesso N1 also updates display state.
//Includes a stuck-reading watchdog: if STUCK_READING_LIMIT consecutive valid-
//status readings return the same distance (the 0xFF / 255 mm hang signature),
//reinitSensor() is called to soft-reset and re-configure the VL53L1X.
void handleDistanceMeasurement(bool isConnected) {
  ulong now = millis();

  //Start a new measurement if SENSOR_READ_INTERVAL_MS has passed
  if (!measurementInProgress && now - lastRead >= SENSOR_READ_INTERVAL_MS) {
    distanceSensor.startRanging();                  //Non-blocking operation
    measurementInProgress = true;
    lastRead = now;
  }

  //Check if measurement is ready
  if (measurementInProgress && distanceSensor.checkForDataReady()) {
    measurementInProgress = false;     //Ready for next measurement cycle

    uint16_t distance_raw = distanceSensor.getDistance();

    byte rangeStatus = distanceSensor.getRangeStatus(); //Get the sensor range status
    /*Range status values:
      0: Valid
      1: Sigma fail
      2: Signal fail
      4: Phase out of valid limits
      5: Hardware fail
      7: Wraparound (ambiguous reading)
      other: Unknown

     Exponential Moving Average (EMA) calculation
     - NOTE: The EMA method takes several readings to change value enough to trigger an output
     - A smaller alpha creates a smoother, slower-reacting line (more weight on historical data)
     - A larger alpha makes the EMA more responsive (more weight on recent data)
     */

    if (distance_raw > 0 && distance_raw < 8000 && rangeStatus == 0) {

      // ── Stuck-reading watchdog ──────────────────────────────────────────────
      // Only count a reading toward a reset when it is BOTH repeated AND below
      // MIN_PLAUSIBLE_DISTANCE_MM.  This prevents false resets during normal
      // operation: a parked car (or an empty bay) produces legitimately stable
      // readings that are always well above the plausible-distance threshold.
      // The watchdog is blind to those; it only reacts to suspiciously short
      // values that persist, which is the signature of the sensor hang bug.
      if (distance_raw < MIN_PLAUSIBLE_DISTANCE_MM && distance_raw == lastValidDistance) {
        stuckReadingCount++;
        Serial.printf("[SENSOR] Suspicious short repeated reading: %u mm (%d/%d)\n",
                      distance_raw, stuckReadingCount, STUCK_READING_LIMIT);
      } else {
        stuckReadingCount = 0;           //Any plausible reading clears the count
      }
      lastValidDistance = distance_raw;  //Always track, regardless of plausibility

      if (stuckReadingCount >= STUCK_READING_LIMIT) {
        Serial.printf("[SENSOR] Stuck-reading watchdog fired: %u mm × %d — resetting sensor.\n",
                      distance_raw, stuckReadingCount);
        badReadings++;                   //Count this as a bad reading
        distanceSensor.clearInterrupt();
        reinitSensor();
        return;                          //Discard this poisoned reading entirely
      }
      // ── End watchdog ────────────────────────────────────────────────────────

      goodReadings++;

      //EMA smoothing
      if (!emaInitialized) {
        averageDistance = distance_raw;
        emaInitialized = true;
      } else {
        averageDistance = EMA_ALPHA * distance_raw + (1.0 - EMA_ALPHA) * averageDistance;
      }

      uint16_t roundedAverage = (uint16_t)round(averageDistance);
      #ifdef BOARD_NESSO_N1
        lastRaw                 = distance_raw;
        lastSmoothed            = roundedAverage;
        lastReadValid           = true;
        newReadingAvailable     = true;     //Signal loop() to redraw the display
      #endif

      //Report to Zigbee if connected; EMA smoothing always active
      if (isConnected) {
        zbDistance.setIlluminance(roundedAverage);   //Endpoint 10: smoothed value (non-blocking)
        zbRaw.setIlluminance(distance_raw);          //Endpoint 11: raw value (non-blocking)
        Serial.printf("Raw: %4u mm (ep11) | Smoothed: %4u mm (ep10) | Status: %d | Reported to Hubitat\n",
                      distance_raw, roundedAverage, rangeStatus);
      } else {
        Serial.printf("Raw: %4u mm | Smoothed: %4u mm | Status: %d | Not connected\n",
                      distance_raw, roundedAverage, rangeStatus);
      }
    } else {
      badReadings++;
      Serial.printf("Invalid reading: %4u mm, Status: %d | Good: %lu, Bad: %lu (%.1f%% success)\n",
                    distance_raw, rangeStatus, goodReadings, badReadings,
                    100.0 * goodReadings / (goodReadings + badReadings));
    }

    distanceSensor.clearInterrupt();                 //MUST clear the interrupt flag to enable the next measurement
  }
}

String getChipIdString() {
  char buf[19];  //"0x" + 16 hex digits + null terminator
  snprintf(buf, sizeof(buf), "0x%016llX", ESP.getEfuseMac());
  return String(buf);
}

String getDeviceName() {
  uint64_t chipId = ESP.getEfuseMac();
  String   devName = "LIDAR-" + String((uint16_t)(chipId & 0xFFFF), HEX);  //Default name

  for (int i = 0; i < (int)(sizeof(deviceMappings) / sizeof(deviceMappings[0])); i++) {
    if (deviceMappings[i].chipId == chipId) {
      devName = String(deviceMappings[i].name);
      break;
    }
  }
  return devName;
}

void WiFiConnectionInfo() {
  uint64_t chipID = ESP.getEfuseMac();
  Serial.println("\n\n*** WiFi Connected Successfully! ***");
  Serial.printf("\tIP address   : %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("\tGateway      : %s\n", WiFi.gatewayIP().toString().c_str());
  Serial.printf("\tSubnet       : %s\n", WiFi.subnetMask().toString().c_str());
  Serial.printf("\tDevice ID    : %s\n", getChipIdString().c_str());
  Serial.printf("\tZigbee MAC   : %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n",
    (uint8_t)(chipID >> 0),  (uint8_t)(chipID >> 8),
    (uint8_t)(chipID >> 16), (uint8_t)(chipID >> 24),
    (uint8_t)(chipID >> 32), (uint8_t)(chipID >> 40),
    (uint8_t)(chipID >> 48), (uint8_t)(chipID >> 56)
  );
  Serial.printf("\tWiFi MAC     : %s\n", WiFi.macAddress().c_str());
  Serial.printf("\tWiFi DNS     : %s\n", WiFi.dnsIP().toString().c_str());
  Serial.printf("\tWiFi RSSI    : %d dBm\n", WiFi.RSSI());
  Serial.printf("\tWiFi Channel : %d\n", WiFi.channel());
}

bool updateZigbeeLED() {                             //Use the built-in LED to signal Zigbee connection status
  static bool lastConnected = false;
  bool currentlyConnected = Zigbee.connected();

  //Only auto-transition BLINKING/SOLID when not in LED_SOS mode.
  //LED_SOS is set externally (OTA window, OTA connect loop, OTA wait loop)
  //and must not be overridden by the Zigbee-not-connected path.
  if (currentLedState != LED_SOS) {
    if (currentlyConnected && !lastConnected) {      //Just connected
      Serial.println("\n*** CONNECTED TO ZIGBEE NETWORK! ***\n");
      currentLedState = LED_SOLID;
    } else if (!currentlyConnected) {
      currentLedState = LED_BLINKING;                //Blink while not connected
    }
  }

  lastConnected = currentlyConnected;
  ulong now     = millis();

  switch (currentLedState) {
    case LED_OFF:
      digitalWrite(LED_PIN, LOW);
      ledOn = false;
      break;

    case LED_SOLID:
      digitalWrite(LED_PIN, HIGH);
      ledOn = true;
      break;

    case LED_BLINKING:
      if (now - lastLedChange >= 300) {              //300 ms blink interval
        ledOn = !ledOn;
        digitalWrite(LED_PIN, ledOn);
        lastLedChange = now;
      }
      break;

    case LED_SOS:                                    //SOS pattern: ... --- ...
      if (now - sosLastChange >= SOS_PATTERN[sosStep]) {
        sosLedOn = !sosLedOn;
        digitalWrite(LED_PIN, sosLedOn ? HIGH : LOW);
        sosLastChange = now;
        sosStep = (sosStep + 1) % SOS_STEPS;
      }
      break;
  }
  return currentlyConnected;
}

//── checkButtonPress ─────────────────────────────────────────────────────────
//BOARD_EQP:      BOOT button held long enough → factory reset & re-pairing
//BOARD_NESSO_N1: Front Button short = cycle display mode, long = OTA mode
//                Side Button short  = reboot
//                Side Button long   = factory reset & re-pairing
#ifdef BOARD_EQP
void checkButtonPress() {
  if (digitalRead(BUTTON_PIN) == LOW) {                   //Check for BOOT button press
    delay(100);                                           //Debounce
    currentLedState = LED_BLINKING;                       //Visual indicator: button pressed
    if (digitalRead(BUTTON_PIN) == LOW) {
      ulong start = millis();
      bool messageShown = false;

      while (digitalRead(BUTTON_PIN) == LOW) {
        yield();                                          //Prevents WDT reset
        if (!messageShown && (millis() - start > 500)) {
          Serial.print("\n\nBOOT button pushed - hold for ");
          Serial.print(BUTTON_HOLD_TIME / 1000);
          Serial.println(" seconds for factory reset...");
          messageShown = true;
        }
        if (millis() - start > BUTTON_HOLD_TIME) {
          Serial.println("\n*** FACTORY RESET ***");
          Serial.println("Resetting Zigbee and restarting...");
          preferences.begin("ota-mode", false);           //CRITICAL: clear OTA flag before reset
          preferences.putULong("lastBoot", 0);
          preferences.end();
          Zigbee.factoryReset();
          delay(1000);
          ESP.restart();
          return;
        }
        delay(50);
      }
      if (messageShown) {
        Serial.println("Button released early - reset cancelled");
      }
    }
  }
}
#endif  //BOARD_EQP checkButtonPress

#ifdef BOARD_NESSO_N1
void checkButtonPress() {

  //── Front Button: short press = cycle display mode; long press = enter OTA mode ──
  if (digitalRead(FRONT_BUTTON) == LOW) {
    delay(100);                                           //Debounce
    currentLedState = LED_BLINKING;
    if (digitalRead(FRONT_BUTTON) == LOW) {
      ulong start = millis();
      bool messageShown = false;

      while (digitalRead(FRONT_BUTTON) == LOW) {
        yield();
        if (!messageShown && (millis() - start > 500)) {
          beepTwice();
          Serial.print("\n\nFront Button held - hold for ");
          Serial.print(BUTTON_HOLD_TIME / 1000);
          Serial.println(" seconds to enter OTA mode...");
          drawStatusScreen("Front Button held...",
                           ("Hold " + String(BUTTON_HOLD_TIME / 1000) + "s for OTA mode").c_str(),
                           "Release to cancel", TFT_YELLOW);
          messageShown = true;
        }
        if (millis() - start > BUTTON_HOLD_TIME) {       //Long press: enter OTA mode
          beepTwice();
          Serial.println("\n*** OTA MODE via Front Button long press ***");
          drawStatusScreen("Entering OTA Mode...", "Rebooting.", "", TFT_YELLOW);
          preferences.begin("ota-mode", false);
          preferences.putBool("directOTA", true);        //SW-reset-safe; bypasses power-on check
          preferences.putULong("lastBoot", 0);           //Clear rapid-reboot flag to avoid double-trigger
          preferences.end();
          delay(1000);
          ESP.restart();
          return;
        }
        delay(50);
      }

      if (messageShown) {                                //Released between 0.5s and BUTTON_HOLD_TIME
        Serial.println("Front Button released early - OTA cancelled");
        drawDistanceScreen();
      } else {                                           //Released before 0.5s — short press: just reboot
        beepOnce();
        displayMode = (displayMode + 1) % 4;
        if (displayMode == 3) {                          //Reset alt-timer when re-entering mode 3
          altShowAvg    = true;
          lastAltSwitch = 0;
        }
        Serial.printf("\nFront Button short press - display mode → %d\n", displayMode);
        drawDistanceScreen();
      }
    }
  }

  //── Side Button: short press = reboot; long press = factory reset & re-pairing ──
  if (digitalRead(SIDE_BUTTON) == LOW) {
    delay(100);                                          //Debounce
    currentLedState = LED_BLINKING;
    if (digitalRead(SIDE_BUTTON) == LOW) {
      ulong start = millis();
      bool messageShown = false;
      beepTwice();
      while (digitalRead(SIDE_BUTTON) == LOW) {
        yield();
        if (!messageShown && (millis() - start > 500)) {
          Serial.print("\n\nSide Button held - hold for ");
          Serial.print(BUTTON_HOLD_TIME / 1000);
          Serial.println(" seconds for factory reset...");
          drawStatusScreen("Side Button held...",
                           ("Hold " + String(BUTTON_HOLD_TIME / 1000) + "s for factory reset").c_str(),
                           "Release to cancel", TFT_RED);
          messageShown = true;
        }
        if (millis() - start > BUTTON_HOLD_TIME) {      //Long press: factory reset
          beepTwice();
          Serial.println("\n*** FACTORY RESET via Side Button long press ***");
          Serial.println("Resetting Zigbee and restarting...");
          drawStatusScreen("FACTORY RESET", "Resetting Zigbee...", "Restarting.", TFT_RED);
          delay(1000);
          preferences.begin("ota-mode", false);         //CRITICAL: clear OTA flag before factory reset
          preferences.putULong("lastBoot", 0);
          preferences.end();
          Zigbee.factoryReset();
          delay(1000);
          ESP.restart();
          return;
        }
        delay(50);
      }
      if (messageShown) {                               //Released between 0.5s and BUTTON_HOLD_TIME
        Serial.println("Side Button released early - reset cancelled");
        drawDistanceScreen();
      } else {                                          //Released before 0.5s — short press: reboot
        beepOnce();
        Serial.println("\nSide Button short press - rebooting...");
        drawStatusScreen("Rebooting...", "", "", TFT_CYAN);
        preferences.begin("ota-mode", false);           //Clear OTA flag: reboot must not trigger OTA mode
        preferences.putULong("lastBoot", 0);
        preferences.end();
        delay(500);
        ESP.restart();
      }
    }
  }
}
#endif  //BOARD_NESSO_N1 checkButtonPress

void checkZigbeeWatchdog() {                           //Reboot to force clean Zigbee rejoin if disconnected too long
  if (!Zigbee.connected()) {
    if (zigbeeDisconnectedSince == 0) {
      zigbeeDisconnectedSince = millis();
      Serial.println("[Zigbee] Lost connection - watchdog started...");
    } else if (millis() - zigbeeDisconnectedSince > ZIGBEE_RECONNECT_TIMEOUT) {
      Serial.println("[Zigbee] Disconnected too long - rebooting to rejoin...");
      //Clear OTA flag before rebooting — a watchdog reboot must never look like
      //a deliberate power cycle to checkForOTAMode() on the next boot.
      preferences.begin("ota-mode", false);
      preferences.putULong("lastBoot", 0);
      preferences.end();
      delay(500);
      ESP.restart();
    }
  } else {
    zigbeeDisconnectedSince = 0;                       //Reset watchdog timer when connected
  }
}

void printVersion(int lineLength, const char* prefix, const char* printText, const char* suffix) {
  if (prefix    == nullptr) prefix    = "";
  if (printText == nullptr) printText = "";
  if (suffix    == nullptr) suffix    = "";

  int content_length = strlen(prefix) + strlen(printText) + strlen(suffix);
  int total_padding  = lineLength - content_length;
  int left_padding   = total_padding / 2;
  int right_padding  = total_padding - left_padding;

  if (left_padding  < 0) left_padding  = 0;
  if (right_padding < 0) right_padding = 0;

  Serial.printf("║%*s%s%s%s%*s║\n",
                left_padding,  "",
                prefix, printText, suffix,
                right_padding, "");
}

//*****************************************
//REBOOT HELPERS (common)
// Called from server.on() lambdas in multiple modes to consolidate
// the repeated flag-clearing + restart sequences.
//*****************************************
void doRebootToZigbee(bool clearWifiFlag) {
  //Clears OTA and (optionally) WiFi-only flags, then restarts into normal Zigbee mode.
  if (clearWifiFlag) {
    preferences.begin("radio-mode", false);
    preferences.putBool("wifiRequested", false);
    preferences.end();
  }
  preferences.begin("ota-mode", false);
  preferences.putULong("lastBoot", 0);    //CRITICAL: prevent false OTA detection on next boot
  preferences.end();
  delay(500);
  ESP.restart();
}

void doRebootToOTA() {
  //Sets the directOTA flag (SW-reset-safe) and restarts into OTA mode.
  //Uses directOTA rather than lastBoot because ESP.restart() produces ESP_RST_SW,
  //which checkForOTAMode() would otherwise reject as a non-power-on reset.
  preferences.begin("ota-mode", false);
  preferences.putBool("directOTA", true); //SW-reset-safe; bypasses power-on check
  preferences.putULong("lastBoot", 0);    //Clear rapid-reboot flag to avoid double-trigger
  preferences.end();
  Serial.println("Rebooting into OTA mode...\n");
  delay(1000);
  ESP.restart();
}

//***********************************************
//REBOOT HELPERS (common)
// Called from server.on() lambdas in multiple modes to consolidate
// the repeated flag-clearing + restart sequences.
//***********************************************

//── doRebootToWiFiMode ────────────────────────────────────────────────────────
//Re-enters WiFi-only mode on the next boot.  Called from the /reboot-wifi
//endpoint in WiFi-only mode; mirrors the pattern of doRebootToZigbee() and
//doRebootToOTA().
//Note: wifiRequested is cleared when WiFi connects (so the flag is not redundantly
//set across reboots); it must be explicitly restored here before restarting.
void doRebootToWiFiMode() {
  preferences.begin("radio-mode", false);
  preferences.putBool("wifiRequested", true);  //Re-enter WiFi-only mode on next boot
  preferences.putInt("wifiRetries", 0);
  preferences.end();
  preferences.begin("ota-mode", false);
  preferences.putULong("lastBoot", 0);         //Prevent false OTA detection on next boot
  preferences.end();
  delay(500);
  ESP.restart();
}

//***********************************************
//WiFi-ONLY MODE (common, with Nesso display additions)
//  Entered when Hubitat sends an ON command to Zigbee endpoint 12.
//  Zigbee is never started. WiFi and web server run at full priority.
//  Returns to Zigbee mode via GET /zigbee-mode, after OTA_MAX_TIME minutes,
//  or after /update completes.
//***********************************************
void runWiFiOnlyMode(int retryCount = 0) {
  delay(50);
  Serial.println("╔════════════════════════════════════════╗");
  printVersion(40, "WiFi-ONLY MODE ACTIVE", "", "");
  printVersion(40, "Zigbee is DISABLED for this session", "", "");
  printVersion(40, "Use /zigbee-mode to return", "", "");
  Serial.println("╚════════════════════════════════════════╝\n");
#ifdef BOARD_NESSO_N1
  drawStatusScreen("WiFi-Only Mode", "Zigbee DISABLED", "Connecting to WiFi...", TFT_ORANGE);
#endif

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);            //Full TX power — Zigbee is off; maximise link margin for reliable upload
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to WiFi (%s)...\n", WIFI_SSID);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 60) {
    Serial.print(".");
    attempts++;
    delay(500);
  }

  if (WiFi.status() != WL_CONNECTED) {
    retryCount++;
    preferences.begin("ota-mode", false);
    preferences.putULong("lastBoot", 0);        //Prevent false rapid-reboot detection
    preferences.end();
    if (retryCount < 3) {
      Serial.printf("\nERROR: WiFi connection failed. Retry %d of 3 — rebooting in 5 seconds...\n\n", retryCount);
#ifdef BOARD_NESSO_N1
      drawStatusScreen("WiFi Failed", ("Retry " + String(retryCount) + " of 3").c_str(), "Rebooting...", TFT_RED);
#endif
      preferences.begin("radio-mode", false);
      preferences.putBool("wifiRequested", true); //Keep flag set so next boot retries WiFi
      preferences.putInt("wifiRetries", retryCount);
      preferences.end();
      delay(5000);
    } else {
      Serial.printf("\nERROR: WiFi failed after %d attempts — returning to Zigbee mode.\n\n", retryCount);
#ifdef BOARD_NESSO_N1
      drawStatusScreen("WiFi Failed", "Returning to Zigbee", "Rebooting...", TFT_RED);
#endif

      preferences.begin("radio-mode", false);
      preferences.putBool("wifiRequested", false); //Give up; next boot is normal Zigbee
      preferences.putInt("wifiRetries", 0);
      preferences.end();
      preferences.begin("ota-mode", false);
      preferences.putULong("lastBoot", 0);        //Prevent false OTA detection on next boot
      preferences.end();
      delay(3000);
    }
    ESP.restart();
  }
  //WiFi connected — clear retry counter
  preferences.begin("radio-mode", false);
  preferences.putBool("wifiRequested", false);   //Clear flag; now in WiFi-only mode
  preferences.putInt("wifiRetries", 0);
  preferences.end();

  //CRITICAL: modem sleep MUST be disabled during OTA.
  //With sleep enabled the radio powers down between beacon intervals (~100 ms gaps),
  //dropping TCP packets mid-upload and producing "status code 0" / "could not activate firmware".
  WiFi.setSleep(false);

  WiFiConnectionInfo();
#ifdef BOARD_NESSO_N1
  drawStatusScreen("WiFi-Only Mode", WiFi.localIP().toString().c_str(), "Zigbee OFF", TFT_ORANGE);
#endif

  //Register web server routes for WiFi-only mode
  server.on("/", HTTP_GET, []() {
    String btns = "<a class='btn' href='/update'>&#8593; Upload OTA Firmware</a>"
                  "<a class='btn' href='/reboot-wifi'>&#8635; Reboot WiFi</a>"
                  "<a class='btn btn-red' href='/zigbee-mode'>&#8634; Return to Zigbee Mode</a>";
    server.send(200, "text/html", buildRootPage("<b>WiFi-Only</b> (Zigbee OFF)", "", btns));
  });

  server.on("/reboot-wifi", HTTP_ANY, []() {
    server.send(200, "text/plain", "Rebooting WiFi connection...");
    Serial.println("\n*** WIFI REBOOT REQUESTED VIA HTTP — rebooting back into WiFi-only mode... ***\n");
    server.client().clear();
    server.client().stop();
    doRebootToWiFiMode();
  });

  server.on("/zigbee-mode", HTTP_ANY, []() {
    server.send(200, "text/plain", "Rebooting into Zigbee mode...");
    Serial.println("\n*** ZIGBEE MODE REQUESTED VIA HTTP — rebooting... ***\n");
    server.client().clear();
    server.client().stop();
    doRebootToZigbee(true);    //true = also clear wifiRequested flag
  });

  server.on("/ota-mode", HTTP_ANY, []() {
    if (server.method() == HTTP_POST || server.method() == HTTP_GET) {
      Serial.println("\n*** OTA MODE REQUESTED VIA HTTP (WiFi-only mode) ***");
      server.send(200, "application/json",
        "{\"status\":\"entering_ota_mode\",\"message\":\"Device will reboot into OTA mode shortly\"}");
      server.client().clear();
      server.client().stop();
      doRebootToOTA();
    } else {
      server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    }
  });

  server.on("/progress", HTTP_GET, []() {           //Kept as a stub so stale browser tabs don't get 404
    server.send(200, "application/json", "{\"removed\":true}");
  });

  ElegantOTA.onStart([]() {
    Serial.println("OTA update started in WiFi-only mode...");
#ifdef BOARD_NESSO_N1
    drawStatusScreen("OTA Update", "Uploading firmware...", "", TFT_YELLOW);
#endif
    //NOTE: wifiRequested flag is intentionally NOT cleared here.
    //It is only cleared in onEnd() on success, so that a failed upload causes
    //ElegantOTA's automatic reboot to land back in WiFi mode (ready to retry)
    //rather than dropping into Zigbee mode.
  });

  ElegantOTA.onProgress([](size_t current, size_t total) {
    static ulong lastPrint = 0;  ulong now = millis();
    if (now - lastPrint >= 1000 || current == total) {
      lastPrint = now;
      size_t fwSize = Update.size();   // Firmware total, set by ElegantOTA via Update.begin()
      int pct = (fwSize > 0) ? (int)(100UL * total / fwSize) : 0;
      Serial.printf("[OTA] Progress: %3d%% | %7u / %7u bytes written | Free heap: %u bytes\n",
                    pct, total, fwSize, ESP.getFreeHeap());
    }
  });

  ElegantOTA.onEnd([](bool success) {
    if (success) {
      Serial.println("OTA update successful! Rebooting to Zigbee mode...\n");
      //Clear wifiRequested only on success — device should return to normal Zigbee operation.
      preferences.begin("radio-mode", false);
      preferences.putBool("wifiRequested", false);
      preferences.end();
#ifdef BOARD_NESSO_N1
      drawStatusScreen("OTA Complete", "Rebooting...", "", TFT_GREEN);
#endif
    } else {
      Serial.println("OTA update FAILED — staying in WiFi mode for retry...\n");
      //wifiRequested intentionally left true: ElegantOTA will reboot the device, and
      //wifiRequested=true causes the next boot to re-enter WiFi mode automatically,
      //so the user can attempt the upload again without going back to Hubitat.
#ifdef BOARD_NESSO_N1
      drawStatusScreen("OTA FAILED", "Rebooting to WiFi...", "", TFT_RED);
#endif
    }
    //Clear lastBoot timestamp so the post-OTA reboot is NOT treated as a rapid
    //power cycle by checkForOTAMode() — without this it incorrectly enters OTA mode again.
    preferences.begin("ota-mode", false);
    preferences.putULong("lastBoot", 0);
    preferences.end();
  });

  ElegantOTA.begin(&server);
  server.begin();

  Serial.println("\n*** Web Server Ready! ***");
  Serial.printf("Control panel     : http://%s\n",             WiFi.localIP().toString().c_str());
  Serial.println("\nAvailable endpoints:");
  Serial.printf("  /               : http://%s/\n",            WiFi.localIP().toString().c_str());
  Serial.printf("  /update         : http://%s/update\n",      WiFi.localIP().toString().c_str());
  Serial.printf("  /zigbee-mode    : http://%s/zigbee-mode\n", WiFi.localIP().toString().c_str());
  Serial.printf("  /ota-mode       : http://%s/ota-mode\n",    WiFi.localIP().toString().c_str());
  Serial.println("=======================================================\n");

  ulong modeStart = millis();
  const ulong MODE_TIMEOUT = (ulong)OTA_MAX_TIME * 60 * 1000;

  while (millis() - modeStart < MODE_TIMEOUT) {
    server.handleClient();
    ElegantOTA.loop();

    static ulong lastMinutePrint = 0;
    if (millis() - lastMinutePrint > 60000) {
      lastMinutePrint = millis();
      ulong remaining = (MODE_TIMEOUT - (millis() - modeStart)) / 60000;
      Serial.printf("[WiFi-Only Mode] Timeout in %lu minutes. Use /zigbee-mode to return now.\n", remaining);
    }

    vTaskDelay(1);                                //Yield to WiFi/TCP FreeRTOS background tasks — essential on ESP32-C6
    yield();
  }

  Serial.println("\n*** WiFi-ONLY MODE TIMEOUT — rebooting to Zigbee mode... ***\n");
#ifdef BOARD_NESSO_N1
  drawStatusScreen("WiFi-Only Timeout", "Returning to Zigbee", "Rebooting...", TFT_ORANGE);
#endif
  preferences.begin("ota-mode", false);
  preferences.putULong("lastBoot", 0);   //Prevent false OTA detection on next boot
  preferences.end();
  delay(2000);
  ESP.restart();   //Returns to normal Zigbee boot (wifiRequested was already cleared)
}

//***********************************************
//***********************************************
//SETUP
//***********************************************
//***********************************************
void setup() {
  //── Hardware init order note ─────────────────────────────────────────────────
  // BOARD_EQP:      LED_PIN is a direct GPIO; init it first so OTA flashes work early.
  // BOARD_NESSO_N1: LED_PIN (LED_BUILTIN) is on the PI4IOE expander managed by M5GFX.
  //                 It is only usable AFTER display.init(), which is called after Serial.

#ifdef BOARD_EQP
  pinMode(LED_PIN, OUTPUT);                         //MUST be first on EQP — LED used for OTA flashes before normal init
  digitalWrite(LED_PIN, LOW);
#endif

  Serial.begin(115200);
  ulong serialStart = millis();
  while (!Serial && (millis() - serialStart) < 2000) {
    delay(10);
  }
  delay(2000);                                      //Some printout gets missed without this delay

  Serial.println("\n\n\n");
  Serial.println("╔════════════════════════════════════════╗");
  printVersion(40, "Zigbee LIDAR Distance Sensor", "", "");
  printVersion(40, "Version ", VERSION, "");
#ifdef BOARD_EQP
  printVersion(40, "ESP32 Qwiic Pocket board", "", "");
#endif
#ifdef BOARD_NESSO_N1
  printVersion(40, "Arduino Nesso N1 board", "", "");
#endif
  printVersion(40, "Device Name : ", getDeviceName().c_str(), "");
  printVersion(40, "Device ID : ", getChipIdString().c_str(), "");
  Serial.println("╚════════════════════════════════════════╝\n");

#ifdef BOARD_NESSO_N1
  //── Display init (MUST come before any expander pin access) ─────────────────
  //M5GFX autodetects the Nesso N1 and configures the ST7789P3 panel,
  //PI4IOE5V6408 expander (backlight/reset), and FT6336U touch in one call.
  //LED_BUILTIN and Front/Side buttons are expander pins; only usable after this.
  display.init();
  display.setRotation(1);             //Landscape, USB-C on right
  display.setBrightness(255);

  pinMode(FRONT_BUTTON, INPUT_PULLUP);
  pinMode(SIDE_BUTTON,  INPUT_PULLUP);
  //NOTE: do NOT call pinMode(LED_PIN, OUTPUT) on Nesso N1.
  //LED_BUILTIN is a PI4IOE expander pin managed by Arduino_Nesso_N1 internally.
  //Calling pinMode() on it disrupts the expander and kills the display backlight.
  digitalWrite(LED_PIN, LOW);

  drawStatusScreen("Starting up...", getDeviceName().c_str(), "v" VERSION, TFT_CYAN);
#endif

  //*****************************************
  //CHECK FOR OTA MODE (before anything else)
  //*****************************************
  /*
    ## How to Enter OTA Mode via Power Cycle:
    1. **Plug in power** — device boots and Zigbee starts
    2. **Watch for LED blinking** (6s cycle: 3s on / 3s off) — flag is saved, OTA window is open
    3. **Unplug power** within 30 seconds of the flashes
    4. **Plug in power again** — device enters OTA mode!
    Note: if you don't unplug within the OTA window, the flag clears and normal
    power interruptions are safe.
    On BOARD_NESSO_N1: you can also use Front Button long-press to enter OTA mode directly.
  */

  ulong bootStart = millis();                       //Wait until uptime > 2.5s so checkForOTAMode sees a valid boot
  while (millis() - bootStart < 2500) {
    yield();
  }

  //Direct OTA flag: set by doRebootToOTA() or Front Button long-press (Nesso N1);
  //works with any reset reason (not just power-on).
  preferences.begin("ota-mode", false);
  bool directOTARequested = preferences.getBool("directOTA", false);
  if (directOTARequested) preferences.putBool("directOTA", false);  //Consume immediately
  preferences.end();

  bool otaModeRequested = directOTARequested || checkForOTAMode();

  //========= Check for WiFi-only mode (triggered by Zigbee switch "On" command) =========
  //  The zbWiFiSwitch callback (endpoint 12) saves this flag before rebooting.
  //  If set, skip Zigbee entirely and run WiFi-only mode.
  {
    preferences.begin("radio-mode", false);
    bool wifiModeRequested = preferences.getBool("wifiRequested", false);
    int  wifiRetryCount    = preferences.getInt("wifiRetries", 0);
    //Do NOT clear the flag here; runWiFiOnlyMode() manages it based on retry outcome
    preferences.end();
    Serial.printf("WiFi-only mode flag: %s\n", wifiModeRequested ? "SET — entering WiFi-only mode" : "not set");
    if (wifiModeRequested) {
      runWiFiOnlyMode(wifiRetryCount);             //Loops forever on success; reboots on failure
    }
  }

  //========= WiFi for OTA setup =========   //MUST initialize OTA BEFORE setting up Zigbee
  if (otaModeRequested) {
    delay(50);
    Serial.println("╔════════════════════════════════════════╗");
    printVersion(40, "OTA FIRMWARE UPDATE MODE ACTIVE", "", "");
    printVersion(40, "Zigbee is DISABLED for this session", "", "");
    Serial.println("╚════════════════════════════════════════╝\n");
#ifdef BOARD_NESSO_N1
    drawStatusScreen("OTA Update Mode", "Connecting to WiFi...", "Zigbee DISABLED", TFT_YELLOW);
#endif

    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);            //Full TX power — Zigbee is off in OTA mode; maximise link margin for reliable upload

    bool wifiConnected = false;
    for (int attempt = 1; attempt <= 3 && !wifiConnected; attempt++) {
      Serial.printf("\nConnecting to WiFi (%s) — attempt %d of 3...\n", WIFI_SSID, attempt);
      WiFi.disconnect(true);
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      ulong attemptStart = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - attemptStart < 30000) {
        currentLedState = LED_SOS;
        updateZigbeeLED();
        if (millis() % 500 < 10) Serial.print(".");
        delay(5);
      }
      if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
      } else {
        Serial.printf("\nAttempt %d failed (status: %d).%s\n", attempt, WiFi.status(),
          attempt < 3 ? " Retrying in 3s..." : " All attempts exhausted.");
        if (attempt < 3) delay(3000);
      }
    }

    if (wifiConnected) {
      WiFiConnectionInfo();

      //CRITICAL: modem sleep MUST be disabled during OTA.
      //With sleep enabled the radio powers down between beacon intervals (~100 ms gaps),
      //dropping TCP packets mid-upload.  The result is a size/MD5 mismatch in Update.end()
      //which surfaces as "Update failed — could not activate the firmware".
      WiFi.setSleep(false);

      int otaRSSI = WiFi.RSSI();
      Serial.printf("[OTA] WiFi RSSI: %d dBm — %s\n", otaRSSI,
        otaRSSI >= -60 ? "Excellent" :
        otaRSSI >= -70 ? "Good" :
        otaRSSI >= -80 ? "Fair (upload may be slow)" :
                         "Poor — OTA likely to fail, move closer to AP");
#ifdef BOARD_NESSO_N1
      drawStatusScreen("OTA Update Mode",
                       ("http://" + WiFi.localIP().toString() + "/update").c_str(),
                       "Zigbee OFF", TFT_YELLOW);
#endif

      server.on("/", HTTP_GET, []() {
        String btns = "<a class='btn' href='/update'>&#8593; Upload OTA Firmware</a>"
                      "<a class='btn' href='/reboot-wifi'>&#8635; Reboot WiFi</a>"
                      "<a class='btn btn-red' href='/zigbee-mode'>&#8634; Return to Zigbee Mode</a>";
        server.send(200, "text/html", buildRootPage("<b>OTA Update Mode</b> (Zigbee OFF)", "", btns));
      });

      server.on("/reboot-wifi", HTTP_ANY, []() {
        server.send(200, "text/plain", "Rebooting WiFi connection...");
        Serial.println("\n*** WIFI REBOOT REQUESTED VIA HTTP (OTA mode) — rebooting back into OTA mode... ***\n");
        server.client().clear();
        server.client().stop();
        //Use doRebootToOTA() so the device comes back in OTA mode (not WiFi-only mode).
        doRebootToOTA();
      });
      server.on("/zigbee-mode", HTTP_ANY, []() {
        server.send(200, "text/plain", "Rebooting into Zigbee mode...");
        Serial.println("\n*** ZIGBEE MODE REQUESTED VIA HTTP (OTA mode) — rebooting... ***\n");
        server.client().clear();
        server.client().stop();
        doRebootToZigbee(false);  //false = don't clear wifiRequested (not in WiFi-only mode)
      });

      server.on("/progress", HTTP_GET, []() {       //Kept as a stub so stale browser tabs don't get 404
        server.send(200, "application/json", "{\"removed\":true}");
      });

      ElegantOTA.onStart([]() {
        Serial.println("\nOTA update started - clearing rapid reboot flag...");
#ifdef BOARD_NESSO_N1
        drawStatusScreen("OTA Update", "Uploading...", "", TFT_YELLOW);
#endif
        preferences.begin("ota-mode", false);
        preferences.putULong("lastBoot", 0);
        preferences.end();
      });

      ElegantOTA.onProgress([](size_t current, size_t total) {
        static ulong lastPrint = 0;  ulong now = millis();
        if (now - lastPrint >= 1000 || current == total) {
          lastPrint = now;
          size_t fwSize = Update.size();   // Firmware total, set by ElegantOTA via Update.begin()
          int pct = (fwSize > 0) ? (int)(100UL * total / fwSize) : 0;
          Serial.printf("[OTA] Progress: %3d%% | %7u / %7u bytes written | Free heap: %u bytes\n",
                        pct, total, fwSize, ESP.getFreeHeap());
        }
      });

      ElegantOTA.onEnd([](bool success) {
        if (success) {
          Serial.println("OTA update successful! Rebooting to normal mode...\n");
#ifdef BOARD_NESSO_N1
          drawStatusScreen("OTA Complete", "Rebooting...", "", TFT_GREEN);
#endif
        } else {
          Serial.println("OTA update failed!\n");
#ifdef BOARD_NESSO_N1
          drawStatusScreen("OTA FAILED", "Try again", "", TFT_RED);
#endif
        }
      });

      ElegantOTA.begin(&server);
      server.begin();

      Serial.println("\n*** OTA Mode Ready! ***");
      Serial.print("Upload firmware at: http://");
      Serial.print(WiFi.localIP());
      Serial.println("/update\n");
    } else {
      Serial.println("\nWiFi connection failed after 3 attempts. OTA not available.\n");
    }

    if (wifiConnected) {
      Serial.println("╔════════════════════════════════════════╗");
      printVersion(40, "Update firmware at:", "", "");
      printVersion(40, "http://", (WiFi.localIP().toString() + "/update").c_str(), "");
      Serial.println("╚════════════════════════════════════════╝\n");
      Serial.println("Device will automatically reboot after firmware update\n");
      Serial.println("Waiting for OTA update...\n");

      ulong otaStartTime = millis();
      const ulong OTA_TIMEOUT = OTA_MAX_TIME * 60 * 1000;

      while (millis() - otaStartTime < OTA_TIMEOUT) {
        server.handleClient();
        ElegantOTA.loop();

#ifdef BOARD_NESSO_N1
        //── Front Button long-press in OTA mode: return to Zigbee mode ──────────
        if (digitalRead(FRONT_BUTTON) == LOW) {
          delay(100);
          if (digitalRead(FRONT_BUTTON) == LOW) {
            ulong btnStart = millis();
            bool  msgShown = false;
            while (digitalRead(FRONT_BUTTON) == LOW) {
              yield();
              if (!msgShown && (millis() - btnStart > 500)) {
                beepTwice();
                Serial.printf("\nFront button held - hold for %d s to return to Zigbee mode...\n", BUTTON_HOLD_TIME / 1000);
                drawStatusScreen("Front Button held...",
                                 ("Hold " + String(BUTTON_HOLD_TIME / 1000) + "s for Zigbee mode").c_str(),
                                 "Release to cancel", TFT_YELLOW);
                msgShown = true;
              }
              if (millis() - btnStart > BUTTON_HOLD_TIME) {
                beepTwice();
                Serial.println("\n*** ZIGBEE MODE via front button long press (OTA mode) ***");
                drawStatusScreen("Returning to Zigbee", "Rebooting...", "", TFT_GREEN);
                doRebootToZigbee(false);
                return;
              }
              delay(50);
            }
            if (msgShown) {
              Serial.println("Front button released early - Zigbee mode cancelled");
              drawStatusScreen("OTA Update Mode",
                               ("http://" + WiFi.localIP().toString() + "/update").c_str(),
                               "Zigbee OFF", TFT_YELLOW);
            }
          }
        }
#endif  //BOARD_NESSO_N1 OTA front-button handler

        currentLedState = LED_SOS;                    //SOS pattern while waiting for OTA upload
        updateZigbeeLED();

        static ulong lastMinutePrint = 0;
        if (millis() - lastMinutePrint > 60000) {
          lastMinutePrint = millis();
          ulong remaining = (OTA_TIMEOUT - (millis() - otaStartTime)) / 60000;
          Serial.printf("OTA mode timeout in %lu minutes...\n", remaining);
        }

        vTaskDelay(1);                                //Yield to WiFi/TCP FreeRTOS background tasks — essential on ESP32-C6
        yield();
      }

      Serial.println("\n*** OTA MODE TIMEOUT - No update received ***");
      Serial.println("Rebooting to normal operation mode in 3 seconds...\n");
#ifdef BOARD_NESSO_N1
      drawStatusScreen("OTA Timeout", "No update received", "Rebooting...", TFT_ORANGE);
#endif
      delay(3000);
      ESP.restart();

    } else {
      Serial.println("\nERROR: WiFi connection failed after 3 attempts in OTA mode!");
      Serial.println("Rebooting to Zigbee mode in 10 seconds...\n");
#ifdef BOARD_NESSO_N1
      drawStatusScreen("OTA WiFi Failed", "Returning to Zigbee", "Rebooting...", TFT_RED);
#endif
      delay(10000);
      ESP.restart();
    }
  }

  //---- Normal operation continues below ----
  Serial.println("\nNormal operation mode ...\n");

  //── I2C setup ────────────────────────────────────────────────────────────────
  // BOARD_EQP:      Wire1 on GPIO 6/7 (Qwiic connector).
  // BOARD_NESSO_N1: Wire (I2C0) on GPIO 10 (SDA) / GPIO 8 (SCL) is the one I2C
  //                 bus shared by M5GFX, FT6336 touch, BMI270 IMU, and the Qwiic
  //                 connector (per TPX00227 pinout p.3). M5GFX initializes Wire
  //                 at boot. Do NOT call Wire.begin() here — doing so triggers a
  //                 hardware I2C reset that disrupts the bus. Wire1 is unavailable
  //                 (GPIO 10/8 are LP I2C pins, not usable as I2C1).
  //
  // sensorBus is a global TwoWire* so reinitSensor() can reach it without
  // needing setup()'s local scope.
  Serial.println("Initializing I2C...");
#ifdef BOARD_EQP
  sensorBus = QWIIC_CONNECTED ? &Wire1 : &Wire;
  sensorBus->begin(SENSOR_SDA, SENSOR_SCL);
  sensorBus->setClock(400000);
#endif
#ifdef BOARD_NESSO_N1
  sensorBus = &Wire;  // Already initialized by M5GFX on GPIO 10/8
#endif
  delay(1000);  //Some sensors need a delay after I2C init before they respond

  //── Optional output pin ───────────────────────────────────────────────────────
  if (OUTPUT_PIN >= 0) {
    pinMode(OUTPUT_PIN, OUTPUT);
    digitalWrite(OUTPUT_PIN, LOW);
  }

  //========= Distance sensor setup =========
  Serial.println("\nInitializing distance sensor...");
#ifdef BOARD_NESSO_N1
  drawStatusScreen("Initializing...", "VL53L1X sensor", "Looking for device...", TFT_CYAN);
#endif
  if (distanceSensor.begin(*sensorBus) == 0) {
    Serial.println("Distance sensor detected!");
  } else {
    Serial.println("Distance sensor not found. Check wiring and relevant parameter values!");
    while (1) delay(1000);
  }

  Serial.println("Configuring sensor...");
  if (DISTANCE_LONG_MODE) distanceSensor.setDistanceModeLong();
  else                    distanceSensor.setDistanceModeShort();

  int mode = distanceSensor.getDistanceMode();
  if      (mode == 1) Serial.println("\tSensor set to Short range mode");
  else if (mode == 2) Serial.println("\tSensor set to Long range mode");
  else                Serial.println("\tUnknown range mode");

  distanceSensor.setTimingBudgetInMs(TIMING_BUDGET);
  Serial.print("\tTiming budget expected value           : "); Serial.println(TIMING_BUDGET);
  Serial.print("\tTiming budget as set (ms)              : "); Serial.println(distanceSensor.getTimingBudgetInMs());

  distanceSensor.setIntermeasurementPeriod(INTERMEASUREMENT_PERIOD);
  Serial.print("\tIntermeasurement period expected value : "); Serial.println(INTERMEASUREMENT_PERIOD);
  Serial.print("\tIntermeasurement period as set (ms)    : "); Serial.println(distanceSensor.getIntermeasurementPeriod());

  distanceSensor.startRanging();
  measurementInProgress = true;

  //========= Web Server route registration =========
  //Routes are registered here, before WiFi or Zigbee start.
  //server.begin() is called after WiFi connects (in WiFi-only or OTA mode paths).

  server.enableCORS(true);

  server.on("/", HTTP_GET, []() {
    String extra = "<tr><td>Zigbee Connected</td><td>" + String(Zigbee.connected() ? "&#9989; Yes" : "&#10060; No") + "</td></tr>";
    String btns  = "<a class='btn' href='/update'>&#8593; Upload OTA Firmware</a>"
                   "<form style='display:inline' action='/ota-mode' method='post'>"
                   "<button class='btn btn-red' type='submit'>&#9889; Enter OTA Mode</button></form>";
    server.send(200, "text/html", buildRootPage("<b>Zigbee-Only</b> (WiFi OFF)", extra, btns));
  });

  server.on("/ota-mode", HTTP_ANY, []() {
    if (server.method() == HTTP_POST || server.method() == HTTP_GET) {
      Serial.println("\n*** OTA MODE REQUESTED VIA HTTP ***");
      server.send(200, "application/json",
        "{\"status\":\"entering_ota_mode\",\"message\":\"Device will reboot into OTA mode shortly\"}");
      server.client().clear();
      server.client().stop();
      doRebootToOTA();
    } else {
      server.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
    }
  });

  server.on("/zigbee-mode", HTTP_ANY, []() {
    //Convenience alias in normal mode; device is already in Zigbee mode.
    server.send(200, "text/plain", "Already in Zigbee mode. No action taken.");
    Serial.println("[/zigbee-mode] Already in Zigbee mode — no action taken.");
  });

  ElegantOTA.begin(&server);

  //========= Zigbee setup — MUST come before WiFi.begin() =========
  //  The Zigbee stack crashes if the WiFi stack is running when Zigbee.begin() is called.
  //  WiFi is started AFTER Zigbee.begin() completes to avoid this conflict.
  Serial.println("\nConfiguring Zigbee endpoint...");
#ifdef BOARD_NESSO_N1
  drawStatusScreen("Starting Zigbee...", "Configuring endpoints", "", TFT_CYAN);
#endif

  //All endpoints share the same model string. Hubitat reads from whichever endpoint it
  //fingerprints on (ep12 for this device). Model string MUST be ≤32 chars or
  //setManufacturerAndModel() silently fails on ALL endpoints.
  zbDistance.setManufacturerAndModel(ZB_MANUFACTURER, zbModelString.c_str());
  zbDistance.setMinMaxValue(0, 4000);
  zbDistance.setTolerance(10);

  zbRaw.setManufacturerAndModel(ZB_MANUFACTURER, zbModelString.c_str());
  zbRaw.setMinMaxValue(0, 4000);
  zbRaw.setTolerance(2);              //Endpoint reports every 2mm change

  zbWiFiSwitch.setManufacturerAndModel(ZB_MANUFACTURER, zbModelString.c_str());

  Serial.println("Adding Zigbee endpoints...");
  Zigbee.addEndpoint(&zbDistance);    //Endpoint 10: smoothed distance values (Illuminance cluster)
  Zigbee.addEndpoint(&zbRaw);         //Endpoint 11: raw distance values (Illuminance cluster)

  //Register the WiFi-mode switch endpoint (endpoint 12).
  //When Hubitat sends an ON command to this endpoint, the device saves a flag to
  //Preferences and reboots into WiFi-only mode (Zigbee is never started on that boot).
  //To return to Zigbee mode, send a GET request to http://<device-ip>/zigbee-mode.
  zbWiFiSwitch.onLightChange([](bool on) {
    if (on) {
      Serial.println("\n*** WiFi-only mode requested via Zigbee — saving flag and rebooting... ***\n");
      preferences.begin("radio-mode", false);
      preferences.putBool("wifiRequested", true);
      preferences.end();                          //end() flushes the NVS write
      preferences.begin("ota-mode", false);
      preferences.putULong("lastBoot", 0);        //Prevent false OTA detection on next boot
      preferences.end();
      delay(1000);
      ESP.restart();
    }
    //Off command is ignored; use /zigbee-mode HTTP endpoint to return from WiFi-only mode
  });
  Zigbee.addEndpoint(&zbWiFiSwitch);              //Endpoint 12: WiFi mode toggle

  Serial.println("Link key exchange disabled for better C-8 Pro pairing compatibility");
  esp_zb_secur_link_key_exchange_required_set(false);

  esp_coex_preference_set(ESP_COEX_PREFER_BT);    //Give Zigbee full radio priority during init
                                                  //"BT" covers 802.15.4 on the ESP32-C6
  Serial.println("Starting Zigbee (End Device mode)...");
  Zigbee.begin();

  bool quickConnect = false;
  for (int i = 0; i < 100; i++) {
    Serial.print(".");
    delay(100);
    yield();
    vTaskDelay(1);
    if (Zigbee.connected()) { quickConnect = true; break; }
  }
  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);  //Restore balanced coexistence after Zigbee init
  Serial.println("\nZigbee started successfully!");

  //Zigbee started — set the OTA flag and flash LED 3 times.
  //User can unplug immediately after the flashes to enter OTA mode on next boot.
  setOTAFlag();

  if (quickConnect) {
    Serial.println("\nAlready commissioned - reconnected to Zigbee network!");
#ifdef BOARD_NESSO_N1
    drawStatusScreen("Zigbee Connected", getDeviceName().c_str(), "v. " VERSION, TFT_GREEN);
    display.fillRect(0, 114, DISPLAY_W, 21, display.color565(20, 20, 20));
    display.setFont(&fonts::Font2);
    display.setTextColor(TFT_WHITE);
    display.setCursor(4, 117);
    display.print("Zigbee");
#endif
  } else {
    Serial.println("\n========================================");
    Serial.println("Put Hubitat in Zigbee pairing mode now!");
    Serial.println("========================================\n");
#ifdef BOARD_NESSO_N1
    drawStatusScreen("Waiting for Zigbee", "Put Hubitat in", "pairing mode now!", TFT_YELLOW);
#endif
  }

  //========= WiFi is OFF in normal Zigbee mode =========
  //  WiFi running alongside Zigbee on the ESP32-C6 causes radio contention that
  //  prevents reliable WiFi association. The radios are kept mutually exclusive:
  //  - Normal operation: Zigbee ON, WiFi OFF
  //  - WiFi-only mode:   WiFi ON,  Zigbee OFF (enter via Hubitat switch or power-cycle OTA trick)

  WiFi.mode(WIFI_OFF);
  Serial.println("\nWiFi is OFF. Zigbee-only mode active.");
  Serial.println("To enable WiFi, use the Hubitat 'Enable WiFi Mode' command button (Zigbee endpoint 12)");
  Serial.println("  or use the power-cycle OTA trigger: plug in, watch for 6s-cycle LED blink, unplug within 15s, plug in.");
  Serial.println("\n========================================");
}

//***********************************************
//LOOP
//***********************************************
void loop() {
  //Auto-clear OTA window flag OTA_WINDOW_MS seconds (initially 15s) after Zigbee starts. 
  //setOTAFlag() sets it each boot; this gives a window to unplug for OTA mode, 
  //then clears it so normal power interruptions after that don't accidentally 
  //trigger OTA mode on the next boot.
  static bool otaWindowClosed = false;
#ifdef BOARD_NESSO_N1
  static bool otaFlashState   = false;  //Tracks current display flash phase (true=red, false=black)
#endif

  if (!otaWindowClosed) {
    if (millis() > OTA_WINDOW_MS) {     //OTA window expired
      preferences.begin("ota-mode", false);
      preferences.putULong("lastBoot", 0);
      preferences.end();
      otaWindowClosed = true;
      //Set LED state based on actual connection: SOLID if already connected, BLINKING if not.
      //Cannot rely on updateZigbeeLED()'s rising-edge detection here because lastConnected
      //is already true when we were connected before the OTA window opened.
      currentLedState = Zigbee.connected() ? LED_SOLID : LED_BLINKING;
      Serial.println("\nOTA power cycle window elapsed — OTA flag cleared. Normal power interruptions are safe.\n");
#ifdef BOARD_NESSO_N1
      drawDistanceScreen();             //Restore normal display immediately
#endif
    } else {
      currentLedState = LED_SOS;        //OTA window open — SOS pattern signals OTA-ready
#ifdef BOARD_NESSO_N1
      bool redPhase = (millis() % (OTA_WINDOW_FLASH_MS * 2)) < OTA_WINDOW_FLASH_MS;
      if (redPhase != otaFlashState) {  //Only redraw on phase transitions
        otaFlashState = redPhase;
        drawOTAWindowScreen(redPhase);
      }
#endif
    }
  }

  checkZigbeeWatchdog();                              //Reboot to force clean Zigbee rejoin if disconnected too long

  bool isConnected = updateZigbeeLED();               //Always call; LED_SOS is not auto-overridden by Zigbee connection state

  handleDistanceMeasurement(isConnected);             //======= Main distance reading and reporting loop =======

#ifdef BOARD_NESSO_N1
  //── Refresh display — only when new sensor data has arrived ─────────────────
  //  Avoids flooding the SPI bus with redundant redraws between readings,
  //  which was starving the Zigbee FreeRTOS tasks and causing SW_CPU resets.
  if (otaWindowClosed && newReadingAvailable) {
    newReadingAvailable = false;
    drawDistanceScreen();
  }

  //── Mode 3: alternating display timer ───────────────────────────────────────
  //  Flips between avg (green) and raw (yellow) every LARGE_DISPLAY_ALT_S seconds.
  //  Timer is armed on the first valid reading to avoid an instant flip at boot.
  if (displayMode == 3 && otaWindowClosed && lastReadValid) {
    ulong now = millis();
    if (lastAltSwitch == 0) {
      lastAltSwitch = now;              //Arm timer — no flip yet
    } else if (now - lastAltSwitch >= (ulong)LARGE_DISPLAY_ALT_S * 1000) {
      altShowAvg    = !altShowAvg;
      lastAltSwitch = now;
      drawDistanceScreen();
    }
  }
#endif  //BOARD_NESSO_N1 display refresh

  checkButtonPress();                                 //Handle button input (platform-specific)

  vTaskDelay(1);                                      //Allow Zigbee FreeRTOS tasks to run — essential on ESP32-C6
  yield();
}
