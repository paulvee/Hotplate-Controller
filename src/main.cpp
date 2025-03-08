#include <Arduino.h>  // so we can use the Arduino specifics (port read/write, Byte, String, etc.)

// Custom DIY reflow controller with graphical user interface and proportional temperature control
// The code belongs to the following YouTube video:
// https://curiousscientist.tech/blog/reflow-hot-plate-update-2024
// Original design by Curious Scientist: https://curiousscientist.tech/
// Shared for non-commercial purposes only. If you have plans for commercial use or want to charge money for it,
// send an email to: contact@curiousscientist.tech
//
// Modifications by paulv 2024-2025
// Adopting the code for a commercial hotplate UYUE 946C 400W 200x200mm

//
const String FW_VERSION = "V5.6.2";  // Firmware version
/*
  Changelog:
  Version V2.0.0:
  original code with updated splash screen

  Version V2.1.0:
  fixing the use of no-name TFT displays, but that requires changing one of the library files.

  Version V2.2.0:
  fixing the drawing of the curve when there is no change
  fixing the field color while in the edit mode and setting it back when exiting the edit mode
  changed the button detection to an ISR to get a better response

  Version V2.3.0:
  Fixed the no-name TFT display color code definitions and replaced all ST77XX definitions to ST7735.
  The library file does no longer need to get edited.

  Version V3.0.0:
  Port to the ESP32 DEVKIT 1 with schematic V2.0
  Changed most of the pin assignments so we can use hardware SPI and use a faster SPI clock.
  This will dramatically speed-up the tft activities, and fixed the screen1/2 redraw with fast encoder activities
  Changed the button code back to (delayed) polling, cannot use an ISR with the ESP and this code due to the many tft activities
  The rotary ISR now required a Schmitt-trigger gate on the CLK pin to eliminate many triggers in the undetermined zone of the edge

  Version V3.1.0:
  Changed the rotary button service to a library, changed the processing
  Fixed the depricated var++ and var-- increments and decrements
  Changed references from Triac to SSR

  Version 4.0.0:
  Changed the display to a Waveshare 2.4" 240x320 with the ILI9341 controller.
  Made the TFT colors easier to assign
  Changed the SSR control to on-off and removed the PID code
  Added "C" and "s" to the temp and time values on the reflow curve
  Moved the cooling temp & time a little up and right to avoid the clobbering of the curves
  Change the cooling temp from red to blue
  Changed hard-coding of x-y limits of TFT display to const vars
  Changed the MAX7765 code to use a library that has more posibilities

  Version 5.0.0:
  Changed the TFT library to TFT_eSPI.h to add more features and make it faster

  Version 5.1.0:
  Redid the screen layout and deleted the second screen.
  There are now buttons on the main screen for free heating and free cooling.
  You can set the maximum temp for free heating and the minimum temp for free cooling.
  Both free heating and free cooling show the actual curve of temp vs time.
  Used drawString() a lot for more flexibility and for all font 2 sizes.
  Changed the layout of the axis and made them larger.
  Added tick marks and values to the axis.
  Moved the actual temperature, target temperature and actual timing displays to new places.
  Started preparation for the selection of a second (or more) solder paste.
  Added degree symbol "°C" for the larger font 2
  Split out code that was used in multiple places.
  Many small enhancements and many more comments and clean-ups

  Version 5.2.0:
  Switched to Virtual Studio Code and PlatformIO, and started using Git for version control.
  Also used the Copilot AI to help with the code for the DrawGraph() function.
  It is now using a more natural curve instead of straight lines between the points,
  which will make it easier to follow for the heaters.
  Added a run-time selection of solder pastes.
  Moved the preheat temp and time fields a bit out of the way of the curve.
  Removed the free heating and free cooling fields from the display when we reflow.

  Version 5.3.0
  Added a "warm-up" phase for the free heating, so we can heat up the board before we start the reflow.
  Added PID controller for the heating elements.
  Fixed the highlighting issue with the buttons. When the action was stopped, the field was not highlighted.
  Fixed an error in the display curve that was using the old offset for the x/y axis.
  Changed the runReflow() function a bit so it can be ran as a simulation at 10x the speed, without actual heating
  Changed the runReflow() function cooling phase and let it run until we are at the end of the display (340s)
  Changed the runReflow() function with updated actual temperature and time display calculations.
  Changed the updateStatus() code so it can handle a single line status message and set the text color and the background
  color of the status message. Setting both to black will erase the field.
  Changed the drawActionButtons() code so it can handle a single line status message and set the text color and the background

  Version 5.4.0
  Switched back to using an a PWM-based control for the heaters. The PID method does not give any benefits and the PID controller
  is too slow, the heater too powerfull and the overshoot especially at the end of the reflow phase is too large.
  The PWM method is much better, but there is still a lot of overshoot after the reflow phase, with the temperature
  going up to 250°C and then dropping back to 200°C. This may not a problem for the solder paste I'm using, but it
  might be.

  Version 5.4.2
  Implemented some kind of early prediction for the preheat and reflow modes to stop the heating a bit earlier,
  so the overshoot is less.
  Used a boost mode for the first 30 seconds of the reflow phase to get the temperature up faster, because the profile
  starts at zero degrees, while the actual temperature is room temperature. Otherwise we already start behind the curve.
  Cleaned-up the old code and revisited or added to the comments and the code layout.

  Version 5.4.3
  Repositioned the cooling temp and time fields to below the curve so with higher paste profiles they don't overlap.
  Went back to try the PID mode, it should be perfect for the free heating and warm-up modes.

  Version 5.5.0
  After many tests, went back from using a PID regulation to a PWM-based mode. worked around the issues and tuned the modes.
  Changed the origin from the reflow graph to start at the estimated room temperature. Added mode status fields like in the
  reflow mode. Tuned the reflow model. Added the cooling phase to the reflow model. Added a status field for the PWM value.
  Refactored the source code to make it more readable, moved functions around and added more comments.

  Version 5.5.1
  Added powercontrol for the TFT. It eliminates the white screen during the booting period.

  Version 5.6.0
  Added an ISR for the rotary button now that I use hardware to produce a clean edge.
  Added labels for the X-Y chart axis.

  Version 5.6.1
  Changed the MAX6675 library to the MAX31855 library.
  I need the MAX6675 chip so I can solder it on the PCB, but want to use the reflow controller
  to do that. The MAX31855 is a drop-in replacement for the MAX6675, but with a 14-bit resolution.
  However, during the soldering I noticed quite a bit of noise on the temperature readings.
  I don't think that the MAX31855 is a good replacement for the MAX6675, so I will go back to the MAX6675

  Version 5.6.2
  Now that the hardware is working, I changed the code to use the MAX6675 library again.
  I also added code to keep the power to the TFT off, so we avoid the several seconds of a white screen
  during the booting of the ESP32.


  Todo:
  No open or desired issues at the moment.

  Nice to have:
  Maybe add a way to store an updated/edited profile in EEPROM and load that at boot.


*/

// Include the libraries we need
#include <SPI.h>
#include <TFT_eSPI.h>  // 2,4" SPI 240x320 - https://github.com/Ambercroft/TFT_eSPI/wiki
#include <math.h>      // for the round() function

#include "MAX6675.h"  // can also use the 14-bit MAX31855K, but the 12-bit MAX6675 is cheaper and works fine
// #include "MAX31855.h"  // 14-bit version of the MAX6675

#define DSO_TRIG 4  // optional: to trace real-time activity on a scope

#define RotaryCLK 27  // CLK pin on the rotary encoder (must be an interrupt pin)
#define RotaryDT 32   // DT pin on the rotary encoder
#define RotarySW 33   // SW pin on the rotary encoder (Button function)

#define SSR_pin 2   // Switching the heater ON/OFF; also the built-in LED so we can see when the SSR is on.
#define Fan_pin 26  // GPIO pin for switching the fans ON/OFF (via a transistor)

#define MAX_CS 13  // CS pin for the MAX6675K
#define MAX_SO 21  // MISO for MAX6675
#define MAX_CLK 3  // SPI clock

// the definitions below are now defined in the platformio.ini file
// #define TFT_MOSI  23  // TFT SDA conn pin 3
// #define TFT_CLK   18  // TFT SCL conn pin 4
// #define TFT_CS    5   // TFF CS conn pin 5
// #define TFT_DC    16  // TFT DC conn pin 6
// #define TFT_RST   17  // TFT RES conn pin 7
// #define TFT_MISO  19  // TFT not used
#define TFT_ON 15  // TFT power and backlight control

// function prototypes
void setup();
void loop();
void rotaryButtonISR();
void rotaryEncoderISR();
void processRotaryButton();
void updateHighlighting();
void runReflow();
void freeHeating();
void freeCooling();
void runWarmup();
void drawAxis();
void drawCurve();
// next three are optional to replace drawCurve() using straight lines to
// curved lines
float cosineInterpolate(float, float, float);        // optional
void drawSmoothCurve(int, int, int, int, uint16_t);  // optional
void drawCurve_new();                                // optional, rename to drawCurve(), and rename
// drawCurve() to something else so it no longer gets called
void drawFreeCurve();
void drawReflowCurve();
void drawActionButtons();
void measureTemperature();
void removeFieldsFromDisplay();
void updateStatus(int, int, const char*);
void printTemp();
void printTargetTemperature();
void printElapsedTime();
void printPWM();
void printFan();

// setup the MAX library
// MAX31855 thermoCouple(MAX_CS, MAX_SO, MAX_CLK);
MAX6675 thermoCouple(MAX_CS, MAX_SO, MAX_CLK);

// Constructor for the TFT screen
// using hardware SPI
TFT_eSPI tft = TFT_eSPI();
const int tftX = 320;          // the width of the TFT
const int tftY = 240;          // the height of the TFT
const int yGraph = tftY - 13;  // the bottom of the graph
const int xGraph = 18;         // the left side of the graph

// Graph characteristics for the 240x320 TFT
// graph area is 18px from the right and 15px from the bottom, we leave 2px free from the right and 60 from the top
// width: (320)  x: [18-318] -> 300px
// height: (240) y: [15-237] -> 222px
// temperature range: 0°C to 250°C
// time range: 0 s to 330 s -> 330 s (5 1/2min)
// The coordinate system of the display has inverted Y-axis.
//  Default small font is 6px wide and 8px tall
//  font 2 size is 7px wide and 10px tall

// different display libraries use special color names. To make editing
// and selecting other libraries easier, I use the definitions below for
// the colors used in this sketch.
#define BLACK TFT_BLACK
#define WHITE TFT_WHITE
#define RED TFT_RED
#define GREEN TFT_GREEN
#define ORANGE TFT_ORANGE
#define MAGENTA TFT_MAGENTA
#define BLUE TFT_BLUE
#define YELLOW TFT_YELLOW
#define CYAN TFT_CYAN
#define DGREEN 0x046B   // dark green
#define VLGREY 0xDF3D   // very light grey
#define BGGREEN 0xD75C  // background green
#define DGREY TFT_DARKGREY

// color picker: https://barth-dev.de/online/rgb565-color-picker/

// predefined colors in TFT_eSPI.h, can be used as is
// #define TFT_BLACK       0x0000      /*   0,   0,   0 */
// #define TFT_NAVY        0x000F      /*   0,   0, 128 */
// #define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
// #define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
// #define TFT_MAROON      0x7800      /* 128,   0,   0 */
// #define TFT_PURPLE      0x780F      /* 128,   0, 128 */
// #define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
// #define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
// #define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
// #define TFT_BLUE        0x001F      /*   0,   0, 255 */
// #define TFT_GREEN       0x07E0      /*   0, 255,   0 */
// #define TFT_CYAN        0x07FF      /*   0, 255, 255 */
// #define TFT_RED         0xF800      /* 255,   0,   0 */
// #define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
// #define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
// #define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
// #define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
// #define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
// #define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F
// #define TFT_BROWN       0x9A60      /* 150,  75,   0 */
// #define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
// #define TFT_SILVER      0xC618      /* 192, 192, 192 */
// #define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
// #define TFT_VIOLET      0x915C      /* 180,  46, 226 */

#define OFF 0
#define ON 255  // for the analogWrite() PWM function

// Rotary encoder related
int selectedItem = 1;  // item number for the active menu item
static bool ButtonPressed = false;
volatile bool buttonPressedFlag = false;  // flag to tell the code that the button was pressed

// Statuses of the DT and CLK pins on the encoder
int CLKNow;
int CLKPrevious;
int DTNow;
int DTPrevious;

//---Thermocouple MAX6675
int TCRaw = 0;                       // raw value coming from the thermocouple module
double TCCelsius = 0;                // Celsius value of the temperature reading
unsigned long temperatureTimer = 0;  // Timer for measuring the temperature

bool coolingFanEnabled = false;  // status that tells the code if the fan is enabled or not
String Fan = "OFF";
bool heatingEnabled = false;  // tells the code if the heating was enabled or not
bool reflow = false;          // tells if the reflow process has been enabled
bool enableFreeHeating = false;
bool enableFreeCooling = false;
bool enableWarmup = false;
unsigned long SSRTimer = 0;       // Timer for switching the SSR
unsigned long SSRInterval = 250;  // 250ms; update interval for switching the SSR
double elapsedHeatingTime = 0;    // Time spent in the heating phase (unit is ms)
double earlyStop;                 // slow down the heating process just before reaching targetTemp
double Output;                    // holds the value for the PWM output to the SSR

// ==================================================================
// Reflow Curve parts for Chipquick Sn42/Bi57.6/Ag0.4 - 138C : I have this paste in a syringe
String pasteName = "Sn42/Bi57.6/Ag0.4";

// these vars will hold the various paste reflow curve values
// From room temperature to soaking temperature
volatile int preheatTemp = 0;
volatile int preheatTime = 0;  // Note: times are always as compared to zero and not the length of the process.

// Soaking temperature (nearly flat curve)
volatile int soakingTemp = 0;
volatile int soakingTime = 0;

// Soaking temperature to peak temperature (slight overshoot to peak temperature)
volatile int reflowTemp = 0;
volatile int reflowTime = 0;

// Cooling temperature - Same temperature as the peak, because this part is more like keeping the solder around Tmelt for a short (~10s) time
volatile int coolingTemp = 0;
volatile int coolingTime = 0;

// struct to group the used values for each solderpaste variation
struct solderpaste {
    char pasteName[30];
    volatile int preheatTemp;  // Temps in degrees C. Preheat is from room temperature to soaking temperature
    volatile int preheatTime;  // Note: times in seconds are always as compared to zero and not the length of the process step.
    volatile int soakingTemp;  // Soaking temperature (nearly flat curve)
    volatile int soakingTime;
    volatile int reflowTemp;  // Soaking temperature to peak temperature (slight overshoot to peak temperature)
    volatile int reflowTime;
    volatile int coolingTemp;  // Cooling temperature - Same temperature as the peak, because this part is more like keeping the solder around Tmelt for a short (~10s) time
    volatile int coolingTime;
};

// Create an array of struct's for the various solder pastes.
// The separator is the number of fields in the struct (9), so the array is created correctly.
// it can hold any number of solderpastes, the code handles that.
// https://www.chipquik.com/store/product_info.php?products_id=473036 for many different pastes and their profiles
//
solderpaste solderpastes[] = {
    // Paste 0
    "Sn42/Bi57.6/Ag0.4",  // Solderpaste
    90,                   // preheatTemp
    90,                   // preheatTime Note: times are always as compared to zero and not the length of the process.
    130,                  // soakingTemp
    180,                  // soakingTime
    165,                  // reflowTemp
    240,                  // reflowTime
    165,                  // coolingTemp
    250,                  // coolingTime start

    // paste 1
    "Sn42/Bi57/Ag1",  // the same as the previous paste, but with a bit more silver
    90,
    90,
    130,
    180,
    165,
    240,
    165,
    250,

    // Paste 2
    "Sn63/Pb37",
    100,
    30,
    150,
    120,
    235,
    210,
    235,
    220,

    // Paste 3
    "Sn63/Pb37 Mod",
    100,
    60,  // changed this from an unrealistic 30s to 60s
    150,
    120,
    235,
    210,
    235,
    220};

int solderPasteSelected = 0;       // hold the index to the array of solderpastes
int prev_solderPasteSelected = 0;  // previous selected solder paste index to avoid screen redraws
int numSolderpastes = 0;           // the number of solderpastes in the array, will be set dynamically based on the size of the array

// Remembers the original value during the editing mode. This is used to eliminate
// a screen redraw when nothing is changed.
volatile int prev_soakingTemp = soakingTemp;
volatile int prev_preheatTemp = preheatTemp;
volatile int prev_preheatTime = preheatTime;
volatile int prev_soakingTime = soakingTime;
volatile int prev_reflowTemp = reflowTemp;
volatile int prev_reflowTime = reflowTime;
volatile int prev_coolingTemp = coolingTemp;
volatile int prev_coolingTime = coolingTime;

// position of the paste name on the TFT screen
const int pasteNamePosX = 50;  // position from the left
const int pasteNamePosY = 1;   // position from the top of the TFT.

double targetTemp = 0;  // A variable that holds the target values temporarily,
// based on the actually calculated part of the active heating phase
volatile int freeHeatingTemp = 200;  // Free heating default target temperature
volatile int freeCoolingTemp = 40;   // Free cooling default target target temperature
volatile int warmupTemp = 38;        // Free heating warm-up default target temperature

// Conversion formulas for converting the physical values (Temp and time) into pixel values.
//  Display is (YxX) 240x320px
//  Y-Axis has 250 degrees max
//  px range is 13 from bottom and 60 from top
double tempPixelFactor = 250.0 / (tftY - (60 + 13));  // y = 250 / 167 = 1.497 ~ 1.5°C per pixel on Y
// X-Axis is shifted by 18 from the bottom.
// 360 seconds max
// px range is 18 from left and 2 from the right (320-2)
double timePixelFactor = 360.0 / (tftX - (18 + 2));  // x = 360 / 300 = 1.2s per pixel on X
// Axis is shifted by 18 from the left and ends 2 pixels before the end of the screen: Available area for plotting: 320-(20) = 300 px.

// Pixel conversions
//- converts the physical values (time, temp) into pixel values for plotting the chart
int preheatTemp_px;
int preheatTime_px;
int soakingTemp_px;
int soakingTime_px;
int reflowTemp_px;
int reflowTime_px;
int coolingTemp_px;
int coolingTime_px;
int measuredTemp_px;
int measuredTime_px;

//---Menu related----
volatile int itemCounter = -1;         // this tells the code the active menu position
volatile int previousItemCounter = 0;  // this tells the code the previous menu position (needed for the highlighting)
bool menuChanged = false;              // it tells if the a new menu item should be drawn
bool menuSelected = false;             // it tells if the user entered a menu (selected an item)
bool editMode = false;                 // Used to avoid the updateHighlighting when in the edit mode

// Selection of the fields by the rotation of the encoder
bool solderpasteFieldSelected = false;
bool preheatTempSelected = false;
bool preheatTimeSelected = false;
bool soakingTempSelected = false;
bool soakingTimeSelected = false;
bool reflowTempSelected = false;
bool reflowTimeSelected = false;
bool coolingTempSelected = false;
bool coolingTimeSelected = false;
bool warmupTempSelected = false;
bool freeWarmUpButtonSelected = false;
bool startStopButtonSelected = false;
bool freeHeatingTargetSelected = false;
bool freeHeatingOnOffSelected = false;
bool freeCoolingTargetSelected = false;
bool freeCoolingOnOffSelected = false;

//--------------------------------------
bool redrawCurve = true;  // tells the code if the reflow curve has to be redrawn
int RectRadius = 2;       // The radius for the rounding of the menu fields

// the reflow phases
enum ReflowPhase {
    PREHEAT = 0,
    SOAK,
    REFLOW,
    HOLD,
    COOLING
};
ReflowPhase currentPhase = PREHEAT;  // Default phase

// forward looking prediction for the heating cut-off in the preheat and reflow phases.
// When the heater is on it is ramping up the temperature. We need to turn the heater off before it
// reaches the target temperature to avoid overshoot due to the inertia of the hardware.
int preheatCutOff = 0;             // the temperature where we want to cut off the heater in the preheat phase
int const preheatCutOffTime = 15;  // cut off the heater 15s before the target temperature
int reflowCutOff = 0;              // the temperature where we want to cut off the heater in the reflow phase
int const reflowCutOffTime = 15;   // cut off the heater 15s before the target temperature
// The actual cutoff times are based on the selected solderpaste and will be defined in setup() or when a
// new solderpaste is selected.

//==================================================

void setup() {
    pinMode(TFT_ON, OUTPUT);    // Define output pin for switching the power to the TFT
    digitalWrite(TFT_ON, LOW);  // Disable power to the TFT

    Serial.begin(9600);
    while (!Serial);
    delay(5000);
    Serial.print("\n\r\n\rReflow controller ");
    Serial.println(FW_VERSION);

    Serial.print("tempPixelFactor = ");
    Serial.println(tempPixelFactor, 3);
    Serial.print("timePixelFactor = ");
    Serial.println(timePixelFactor, 3);

    SPI.begin();  // start hardware SPI

    //------
    // PORT/PIN definitions
    pinMode(DSO_TRIG, OUTPUT);  // optional for tracing real-time events with a DSO
    // Rotary encoder-related
    pinMode(RotaryCLK, INPUT);                                                    // CLK - has pull-up resistor
    pinMode(RotaryDT, INPUT);                                                     // DT - has pull-up resistor
    pinMode(RotarySW, INPUT);                                                     // SW (Button function) has pull-up resistor
    attachInterrupt(digitalPinToInterrupt(RotarySW), rotaryButtonISR, FALLING);   // Attach interrupt to the button pin                                                  // set debounce time for the rotary button to 20 milliseconds
    attachInterrupt(digitalPinToInterrupt(RotaryCLK), rotaryEncoderISR, CHANGE);  // CLK pin is inverted by a Schmitt-trigger gate
    // Reading the current status of the encoder for preparing it for the first movement/change
    CLKPrevious = digitalRead(RotaryCLK);
    DTPrevious = digitalRead(RotaryDT);
    //-----
    pinMode(SSR_pin, OUTPUT);   // Define output pin for switching the SSR
    analogWrite(SSR_pin, OFF);  // SSR is OFF by default
    //----
    pinMode(Fan_pin, OUTPUT);     // Define output pin for switching the fan (transistor)
    digitalWrite(Fan_pin, HIGH);  // Enable fan - turn them on as a test to see if they spin up
    //-----
    Serial.println("setting up tft");
    digitalWrite(TFT_ON, HIGH);  // Enable power for the TFT
    // vTaskDelay(1 / portTICK_PERIOD_MS);  // give it 1ms to initialize
    tft.init();             // Initialize the display
    tft.setRotation(1);     // Select the Landscape alignment - Use 3 to flip horizontally
    tft.fillScreen(BLACK);  // Clear the screen and set it to black
    //-----
    thermoCouple.begin();
    thermoCouple.setSPIspeed(40000000);

    //----- set the initial solderpaste and values
    numSolderpastes = sizeof(solderpastes) / sizeof(solderpaste);  // the size of the array of solderpastes

    solderpaste current = solderpastes[solderPasteSelected];
    // select the first one as the default
    // use the struct on the selected array to access the elements
    // and transpose the values
    pasteName = current.pasteName;
    preheatTemp = current.preheatTemp;
    preheatTime = current.preheatTime;
    soakingTemp = current.soakingTemp;
    soakingTime = current.soakingTime;
    reflowTemp = current.reflowTemp;
    reflowTime = current.reflowTime;
    coolingTemp = current.coolingTemp;
    coolingTime = current.coolingTime;

    //-----
    // forward prediction for the heating cut-off in the preheat and reflow phases
    preheatCutOff = preheatTime - preheatCutOffTime;  // cut off the heater before the target temperature
    reflowCutOff = reflowTime - reflowCutOffTime;     // cut off the heater before the target temperature

    //-----
    Serial.println("show welcome screen on tft");
    tft.setTextColor(WHITE);

    tft.setTextDatum(MC_DATUM);  // center text on display; works on current font only
    tft.drawString("Automated reflow station " + FW_VERSION, tft.width() / 2, 40, 2);
    tft.drawString("2025 paulv", tft.width() / 2, 80, 2);
    tft.drawString("based on code from", tft.width() / 2, 100, 2);
    tft.drawString("www.curiousscientist.tech", tft.width() / 2, 120, 2);
    tft.setTextDatum(TL_DATUM);  // switch back to left formatted

    vTaskDelay(500 / portTICK_PERIOD_MS);  // wait a little to show it

    Serial.println("writing reflow curve");
    // Erase the screen, then draw the starting graph
    tft.fillScreen(BLACK);
    drawReflowCurve();
    drawActionButtons();
    digitalWrite(Fan_pin, LOW);  // Disable fan - turn off the "spinning test" of the fans

    Serial.println("setup is done...");
}

/*
  The main loop of the code
  Statemachines will activate the functionality of the code.
  There are interrupts for the rotary encoder and the rotary button.
  The code will measure the temperature and update the display.
  The reflow, warmup, heating and cooling modes will execute when the user
  selects them.
  The code will also handle the selection of the solderpaste and the editing
  of the reflow profile.

*/
void loop() {
    measureTemperature();
    updateHighlighting();
    runReflow();
    runWarmup();
    freeHeating();
    freeCooling();

    if (buttonPressedFlag) {
        processRotaryButton();
        buttonPressedFlag = false;  // Reset the flag
    }
}

/*
  When the rotary button is pressed, we set a flag to signal the code that the button was pressed.
  The flag is used in the main loop to process the button press.
  The button press will either enter the edit mode for a reflow field to change the temperature or time,
  or you can select another solderpaste or you activate one of the warm-up, reflow, heating, cooling modes.

*/
void IRAM_ATTR rotaryButtonISR() {
    buttonPressedFlag = true;
}

/*
  Interrupt Service Routine for the Rotary Encoder

  This is the routine that is used to move from field to field and when the button is pressed,
  it enters the edit mode in which the information in the field can be changed by rotation.

  The interrupt is generated when a change on the RotaryCLK signal is detected.
  The ISR takes a little time and we also have a hardware r/c delay, so by reading it again now,
  we should have a stable level.

  Depending on the field we're in, we can adjust the value of temp and time

  The recommended method is to limit the execution time in an ISR to the absolute minimum.
  In this case however, we're not expecting other interrupts, and the actual time spent in the ISR
  is very short, despite the many lines of code.

*/
void IRAM_ATTR rotaryEncoderISR() {  // IRAM_ATTR
    // we get here because the ISR detected a change on the CLK pin
    // digitalWrite(DSO_TRIG, HIGH); // track duration, typically 1.5us

    CLKNow = digitalRead(RotaryCLK);  // Read the state of the CLK pin again
    if (preheatTempSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow)  // read the DT level and determine the direction (CW or CCW)
            {
                if (preheatTemp > 20) {
                    preheatTemp = preheatTemp - 1;
                }
            } else {
                if (preheatTemp < 150) {  // typical max value for preheat phase - feel free to change it
                    preheatTemp = preheatTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (preheatTimeSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (preheatTime > 0) {
                    preheatTime = preheatTime - 1;
                }
            } else {
                if (preheatTime < 90) {  // Typical preheat time
                    preheatTime = preheatTime + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (soakingTempSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (soakingTemp > 20) {
                    soakingTemp = soakingTemp - 1;
                }
            } else {
                if (soakingTemp < 180) {  // typical soaking temperature
                    soakingTemp = soakingTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (soakingTimeSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (soakingTime > 0) {
                    soakingTime = soakingTime - 1;
                }
            } else {
                if (soakingTime < 180) {  // typical (total) time at the end of the soaking period
                    soakingTime = soakingTime + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (reflowTempSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (reflowTemp > 0) {
                    reflowTemp = reflowTemp - 1;
                }
            } else {
                if (reflowTemp < 250) {  // typical peak temp for reflow
                    reflowTemp = reflowTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (reflowTimeSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (reflowTime > 0) {
                    reflowTime = reflowTime - 1;
                }
            } else {
                if (reflowTime < 240) {
                    reflowTime = reflowTime + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (coolingTempSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (coolingTemp > 0) {
                    coolingTemp = coolingTemp - 1;
                }
            } else {
                if (coolingTemp < 250) {  // holding temperature before enterint the cooling phase
                    coolingTemp = coolingTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state

    } else if (coolingTimeSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (coolingTime > 0) {
                    coolingTime = coolingTime - 1;
                }
            } else {
                if (coolingTime < 250) {  // total elapsed seconds before entering the cooling phase
                    coolingTime = coolingTime + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (warmupTempSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (warmupTemp > 20) {  // lowest at 20°C
                    warmupTemp = warmupTemp - 1;
                }
            } else {
                if (warmupTemp < 60) {  // Up to 60°C
                    warmupTemp = warmupTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (freeWarmUpButtonSelected == true) {
        // freeWarmUpButtonSelected does not do anything with the rotation of the encoder
    } else if (freeHeatingTargetSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (freeHeatingTemp > 20) {
                    freeHeatingTemp = freeHeatingTemp - 1;
                }
            } else {
                if (freeHeatingTemp < 300) {  // Here we allow a little higher temperature than the reflow curve temperature
                    freeHeatingTemp = freeHeatingTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (startStopButtonSelected == true) {
        // start/stop button does not do anything with the rotation of the encoder
    } else if (freeCoolingTargetSelected == true) {
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (freeCoolingTemp > 20) {
                    freeCoolingTemp = freeCoolingTemp - 1;
                }
            } else {
                if (freeCoolingTemp < 200) {  // Here we allow a little higher temperature than the reflow curve temperature
                    freeCoolingTemp = freeCoolingTemp + 1;
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else if (freeHeatingOnOffSelected == true) {
        // freeHeatingOnOffSelected does not do anything with the rotation of the encoder
    } else if (freeCoolingOnOffSelected == true) {
        // freeCoolingOnOffSelected does not do anything with the rotation of the encoder
    } else if (solderpasteFieldSelected == true) {  // Add the new field logic here
        if (CLKNow != CLKPrevious && CLKNow == 1) {
            if (digitalRead(RotaryDT) != CLKNow) {
                if (solderPasteSelected > 0) {
                    solderPasteSelected = solderPasteSelected - 1;
                } else {
                    solderPasteSelected = numSolderpastes - 1;  // Wrap around to the last index
                }
            } else {
                if (solderPasteSelected < numSolderpastes - 1) {
                    solderPasteSelected = solderPasteSelected + 1;
                } else {
                    solderPasteSelected = 0;  // Wrap around to the first index
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    } else {                   // This navigates through the fields in the menu

        if (CLKNow != CLKPrevious && CLKNow == 1) {
            previousItemCounter = itemCounter;
            if (digitalRead(RotaryDT) != CLKNow) {
                if (itemCounter > 0) {
                    itemCounter = itemCounter - 1;
                } else {
                    itemCounter = 15;  // after the first menu item, we go back to the last menu item
                }
            } else {
                if (itemCounter < 15) {
                    itemCounter = itemCounter + 1;
                } else {
                    itemCounter = 0;  // after the last menu item, we go back to the first menu item
                }
            }
        }
        menuChanged = true;
        CLKPrevious = CLKNow;  // Store last CLK state
    }
    // digitalWrite(DSO_TRIG, LOW); // track duration
}

/*
  The rotary button has been pressed on a field so we enter the processing of the
  menu and values

  Unfortunately, the way the code was written with many tft activities,
  you can't turn this function into an ISR, so we have to use a seperate ISR function
  to watch the activity of the button, and then call this function. A flag is used in
  the ISR so this function cannot be interrupted while we're executing it.

  The fields are selected by turning the rotary encoder itself in a loop.
  You can rotate both ways through the fields. Rotating the encoder will go through
  all the fields in the menu, and returns to the first element after the last one.
  A slected field turns yellow to indicate that it is selected.

  A press of the button enters the edit mode for the contend of the field it is on.
  When you enter the edit mode for the field, the background of the field changes
  from yellow to green to indicate the edit mode. The rotary encoder turning is used
  to select a new value in the active field.

  Another press of the button ends the edit mode, turns the field back to yellow and
  if there was a change, the complete graph is redrawn based on the new value.

  There are special fields for the solder paste selection, and the "buttons" for Warmup,
  Reflow, Heating and cooling.
  Pressing a button when one of these fields is selected activates the selected mode.
  Pressing again stops the activity of the mode.

*/
void processRotaryButton() {
    switch (itemCounter)  // selects the menu item that was selected
    {
        //--Preheat temperature
        case 0:
            preheatTempSelected = !preheatTempSelected;  // flip the status

            if (preheatTempSelected == true) {
                // Edit mode: Green background, red number
                editMode = true;
                tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, GREEN);  // highlight
                tft.setTextColor(RED);
                tft.drawString(String(preheatTemp), preheatTime_px - 10, preheatTemp_px - 30, 1);

            } else {
                if (prev_preheatTemp != preheatTemp)  // only redraw when there is a change
                {
                    redrawCurve = true;  // when the status becomes false, we exit the menu, thus we need to redraw the curve
                    drawReflowCurve();   // Fix: this function erases the field background, so we call for it here.
                    prev_preheatTemp = preheatTemp;
                }
                // Ending edit mode
                tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30);
                tft.setTextColor(RED);
                tft.print(preheatTemp);
                tft.print("C");
                editMode = false;
            }
            break;

        //--Preheat time
        case 1:
            preheatTimeSelected = !preheatTimeSelected;

            if (preheatTimeSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(preheatTime);
            } else {
                if (prev_preheatTime != preheatTime)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_preheatTime = preheatTime;
                }
                // Ending edit mode
                tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(preheatTime);
                tft.print("s");
                editMode = false;
            }
            break;

        //--Soaking temperature
        case 2:
            soakingTempSelected = !soakingTempSelected;

            if (soakingTempSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(soakingTemp);
            } else {
                if (prev_soakingTemp != soakingTemp)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_soakingTemp = soakingTemp;
                }
                // Ending edit mode
                tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(soakingTemp);
                tft.print("C");
                editMode = false;
            }
            break;

        //--Soaking time
        case 3:
            soakingTimeSelected = !soakingTimeSelected;

            if (soakingTimeSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
                tft.setTextColor(RED);
                tft.print(soakingTime);
            } else {
                if (prev_soakingTime != soakingTime)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_soakingTime = soakingTime;
                }
                // Ending edit mode
                tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
                tft.setTextColor(RED);
                tft.print(soakingTime);
                tft.print("s");
                editMode = false;
            }
            break;

        //--Reflow temperature
        case 4:
            reflowTempSelected = !reflowTempSelected;

            if (reflowTempSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
                tft.setTextColor(RED);
                tft.print(reflowTemp);
            } else {
                if (prev_reflowTemp != reflowTemp)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_reflowTemp = reflowTemp;
                }
                // Ending edit mode
                tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
                tft.setTextColor(RED);
                tft.print(reflowTemp);
                tft.print("C");
                editMode = false;
            }
            break;

        //--Reflow time
        case 5:
            reflowTimeSelected = !reflowTimeSelected;

            if (reflowTimeSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
                tft.setTextColor(RED);
                tft.print(reflowTime);
            } else {
                if (prev_reflowTime != reflowTime)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_reflowTime = reflowTime;
                }
                // Ending edit mode
                tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
                tft.setTextColor(RED);
                tft.print(reflowTime);
                tft.print("s");
                editMode = false;
            }
            break;

        //--Cooling temperature
        case 6:
            coolingTempSelected = !coolingTempSelected;

            if (coolingTempSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 29, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 30);
                tft.setTextColor(RED);
                tft.print(coolingTemp);
            } else {
                if (prev_coolingTemp != coolingTemp)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_coolingTemp = coolingTemp;
                }
                // Ending edit mode
                tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 29, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 30);
                tft.setTextColor(BLUE);
                tft.print(coolingTemp);
                tft.print("C");
                editMode = false;
            }
            break;

        //--Cooling time
        case 7:
            coolingTimeSelected = !coolingTimeSelected;

            if (coolingTimeSelected == true) {
                editMode = true;
                // Green background
                tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 19, 24, 9, RectRadius, GREEN);  // highlight
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 20);
                tft.setTextColor(RED);
                tft.print(coolingTime);
            } else {
                if (prev_coolingTime != coolingTime)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    drawReflowCurve();
                    prev_coolingTime = coolingTime;
                }
                // Ending edit mode
                tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 19, 24, 9, RectRadius, YELLOW);  // highlight
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 20);
                tft.setTextColor(RED);
                tft.print(coolingTime);
                tft.print("s");
                editMode = false;
            }
            break;
        //================================================================================================
        // process the special warmup, free heating, cooling and solderpaste "button" fields
        //================================================================================================
        case 8:  //--Warmup temperature
            warmupTempSelected = !warmupTempSelected;

            if (warmupTempSelected == true) {
                editMode = true;
                tft.fillRoundRect(220, 2, 32, 12, RectRadius, GREEN);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString(String(warmupTemp), 228, 4, 1);
            } else {
                tft.fillRoundRect(220, 2, 32, 12, RectRadius, YELLOW);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString(String(warmupTemp), 228, 4, 1);
                editMode = false;
            }
            break;

        case 9:  //--Warmup button
            freeWarmUpButtonSelected = !freeWarmUpButtonSelected;

            if (freeWarmUpButtonSelected == true) {
                reflow = false;
                // clean the curve area
                drawFreeCurve();

                tft.fillRoundRect(260, 0, 60, 15, RectRadius, DGREEN);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("STOP", 265, 0, 2);
                enableWarmup = true;
                heatingEnabled = true;   // start heating
                elapsedHeatingTime = 0;  // set the elapsed time to 0
            } else {
                // First draw all the buttons (the easy way out)
                drawActionButtons();
                // Then update the warmup field so it's still marked as selected so we know where we are
                tft.fillRoundRect(260, 0, 60, 15, RectRadius, YELLOW);
                tft.setTextColor(WHITE);
                tft.drawString("WARMUP", 265, 0, 2);
                enableWarmup = false;
                analogWrite(SSR_pin, OFF);  // turn the heater off
                //---------------------------
                // Put back all the values after stop
                reflow = false;                    // Reset reflow status flag to false (so free heating can run)
                redrawCurve = true;                // simply redraw the whole graph
                heatingEnabled = false;            // stop heating
                tft.fillCircle(237, 7, 6, BLACK);  // remove the SSR on/off signal
                drawReflowCurve();                 // redraw the curve with the values
                // Reapply the highlight to the selected field
                menuChanged = true;  // Ensure menuChanged is set to true
                updateHighlighting();
            }
            break;

        case 10:  //--Start/Stop reflow
            startStopButtonSelected = !startStopButtonSelected;

            if (startStopButtonSelected == true) {
                editMode = true;
                // Remove all the numbers -> It makes the display cleaner, easier to read
                removeFieldsFromDisplay();
                drawCurve();  // redraw the curve

                // Update the Reflow button to a green background and label it stop
                tft.fillRoundRect(260, 20, 60, 15, RectRadius, GREEN);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString("STOP", 265, 20, 2);

                currentPhase = PREHEAT;  // Set the current phase to preheat (in case we do a second reflow round)
                reflow = true;           // Enable reflow
                heatingEnabled = true;   // start heating
                elapsedHeatingTime = 0;  // set the elapsed time to 0
            } else {
                // First, update all the buttons (easy way out)
                drawActionButtons();
                // update the reflow field so it's still marked as selected so we know where we are
                tft.fillRoundRect(260, 20, 60, 15, RectRadius, YELLOW);  // still highlighted
                tft.setTextColor(WHITE);
                tft.drawString("REFLOW", 265, 20, 2);
                analogWrite(SSR_pin, OFF);  // turn the heater off
                //---------------------------
                // Put back all the values after stop
                reflow = false;              // Reset reflow status flag to false (so free heating can run)
                redrawCurve = true;          // simply redraw the whole graph
                heatingEnabled = false;      // stop heating
                digitalWrite(Fan_pin, OFF);  // turn the cooling fan off, the user can select the free cooling mode if desired
                // ending edit mode
                editMode = false;
                drawReflowCurve();               // redraw the curve with the values
                updateStatus(BLACK, BLACK, "");  // erase the status field
                // Reapply the highlight to the selected field
                menuChanged = true;  // Ensure menuChanged is set to true
                updateHighlighting();
            }
            break;

        case 11:  //--Free heating target temperature
            freeHeatingTargetSelected = !freeHeatingTargetSelected;

            if (freeHeatingTargetSelected == true) {
                editMode = true;
                tft.fillRoundRect(220, 42, 32, 12, RectRadius, GREEN);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString(String(freeHeatingTemp), 228, 44, 1);
            } else {
                tft.fillRoundRect(220, 42, 32, 12, RectRadius, YELLOW);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString(String(freeHeatingTemp), 228, 44, 1);
                editMode = false;
            }
            break;

        case 12:  //--Start/stop free heating
            freeHeatingOnOffSelected = !freeHeatingOnOffSelected;

            if (freeHeatingOnOffSelected == true) {
                reflow = false;
                // clean the curve area
                drawFreeCurve();

                tft.fillRoundRect(260, 40, 60, 15, RectRadius, RED);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("STOP", 265, 40, 2);
                enableFreeHeating = true;
                heatingEnabled = true;   // start heating
                elapsedHeatingTime = 0;  // set the elapsed time to 0
            } else {
                // First draw all the buttons (easy way out)
                drawActionButtons();
                // update the heating field so it's still marked as selected so we know where we are
                tft.fillRoundRect(260, 40, 60, 15, RectRadius, YELLOW);  // still highlighted
                tft.setTextColor(WHITE);
                tft.drawString("HEATING", 265, 40, 2);
                enableFreeHeating = false;
                freeHeatingOnOffSelected = false;
                //---------------------------
                // Put back all the values after stop
                analogWrite(SSR_pin, OFF);  // turn the heater off
                reflow = false;             // Reset reflow status flag to false (so free heating can run)
                redrawCurve = true;         // simply redraw the whole graph
                heatingEnabled = false;     // stop heating
                drawReflowCurve();          // redraw the curve with the values
                // Reapply the highlight to the selected field
                updateStatus(BLACK, BLACK, "");  // erase the status field
                menuChanged = true;              // Ensure menuChanged is set to true
                updateHighlighting();
            }
            break;

        case 13:  //--Free cooling temperature
            freeCoolingTargetSelected = !freeCoolingTargetSelected;

            if (freeCoolingTargetSelected == true) {
                editMode = true;
                tft.fillRoundRect(220, 62, 32, 12, RectRadius, GREEN);  // X,Y, W,H, Color
                tft.setTextColor(BLUE);
                tft.drawString(String(freeCoolingTemp), 228, 64, 1);
            } else {
                // ending edit mode
                tft.fillRoundRect(220, 62, 32, 12, RectRadius, YELLOW);  // X,Y, W,H, Color
                tft.setTextColor(BLUE);
                tft.drawString(String(freeCoolingTemp), 228, 64, 1);
                editMode = false;
            }
            break;

        case 14:  //-- Start/stop free Cooling
            freeCoolingOnOffSelected = !freeCoolingOnOffSelected;

            if (freeCoolingOnOffSelected == true) {
                reflow = false;
                // clean the curve area
                drawFreeCurve();

                tft.fillRoundRect(260, 60, 60, 15, RectRadius, BLUE);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("STOP", 265, 60, 2);
                enableFreeCooling = true;
                elapsedHeatingTime = 0;     // set the elapsed time to 0
                analogWrite(SSR_pin, OFF);  // just in case it's still on when we select freecooling after freeheating
            } else {
                // First draw all the buttons (easy way out)
                drawActionButtons();
                // Then update the cooling field so it's still marked as selected so we know where we are
                tft.fillRoundRect(260, 60, 60, 15, RectRadius, YELLOW);
                tft.setTextColor(WHITE);
                tft.drawString("COOLING", 265, 60, 2);
                enableFreeCooling = false;
                freeCoolingOnOffSelected = false;
                heatingEnabled = false;  // stop heating if still on
                //---------------------------
                // Put back all the values after stop
                reflow = false;             // Reset reflow status flag to false (so free heating can run)
                redrawCurve = true;         // simply redraw the whole graph
                heatingEnabled = false;     // stop heating
                coolingFanEnabled = false;  // stop cooling fan.
                drawReflowCurve();          // redraw the curve with the values
                // Reapply the highlight to the selected field
                updateStatus(BLACK, BLACK, "");  // erase the status field
                menuChanged = true;              // Ensure menuChanged is set to true
                updateHighlighting();
            }
            break;

        case 15:  // change the solder paste
            solderpasteFieldSelected = !solderpasteFieldSelected;
            if (solderpasteFieldSelected == true) {
                editMode = true;
                // highlighting the edit mode
                tft.fillRoundRect(48, 0, 150, 18, RectRadius, GREEN);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                // Fetch the paste name from the array using solderPasteSelected
                pasteName = solderpastes[solderPasteSelected].pasteName;
                tft.drawString(pasteName, pasteNamePosX, pasteNamePosY, 2);
            } else {
                if (prev_solderPasteSelected != solderPasteSelected)  // only redraw when there is a change
                {
                    redrawCurve = true;
                    // Fetch the values from the array using solderPasteSelected
                    // and transpose the values to make them active
                    preheatTemp = solderpastes[solderPasteSelected].preheatTemp;
                    preheatTime = solderpastes[solderPasteSelected].preheatTime;
                    soakingTemp = solderpastes[solderPasteSelected].soakingTemp;
                    soakingTime = solderpastes[solderPasteSelected].soakingTime;
                    reflowTemp = solderpastes[solderPasteSelected].reflowTemp;
                    reflowTime = solderpastes[solderPasteSelected].reflowTime;
                    coolingTemp = solderpastes[solderPasteSelected].coolingTemp;
                    coolingTime = solderpastes[solderPasteSelected].coolingTime;

                    drawReflowCurve();
                    prev_solderPasteSelected = solderPasteSelected;

                    // forward prediction for the heating cut-off in the preheat and reflow phases
                    preheatCutOff = preheatTime - preheatCutOffTime;  // cut off the heater 10 seconds before the target temperature
                    reflowCutOff = reflowTime - reflowCutOffTime;     // cut off the heater 10 seconds before the target temperature
                }
                // ending edit mode
                tft.fillRoundRect(48, 0, 150, 18, RectRadius, YELLOW);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                pasteName = solderpastes[solderPasteSelected].pasteName;
                tft.drawString(pasteName, pasteNamePosX, pasteNamePosY, 2);
                editMode = false;
            }
            break;
    }
    menuChanged = false;
}

/*
  updateHighlighting

  This function is called from various places in the code.
  Depending on the menu (=field), determined by the rotary encoder, update the information
  on the display.

  Added a check for the edit mode so we keep the proper highlighting when we return from
  making changes to the values.
*/
void updateHighlighting() {
    if (menuChanged == true)  // if somewhere in the code we changed the menu, we will be able to enter the part below
    {
        switch (itemCounter)  // check which menu was changed
        {
            case 0:  // Preheat temp
                if (editMode) {
                    tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30);  // set the cursor to the corresponding spot
                tft.setTextColor(RED);                                    // set text color
                tft.print(preheatTemp);                                   // print the value

                // Note: All the following lines are doing the same, they just print to different coordinates and different values.
                break;

            case 1:  // Preheat time
                if (editMode) {
                    tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(preheatTime);
                break;

            case 2:  // Soaking temp
                if (editMode) {
                    tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(soakingTemp);
                break;

            case 3:  // Soaking time
                if (editMode) {
                    tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
                tft.setTextColor(RED);
                tft.print(soakingTime);
                break;

            case 4:  // Reflow temp
                if (editMode) {
                    tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
                tft.setTextColor(RED);
                tft.print(reflowTemp);
                break;

            case 5:  // Reflow time
                if (editMode) {
                    tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
                tft.setTextColor(RED);
                tft.print(reflowTime);
                break;

            case 6:  // Cooling temp
                if (editMode) {
                    tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 29, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 29, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 30);
                tft.setTextColor(BLUE);
                tft.print(coolingTemp);
                break;

            case 7:  // Cooling time
                if (editMode) {
                    tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 19, 24, 9, RectRadius, GREEN);  // highlight edit mode
                } else {
                    tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 19, 24, 9, RectRadius, YELLOW);  // highlight
                }
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 20);
                tft.setTextColor(RED);
                tft.print(coolingTime);
                break;

            case 8:  // warmup temp
                if (editMode) {
                    tft.fillRoundRect(220, 2, 32, 12, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(220, 2, 32, 12, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(BLUE);
                tft.drawString(String(warmupTemp), 228, 4, 1);
                break;

            case 9:  // Warmup on/off
                if (editMode) {
                    tft.fillRoundRect(260, 0, 60, 15, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(260, 0, 60, 15, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(BLACK);
                tft.drawString("WARMUP", 265, 0, 2);
                break;

            case 10:  // Reflow start/stop
                if (editMode) {
                    tft.fillRoundRect(260, 20, 60, 15, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(260, 20, 60, 15, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(BLACK);
                tft.drawString("REFLOW", 265, 20, 2);
                break;

            case 11:  // free heating target temp
                if (editMode) {
                    tft.fillRoundRect(220, 42, 32, 12, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(220, 42, 32, 12, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(RED);
                tft.drawString(String(freeHeatingTemp), 228, 44, 1);
                break;

            case 12:  // Free heating on/off
                if (editMode) {
                    tft.fillRoundRect(260, 40, 60, 15, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(260, 40, 60, 15, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(BLACK);
                tft.drawString("HEATING", 265, 40, 2);
                break;

            case 13:  // free cooling temp
                if (editMode) {
                    tft.fillRoundRect(220, 62, 32, 12, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(220, 62, 32, 12, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(BLUE);
                tft.drawString(String(freeCoolingTemp), 228, 64, 1);
                break;

            case 14:  // Free cooling selected on/off
                if (editMode) {
                    tft.fillRoundRect(260, 60, 60, 15, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(260, 60, 60, 15, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(BLACK);
                tft.drawString("COOLING", 265, 60, 2);

                break;

            case 15:  // solderpast field
                if (editMode) {
                    tft.fillRoundRect(48, 0, 150, 18, RectRadius, GREEN);  // X,Y, W,H, Color
                } else {
                    tft.fillRoundRect(48, 0, 150, 18, RectRadius, YELLOW);  // X,Y, W,H, Color
                }
                tft.setTextColor(RED);
                pasteName = solderpastes[solderPasteSelected].pasteName;
                tft.drawString(pasteName, pasteNamePosX, pasteNamePosY, 2);
                break;
        }
        //--------------------------------------------------------------------------------------------
        // Remove the highlighting of the previous field
        //--------------------------------------------------------------------------------------------
        switch (previousItemCounter)  // check which item was previously highlighted so we can restore its original look (no highlighting)
        {
            case 0:                                                                                     // preheat temp
                tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, BLACK);  // restore original background (black)
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30);                                // set the cursor to the corresponding place (inside the rectangle)
                tft.setTextColor(RED);                                                                  // set text color
                tft.print(preheatTemp);                                                                 // print value
                tft.print("C");
                break;
            case 1:  // preheat time
                tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, BLACK);
                tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
                tft.setTextColor(WHITE);
                tft.print(preheatTime);
                tft.print("s");
                break;
            case 2:  // soaking temp
                tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, BLACK);
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
                tft.setTextColor(RED);
                tft.print(soakingTemp);
                tft.print("C");
                break;
            case 3:  // soaking time
                tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, BLACK);
                tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
                tft.setTextColor(WHITE);
                tft.print(soakingTime);
                tft.print("s");
                break;
            case 4:  // reflow temp
                tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, BLACK);
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
                tft.setTextColor(RED);
                tft.print(reflowTemp);
                tft.print("C");
                break;
            case 5:  // reflow time
                tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, BLACK);
                tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
                tft.setTextColor(WHITE);
                tft.print(reflowTime);
                tft.print("s");
                break;
            case 6:  // cooling temp
                tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 29, 24, 9, RectRadius, BLACK);
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 30);
                tft.setTextColor(BLUE);
                tft.print(coolingTemp);
                tft.print("C");
                break;
            case 7:  // cooling time
                tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 19, 24, 9, RectRadius, BLACK);
                tft.setCursor(coolingTime_px + 20, coolingTemp_px + 20);
                tft.setTextColor(WHITE);
                tft.print(coolingTime);
                tft.print("s");
                break;
            case 8:                                                    // warmup temp
                tft.fillRoundRect(220, 2, 32, 12, RectRadius, BLACK);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString(String(warmupTemp) + "C", 228, 4, 1);
                break;
            case 9:                                                     // warmup on/off
                tft.fillRoundRect(260, 0, 60, 15, RectRadius, DGREEN);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("WARMUP", 265, 0, 2);
                break;
            case 10:                                                     // Reflow start/stop
                tft.fillRoundRect(260, 20, 60, 15, RectRadius, ORANGE);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("REFLOW", 265, 20, 2);
                break;
            case 11:                                                    // free heating target temp
                tft.fillRoundRect(220, 42, 32, 12, RectRadius, BLACK);  // X,Y, W,H, Color
                tft.setTextColor(RED);
                tft.drawString(String(freeHeatingTemp) + "C", 228, 44, 1);
                break;
            case 12:                                                  // free heating on/off
                tft.fillRoundRect(260, 40, 60, 15, RectRadius, RED);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("HEATING", 265, 40, 2);
                break;
            case 13:                                                    // free cooling temp
                tft.fillRoundRect(220, 62, 32, 12, RectRadius, BLACK);  // X,Y, W,H, Color
                tft.setTextColor(BLUE);
                tft.drawString(String(freeCoolingTemp) + "C", 228, 64, 1);
                break;
            case 14:                                                   // free cooling on/off
                tft.fillRoundRect(260, 60, 60, 15, RectRadius, BLUE);  // X,Y, W,H, Color
                tft.setTextColor(WHITE);
                tft.drawString("COOLING", 265, 60, 2);
                break;
            case 15:                                                   // solderpaste field
                tft.fillRoundRect(46, 0, 152, 20, RectRadius, BLACK);  // erase the previous
                tft.setTextColor(WHITE);
                pasteName = solderpastes[solderPasteSelected].pasteName;
                tft.drawString(pasteName, pasteNamePosX, pasteNamePosY, 2);
                break;
        }
        menuChanged = false;
    }
}

/*
Run the reflow mode
The reflow process moves from one phase to the next based on an actual measurement
of where we are in time, and at what temperature.

The calculation of the target temperature is based on the elapsed time and the temperature
we want to reach at the end of each phase.
This calculation follows the initial reflow curve (straight lines) but not exactly.
When you want to see the curve with the calculated values, you can simulate the reflow
mode and see the calculation of the target temperature over time,
This also allows you to test the transitions from one phase to the next without actually
using the heaters.

To activate the simulation, uncomment the line:
 1. TCCelsius = targetTemp; // this links the TCCelsius to the calculated targetTemp and not
 the actual sensor value.
 2. Set elapsedHeatingTime at the end of the function to 100.0 to run the simulation 10x faster .

The temperature of the hotplate is regulated by the controlling the SSR through
PWM-based activation.

There are "early" calculations to prevent the temperature from overshooting the target.

*/

void runReflow() {
    if (reflow == true)  // Only proceed if the reflow was enabled by the press of the start button.
    {
        if (elapsedHeatingTime < 340)  // continue to run until the end of the time scale or when user stopped it
        {
            unsigned long timeNow = millis();

            if (timeNow - SSRTimer > SSRInterval)  // Update frequency = 250 ms - should be less frequent than the temperature readings
            {
                // *** Simulate the temperature value in TCCelsius
                // TCCelsius = targetTemp; // To simulate, link the calculated temperature to the target temperature

                // Calculate the x-y coordinate of a pixel to show the temperature measurement over time
                measuredTemp_px = (int)((yGraph) - ((TCCelsius / tempPixelFactor)));
                measuredTime_px = (int)(xGraph + (elapsedHeatingTime / timePixelFactor));

                // Draw the pixel (time vs. temperature) on the graph
                tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
                // you can draw a thicker line by activating the next statement
                // by putting another pixel next (on Y) the original, to fake "a thicker line"
                // tft.drawPixel(measuredTime_px, measuredTemp_px + 1, CYAN);

                printElapsedTime();  // Print the elapsed time in seconds

                switch (currentPhase)  // This part determines the code's progress along the reflow curve
                {
                    case PREHEAT:
                        // calculate the desired temperature from the reflow profile, based on the elapsed time,
                        // and thus trying to follow the reflow curve in real-time
                        targetTemp = 20 + (elapsedHeatingTime * (1.0 / preheatTime) * (preheatTemp - 20));

                        // show the phase on the display
                        updateStatus(DGREEN, WHITE, "Preheat");
                        // print the target temperature on the display
                        printTargetTemperature();

                        // Do some kind of early turn-off to avoid temperature overshooting
                        // if we are almost there and close to the targetTemp, we can stop heating
                        // We start to see if we can stop heating a little while before we reach the end of the time,
                        // and check to see if we are within a certain range of the target temperature.
                        // We should be above the target temperature and not too far below it.
                        // Note that the target temperature changes every cycle, so we have to check it every loop.
                        if ((elapsedHeatingTime >= preheatCutOff) && (TCCelsius >= targetTemp - 15) || (TCCelsius >= targetTemp)) {
                            Output = 0;
                        } else {
                            Output = 255;
                        }
                        analogWrite(SSR_pin, Output);

                        // determine if we can switch to the next phase
                        if (TCCelsius > preheatTemp && elapsedHeatingTime > preheatTime) {
                            currentPhase = SOAK;
                        }
                        break;

                    case SOAK:
                        // calculate the desired temperature from the reflow profile, based on the elapsed time and the temp from the profile
                        targetTemp = preheatTemp + ((elapsedHeatingTime - preheatTime) / (soakingTime - preheatTime)) * (soakingTemp - preheatTemp);
                        printTargetTemperature();
                        updateStatus(DGREEN, WHITE, "Soaking");
                        if (TCCelsius < targetTemp) {
                            Output = 150;  // reduce the power
                        } else {
                            Output = 0;
                        }
                        analogWrite(SSR_pin, Output);

                        if (TCCelsius > soakingTemp && elapsedHeatingTime > soakingTime) {
                            currentPhase = REFLOW;
                        }
                        break;

                    case REFLOW:
                        // calculate the desired temperature from the reflow profile, based on the elapsed time and the temp from the profile
                        targetTemp = soakingTemp + ((elapsedHeatingTime - soakingTime) / (reflowTime - soakingTime)) * (reflowTemp - soakingTemp);
                        printTargetTemperature();
                        updateStatus(DGREEN, WHITE, "Reflow");

                        // if we are almost there and above the targetTemp, we can stop heating to avoid overshooting
                        if ((elapsedHeatingTime >= reflowCutOff) && (TCCelsius >= targetTemp - 15) || (TCCelsius >= targetTemp)) {
                            Output = 0;
                        } else {
                            Output = 255;  // max power
                        }
                        analogWrite(SSR_pin, Output);

                        if (TCCelsius > reflowTemp || elapsedHeatingTime > reflowTime)
                        // when we have reached the reflowTemp or past the time, we can move to the hold phase
                        {
                            currentPhase = HOLD;
                        }
                        break;

                    case HOLD:
                        // calculate the desired temperature from the reflow profile, based on the elapsed time and the temp from the profile
                        targetTemp = reflowTemp + ((elapsedHeatingTime - reflowTime) / (coolingTime - reflowTime)) * (coolingTemp - reflowTemp);

                        printTargetTemperature();
                        updateStatus(DGREEN, WHITE, "Holding");
                        if (TCCelsius < targetTemp) {
                            Output = 50;  // reduce the power to maintain the temperature
                        } else {
                            Output = 0;
                        }
                        analogWrite(SSR_pin, Output);

                        if (TCCelsius > coolingTemp && elapsedHeatingTime > coolingTime)
                        // when we have reached the coolingTime limit, we can move to the cooling phase

                        // in case the time runs out and we don't reach the cooling phase, you can use this:
                        // if (elapsedHeatingTime > coolingTime)
                        {
                            currentPhase = COOLING;
                        }
                        break;

                    case COOLING:
                        // turn of the heater, turn on the fans and allow them to cool the plate down to 40 degrees
                        targetTemp = 40;
                        Output = 0;
                        analogWrite(SSR_pin, Output);            // stop heating
                        updateStatus(DGREEN, WHITE, "Cooling");  // start cooling
                        heatingEnabled = false;                  // Disable heating
                        coolingFanEnabled = true;                // Enable cooling
                        if (TCCelsius > targetTemp) {
                            digitalWrite(Fan_pin, HIGH);  // Turn on the fan(s)
                            Fan = "ON";                   // so we can show the status with printFan()
                        } else {
                            digitalWrite(Fan_pin, LOW);  // Turn on the fan(s)
                            Fan = "OFF";
                        }
                        break;
                }
                if (heatingEnabled == true) {
                    // show the PWM output on the screen
                    printPWM();
                } else {
                    // show the Fan status on the screen
                    printFan();
                }
                // *** when simulating: set interval to 100.0 (10x faster)
                elapsedHeatingTime += (SSRInterval / 1000.0);  // SSRInterval is in ms, so it has to be divided by 1000
                SSRTimer = millis();
            }
        }
    }
}

// run the free heating mode
void freeHeating() {
    if (enableFreeHeating == true) {
        unsigned long timeNow = millis();
        static bool slowdown = false;
        static bool rampup = true;

        if (timeNow - SSRTimer > SSRInterval)  // update frequency = 250 ms - should be less frequent than the temperature readings
        {
            // Draw a pixel for the temperature measurement - Calculate the position
            measuredTemp_px = (int)(yGraph - ((TCCelsius / tempPixelFactor)));  // 220 -> 200 offset is 3
            measuredTime_px = (int)(xGraph + (elapsedHeatingTime / timePixelFactor));

            printElapsedTime();        // Print the elapsed time in seconds
            printTargetTemperature();  // Print the actual target temperature

            // Draw the pixel (time vs. temperature) on the chart
            tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
            tft.drawPixel(measuredTime_px, measuredTemp_px + 1, CYAN);  // putting another pixel next (on Y) the original, "fake a thick line"

            targetTemp = freeHeatingTemp;

            // If we are almost there and just below the targetTemp,
            // we can use conservative parameters to reduce overshooting

            double gap = abs(targetTemp - TCCelsius);

            // initial rampup
            if (slowdown == false && rampup == true) {
                Output = 255;
                analogWrite(SSR_pin, int(Output));
                tft.fillRoundRect(130, 80, 80, 20, RectRadius, BLACK);
                tft.setTextColor(WHITE);
                tft.drawString("rampup", 132, 80, 1);
            }

            // when we are ramping up and close to the target, but still below it, slow down
            if (gap < 25 && TCCelsius < freeHeatingTemp && rampup == true) {
                // slow down based on the targetTemp so we will get there
                if (freeHeatingTemp < 100) {
                    Output = 10;
                } else if (freeHeatingTemp < 200) {
                    Output = 20;
                } else {
                    Output = 30;
                }

                analogWrite(SSR_pin, Output);  // let it still creep up
                tft.fillRoundRect(130, 80, 80, 20, RectRadius, BLACK);
                tft.setTextColor(WHITE);
                tft.drawString("slow down", 132, 80, 1);
                slowdown = true;
            }

            // if we are now above the target, return to normal regulation
            if (TCCelsius >= freeHeatingTemp && slowdown == true && rampup == true) {
                // back to normal regulation
                slowdown = false;
                rampup = false;
            }

            // normal regulation
            if (slowdown == false && rampup == false) {
                if (TCCelsius < freeHeatingTemp) {
                    Output = 40;  // curb the power to make the regulation smoother
                    analogWrite(SSR_pin, int(Output));
                } else {
                    Output = 0;
                    analogWrite(SSR_pin, int(Output));
                }
                tft.fillRoundRect(130, 80, 80, 20, RectRadius, BLACK);
                tft.setTextColor(WHITE);
                tft.drawString("regulate", 132, 80, 1);
            }

            // show the PWM output on the screen
            printPWM();

            updateStatus(DGREEN, WHITE, "Heating");

            elapsedHeatingTime += (SSRInterval / 1000.0);  // SSRInterval is in ms, so it has to be divided by 1000

            SSRTimer = millis();
        }
    }
}

// run the free cooling mode
void freeCooling() {
    if (enableFreeCooling == true)  // If heating was enabled somewhere in the code, we can enter the code below
    {
        unsigned long timeNow = millis();

        if (timeNow - SSRTimer > SSRInterval)  // update frequency = 250 ms - should be less frequent than the temperature readings
        {
            // Calculate the x-y pixel position for the temperature measurement over time
            measuredTemp_px = (int)(yGraph - ((TCCelsius / tempPixelFactor)));
            measuredTime_px = (int)(xGraph + (elapsedHeatingTime / timePixelFactor));

            // Draw the pixel (time vs. temperature) on the chart
            tft.drawPixel(measuredTime_px, measuredTemp_px, BLUE);
            tft.drawPixel(measuredTime_px, measuredTemp_px + 1, BLUE);
            // putting another pixel next (on Y) the original, to fake a "thick line"
            // tft.drawLine(measuredTime_px, measuredTemp_px, measuredTime_px-50, measuredTime_px+50, WHITE);

            // Also print the elapsed time in second
            printElapsedTime();

            targetTemp = freeCoolingTemp;
            printTargetTemperature();  // Print the target temperature that we calculated above

            if (TCCelsius > freeCoolingTemp)  // Turn the fans ON or OFF depending on the flag
            {
                digitalWrite(Fan_pin, HIGH);
            } else {
                digitalWrite(Fan_pin, LOW);
            }

            updateStatus(DGREEN, WHITE, "Cooling");
            // Print the Fan status on the TFT
            printFan();

            elapsedHeatingTime += (SSRInterval / 1000.0);  // SSRInterval is in ms, so it has to be divided by 1000

            SSRTimer = millis();
        }
    }
}

// Run warmup mode, a more gentile warmup for lower temperatures than free heating
void runWarmup() {
    static bool slowdown = false;
    static bool rampup = true;

    if (enableWarmup == true) {
        unsigned long timeNow = millis();

        if (timeNow - SSRTimer > SSRInterval)  // update frequency = 250 ms - should be less frequent than the temperature readings
        {
            // Calculate the position of the coordinates for the pixel so we can plot it on the chart
            measuredTemp_px = (int)(yGraph - ((TCCelsius / tempPixelFactor)));         // 220 -> 200 offset is 13
            measuredTime_px = (int)(xGraph + (elapsedHeatingTime / timePixelFactor));  // 18px from the left

            // Draw the calculated pixel position (time vs. temperature) on the chart
            tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
            tft.drawPixel(measuredTime_px, measuredTemp_px + 1, CYAN);  // putting another pixel next (on Y) the original, "fake a thick line"

            // Also print the elapsed time in second
            printElapsedTime();
            printTargetTemperature();

            targetTemp = warmupTemp;
            double gap = abs(targetTemp - TCCelsius);  // to stop the rampup mode early

            // initial rampup
            if (slowdown == false && rampup == true) {
                Output = 125;  // curb the output in this mode to half power
                analogWrite(SSR_pin, int(Output));
                tft.fillRoundRect(130, 80, 80, 20, RectRadius, BLACK);
                tft.setTextColor(WHITE);
                tft.drawString("rampup", 132, 80, 1);
            }

            // when we are ramping up and close to the target, but still below it, slow down
            if (gap < 10 && TCCelsius < warmupTemp && rampup == true) {
                Output = 4;  // slow down and let it creep up
                analogWrite(SSR_pin, Output);
                tft.fillRoundRect(130, 80, 80, 20, RectRadius, BLACK);
                tft.setTextColor(WHITE);
                tft.drawString("slow down", 132, 80, 1);
                slowdown = true;
            }

            // if we are now above the target, return to normal regulation
            if (TCCelsius >= warmupTemp && slowdown == true && rampup == true) {
                // back to normal regulation
                slowdown = false;
                rampup = false;
            }

            // normal regulation
            if (slowdown == false && rampup == false) {
                if (TCCelsius < warmupTemp) {
                    Output = 40;  // reduce the power even more
                    analogWrite(SSR_pin, int(Output));
                } else {
                    Output = 0;
                    analogWrite(SSR_pin, int(Output));
                }
                tft.fillRoundRect(130, 80, 80, 20, RectRadius, BLACK);
                tft.setTextColor(WHITE);
                tft.drawString("regulate", 132, 80, 1);
            }

            // show the PWM output on the screen
            printPWM();

            updateStatus(DGREEN, WHITE, "Warmup");

            elapsedHeatingTime += (SSRInterval / 1000.0);  // SSRInterval is in ms, so it has to be divided by 1000

            SSRTimer = millis();
        }
    }
}

// Draw the chart axes, tickmarks and values
void drawAxis() {
    // Y-axis line (vertical - temperature): total 320px
    tft.drawLine(xGraph, ((int)238 - (250 / tempPixelFactor)) - 12, xGraph, 238 - 13, RED);  // X0, Y0, X1, Y1, Color

    // Horizontal lines (ticks) at every 50C (a line from from 13 left to 22 right)
    tft.drawLine(13, (int)yGraph - (50 / tempPixelFactor), 22, (int)yGraph - (50 / tempPixelFactor), RED);    // 50C
    tft.drawLine(13, (int)yGraph - (100 / tempPixelFactor), 22, (int)yGraph - (100 / tempPixelFactor), RED);  // 100C
    tft.drawLine(13, (int)yGraph - (150 / tempPixelFactor), 22, (int)yGraph - (150 / tempPixelFactor), RED);  // 150C
    tft.drawLine(13, (int)yGraph - (200 / tempPixelFactor), 22, (int)yGraph - (200 / tempPixelFactor), RED);  // 200C
    tft.drawLine(13, (int)yGraph - (250 / tempPixelFactor), 22, (int)yGraph - (250 / tempPixelFactor), RED);  // 250C

    // Y-axis is Temperature in Celsius
    tft.drawString("`c", 4, (int)tftY - 35, 2);

    // tick values
    tft.drawString("50", 5, (int)tftY - 17 - (50 / tempPixelFactor), 1);  // text, x, y color
    tft.drawString("100", 0, (int)tftY - 17 - (100 / tempPixelFactor), 1);
    tft.drawString("150", 0, (int)tftY - 17 - (150 / tempPixelFactor), 1);
    tft.drawString("200", 0, (int)tftY - 17 - (200 / tempPixelFactor), 1);
    tft.drawString("250", 0, (int)tftY - 17 - (250 / tempPixelFactor), 1);

    // X-axis line (horizontal - time) : total 240px
    tft.drawLine(xGraph, yGraph, ((int)360 / timePixelFactor), yGraph, WHITE);  // X0, Y0, X1, Y1, Color

    // Vertical lines (ticks) at every 30s; generate small (4px) and larger ticks (6px) at every 60s
    tft.drawLine(xGraph + (int)30 / timePixelFactor, 226, xGraph + (int)30 / timePixelFactor, 222, WHITE);    // 30S
    tft.drawLine(xGraph + (int)60 / timePixelFactor, 226, xGraph + (int)60 / timePixelFactor, 220, WHITE);    // 60S
    tft.drawLine(xGraph + (int)90 / timePixelFactor, 226, xGraph + (int)90 / timePixelFactor, 222, WHITE);    // 90S
    tft.drawLine(xGraph + (int)120 / timePixelFactor, 226, xGraph + (int)120 / timePixelFactor, 220, WHITE);  // 120S
    tft.drawLine(xGraph + (int)150 / timePixelFactor, 226, xGraph + (int)150 / timePixelFactor, 222, WHITE);  // 150S
    tft.drawLine(xGraph + (int)180 / timePixelFactor, 226, xGraph + (int)180 / timePixelFactor, 220, WHITE);  // 180S
    tft.drawLine(xGraph + (int)210 / timePixelFactor, 226, xGraph + (int)210 / timePixelFactor, 222, WHITE);  // 210S
    tft.drawLine(xGraph + (int)240 / timePixelFactor, 226, xGraph + (int)240 / timePixelFactor, 220, WHITE);  // 240S
    tft.drawLine(xGraph + (int)270 / timePixelFactor, 226, xGraph + (int)270 / timePixelFactor, 222, WHITE);  // 270S
    tft.drawLine(xGraph + (int)300 / timePixelFactor, 226, xGraph + (int)300 / timePixelFactor, 220, WHITE);  // 300S
    tft.drawLine(xGraph + (int)330 / timePixelFactor, 226, xGraph + (int)330 / timePixelFactor, 222, WHITE);  // 330S

    // X-axis is Time in seconds
    tft.drawString("seconds", 15, (int)tftY - 10, 1);

    // tick values with justified numbers
    tft.drawString("60", (xGraph + (int)60 / timePixelFactor) - 5, 230, 1);
    tft.drawString("120", (xGraph + (int)120 / timePixelFactor) - 8, 230, 1);
    tft.drawString("180", (xGraph + (int)180 / timePixelFactor) - 8, 230, 1);
    tft.drawString("240", (xGraph + (int)240 / timePixelFactor) - 8, 230, 1);
    tft.drawString("300", (xGraph + (int)300 / timePixelFactor) - 8, 230, 1);
}

/*
        This function draws the reflow curve based on the selected reflow paste.
        Since the axes are slightly shifted from the edge of the display, there is a
  shift for the start of the preheat curve.
        Different sections of the reflow curve are using different colors.
  The sections are drawn with straight lines.
        The cooling time +20 is just an arbitrary value, just to illustrate the cooling part
  (decreasing temperature) on the reflow curve.
        It has no physical meaning other than it illustrates the cooling phase.
*/
void drawCurve() {
    tft.setTextSize(1);  // Reset the size to 1 in case the code is coming from someplace else

    drawAxis();

    // Draw the curve
    // starting in the origin of the x line, but at the room temperature (est. @20 degrees) of the y-line
    tft.drawLine(xGraph, yGraph - (20 / tempPixelFactor), preheatTime_px, preheatTemp_px, YELLOW);
    tft.drawLine(preheatTime_px, preheatTemp_px, soakingTime_px, soakingTemp_px, ORANGE);
    tft.drawLine(soakingTime_px, soakingTemp_px, reflowTime_px, reflowTemp_px, RED);
    tft.drawLine(reflowTime_px, reflowTemp_px, coolingTime_px, coolingTemp_px, RED);
    tft.drawLine(coolingTime_px, coolingTemp_px, coolingTime_px + 40, coolingTemp_px + 20, BLUE);  // fake a downward cooling curve
}

// =================================================================================================

// the next three functions are for a different reflow curve based on curved lines
// the curve is drawn using cosine interpolation between the points.
//
// this code can be activated by renaming the drawCurve() name above, to say drawCurve_line()
// and rename the drawCurve_new() below to drawCurve().

// Function to interpolate between two points using cosine interpolation
float cosineInterpolate(float y1, float y2, float mu) {
    float mu2 = (1 - cos(mu * PI)) / 2;
    return (y1 * (1 - mu2) + y2 * mu2);
}

// use the cosimne intrpolation to draw a smooth curve between two points
void drawSmoothCurve(int x0, int y0, int x1, int y1, uint16_t color) {
    int steps = 100;  // Number of interpolation steps
    for (int i = 0; i < steps; i++) {
        float mu = (float)i / (float)(steps - 1);
        int x = x0 + (x1 - x0) * mu;
        int y = cosineInterpolate(y0, y1, mu);
        tft.drawPixel(x, y, color);
    }
}

/*
        This function draws the reflow curve based on the selected reflow paste.
  It follows the calculated x-y position much closer than the streight line version above.

        Since the axes are slightly shifted from the edge of the display, there is a
  shift for the start of the preheat curve.
  The sections are drawn with curved lines.
        Different sections of the reflow curve are using different colors.
        The cooling time +20 is just an arbitrary value, just to illustrate the cooling part
  (decreasing temperature) on the reflow curve.
        It has no physical meaning other than it illustrates the cooling phase.
*/
void drawCurve_new() {
    tft.setTextSize(1);  // Reset the size to 1 in case the code is coming from someplace else

    drawAxis();

    // Draw the curve using smooth interpolation
    drawSmoothCurve(xGraph, yGraph, preheatTime_px, preheatTemp_px, YELLOW);
    drawSmoothCurve(preheatTime_px, preheatTemp_px, soakingTime_px, soakingTemp_px, ORANGE);
    drawSmoothCurve(soakingTime_px, soakingTemp_px, reflowTime_px, reflowTemp_px, RED);
    drawSmoothCurve(reflowTime_px, reflowTemp_px, coolingTime_px, coolingTemp_px, RED);
    drawSmoothCurve(coolingTime_px, coolingTemp_px, coolingTime_px + 40, coolingTemp_px + 20, BLUE);
}

//=================================================================================================

/*
  Function to create the basic graph display on the TFT screen
  It is used for the warmup, free heating and free cooling functions to
  plot their curves.
*/
void drawFreeCurve() {
    // first update the hotplate temperature reading
    tft.fillRoundRect(4, 10, 156, 115, RectRadius, BLACK);  // Erase previous content
    tft.fillScreen(BLACK);                                  // Repaint with black
    //-------------------------------------

    // Print the name of the default paste
    tft.setTextColor(WHITE);
    tft.drawString(pasteName, pasteNamePosX, pasteNamePosY, 2);

    drawAxis();
}

/*
  Draw the reflow curve on the display

  First we redraw the hotplate temperature reading, then we start to draw
  the graph by first drawing the x-y axis and graticule.
  We then calculate the pixel values for the portions of the graph, after which
  we can actually draw the graph with the colored segments.

  After that, we will position the actual temperatures and times above and below
  the graph segments.

*/
void drawReflowCurve() {
    if (redrawCurve == true) {
        // first update the hotplate temperature reading
        tft.fillRoundRect(4, 10, 156, 115, RectRadius, BLACK);  // Erase previous content
        tft.fillScreen(BLACK);                                  // Repaint with black

        // Print the name of the paste
        tft.setTextColor(WHITE);
        tft.drawString(pasteName, pasteNamePosX, pasteNamePosY, 2);

        // Calculate the portions of the curve to be plotted
        // Temperature and time values converted into pixel values
        // values are casted into integers (rounding errors can occur: (0.5 is rounded down to 0)
        preheatTemp_px = (int)(yGraph - (double)preheatTemp / tempPixelFactor);
        preheatTime_px = (int)(xGraph + (double)preheatTime / timePixelFactor);
        //--
        soakingTemp_px = (int)(yGraph - (double)soakingTemp / tempPixelFactor);
        soakingTime_px = (int)(xGraph + (double)soakingTime / timePixelFactor);
        //--
        reflowTemp_px = (int)(yGraph - (double)reflowTemp / tempPixelFactor);
        reflowTime_px = (int)(xGraph + (double)reflowTime / timePixelFactor);
        //--
        coolingTemp_px = (int)(yGraph - (double)coolingTemp / tempPixelFactor);
        coolingTime_px = (int)(xGraph + (double)coolingTime / timePixelFactor);

        // Draw the reflow curve
        drawCurve();

        // Draw the values of the portions of the curve
        // Preheat
        tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30);
        tft.setTextColor(RED);
        tft.print(preheatTemp);
        tft.print("C");
        tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
        tft.setTextColor(WHITE);
        tft.print(preheatTime);
        tft.print("s");
        //--
        // Soak
        tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
        tft.setTextColor(RED);
        tft.print(soakingTemp);
        tft.print("C");
        tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
        tft.setTextColor(WHITE);
        tft.print(soakingTime);
        tft.print("s");
        //--
        // Reflow
        tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
        tft.setTextColor(RED);
        tft.print(reflowTemp);
        tft.print("C");
        tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
        tft.setTextColor(WHITE);
        tft.print(reflowTime);
        tft.print("s");
        //--
        // Reflow-holding (cooling)
        tft.setCursor(coolingTime_px + 20, coolingTemp_px + 30);
        tft.setTextColor(BLUE);
        tft.print(coolingTemp);
        tft.print("C");
        tft.setCursor(coolingTime_px + 20, coolingTemp_px + 20);
        tft.setTextColor(WHITE);
        tft.print(coolingTime);
        tft.print("s");

        drawActionButtons();
        redrawCurve = false;  // disable the redraw (user will trigger a new redraw process)
    }
}

/*
  Function to draw the action "buttons" on the display for the warmup, reflow, free
  heating and free cooling functions. When selecting these fields, the user can
  either change the initial values, or start the action by pushing ther button,
  and pushing again to stop it.
*/
void drawActionButtons() {
    // place the warmup button
    tft.fillRoundRect(260, 0, 60, 15, RectRadius, DGREEN);  // X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("WARMUP", 265, 0, 2);

    // Free warmup value
    tft.fillRoundRect(220, 2, 32, 12, RectRadius, BLACK);  // X,Y, W,H, Color
    tft.setTextColor(RED);
    tft.drawString(String(warmupTemp) + "C", 228, 4, 1);

    // Place the reflow/stop button
    tft.fillRoundRect(260, 20, 60, 15, RectRadius, ORANGE);  // X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("REFLOW", 265, 20, 2);

    // Place the Heating/stop button
    tft.fillRoundRect(260, 40, 60, 15, RectRadius, RED);  // X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("HEATING", 265, 40, 2);

    // Free maximum Heating value
    tft.fillRoundRect(220, 42, 32, 12, RectRadius, BLACK);  // X,Y, W,H, Color
    tft.setTextColor(RED);
    tft.drawString(String(freeHeatingTemp) + "C", 228, 44, 1);

    // Place the Free Cooling/stop button
    tft.fillRoundRect(260, 60, 60, 15, RectRadius, BLUE);  // X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("COOLING", 265, 60, 2);

    // Free minimum Cooling value
    tft.fillRoundRect(220, 62, 32, 12, RectRadius, BLACK);  // X,Y, W,H, Color
    tft.setTextColor(BLUE);
    tft.drawString(String(freeCoolingTemp) + "C", 228, 64, 1);
}

/*
 Obtain the hot plate temperature using a thermocouple and the MAX6675
 Do regular readings and update the display every 0.25s (could be slower)

*/
void measureTemperature() {
    // Relevant YouTube video for this part: https://www.youtube.com/watch?v=PdS6-TccgK4
    if (millis() - temperatureTimer > 250)  // update frequency = 0.25s - faster than checking the heating (2s)
    {
        int status = thermoCouple.read();  // Do one read to make sure we get the valid temp reading

        /*
          If there is an issue, activate the code below
          to see the error code on the serial monitor

        if (status != 0) {
           Serial.print("Max status: ");
           Serial.print(status);
          Serial.print("\t");
        }
        */

        TCCelsius = thermoCouple.getTemperature();

        // Serial.print("Temp: ");
        // Serial.println(TCCelsius); //print converted data on the serial terminal

        // Update the text on the TFT display whenever a reading is finished
        printTemp();

        temperatureTimer = millis();  // reset timer
    }
}

/*
  Remove all the temp and time fields from the initial solder paste setup display
  when we go to reflow, free heating or free cooling.
  Also remove the action buttons

*/
void removeFieldsFromDisplay() {
    // When we select Reflow
    // Remove all the numbers, keep only the curve -> It makes the display cleaner, easier to read
    tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 30, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 20, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 20, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 10, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 10, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 20, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 30, 24, 9, RectRadius, BLACK);
    tft.fillRoundRect(coolingTime_px + 20, coolingTemp_px + 20, 24, 9, RectRadius, BLACK);

    // Also remove the free warmup, cooling and heating buttons and values
    tft.fillRoundRect(220, 0, 100, 15, RectRadius, BLACK);   // Warmup
    tft.fillRoundRect(220, 40, 100, 15, RectRadius, BLACK);  // Free Heating
    tft.fillRoundRect(220, 60, 100, 15, RectRadius, BLACK);  // Free cooling
}

// show and update the status field on the display
void updateStatus(int fieldColor, int textColor, const char* text) {
    // Erase the previously printed text
    tft.fillRoundRect(160, 190, 70, 18, RectRadius, fieldColor);
    tft.setTextColor(textColor);
    tft.drawString(text, 170, 190, 2);
}

// show and update the actual temperature to the TFT
void printTemp() {
    if (TCCelsius > 500)  // is incorrect value, could be a grounding issue?
    {
        tft.fillRoundRect(30, 40, 80, 16, RectRadius, RED);  // X,Y, W,H, Color
        tft.setTextColor(WHITE);
        tft.drawString("Temp ERROR", 32, 40, 2);
        TCCelsius = 55;  // so the graph is on the TFT area
    } else {
        tft.fillRoundRect(30, 40, 80, 16, RectRadius, DGREEN);  // X,Y, W,H, Color
        tft.setTextColor(WHITE);
        // can only print "°C" with font 2
        tft.drawString("Temp " + String(int(TCCelsius)) + "`C", 32, 40, 2);
    }
}

/*
 Print the calculated target temperature which is derived from the elapsed time
 and the reflow curve shown on the TFT
*/
void printTargetTemperature() {
    tft.fillRoundRect(30, 60, 80, 16, RectRadius, DGREEN);
    tft.setTextColor(WHITE);
    tft.drawString("Targt " + String(int(targetTemp)) + "`C", 32, 60, 2);  // can only print "°C" with font 2
}

// function to print the elapsed time on the TFT when one of the modes is active
void printElapsedTime() {
    tft.fillRoundRect(120, 40, 80, 16, RectRadius, DGREEN);
    tft.setTextColor(WHITE);
    tft.drawString("Time " + String(int(elapsedHeatingTime)) + "s", 122, 40, 2);
}

// Print the PWM information to the TFT
void printPWM() {
    tft.fillRoundRect(120, 60, 80, 16, RectRadius, DGREEN);
    tft.setTextColor(WHITE);
    tft.drawString("PID : " + String(int(Output)), 122, 60, 2);
}

// Print the fan information to the TFT
void printFan() {
    tft.fillRoundRect(120, 60, 80, 16, RectRadius, DGREEN);
    tft.setTextColor(WHITE);
    tft.drawString("FAN : " + Fan, 122, 60, 2);
}

// ============== End of code