#include <Arduino.h> // so we can use the Arduino specifics (port read/write, Byte, String, etc.)

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
const String FW_VERSION = "V5.0.3"; 
/*
  Changelog:
  Version V2.0.0:
  original code with updated splash screen

  Version V2.0.1:
  fixing the use of no-name TFT displays, but that requires changing one of the library files.

  Version V2.0.2:
  fixing the drawing of the curve when there is no change
  fixing the field color while in the edit mode and setting it back when exiting the edit mode
  changed the button detection to an ISR to get a better response

  Version V2.0.3:
  Fixed the no-name TFT display color code definitions and replaced all ST77XX definitions to ST7735.
  The library file does no longer need to get edited.

  Version V3.0.0:
  Port to the ESP32 DEVKIT 1 with schematic V2.0
  Changed most of the pin assignments so we can use hardware SPI and use a faster SPI clock.
  This will dramatically speed-up the tft activities, and fixed the screen1/2 redraw with fast encoder activities
  Changed the button code back to (delayed) polling, cannot use an ISR with the ESP and this code due to the many tft activities
  The rotary ISR now required a Schmitt-trigger gate on the CLK pin to eliminate many triggers in the undetermined zone of the edge

  Version V3.0.1:
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

  Version 5.0.1:
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

  Version 5.0.2:
  Switched to Virtual Studio Code and PlatformIO, and started using Git for version control.
  Also used the Copilot AI to help with the code for the DrawGraph() function.
  It is now using a more natural curve instead of straight lines between the points,
  which will make it easier to follow for the heaters.
  Added a run-time selection of solder pastes.
  Moved the preheat temp and time fields a bit out of the way of the curve.
  Removed the free heating and free cooling fields from the display when we reflow.
  
  Version 5.0.3
  Fixed the highlighting issue with the buttons. When the action was stopped, the field was not highlighted.

  Todo:

  Test hardware heat-up times with additional heaters and see if we need to change 
  the schematic or the code (two SSR's?, more K-type temp sensors?)

  Nice to have:
  Maybe add a way to store an updated/edited profile in EEPROM and load that at boot.
  Add a beeper to signal end-of-reflow

*/

//#include <arduino.h> // Just in case I use some Arduino specific functionality
#include <SPI.h>
#include <TFT_eSPI.h> // 2,4" SPI 240x320 - https://github.com/Ambercroft/TFT_eSPI/wiki
#include <ezButton.h> // for the rotary button press
#include "MAX6675.h"
#include <math.h>  // for the round() function

#define DSO_TRIG 4      // optional: to trace real-time activity on a scope

#define RotaryCLK 27  // CLK pin on the rotary encoder (must be an interrupt pin)
#define RotaryDT 32   // DT pin on the rotary encoder
#define RotarySW 33   // SW pin on the rotary encoder (Button function)

#define SSR_pin   2   // Switching the heater ON/OFF; also the built-in LED so we can see when the SSR is on.
#define Fan_pin   26  // GPIO pin for switching the fans ON/OFF (via a transistor)

#define MAX_CS    13  // CS pin for the MAX6675K
#define MAX_SO    21  // MISO for MAX6675
#define MAX_CLK    3  // SPI clock

// the definitions below are now defined in the platformio.ini file
//#define TFT_MOSI  23  // TFT SDA conn pin 3
//#define TFT_CLK   18  // TFT SCL conn pin 4
//#define TFT_CS    5   // TFF CS conn pin 5
//#define TFT_DC    16  // TFT DC conn pin 6
//#define TFT_RST   17  // TFT RES conn pin 7
//#define TFT_MISO  19  // TFT not used

// function prototypes
void rotaryEncoderISR();
void drawAxis();
void drawCurve();
void updateReflowState(double, double, const char*);
void heating();
void printTargetTemperature();
void printElapsedTime();
void controlSSR(double, double, unsigned long, unsigned long);
void updateStatus(uint16_t, const char*);
void processRotaryButton();
void measureTemperature();
void updateHighlighting();
void drawReflowCurve();
void freeHeating();
void freeCooling();
void drawActionButtons();

// setup the MAX library
MAX6675 thermoCouple(MAX_CS, MAX_SO, MAX_CLK);

// Constructor for the TFT screen
// using hardware SPI
TFT_eSPI tft = TFT_eSPI();
const int tftX = 320;
const int tftY = 240;

#define BLACK TFT_BLACK
#define WHITE TFT_WHITE
#define RED TFT_RED
#define GREEN TFT_GREEN
#define ORANGE TFT_ORANGE
#define MAGENTA TFT_MAGENTA
#define BLUE TFT_BLUE
#define YELLOW TFT_YELLOW
#define CYAN TFT_CYAN
#define DGREEN 0x046B // dark green
#define VLGREY 0xDF3D // very light grey
#define BGGREEN 0xD75C // background green
#define DGREY TFT_DARKGREY

// color picker: https://barth-dev.de/online/rgb565-color-picker/

// predefined colors in TFT_eSPI.h
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



//Rotary encoder related
int selectedItem = 1; //item number for the active menu item (was 0)
static bool ButtonPressed = false;
ezButton button(RotarySW); // create a ezButton object

//Statuses of the DT and CLK pins on the encoder
int CLKNow;
int CLKPrevious;
int DTNow;
int DTPrevious;


//---Thermocouple MAX6675
int TCRaw = 0; //raw value coming from the thermocouple module
double TCCelsius = 0; //Celsius value of the temperature reading
unsigned long temperatureTimer = 0; //Timer for measuring the temperature

bool coolingFanEnabled = false; //status that tells the code if the fan is enabled or not
bool heatingEnabled = false; //tells the code if the heating was enabled or not
bool reflow = false; //tells if the reflow process has been enabled
bool enableFreeHeating = false;
bool enableFreeCooling = false;
unsigned long SSRTimer = 0; //Timer for switching the SSR
unsigned long SSRInterval = 250; //update interval for switching the SSR - User is encouraged to experiment with this value
double fanTimer = 0; //Time length for the ON period for of the fan - User is encouraged to experiment with this value
double elapsedHeatingTime = 0; //Time spent in the heating phase (unit is ms)

// ==================================================================
// Reflow Curve parts for Chipquick Sn42/Bi57.6/Ag0.4 - 138C : I have this paste in a syringe
String pasteName = "Sn42/Bi57.6/Ag0.4";

// these vars will hold the various paste reflow curve values
// From room temperature to soaking temperature
volatile int preheatTemp = 0;
volatile int preheatTime = 0; //Note: times are always as compared to zero and not the length of the process.

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
  volatile int preheatTemp; // Temps in degrees C. Preheat is from room temperature to soaking temperature
  volatile int preheatTime; // Note: times in seconds are always as compared to zero and not the length of the process step.
  volatile int soakingTemp; // Soaking temperature (nearly flat curve)
  volatile int soakingTime;
  volatile int reflowTemp; // Soaking temperature to peak temperature (slight overshoot to peak temperature)
  volatile int reflowTime;
  volatile int coolingTemp; // Cooling temperature - Same temperature as the peak, because this part is more like keeping the solder around Tmelt for a short (~10s) time
  volatile int coolingTime;  
};

// Create an array of struct's for the various solder pastes. 
// The separator is the number of fields in the struct (9), so the array is created correctly.
// it can hold any number of solderpastes, the code handles that.
solderpaste solderpastes[] = {
  // Paste 0
  "Sn42/Bi57.6/Ag0.4", // Solderpaste
  90,   // preheatTemp
  90,   // preheatTime Note: times are always as compared to zero and not the length of the process.
  130,  // soakingTemp
  180,  // soakingTime
  165,  // reflowTemp
  240,  // reflowTime
  165,  // coolingTemp 
  260,  // coolingTime
  
  // Paste 1
  "Sn63/Pb37",
  100,
  30,
  150,
  120,
  235,
  210,
  235,
  220,

  // Paste 2
  "Sn63/Pb37 Mod",
  100,
  60, // changes this from unrealistic 30s to 60s
  150,
  120,
  235,
  210,
  235,
  220   
};

int solderPasteSelected = 0; // hold the index to the array of solderpastes
int prev_solderPasteSelected = 0; // previous selected solder paste index to avoid screen redraws
int numSolderpastes = 0; // the number of solderpastes in the array, will be set dynamically based on the size of the array


// Remember the original value during the editing mode. Used to eliminate
// a screen redraw when nothing is changed.
volatile int prev_soakingTemp = soakingTemp;
volatile int prev_preheatTemp = preheatTemp;
volatile int prev_preheatTime = preheatTime;
volatile int prev_soakingTime = soakingTime;
volatile int prev_reflowTemp = reflowTemp;
volatile int prev_reflowTime = reflowTime;
volatile int prev_coolingTemp = coolingTemp;
volatile int prev_coolingTime = coolingTime;
volatile int totalTime = coolingTime; // used in the new plotReflowProfile() function


// display the paste name on the TFT screen
const int pasteNamePosX = 50; // position from the left
const int pasteNamePosY = 1; // position from the top of the TFT. 

double targetTemp = 0; //A variable that holds one of the above 4 target values temporarily, based on the actual part of the active heating phase
volatile int freeHeatingTemp = 200; // Free heating target temperature
volatile int freeCoolingTemp = 40;  // Free cooling target temperature
volatile int freeWarmUpTemp = 40;   // Free heating warm-up temperature

//Conversion formulas for converting the physical values (Temp and time) into pixel values.
// Display is (YxX) 240x320px
// 250 degrees max
// px range is 18 from bottom and 80 from top
double tempPixelFactor = 250.0 / (tftY - (60+18)); // y = 250 / 162 = 1.54 ~ 1.5°C per pixel on Y
//-10: Axis is shifted by 18 from the bottom. 
// 360 seconds max
// px range is 18 from left and 2 from the right
double timePixelFactor = 360.0 / (tftX - (18+2)); // x = 360 / 154 = 1.2 ~1.2s per pixel on X
//-18: Axis is shifted by 18 from the left and ends 2 pixels before the end of the screen: Available area for plotting: 320-(20) = 300 px.

//Pixel conversions - converts the physical values (t, T) into pixel values for plotting the chart
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
volatile int itemCounter = -1; //this tells the code the active menu position
volatile int previousItemCounter = 0; //this tells the code the previous menu position (needed for the highlighting)
bool menuChanged = false; //it tells if the a new menu item should be drawn
bool menuSelected = false; //it tells if the user entered a menu (selected an item)
bool editMode = false; // Used to avoid the updateHighlighting when in the edit mode
//Selection of the fields by the rotation of the encoder
bool solderpasteFieldSelected = false;
bool preheatTempSelected = false;
bool preheatTimeSelected = false;
bool soakingTempSelected = false;
bool soakingTimeSelected = false;
bool reflowTempSelected = false;
bool reflowTimeSelected = false;
bool coolingTempSelected = false;
bool coolingTimeSelected = false;
bool startStopButtonSelected = false;
bool freeHeatingTargetSelected = false;
bool freeHeatingOnOffSelected = false;
bool freeCoolingTargetSelected = false;
bool freeCoolingOnOffSelected = false;


//--------------------------------------
bool redrawCurve = true; //tells the code if the reflow curve has to be redrawn
int RectRadius = 2; // The radius for the rounding of the menu fields


enum ReflowPhase
{
	PREHEAT = 0,
	SOAK,
	REFLOW,
	HOLD,
	COOLING
};
ReflowPhase currentPhase = PREHEAT; //Default phase

//Graph characteristics for the 240x320 TFT
// graph is 18px from the right and 15px from the bottom, we leave 3px free from the top and the right
//width: (320)  x: [18-317] -> 299px
//height: (240) y: [15-237] -> 222px
//temperature range: 20°C to 250°C -> 230°C
//time range: 0 s to 330 s -> 330 s (5 1/2min)
//The coordinate system of the display has inverted Y-axis.
// Default small font is 6px wide and 8px tall
// font 2 size is 7px wide and 10px tall
//----------------------------------------------------------------------------------------------------------------------------------

void setup()
{
	Serial.begin(9600);
  while (!Serial);
  delay(5000);
	Serial.print("\n\r\n\rReflow controller ");
  Serial.println(FW_VERSION);

  //Serial.print("tempPixelFactor = ");Serial.println(tempPixelFactor,3);
  //Serial.print("timePixelFactor = ");Serial.println(timePixelFactor,3);

  SPI.begin(); //start hardware SPI

	//----------------------------------------------------------------------------
	//PINS
  pinMode(DSO_TRIG, OUTPUT); // optional for tracing real-time events with a DSO
	//Rotary encoder-related
	pinMode(RotaryCLK, INPUT); //CLK
	pinMode(RotaryDT, INPUT);  //DT
  // the Rotary button is done by the library
  button.setDebounceTime(20); // set debounce time for the rotary button to 20 milliseconds
	attachInterrupt(digitalPinToInterrupt(RotaryCLK), rotaryEncoderISR, CHANGE); //CLK pin is inverted by a Schmitt-trigger gate
	//Reading the current status of the encoder for preparing it for the first movement/change
	CLKPrevious = digitalRead(RotaryCLK);
	DTPrevious = digitalRead(RotaryDT);
	//-----
	pinMode(SSR_pin, OUTPUT); //Define output pin for switching the SSR
	digitalWrite(SSR_pin, LOW); //SSR is OFF by default
	//----
	pinMode(Fan_pin, OUTPUT); //Define output pin for switching the fan (transistor)
	digitalWrite(Fan_pin, HIGH); //Enable fan - turn them on as a test to see if they spin up
  //-----
  Serial.println("setting up tft");
  tft.init();
  tft.setRotation(1); //Select the Landscape alignment - Use 3 to flip horizontally
  //-----
  thermoCouple.begin();
  thermoCouple.setSPIspeed(40000000);

  //----- set the initial solderpaste values
  numSolderpastes = sizeof(solderpastes) / sizeof(solderpaste); // the size of the array of solderpastes
  
  solderpaste current = solderpastes[solderPasteSelected]; // select the first one as the default 
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
  totalTime = coolingTime;
  
  //-----
  Serial.println("welcome screen on tft");

	tft.fillScreen(BLACK); //erase screen
  tft.setTextColor(WHITE);
	tft.setRotation(1); //Landscape alignment

  tft.setTextDatum(MC_DATUM); // center text on display; works on current font only
	tft.drawString("Automated reflow station "+FW_VERSION, tft.width() / 2, 40, 2);
  tft.drawString("2025 paulv", tft.width() / 2, 80, 2);
  tft.drawString("based on code from", tft.width() / 2, 100, 2);
  tft.drawString("www.curiousscientist.tech", tft.width() / 2, 120, 2);
  tft.setTextDatum(TL_DATUM); // switch back to left formatted

  vTaskDelay(500 / portTICK_PERIOD_MS); // wait a little to show it

  Serial.println("writing reflow curve");
	//Erase the screen, then draw the starting graph
	tft.fillScreen(BLACK);
	drawReflowCurve();
  drawActionButtons();
	digitalWrite(Fan_pin, LOW); //Disable fan - turn off the "spinning test" of the fans

  Serial.println("setup is done...");
}



void loop()
{
  button.loop(); // MUST call the ezButton loop() function first

  if (button.isPressed()) processRotaryButton();
  measureTemperature();
	updateHighlighting();
	//drawReflowCurve(); //This is now done in the various functions
	heating();
	freeHeating();
	freeCooling();
}


/*
  Interrupt Service Routine for the Rotary Encoder

  The interrupt was generated when a change on the RotaryCLK signal was detected.
  The ISR takes a little and we also have a hardware r/c delay, so by reading it again now, 
  we should have a stable level.

  Depending on the field we're in, we can adjust the value of temp and time

  The recommended method is to limit the execution time in an ISR to the absolute minimum.
  In this case however, we're not expecting other interrupts, and the actual time spent in the ISR
  is very short, despite the many lines of code.

  Todo:
  When you're rapidly scrolling forward, going through page 2 back to page 1, the screen does not refresh.
  That process gets interrupted and does not run the whole way.

*/
void IRAM_ATTR rotaryEncoderISR() // IRAM_ATTR
// we get here because the ISR detected a change on the CLK pin
{
  //digitalWrite(DSO_TRIG, HIGH); // track duration, typically 1.5us

  CLKNow = digitalRead(RotaryCLK); //Read the state of the CLK pin again
  if (preheatTempSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow) // read the DT level and determine the direction (CW or CCW)
      {
        if (preheatTemp > 20)
        {
          preheatTemp = preheatTemp -1;
        }
      }
      else
      {
        if (preheatTemp < 150) //typical max value for preheat phase - feel free to change it
        {
          preheatTemp = preheatTemp +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (preheatTimeSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (preheatTime > 0)
        {
          preheatTime = preheatTime -1;
        }
      }
      else
      {
        if (preheatTime < 90) //Typical preheat time
        {
          preheatTime = preheatTime +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (soakingTempSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (soakingTemp > 20)
        {
          soakingTemp = soakingTemp -1;
        }
      }
      else
      {
        if (soakingTemp < 180) //typical soaking temperature
        {
          soakingTemp = soakingTemp +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (soakingTimeSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (soakingTime > 0)
        {
          soakingTime = soakingTime -1;
        }
      }
      else
      {
        if (soakingTime < 180) //typical (total) time at the end of the soaking period
        {
          soakingTime = soakingTime +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (reflowTempSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (reflowTemp > 0)
        {
          reflowTemp = reflowTemp -1;
        }
      }
      else
      {
        if (reflowTemp < 250) //typical peak temp for reflow
        {
          reflowTemp = reflowTemp +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (reflowTimeSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (reflowTime > 0)
        {
          reflowTime = reflowTime -1;
        }
      }
      else
      {
        if (reflowTime < 240)
        {
          reflowTime = reflowTime +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (coolingTempSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (coolingTemp > 0)
        {
          coolingTemp = coolingTemp -1;
        }
      }
      else
      {
        if (coolingTemp < 250) //holding temperature before enterint the cooling phase
        {
          coolingTemp = coolingTemp +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state

  }
  else if (coolingTimeSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (coolingTime > 0)
        {
          coolingTime = coolingTime -1;
        }
      }
      else
      {
        if (coolingTime < 250) //total elapsed seconds before entering the cooling phase
        {
          coolingTime = coolingTime +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (freeHeatingTargetSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (freeHeatingTemp > 20)
        {
          freeHeatingTemp = freeHeatingTemp -1;
        }
      }
      else
      {
        if (freeHeatingTemp < 300) //Here we allow a little higher temperature than the reflow curve temperature
        {
          freeHeatingTemp = freeHeatingTemp +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (startStopButtonSelected == true)
  {
    // start/stop button does not do anything with the rotation of the encoder
  }
  else if (freeCoolingTargetSelected == true)
  {
    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (freeCoolingTemp > 20)
        {
          freeCoolingTemp = freeCoolingTemp -1;
        }
      }
      else
      {
        if (freeCoolingTemp < 200) //Here we allow a little higher temperature than the reflow curve temperature
        {
          freeCoolingTemp = freeCoolingTemp +1;
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  else if (freeHeatingOnOffSelected == true)
  {
    // freeHeatingOnOffSelected does not do anything with the rotation of the encoder
  }
  else if (freeCoolingOnOffSelected == true)
  {
    // freeCoolingOnOffSelected does not do anything with the rotation of the encoder
  } else if (solderpasteFieldSelected == true) { // Add the new field logic here
    if (CLKNow != CLKPrevious && CLKNow == 1) {
      if (digitalRead(RotaryDT) != CLKNow) {
        if (solderPasteSelected > 0) {
          solderPasteSelected = solderPasteSelected - 1;
        } else {
          solderPasteSelected = numSolderpastes - 1; // Wrap around to the last index
        }
      } else {
        if (solderPasteSelected < numSolderpastes - 1) {
          solderPasteSelected = solderPasteSelected + 1;
        } else {
          solderPasteSelected = 0; // Wrap around to the first index
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow; // Store last CLK state
  } else { //This navigates through the fields in the menu

    if (CLKNow != CLKPrevious && CLKNow == 1)
    {
      previousItemCounter = itemCounter;
      if (digitalRead(RotaryDT) != CLKNow)
      {
        if (itemCounter > 0)
        {
          itemCounter = itemCounter -1;
        }
        else
        {
          itemCounter = 13; //after the first menu item, we go back to the last menu item
        }
      }
      else
      {
        if (itemCounter < 13)
        {
          itemCounter = itemCounter +1;
        }
        else
        {
          itemCounter = 0;  //after the last menu item, we go back to the first menu item
        }
      }
    }
    menuChanged = true;
    CLKPrevious = CLKNow;  // Store last CLK state
  }
  //digitalWrite(DSO_TRIG, LOW); // track duration
}

void drawFreeCurve()
{

  // first update the hotplate temperature reading
  tft.fillRoundRect(4, 10, 156, 115, RectRadius, BLACK); //Erase previous content
  tft.fillScreen(BLACK); //Repaint with black - will cause a blink and make the thermometer disappear for 1 s
  //-------------------------------------
  //Axis pixel offset = 3

  // Print the name of the paste
  //tft.setCursor(pasteNamePosX, pasteNamePosY);
  tft.setTextColor(WHITE);
  //tft.print(pasteName);
  tft.drawString(pasteName,pasteNamePosX,pasteNamePosY,2);

  drawAxis();
}


/*
  Draw the reflow curve on the display

  First we redraw the hotplate temperature reading, then we start to draw
  the graph by first drawing the x-y axis.
  We then calculate the pixel values for the portions of the graph, after which
  we can actually draw the graph with the colored segments.
  
  After that, we will position the actual temperatures and times above and below 
  the graph segments.

*/
void drawReflowCurve()
{
	if (redrawCurve == true)
	{
    // first update the hotplate temperature reading
		tft.fillRoundRect(4, 10, 156, 115, RectRadius, BLACK); //Erase previous content
		tft.fillScreen(BLACK); //Repaint with black - will cause a blink and make the thermometer disappear for 1 s
		//-------------------------------------
		//Axis pixel offset = 3

    // Print the name of the paste
    //tft.setCursor(pasteNamePosX, pasteNamePosY);
		tft.setTextColor(WHITE);
		//tft.print(pasteName);
    tft.drawString(pasteName,pasteNamePosX,pasteNamePosY,2);

		//Calculate the portions of the curve to be plotted
		//Temperature values converted into pixel values - values are casted into integers (rounding errors can occur: +/-2°C and +/-2s error)
    preheatTemp_px = (int)((tftY - 3) - (((double)preheatTemp / tempPixelFactor)));
		preheatTime_px = (int)(3 + ((double)preheatTime / timePixelFactor));
		//--
    soakingTemp_px = (int)((tftY - 3) - (((double)soakingTemp / tempPixelFactor)));
		soakingTime_px = (int)(3 + ((double)soakingTime / timePixelFactor));
		//--
    reflowTemp_px = (int)((tftY - 3) - (((double)reflowTemp / tempPixelFactor)));
		reflowTime_px = (int)(3 + ((double)reflowTime / timePixelFactor));
		//--
    coolingTemp_px = (int)((tftY - 3) - (((double)coolingTemp / tempPixelFactor)));
		coolingTime_px = (int)(3 + ((double)coolingTime / timePixelFactor));

		//Draw reflow curve
		drawCurve();

		//Draw the values of the portions of the curve
		//Preheat
		//tft.setCursor(preheatTime_px - 25, preheatTemp_px - 20);
    tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30);
		tft.setTextColor(RED);
		tft.print(preheatTemp);
    tft.print("C");
		//tft.setCursor(preheatTime_px - 25, preheatTemp_px - 10);
    tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
		tft.setTextColor(WHITE);
		tft.print(preheatTime);
    tft.print("s");
		//--
		//Soak
		tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
		tft.setTextColor(RED);
		tft.print(soakingTemp);
    tft.print("C");
		tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
		tft.setTextColor(WHITE);
		tft.print(soakingTime);
    tft.print("s");
		//--
		//Reflow
		tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
		tft.setTextColor(RED);
		tft.print(reflowTemp);
    tft.print("C");
		tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
		tft.setTextColor(WHITE);
		tft.print(reflowTime);
    tft.print("s");
		//--
		//Reflow-holding (cooling)
		tft.setCursor(coolingTime_px+15, coolingTemp_px - 25);
		tft.setTextColor(BLUE);
		tft.print(coolingTemp);
    tft.print("C");
		tft.setCursor(coolingTime_px+15, coolingTemp_px - 15);
		tft.setTextColor(WHITE);
		tft.print(coolingTime);
    tft.print("s");

  	drawActionButtons();
		redrawCurve = false; //disable the redraw (user will trigger a new redraw process)
	}
}

void drawActionButtons(){

    // place the pre-warm button
    tft.fillRoundRect(260, 0, 60, 15, RectRadius, DGREEN); //X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("WARMUP", 265, 0, 2);

    // Free warmup value
    tft.fillRoundRect(220, 0, 32, 12, RectRadius, BLACK); //X,Y, W,H, Color
    tft.setTextColor(RED);
    tft.drawString(String(freeWarmUpTemp)+"C", 228, 4, 1);

    // Place the reflow/stop button
    tft.fillRoundRect(260, 20, 60, 15, RectRadius, ORANGE); //X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("REFLOW", 265, 20, 2);

    // Place the Heating/stop button
    tft.fillRoundRect(260, 40, 60, 15, RectRadius, RED); //X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("HEATING", 265, 40, 2);

    // Free maximum Heating value
    tft.fillRoundRect(220, 42, 32, 12, RectRadius, BLACK); //X,Y, W,H, Color
    tft.setTextColor(RED);
    tft.drawString(String(freeHeatingTemp)+"C", 228, 44, 1);

    // Place the Free Cooling/stop button
    tft.fillRoundRect(260, 60, 60, 15, RectRadius, BLUE); //X,Y, W,H, Color
    tft.setTextColor(WHITE);
    tft.drawString("COOLING", 265, 60, 2);

    // Free minimum Cooling value
    tft.fillRoundRect(220, 62, 32, 12, RectRadius, BLACK); //X,Y, W,H, Color
    tft.setTextColor(BLUE);
    tft.drawString(String(freeCoolingTemp)+"C", 228, 64, 1);

}




/*
 Obtain the hot plate temperature using a thermocouple and the MAX6675

*/
void measureTemperature()
{
	//Relevant YouTube video for this part: https://www.youtube.com/watch?v=PdS6-TccgK4
	if (millis() - temperatureTimer > 250) //update frequency = 0.25s - faster than checking the heating (2s)
	{
    int status = thermoCouple.read(); // do one read to make sure we get the valid temp reading
    // if (status != 0) {
    //   Serial.print("Max status: ");
    //   Serial.print(status);
    //   Serial.print("\t");
    // }

    TCCelsius = thermoCouple.getTemperature();
    if (TCCelsius > 1000)
    {
      // sensor is probably not connected, are we testing?
      TCCelsius = 150; // so we can test the phases
    }
		// Serial.print("Temp: ");
		// Serial.println(TCCelsius); //print converted data on the serial terminal

    //Update the text on the display whenever a reading is finished
    tft.fillRoundRect(30, 40, 80, 16, RectRadius, DGREEN); //X,Y, W,H, Color
		tft.setTextColor(WHITE);
    tft.drawString("Temp "+String(int(TCCelsius))+"`C", 32, 40, 2); // can only print "°C" with font 2

    temperatureTimer = millis(); //reset timer
	}
}



/*
  Remove all the fields from the initial setup display when we go to reflow, free heating or free cooling

*/
void removeFieldsFromDisplay()
{
  // When we select Reflow
  // Remove all the numbers, keep only the curve -> It makes the display cleaner, easier to read
  tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 30, 24, 9, RectRadius, BLACK);
  tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 20, 24, 9, RectRadius, BLACK);
  tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 20, 24, 9, RectRadius, BLACK);
  tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 10, 24, 9,  RectRadius,BLACK);
  tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 10, 24, 9, RectRadius, BLACK);
  tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 20, 24, 9, RectRadius, BLACK);
  tft.fillRoundRect(coolingTime_px + 15, coolingTemp_px - 26, 24, 9, RectRadius, BLACK);
  tft.fillRoundRect(coolingTime_px + 15, coolingTemp_px - 16, 24, 9, RectRadius, BLACK);

  // Also remove the free warmup, cooling and heating buttons and values
  tft.fillRoundRect(220, 0, 100, 15, RectRadius, BLACK); //X,Y, W,H, Color
  tft.fillRoundRect(220, 40, 100, 15, RectRadius, BLACK); //X,Y, W,H, Color
  tft.fillRoundRect(220, 60, 100, 15, RectRadius, BLACK); //X,Y, W,H, Color

}


/*
  The rotary button has been pressed so we enter the processing of the menu and values

  Unfortunately, the way the code was written, with many tft activities, 
  you can't turn this into an ISR, so we have to poll the activity in the main loop

  A press of the button enters the edit mode for the contend of the field it is on.
  The fields are selected by turning the rotary encoder itself. The selected field background it turned yellow.
  When you enter the edit mode for the field, the background of the field changes from yellow to green.
  The rotary encoder is used to select a new value.
  
  Another press of the button ends the edit mode and the complete graph is redrawn based on the new value.
  The last field on the normal display is the Start/Stop field. Pressing the button toggles between them.
  Going beyond the Start-Stop field will enter the optional "free" setting mode.
  In this mode, you can select the temperature and the Heating and Cooling fields.
  Entering edit in the temperature field allows you to set any temperature, selecting the 
  Heating and Cooling fields will turn them on or off with the button.

  Todo:
  Previously, leaving the edit mode would not highlight the background of the field you're on in
  yellow anymore. There is no background. The redraw function erases the field. - Fixed
  Not changing a value will still redraw the complete graph. - Fixed
  The button action is not very reliable, so I now use a library

*/
void processRotaryButton()
{
  switch (itemCounter) //checks the position in the menu
  {
  //--Preheat temperature
  case 0:
    preheatTempSelected = !preheatTempSelected; //flip the status

    if (preheatTempSelected == true)
    {
      //Edit mode: Green background, red number
      editMode = true;
      tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 20, 9, RectRadius, GREEN); //highlight
      //tft.setCursor(preheatTime_px - 25, preheatTemp_px - 20);
      tft.setTextColor(RED);
      //tft.print(preheatTemp);     
      tft.drawString(String(preheatTemp),preheatTime_px - 10, preheatTemp_px - 30, 1);

    }
    else
    {
      if (prev_preheatTemp != preheatTemp) // only redraw when there is a change
      {
        redrawCurve = true; //when the status becomes false, we exit the menu, thus we need to redraw the curve
        drawReflowCurve(); // Fix: this function erases the field background, so we call for it here.
        prev_preheatTemp = preheatTemp;
      }
      //Ending edit mode
      tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, YELLOW); //highlight
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

    if (preheatTimeSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
      tft.setTextColor(RED);
      tft.print(preheatTime);
    }
    else
    {
      if (prev_preheatTime != preheatTime) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_preheatTime = preheatTime;
      }
      //Ending edit mode
      tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, YELLOW); //highlight
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

    if (soakingTempSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
      tft.setTextColor(RED);
      tft.print(soakingTemp);
    }
    else
    {
      if (prev_soakingTemp != soakingTemp) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_soakingTemp = soakingTemp;
      }
      //Ending edit mode
      tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, YELLOW); //highlight
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

    if (soakingTimeSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
      tft.setTextColor(RED);
      tft.print(soakingTime);
    }
    else
    {
      if (prev_soakingTime != soakingTime) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_soakingTime = soakingTime;
      }
      //Ending edit mode
      tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, YELLOW); //highlight
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

    if (reflowTempSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
      tft.setTextColor(RED);
      tft.print(reflowTemp);
    }
    else
    {
      if (prev_reflowTemp != reflowTemp) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_reflowTemp = reflowTemp;
      }
      //Ending edit mode
      tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, YELLOW); //highlight
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

    if (reflowTimeSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
      tft.setTextColor(RED);
      tft.print(reflowTime);
    }
    else
    {
    if (prev_reflowTime != reflowTime) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_reflowTime = reflowTime;
      }
      //Ending edit mode
      tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, YELLOW); //highlight
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

    if (coolingTempSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 26, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(coolingTime_px+15, coolingTemp_px - 25);
      tft.setTextColor(RED);
      tft.print(coolingTemp);
    }
    else
    {
      if (prev_coolingTemp != coolingTemp) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_coolingTemp = coolingTemp;
      }
      //Ending edit mode
      tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 26, 24, 9, RectRadius, YELLOW); //highlight
      tft.setCursor(coolingTime_px+15, coolingTemp_px - 25);
      tft.setTextColor(BLUE);
      tft.print(coolingTemp);
      tft.print("C");
      editMode = false;
    }
    break;

  //--Cooling time
  case 7:
    coolingTimeSelected = !coolingTimeSelected;

    if (coolingTimeSelected == true)
    {
      editMode = true;
      //Green background
      tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 16, 24, 9, RectRadius, GREEN); //highlight
      tft.setCursor(coolingTime_px+15, coolingTemp_px - 15);
      tft.setTextColor(RED);
      tft.print(coolingTime);
    }
    else
    {
    if (prev_coolingTime != coolingTime) // only redraw when there is a change
      {
        redrawCurve = true;
        drawReflowCurve();
        prev_coolingTime = coolingTime;
      }
      //Ending edit mode
      tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 16, 24, 9, RectRadius, YELLOW); //highlight
      tft.setCursor(coolingTime_px+15, coolingTemp_px - 15);
      tft.setTextColor(RED);
      tft.print(coolingTime);
      tft.print("s");
      editMode = false;
    }
    break;

  case 8: //--Start/Stop reflow
    startStopButtonSelected = !startStopButtonSelected;

    if (startStopButtonSelected == true)
    {
      editMode = true;
      //Remove all the numbers, keep only the curve -> It makes the display cleaner, easier to read
      removeFieldsFromDisplay();
      drawCurve(); // redraw the curve

      //Update the Reflow button to a green background and label it stop
      tft.fillRoundRect(260, 20, 60, 15, RectRadius, GREEN); //X,Y, W,H, Color
      tft.setTextColor(RED);
      tft.drawString("STOP", 265, 20, 2);

      currentPhase = PREHEAT; //Set the current phase to preheat (in case we do a second reflow round)
      reflow = true; //Enable reflow
      heatingEnabled = true; //start heating
      elapsedHeatingTime = 0; //set the elapsed time to 0
    }else{
      // First, update all the buttons (easy way out)
      drawActionButtons();
      // update the reflow field so it's still marked as selected so we know where we are
      tft.fillRoundRect(260, 20, 60, 15, RectRadius, YELLOW); // still highlighted
		  tft.setTextColor(WHITE);
      tft.drawString("REFLOW", 265, 20, 2);

      //---------------------------
      //Put back all the values after stop
      reflow = false; //Reset reflow status flag to false (so free heating can run)
      redrawCurve = true; //simply redraw the whole graph
      heatingEnabled = false; //stop heating
      coolingFanEnabled = false; //stop cooling fan.
      //If user presses stop before the program is finished, we assume also that the fan is not needed
      // ending edit mode
      editMode = false;
      drawReflowCurve(); // redraw the curve with the values

      // Reapply the highlight to the selected field
      menuChanged = true; // Ensure menuChanged is set to true
      updateHighlighting();
    }
    break;

  case 9:  //--Free heating target temperature
    freeHeatingTargetSelected = !freeHeatingTargetSelected;

    if (freeHeatingTargetSelected == true)
    {
      editMode = true;
      tft.fillRoundRect(220, 42, 32, 12, RectRadius, GREEN); //X,Y, W,H, Color
      tft.setTextColor(RED);
      tft.drawString(String(freeHeatingTemp), 228, 44, 1);
    }else{
      tft.fillRoundRect(220, 42, 32, 12, RectRadius, YELLOW); //X,Y, W,H, Color
      tft.setTextColor(RED);
      tft.drawString(String(freeHeatingTemp), 228, 44, 1);
      editMode = false;
    }
    break;

  case 10: //--Start/stop free heating
    freeHeatingOnOffSelected = !freeHeatingOnOffSelected;

    if (freeHeatingOnOffSelected == true)
    {
      reflow = false;
      // clean the curve area
      drawFreeCurve();

      tft.fillRoundRect(260, 40, 60, 15, RectRadius, RED); //X,Y, W,H, Color
      tft.setTextColor(WHITE);
      tft.drawString("STOP", 265, 40, 2);
      enableFreeHeating = true;
      heatingEnabled = true; //start heating
      elapsedHeatingTime = 0; //set the elapsed time to 0
    }else{
      // First draw all the buttons (easy way out)
      drawActionButtons();
      // update the heating field so it's still marked as selected so we know where we are
      tft.fillRoundRect(260, 40, 60, 15, RectRadius, YELLOW); // still highlighted
		  tft.setTextColor(WHITE);
      tft.drawString("HEATING", 265, 40, 2);
      enableFreeHeating = false;
      freeHeatingOnOffSelected = false;
      //---------------------------
      //Put back all the values after stop
      digitalWrite(SSR_pin, LOW); // turn the heater off
      reflow = false; //Reset reflow status flag to false (so free heating can run)
      redrawCurve = true; //simply redraw the whole graph
      heatingEnabled = false; //stop heating
      coolingFanEnabled = false; //stop cooling fan.
      drawReflowCurve(); // redraw the curve with the values
      // Reapply the highlight to the selected field
      menuChanged = true; // Ensure menuChanged is set to true
      updateHighlighting();
}
    break;

  case 11: //--Free cooling temperature
    freeCoolingTargetSelected = !freeCoolingTargetSelected;

    if (freeCoolingTargetSelected == true)
    {
      editMode = true;
      tft.fillRoundRect(220, 62, 32, 12, RectRadius, GREEN); //X,Y, W,H, Color
      tft.setTextColor(BLUE);
      tft.drawString(String(freeCoolingTemp), 228, 64, 1);
    }else{
      // ending edit mode
      tft.fillRoundRect(220, 62, 32, 12, RectRadius, YELLOW); //X,Y, W,H, Color
      tft.setTextColor(BLUE);
      tft.drawString(String(freeCoolingTemp), 228, 64, 1);
      editMode = false;
    }
    break;

  case 12: //-- Start/stop free Cooling
    freeCoolingOnOffSelected = !freeCoolingOnOffSelected;

    if (freeCoolingOnOffSelected == true)
    {
      reflow = false;
      // clean the curve area
      drawFreeCurve();

      tft.fillRoundRect(260, 60, 60, 15, RectRadius, BLUE); //X,Y, W,H, Color
      tft.setTextColor(WHITE);
      tft.drawString("STOP", 265, 60, 2);
      enableFreeCooling = true;
      elapsedHeatingTime = 0; //set the elapsed time to 0
      digitalWrite(SSR_pin, LOW); // just in case it's still on when we select freecooling after freeheating
    }else{
      // First draw all the buttons (easy way out)
      drawActionButtons();
      // Then update the cooling field so it's still marked as selected so we know where we are
      tft.fillRoundRect(260, 60, 60, 15, RectRadius, YELLOW);
      tft.setTextColor(WHITE);
      tft.drawString("COOLING", 265, 60, 2);
      enableFreeCooling = false;
      freeCoolingOnOffSelected = false;
      //---------------------------
      //Put back all the values after stop
      reflow = false; //Reset reflow status flag to false (so free heating can run)
      redrawCurve = true; //simply redraw the whole graph
      heatingEnabled = false; //stop heating
      coolingFanEnabled = false; //stop cooling fan.
      drawReflowCurve(); // redraw the curve with the values
      // Reapply the highlight to the selected field
      menuChanged = true; // Ensure menuChanged is set to true
      updateHighlighting();

    }
    break;

  case 13: // change the solder paste
    solderpasteFieldSelected = !solderpasteFieldSelected;
    if (solderpasteFieldSelected == true) {
      editMode = true;
      // highlighting the edit mode
      tft.fillRoundRect(48, 0, 150, 18, RectRadius, GREEN); //X,Y, W,H, Color
      tft.setTextColor(RED);
      // Fetch the paste name from the array using solderPasteSelected
      pasteName = solderpastes[solderPasteSelected].pasteName;
      tft.drawString(pasteName,pasteNamePosX,pasteNamePosY,2);
     } else {
      if (prev_solderPasteSelected != solderPasteSelected) // only redraw when there is a change
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
          totalTime = coolingTime;

          drawReflowCurve();
          prev_solderPasteSelected = solderPasteSelected;
        }
      // ending edit mode
      tft.fillRoundRect(48, 0, 150, 18, RectRadius, YELLOW); //X,Y, W,H, Color
      tft.setTextColor(RED);
      pasteName = solderpastes[solderPasteSelected].pasteName;
      tft.drawString(pasteName,pasteNamePosX,pasteNamePosY,2);
      editMode = false;
    }
    break;
  }
  menuChanged = false;
}

/*
  updateHighlighting

  This function is called every main loop cycle.
  Depending on the menu (=field), determined by the rotary encoder, update the information on the display

  Added a check for the edit mode so we keep the proper highlighting
*/
void updateHighlighting()
{
	if (menuChanged == true) //if somewhere in the code we changed the menu, we will be able to enter the part below
	{
		switch (itemCounter) //check which menu was changed
		{
    case 0: // Preheat temp
      if (editMode)
      {
        tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, YELLOW); //highlight
      }
      tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30); //set the cursor to the corresponding spot
      tft.setTextColor(RED); //set text color
      tft.print(preheatTemp); //print the value
    
      //Note: All the following lines are doing the same, they just print to different coordinates and different values.
      break;

		case 1: // Preheat time
      if (editMode)
      {
        tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, YELLOW); //highlight
      }			
			tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
			tft.setTextColor(RED);
			tft.print(preheatTime);
			break;

		case 2: // Soaking temp
      if (editMode)
      {
			  tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, YELLOW); //highlight
      }
			tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
			tft.setTextColor(RED);
			tft.print(soakingTemp);
			break;

		case 3: // Soaking time
      if (editMode)
      {
        tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, YELLOW); //highlight
      }
			tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
			tft.setTextColor(RED);
			tft.print(soakingTime);
			break;

		case 4: // Reflow temp
      if (editMode)
      {
        tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, YELLOW); //highlight
      }
			tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
			tft.setTextColor(RED);
			tft.print(reflowTemp);
			break;

		case 5: // Reflow time
      if (editMode)
      {
        tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, YELLOW); //highlight
      }
			tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
			tft.setTextColor(RED);
			tft.print(reflowTime);
			break;

		case 6: // Cooling temp
      if (editMode)
      {
        tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 26, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
        tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 26, 24, 9, RectRadius, YELLOW); //highlight
      }
			tft.setCursor(coolingTime_px+15, coolingTemp_px - 25);
			tft.setTextColor(BLUE);
			tft.print(coolingTemp);
			break;

		case 7: // Cooling time
      if (editMode)
      {
			  tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 16, 24, 9, RectRadius, GREEN); //highlight edit mode
      }else{
			  tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 16, 24, 9, RectRadius, YELLOW); //highlight
      }
			tft.setCursor(coolingTime_px+15, coolingTemp_px - 15);
			tft.setTextColor(RED);
			tft.print(coolingTime);
			break;

		case 8: // Reflow start/stop
      if (editMode)
      {
        tft.fillRoundRect(260, 20, 60, 15, RectRadius, GREEN); //X,Y, W,H, Color
      }else{
        tft.fillRoundRect(260, 20, 60, 15, RectRadius, YELLOW); //X,Y, W,H, Color
      }   
      tft.setTextColor(BLACK);
      tft.drawString("REFLOW", 265, 20, 2);
			break;

		case 9: // free heating target temp
      if (editMode)
      {
        tft.fillRoundRect(220, 42, 32, 12, RectRadius, GREEN); //X,Y, W,H, Color
      }else{
        tft.fillRoundRect(220, 42, 32, 12, RectRadius, YELLOW); //X,Y, W,H, Color
      }
      tft.setTextColor(RED);
      tft.drawString(String(freeHeatingTemp), 228, 44, 1);		
			break;

		case 10: //Free heating on/off
      if (editMode)
			{
        tft.fillRoundRect(260, 40, 60, 15, RectRadius, GREEN); //X,Y, W,H, Color
      }else{
        tft.fillRoundRect(260, 40, 60, 15, RectRadius, YELLOW); //X,Y, W,H, Color
      }
      tft.setTextColor(BLACK);
      tft.drawString("HEATING", 265, 40, 2);
			break;

		case 11: //free cooling temp
			if (editMode)
			{
        tft.fillRoundRect(220, 62, 32, 12, RectRadius, GREEN); //X,Y, W,H, Color
			}else{
        tft.fillRoundRect(220, 62, 32, 12, RectRadius, YELLOW); //X,Y, W,H, Color
      }
      tft.setTextColor(BLUE);
      tft.drawString(String(freeCoolingTemp), 228, 64, 1);
			break;

    case 12: // Free cooling selected on/off
      if (editMode)
      {
        tft.fillRoundRect(260, 60, 60, 15, RectRadius, GREEN); //X,Y, W,H, Color
      }else{
        tft.fillRoundRect(260, 60, 60, 15, RectRadius, YELLOW); //X,Y, W,H, Color
      }
      tft.setTextColor(BLACK);
      tft.drawString("COOLING", 265, 60, 2);

      //previousItemCounter = -1; //clear highlight status
      break;

    case 13: // solderpast field
      if (editMode) {
        tft.fillRoundRect(48, 0, 150, 18, RectRadius, GREEN); //X,Y, W,H, Color
      }else{
        tft.fillRoundRect(48, 0, 150, 18, RectRadius, YELLOW); //X,Y, W,H, Color
      }
      tft.setTextColor(RED);
      pasteName = solderpastes[solderPasteSelected].pasteName;
      tft.drawString(pasteName,pasteNamePosX,pasteNamePosY,2);
      break;

		}
		//--------------------------------------------------------------------------------------------
		//Remove the previous highlighting
		//--------------------------------------------------------------------------------------------
		switch (previousItemCounter) //check which item was previously highlighted so we can restore its original look (no highlighting)
		{
		case 0: // preheat temp
			tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 31, 24, 9, RectRadius, BLACK); //restore original background (black)
			tft.setCursor(preheatTime_px - 10, preheatTemp_px - 30); //set the cursor to the corresponding place (inside the rectangle)
			tft.setTextColor(RED); //set text color
			tft.print(preheatTemp); //print value	
      tft.print("C");
			break;
		case 1: // preheat time
			tft.fillRoundRect(preheatTime_px - 10, preheatTemp_px - 21, 24, 9, RectRadius, BLACK);
			tft.setCursor(preheatTime_px - 10, preheatTemp_px - 20);
			tft.setTextColor(WHITE);
			tft.print(preheatTime);
      tft.print("s");
			break;
		case 2: // soaking temp
			tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 21, 24, 9, RectRadius, BLACK);
			tft.setCursor(soakingTime_px - 25, soakingTemp_px - 20);
			tft.setTextColor(RED);
			tft.print(soakingTemp);
      tft.print("C");
			break;
		case 3: // soaking time
			tft.fillRoundRect(soakingTime_px - 25, soakingTemp_px - 11, 24, 9, RectRadius, BLACK);
			tft.setCursor(soakingTime_px - 25, soakingTemp_px - 10);
			tft.setTextColor(WHITE);
			tft.print(soakingTime);
      tft.print("s");
			break;
		case 4: //reflow temp
			tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 9, 24, 9, RectRadius, BLACK);
			tft.setCursor(reflowTime_px - 5, reflowTemp_px + 10);
			tft.setTextColor(RED);
			tft.print(reflowTemp);
      tft.print("C");
			break;
		case 5: // reflow time
			tft.fillRoundRect(reflowTime_px - 5, reflowTemp_px + 19, 24, 9, RectRadius, BLACK);
			tft.setCursor(reflowTime_px - 5, reflowTemp_px + 20);
			tft.setTextColor(WHITE);
			tft.print(reflowTime);
      tft.print("s");
			break;
		case 6: // cooling temp
			tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 26, 24, 9, RectRadius, BLACK);
			tft.setCursor(coolingTime_px+15, coolingTemp_px - 25);
			tft.setTextColor(BLUE);
			tft.print(coolingTemp);
      tft.print("C");
			break;
		case 7: // cooling time
			tft.fillRoundRect(coolingTime_px+15, coolingTemp_px - 16, 24, 9, RectRadius, BLACK);
			tft.setCursor(coolingTime_px+15, coolingTemp_px - 16);
			tft.setTextColor(WHITE);
			tft.print(coolingTime);
      tft.print("s");
			break;
		case 8: // Reflow start/stop
      tft.fillRoundRect(260, 20, 60, 15, RectRadius, ORANGE); //X,Y, W,H, Color
      tft.setTextColor(WHITE);
      tft.drawString("REFLOW", 265, 20, 2);
			break;
		case 9: // free heating target temp
      tft.fillRoundRect(220, 42, 32, 12, RectRadius, BLACK); //X,Y, W,H, Color
      tft.setTextColor(RED);
      tft.drawString(String(freeHeatingTemp)+"C",228, 44, 1);
			break;
		case 10: // free heating on/off
      tft.fillRoundRect(260, 40, 60, 15, RectRadius, RED); //X,Y, W,H, Color
      tft.setTextColor(WHITE);
      tft.drawString("HEATING", 265, 40, 2);
			break;
		case 11: // free cooling temp
      tft.fillRoundRect(220, 62, 32, 12, RectRadius, BLACK); //X,Y, W,H, Color
      tft.setTextColor(BLUE);
      tft.drawString(String(freeCoolingTemp)+"C", 228, 64, 1);
			break;
    case 12: // free cooling on/off
      tft.fillRoundRect(260, 60, 60, 15, RectRadius, BLUE); //X,Y, W,H, Color
      tft.setTextColor(WHITE);
      tft.drawString("COOLING", 265, 60, 2);
      break;    
    case 13: // solderpaste field
      tft.fillRoundRect(46, 0, 152, 20, RectRadius, BLACK); //erase the previous
      tft.setTextColor(WHITE);
      pasteName = solderpastes[solderPasteSelected].pasteName;
      tft.drawString(pasteName,pasteNamePosX,pasteNamePosY,2);
      break;
		}
		menuChanged = false;
	}
}


void heating()
{
	if (reflow == true)//Only proceed if the reflow was enabled by the press of the start button.
	{
		if (heatingEnabled == true) //If heating was enabled somewhere in the code, we can enter the code below
		{
			unsigned long timeNow = millis();

			if (timeNow - SSRTimer > SSRInterval) //update frequency = 250 ms - should be less frequent than the temperature readings
			{
				//Draw a pixel for the temperature measurement - Calculate the position
				measuredTemp_px = (int)((tftY - 13) - ((TCCelsius / tempPixelFactor)));
        measuredTime_px = (int)(18 + (elapsedHeatingTime / timePixelFactor));

				//Also print the elapsed time in second"
        printElapsedTime();

				//Send time and temperature to the serial port with a space (" ") separator, so it can be easily plotted
				/*
					Serial.print(elapsedHeatingTime);
					Serial.print("  ");
					Serial.println(TCCelsius);
					Serial.println(" ");
					//Send pixel values too
					Serial.print(measuredTime_px);
					Serial.print("  ");
					Serial.println(measuredTemp_px);
				*/

				//Draw the pixel (time vs. temperature) on the chart
				tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
				tft.drawPixel(measuredTime_px, measuredTemp_px + 1, CYAN); //putting another pixel next (on Y) the original, "fake a thick line"

				switch (currentPhase) //This part determines the code's progress along the reflow curve
				{
				case PREHEAT:
					targetTemp = 20 + (elapsedHeatingTime * (1.0 / preheatTime) * (preheatTemp - 20)); //20! - this is important because it is also the zero of the axis
					//Example: 20 + (0 * (1/90) * (90-20)) = 20 + 0 = 20
					// 2s later: 20 + (2 * (1/90) * (90-20)) = 20 + 1.56 = 21.6
					// 10s later: 20 + (10 * (1/90) * (90-20)) = 20 + 7.78 = 27.8
					//...
					//updateReflowState(TCCelsius, targetTemp, "Preheat");
          updateReflowState(TCCelsius, targetTemp, "Preheat");
					printTargetTemperature(); //Print the target temperature that we calculated above

          digitalWrite(SSR_pin, HIGH);  // heater at full blast 
          tft.fillCircle(237, 7, 6, RED);  // show SSR is on 

          //Serial.print("delta time : "); Serial.println(preheatTime - elapsedHeatingTime); //Serial.print("  preheatTime : ");Serial.println(preheatTime);
          
          // Serial.print("Preheat target: ");
          // Serial.print(targetTemp);
          // Serial.println(" C");
          // Serial.print("Current temp: ");
          // Serial.print(TCCelsius);
          // Serial.println(" C");
          // Serial.print("Delta temp: ");
          // Serial.print(targetTemp - TCCelsius);
          // Serial.println(" C");


					if (TCCelsius > preheatTemp && elapsedHeatingTime > preheatTime) //check if we have reached the program target
					{
						currentPhase = SOAK;
					}
					//Note: The following parts follow the same strategy. We calculate the expected temperature at the moment and based on the result
					//we either turn the heater on (or keep it on), or turn the heater off (or keep it off). This is a very primitive control and it is
					//not the most precise strategy, but at this moment, I go with this simple approach.
					break;

				case SOAK:

					targetTemp = preheatTemp + ((elapsedHeatingTime - preheatTime) * (1.0 / (soakingTime - preheatTime)) * (soakingTemp - preheatTemp));
					//Example: 90 + ((90-90) * (1/(180-90)) * (130-90) = 90
					// 2s later: 90 +((92-90) * (1/90) * (40)) = 90 + 0.89 = 90.9
					// 10s later: 90 +((100-90) * (1/90) * (40)) = 90 + 4.44 = 94.4
					//. . .

					printTargetTemperature();
					updateReflowState(TCCelsius, targetTemp, "Soaking");
          controlSSR(targetTemp, TCCelsius, timeNow, SSRInterval); //Control the SSR

					/*
					  Serial.print("Soak target: ");
					  Serial.print(targetTemp);
					  Serial.println(" C");
            Serial.print("Current temp: ");
					  Serial.print(TCCelsius);
					  Serial.println(" C");
					*/

					if (TCCelsius > soakingTemp && elapsedHeatingTime > soakingTime) //check if we have reached the program target
					{
						currentPhase = REFLOW;
					}
					break;

				case REFLOW:

					targetTemp = soakingTemp + ((elapsedHeatingTime - soakingTime) * (1.0 / (reflowTime - soakingTime)) * (reflowTemp - soakingTemp));
					//Example: 130 + ((180-180) * (1/(240-180)) * (165-130)) = 130
					//2s later: 130 + ((182-180) * (1/60) * (35)) = 130 + 1.17 = 131.2
					// 10s later: 130 + ((190-180) * (1/60) * (35)) = 130 + 5.83 = 135.8

					printTargetTemperature();
					updateReflowState(TCCelsius, targetTemp, "Reflow");
					controlSSR(targetTemp, TCCelsius, timeNow, SSRInterval);

					/*
					  Serial.print("Reflow target: ");
					  Serial.print(targetTemp);
					  Serial.println(" C");
					*/

					if (TCCelsius > reflowTemp && elapsedHeatingTime > reflowTime) //check if we have reached the program target
					{
						currentPhase = HOLD;
					}
					break;

				case HOLD:

					targetTemp = reflowTemp + ((elapsedHeatingTime - reflowTime) * (1.0 / (coolingTime - reflowTime)) * (coolingTemp - reflowTemp));
					//Example: 165 + ((240-240) * (1/(250-240)) * (165-165)) = 165
					//2s later: 165 + ((242-240) * (1/10) * (0) = 165 + 2 = 165
					//10s later: 165 + ((250-240) * (1/10) * (0) = 165 + 10 = 165

					printTargetTemperature();
					updateReflowState(TCCelsius, targetTemp, "Holding");
					controlSSR(targetTemp, TCCelsius, timeNow, SSRInterval);

					/*
					  Serial.print("Reflow-holding target: ");
					  Serial.print(targetTemp);
					  Serial.println(" C");
					*/

					if (TCCelsius > coolingTemp && elapsedHeatingTime > coolingTime) //check if we have reached the program target
					{
						currentPhase = COOLING;
					}
					break;

				case COOLING:

					//Serial.print("Cooling started.");
          updateReflowState(TCCelsius, targetTemp, "Cooling");
    
					//Turn off heating
					heatingEnabled = false; //disable heating
					digitalWrite(SSR_pin, LOW); //turn off the SSR - heating is OFF

					//-----------------------------------------------------------------
					//Turn on cooling
					coolingFanEnabled = true; //enable cooling
					digitalWrite(Fan_pin, HIGH); //turn on the fan
					fanTimer = millis(); //start fan timer from this period
					break;
				}

				elapsedHeatingTime += (SSRInterval / 1000.0); //SSRInterval is in ms, so it has to be divided by 1000

				/*
				  Serial.print("Elapsed heating time: ");
				  Serial.println(elapsedHeatingTime);
				*/

				SSRTimer = millis();
			}
		}
		else //heating is NOT enabled (disabled) but we're still in the reflow process (last section of the curve)
		{
			if (millis() - SSRTimer > SSRInterval) //update frequency = 1s - should be less frequent than the temperature readings
			{
				if (coolingFanEnabled == true && millis() - fanTimer > 120000) //If fan is enabled and 2 min elapsed
				{
					digitalWrite(Fan_pin, LOW); //turn off the fan after 2 min
					coolingFanEnabled = false; //Set the flag to the corresponding status
					reflow = false; //Reflow is finished

					//Upon exiting, redraw the whole display and default everything
          tft.fillRoundRect(260, 0, 60, 15, RectRadius, RED); //X,Y, W,H, Color
          tft.setTextColor(WHITE);
          tft.drawString("REFLOW", 265, 20, 2);

					//---------------------------
					//Put back all the values after stop
					redrawCurve = true; //simply redraw the whole graph
					drawReflowCurve();
					heatingEnabled = false; //stop heating
				}

				if (coolingFanEnabled == true) //If heating is disabled, but the cooling is enabled, keep drawing the chart
				{
					elapsedHeatingTime += (SSRInterval / 1000.0); //keep interval ticking

					//Keep drawing the realtime temperature curve while the cooling is ongoing
          measuredTemp_px = (int)((tftY - 5) - ((TCCelsius / tempPixelFactor)));
					measuredTime_px = (int)(3 + (elapsedHeatingTime / timePixelFactor));

					//Keep printing the elapsed time and keep plotting the cooling portion of the curve
          printElapsedTime();
          
          // draw the actual curve
					tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
					tft.drawPixel(measuredTime_px, measuredTemp_px + 1, CYAN); //putting another pixel next (on Y) the original, fake thick line
				}

				SSRTimer = millis();
			}
		}
	}
}


void printElapsedTime(){

	tft.fillRoundRect(120, 40, 80, 16, RectRadius, DGREEN); 
  tft.setTextColor(WHITE);
  tft.drawString("Time "+String(int(elapsedHeatingTime))+"s", 122, 40, 2);

}


void freeHeating()
{
	if (reflow == false) //Only proceed if the reflow was enabled by the press of the start button.
	{
		if (enableFreeHeating == true) //If heating was enabled somewhere in the code, we can enter the code below
		{
			unsigned long timeNow = millis();

			if (timeNow - SSRTimer > SSRInterval) //update frequency = 250 ms - should be less frequent than the temperature readings
			{
				//Draw a pixel for the temperature measurement - Calculate the position
				measuredTemp_px = (int)((tftY - 13) - ((TCCelsius / tempPixelFactor))); // 220 -> 200 offset is 3
        measuredTime_px = (int)(18 + (elapsedHeatingTime / timePixelFactor));

				//Also print the elapsed time in second
        printElapsedTime();

				//Draw the pixel (time vs. temperature) on the chart
				tft.drawPixel(measuredTime_px, measuredTemp_px, CYAN);
				tft.drawPixel(measuredTime_px, measuredTemp_px + 1, CYAN); //putting another pixel next (on Y) the original, "fake a thick line"

        //targetTemp = 20 + (elapsedHeatingTime * (1.0 / preheatTime) * (preheatTemp - 20)); //20! - this is important because it is also the zero of the axis
        //Example: 20 + (0 * (1/90) * (90-20)) = 20 + 0 = 20
        // 2s later: 20 + (2 * (1/90) * (90-20)) = 20 + 1.56 = 21.6
        // 10s later: 20 + (10 * (1/90) * (90-20)) = 20 + 7.78 = 27.8
        //...
        targetTemp = freeHeatingTemp;
        updateReflowState(TCCelsius, targetTemp, "Heating");
        printTargetTemperature(); //Print the target temperature that we calculated above
        controlSSR(targetTemp, TCCelsius, timeNow, SSRInterval);
				elapsedHeatingTime += (SSRInterval / 1000.0); //SSRInterval is in ms, so it has to be divided by 1000

				SSRTimer = millis();
			}
		}
	}
}



void freeCooling()
{
	if (reflow == false) //Only proceed if the reflow was enabled by the press of the start button.
	{
		if (enableFreeCooling == true) //If heating was enabled somewhere in the code, we can enter the code below
		{
			unsigned long timeNow = millis();

			if (timeNow - SSRTimer > SSRInterval) //update frequency = 250 ms - should be less frequent than the temperature readings
			{
				//Draw a pixel for the temperature measurement - Calculate the position
				measuredTemp_px = (int)((tftY - 13) - ((TCCelsius / tempPixelFactor))); // 220 -> 200 offset is 3
        measuredTime_px = (int)(18 + (elapsedHeatingTime / timePixelFactor));

				//Also print the elapsed time in second
        printElapsedTime();

				//Draw the pixel (time vs. temperature) on the chart
				tft.drawPixel(measuredTime_px, measuredTemp_px, BLUE);
				tft.drawPixel(measuredTime_px, measuredTemp_px + 1, BLUE); //putting another pixel next (on Y) the original, "fake a thick line"
        //tft.drawLine(measuredTime_px, measuredTemp_px, measuredTime_px-50, measuredTime_px+50, WHITE);

        //targetTemp = 20 + (elapsedHeatingTime * (1.0 / preheatTime) * (preheatTemp - 20)); //20! - this is important because it is also the zero of the axis
        //Example: 20 + (0 * (1/90) * (90-20)) = 20 + 0 = 20
        // 2s later: 20 + (2 * (1/90) * (90-20)) = 20 + 1.56 = 21.6
        // 10s later: 20 + (10 * (1/90) * (90-20)) = 20 + 7.78 = 27.8
        //...
        targetTemp = freeCoolingTemp;
        updateReflowState(TCCelsius, targetTemp, "Cooling");
        printTargetTemperature(); //Print the target temperature that we calculated above
        //controlSSR(targetTemp, TCCelsius, timeNow, SSRInterval);
        //digitalWrite(SSR_pin, HIGH);  // heater at full blast 
        //tft.fillCircle(237, 7, 6, RED);  // show SSR is on 
        if (TCCelsius > freeCoolingTemp) //Turn the fans ON or OFF depending on the flag
          {
            digitalWrite(Fan_pin, HIGH);
          }else{
            digitalWrite(Fan_pin, LOW);
          }         

				elapsedHeatingTime += (SSRInterval / 1000.0); //SSRInterval is in ms, so it has to be divided by 1000

				SSRTimer = millis();
			}
		}
	}
}


void drawAxis()
{

  // Draw the chart axes
   
  //Y-axis line (vertical - temperature): total 320px
  //tft.drawLine(3, ((int)237-(250/tempPixelFactor)-10), 3, 237, RED); //X0, Y0, X1, Y1, Color  
  //tft.drawLine(18, ((int)237-(250/tempPixelFactor)-25), 18, 237-13, RED); //X0, Y0, X1, Y1, Color
  tft.drawLine(18, ((int)238-(250/tempPixelFactor))-12, 18, 238-13, RED); //X0, Y0, X1, Y1, Color

  //Horizontal lines (ticks) at every 50C
  tft.drawLine(18, (int)237-(50/tempPixelFactor), 22, (int)237-(50/tempPixelFactor), RED);   // 50C
  tft.drawLine(18, (int)237-(100/tempPixelFactor), 22, (int)237-(100/tempPixelFactor), RED); //100C
	tft.drawLine(18, (int)237-(150/tempPixelFactor), 22, (int)237-(150/tempPixelFactor), RED); //150C
	tft.drawLine(18, (int)237-(200/tempPixelFactor), 22, (int)237-(200/tempPixelFactor), RED); //200C
  tft.drawLine(18, (int)237-(250/tempPixelFactor), 22, (int)237-(250/tempPixelFactor), RED); //250C

  // tick values
  tft.drawString("50", 5, (int)237-(50/tempPixelFactor)-3, 1); // text, x, y color
  tft.drawString("100", 0, (int)237-(100/tempPixelFactor)-3, 1);
  tft.drawString("150", 0, (int)237-(150/tempPixelFactor)-3, 1);
  tft.drawString("200", 0, (int)237-(200/tempPixelFactor)-3, 1);
  tft.drawString("250", 0, (int)237-(250/tempPixelFactor)-3, 1);

  //X-axis line (horizontal - time) : total 240px
  tft.drawLine(18, 240-13, ((int)360/timePixelFactor), 240-13, WHITE); //X0, Y0, X1, Y1, Color
  //tft.drawLine(18, 240-13, ((int)360/timePixelFactor)+10, 240-13, WHITE); //X0, Y0, X1, Y1, Color
  //tft.drawLine(3, 237, ((int)300/timePixelFactor)+10, 237, WHITE); //X0, Y0, X1, Y1, Color

	//Vertical lines (ticks) at every 30s
  tft.drawLine(18+(int)30/timePixelFactor, 226, 18+(int)30/timePixelFactor, 222, WHITE);    // 30S
	tft.drawLine(18+(int)60/timePixelFactor, 226, 18+(int)60/timePixelFactor, 220, WHITE);    // 60S
	tft.drawLine(18+(int)90/timePixelFactor, 226, 18+(int)90/timePixelFactor, 222, WHITE);    // 90S
	tft.drawLine(18+(int)120/timePixelFactor, 226, 18+(int)120/timePixelFactor, 220, WHITE);  //120S
	tft.drawLine(18+(int)150/timePixelFactor, 226, 18+(int)150/timePixelFactor, 222, WHITE);  //150S
	tft.drawLine(18+(int)180/timePixelFactor, 226, 18+(int)180/timePixelFactor, 220, WHITE);  //180S
	tft.drawLine(18+(int)210/timePixelFactor, 226, 18+(int)210/timePixelFactor, 222, WHITE);  //210S
	tft.drawLine(18+(int)240/timePixelFactor, 226, 18+(int)240/timePixelFactor, 220, WHITE);  //240S
	tft.drawLine(18+(int)270/timePixelFactor, 226, 18+(int)270/timePixelFactor, 222, WHITE);  //270S	
  tft.drawLine(18+(int)300/timePixelFactor, 226, 18+(int)300/timePixelFactor, 220, WHITE);  //300S
  tft.drawLine(18+(int)330/timePixelFactor, 226, 18+(int)330/timePixelFactor, 222, WHITE);  //330S	
  //tft.drawLine(18+(int)360/timePixelFactor, 226, 18+(int)360/timePixelFactor, 220, WHITE);  //360S
  
  // tick values
  //tft.drawString("30", ((int)30/timePixelFactor)-5, 230, 1);
  tft.drawString("60", (18+(int)60/timePixelFactor)-5, 230, 1);
  //tft.drawString("90", ((int)90/timePixelFactor)-5, 230, 1);
  tft.drawString("120", (18+(int)120/timePixelFactor)-8, 230, 1);
  //tft.drawString("150", ((int)150/timePixelFactor)-8, 230, 1);
  tft.drawString("180", (18+(int)180/timePixelFactor)-8, 230, 1);
  //tft.drawString("210", ((int)210/timePixelFactor)-8, 230, 1);
  tft.drawString("240", (18+(int)240/timePixelFactor)-8, 230, 1);
  //tft.drawString("270", ((int)270/timePixelFactor)-8, 230, 1);
  tft.drawString("300", (18+(int)300/timePixelFactor)-8, 230, 1);
  //tft.drawString("330", ((int)330/timePixelFactor)-8, 230, 1);
  //tft.drawString("360", (18+(int)360/timePixelFactor)-8, 230, 1);

}


void drawCurve_old()
{
	//This function draws the user-made reflow curve.
	//Since the axes are slightly shifted from the edge of the display, there is a 3px shift for the start of the preheat curve
	//I indicated different sections of the reflow curve by different colors.
	//The cooling time +20 is just an arbitrary value, just to illustrate the cooling part (decreasing temperature) on the reflow curve.
	//It has no physical meaning other than it illustrates the cooling.

	tft.setTextSize(1); //Reset the size to 1 in case the code is coming from someplace else

  drawAxis();

  // Draw the curve
  // starting center of x-y lines

  tft.drawLine(18, 240 - 4-10, preheatTime_px, preheatTemp_px, YELLOW);
	tft.drawLine(preheatTime_px, preheatTemp_px, soakingTime_px, soakingTemp_px, ORANGE);
	tft.drawLine(soakingTime_px, soakingTemp_px, reflowTime_px, reflowTemp_px, RED);
	tft.drawLine(reflowTime_px, reflowTemp_px, coolingTime_px, coolingTemp_px, RED);
	tft.drawLine(coolingTime_px, coolingTemp_px, coolingTime_px + 40, coolingTemp_px + 20, BLUE);
  
}

// =================================================================================================

// Function to interpolate between two points using cosine interpolation
float cosineInterpolate(float y1, float y2, float mu) {
  float mu2 = (1 - cos(mu * PI)) / 2;
  return (y1 * (1 - mu2) + y2 * mu2);
}

void drawSmoothCurve(int x0, int y0, int x1, int y1, uint16_t color) {
  int steps = 100; // Number of interpolation steps
  for (int i = 0; i < steps; i++) {
      float mu = (float)i / (float)(steps - 1);
      int x = x0 + (x1 - x0) * mu;
      int y = cosineInterpolate(y0, y1, mu);
      tft.drawPixel(x, y, color);
  }
}

void drawCurve() {
  // This function draws the user-made reflow curve.
  // Since the axes are slightly shifted from the edge of the display, there is a 3px shift for the start of the preheat curve
  // I indicated different sections of the reflow curve by different colors.
  // The cooling time +20 is just an arbitrary value, just to illustrate the cooling part (decreasing temperature) on the reflow curve.
  // It has no physical meaning other than it illustrates the cooling.

  tft.setTextSize(1); // Reset the size to 1 in case the code is coming from someplace else

  drawAxis();

  // Draw the curve using smooth interpolation
  drawSmoothCurve(18, 240 - 14, preheatTime_px, preheatTemp_px, YELLOW);
  drawSmoothCurve(preheatTime_px, preheatTemp_px, soakingTime_px, soakingTemp_px, ORANGE);
  drawSmoothCurve(soakingTime_px, soakingTemp_px, reflowTime_px, reflowTemp_px, RED);
  drawSmoothCurve(reflowTime_px, reflowTemp_px, coolingTime_px, coolingTemp_px, RED);
  drawSmoothCurve(coolingTime_px, coolingTemp_px, coolingTime_px + 40, coolingTemp_px + 20, BLUE);
}


//=================================================================================================

void printTargetTemperature() //Momentary target temperature which is derived from the elapsed time
{
	tft.fillRoundRect(30, 60, 80, 16, RectRadius, DGREEN); 
  tft.setTextColor(WHITE);
  tft.drawString("Targt "+String(int(targetTemp))+"`C", 32, 60, 2); // can only print "°C" with font 2

}


void updateReflowState(double currentTemp, double targetTemperature, const char* statusText)
{	
	if (currentTemp < targetTemperature) //Check if the measured temperature is below the target
	{
		updateStatus(DGREEN, statusText); //print reflow phase in green box to indicate approach
	}else{ //measured temperature is ABOVE the target
		updateStatus(ORANGE, statusText); //Print reflow phase in red box to indicate overshoot
	}
  if (statusText == "Cooling")
  {
    updateStatus(BLUE, statusText); //Blue box to show the free cooling phase
  }
    if (statusText == "Heating")
  {
    updateStatus(RED, statusText); //Blue box to show the free heating phase
  }
}


void updateStatus(uint16_t color, const char* text)
{
  tft.fillRoundRect(160, 190, 70, 18, RectRadius, color); //Erase the previously printed text
	tft.setTextColor(WHITE);
  tft.drawString(text, 170, 190, 2);
}


void controlSSR(double targetTemp, double currentTemp, unsigned long currentTime, unsigned long period)
{	
  if (currentTemp < targetTemp)
  {
    digitalWrite(SSR_pin, HIGH);
    if (reflow == true)
      tft.fillCircle(237, 7, 6, RED); // show SSR is on in reflow mode
  }else{
    digitalWrite(SSR_pin, LOW);
    if (reflow == true)
      tft.fillCircle(237, 7, 6, GREEN); // show SSR is off in reflow mode
  }
}