//    rDUINOScope - Arduino based telescope control system (GOTO).
//    Copyright (C) 2016 Dessislav Gouzgounov (Desso)
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    any later version.
//
//    PROJECT Website: http://rduinoscope.byethost24.com
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//    ALLIGNMENT Method: 1 Star Alignment - The method I have implemented is part of Mr. Ralph Pass alignment procedure described on http://rppass.com/
//                       Mr. Pass helped rDUINOScope by providing the calculations needed to implement the method. http://rppass.com/align.pdf - the actual PDF
//
//                       Iterative alignment - The method is based on article from "New" Hamilton Astronomy.com website: http://astro.hcadvantage.com
//                       Actual PDF document: http://astro.hcadvantage.com/ASTRO_ARTICLES/Polar_Alignment_Part_II.pdf
//
//

/*    
 *     Major differences from main release:
 *     
 *     -  Uses ILI9488 480x320 px display in SPI, instead of HX8352B 400x240 px display, so bigger screen and less pin used.
 *     -  Small code optimization to maximize speed (the ILI9488 supports only 24bit/px so it's quite slow as screen compared to HX... ones)
 *     -  Joypad calibration at startup. No more panic to find right values for the joypad.
 *     -  Empirial March sound function moved to confirm good initialization of the device. If no sound then you're having problems! :'(
 *     -  Automatic data and time set on clockScreen() derived from GPS data: the software is able to calculate the location's time zone and to auto update to summer or winter time.
 *     -  Using AD7873 touch IC. Also compatibile with TSC2046/XPT2046 and ADS7843/AD7843 ICs. The software is able to detect and display any communications error with the touch IC.
 *     -  Custom line of text can be appended to any .csv file to display custom "Description" strings. Example: I added "rich cluster with more than 100 stars" to M11 in messier.csv
 *        color for Name and Description when tracking objects.
 *     -  Different pin arrangement
 *     -  Added custom.csv wich can be used to add more custom sky objects (still to be implemented)
*/

#define reverse_logic true //set true if the stepper drivers logic is "neglected enabled"
#define serial_debug    // comment out to deactivate the serial debug mode

// HERE GOES THE Mount, Gears and Drive information.
// ... used to calculate the HourAngle to microSteps ratio
// UPDATE THIS PART according to your SET-UP
// ---------------------------------------------
// NB: RA and DEC uses the same gear ratio (144 tooth in my case)!
//----------------------------------------------
#ifdef serial_debug
  int WORM = 144;
#else
  int WORM = 144;
#endif
int REDUCTOR = 4;      // 1:4 gear reduction
int DRIVE_STP = 200;   // Stepper drive have 200 steps per revolution
int MICROSteps = 16;   // I'll use 1/16 microsteps mode to drive sidereal - also determines the LOWEST speed.

// below variables are used to calculate the paramters where the drive works
int ARCSEC_F_ROTAT = 1296000;    // ArcSeconds in a Full earth rotation;
float SIDEREAL_DAY = 86164.0905;   // Sidereal day in seconds
float ArcSECstep;
int MicroSteps_360;
int RA_90;  // How much in microSteps the RA motor have to turn in order to make 6h = 90 degrees;
int DEC_90;   // How mich in microSteps the DEC motor have to turn in order to make 6h = 90 degrees;
int HA_H_CONST;
int HA_M_CONST;
int DEC_D_CONST;
int DEC_M_CONST;
int MIN_TO_MERIDIAN_FLIP = 2;   // This constant tells the system when to do the Meridian Flip. "= 2" means 2 minutes before 24:00h (e.g. 23:58h)
int MIN_SOUND_BEFORE_FLIP = 3;   // This constant tells the system to start Sound before it makes Meridian Flip
float mer_flp;                   // The calculateLST_HA() function depending on this timer will convert the HA and DEC to the propper ones to do the flip.
boolean MERIDIAN_FLIP_DO = false;
int Tracking_type = 1;  // 1: Sidereal, 2: Solar, 0: Lunar;
int Clock_Sidereal;  // Variable for the Interruptions. nterruption is initialized depending on the DATA above -in miliseconds
int Clock_Solar;  // Variable for the Interruptions. nterruption is initialized depending on the DATA above -in miliseconds
int Clock_Lunar;  // Variable for the Interruptions. nterruption is initialized depending on the DATA above -in miliseconds

////////////////////////////////////////////////
#include "DHT.h"
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <ILI9488.h>  //Use DMA version to increase speed
#include <DueTimer.h> // interruptions library0
#include <DS3231.h>
#include <math.h>

#include "defines.h"  //notes, colors and stars

////////////////////////////////////////////////
/** ILI9488 pin map */
#define TFT_CS  47
#define TFT_DC  48
#define TFT_RST 46

ILI9488 tft(TFT_CS, TFT_DC, TFT_RST);

/** ADS7873 pin map */
#define TP_CS  4
#define TP_IRQ 2

XPT2046_Touchscreen myTouch(TP_CS, TP_IRQ);
TS_Point p;

/** DHT22 pin map */
#define DHTPIN 3
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

/** U-BLOX NEO GPS pin map */
TinyGPSPlus gps;

/** DS3231 RTC pin map */
DS3231 rtc(20, 21);           // (SDA, SCL) from the RTC board

/** SD CARD pin map */
int SD_CS = 42;

/** NIGHT MODE ANALOG pin */
#define DAY_NIGHT_PIN A6

const String FirmwareDate   = "03 08 17";
const String FirmwareNumber = "v2.3.1 Boiana EQ - giacu92";
const String FirmwareName   = "rDUINOScope";
const String FirmwareTime   = "03:14:15";

// Variables:
String Messier_Array[112];
String Treasure_Array[130];
String custom_Array[100];
String ObservedObjects[50];
String Iter_Stars[50];
int int_star_count = 0;       // Counter for how many stars are loaded into Iter_Stars[] array.... to be used with Pages, so that it does not show more pages than needed
//String Stars[] = {};
int Observed_Obj_Count = 0;
int ALLIGN_STEP = 0;  // Using this variable to count the allignment steps - 1: Synchronize, 2: Allign and Center, 3:....
int ALLIGN_TYPE = 0;  // Variable to store the alignment type (0-Skip Alignment, 1-1 Star alignment, 2-2 Star alignment
float delta_a_RA = 0;
float delta_a_DEC = 0;
int Iterative_Star_Index = 0;
String Prev_Obj_Start;
int lastScreenUpdateTimer;
unsigned long Slew_timer, Slew_RA_timer = 0;
int OBJECT_Index;
String OBJECT_NAME;
String OBJECT_DESCR;
String OBJECT_DETAILS;
String BT_COMMAND_STR;
String START_TIME;
int STP_FWD = LOW;
int STP_BACK = HIGH;
float OBJECT_RA_H;
float OBJECT_RA_M;
float OBJECT_DEC_D;
float OBJECT_DEC_M;
float OBJECT_MAG;
float curr_RA_H, curr_RA_M, curr_RA_S, curr_DEC_D, curr_DEC_M, curr_DEC_S;    // Global variables to store Mount's current RA and DEC.
char curr_RA_lz[9], curr_DEC_lz[10], curr_HA_lz[9];                                                // Global variable to store current RA and DEC with Leading Zeroes and sign (RA: 00:00:00; DEC: +/-00*00:00)
int SELECTED_STAR = 0;
double DELTA_RA_ADJUST = 1; // cos RA
double DELTA_DEC_ADJUST = 1; // cos DEC
// Default values to load when CANCEL button is hit on the GPS screen
float OBSERVATION_LONGITUDE = 14.1497; // (23.3333* - Home)
float OBSERVATION_LATTITUDE = 42.3631; // (42.6378* - Home)
float OBSERVATION_ALTITUDE = 130.00; // Chieti, Italy
int TIME_ZONE = 1;
// .............................................................
int GPS_iterrations = 0;
double LST, HAHour, HAMin, ALT, AZ;
double JD;
String BTs;

int last_button, MESS_PAGER, TREAS_PAGER, STARS_PAGER, CUSTOM_PAGER;
boolean IS_TFT_ON = true;
boolean IS_STEPPERS_ON = true;
boolean IS_OBJ_VISIBLE = false;
boolean IS_IN_OPERATION = false;  // This variable becomes True when Main screen appears
boolean IS_TRACKING = false;
boolean IS_NIGHTMODE;
boolean IS_OBJ_FOUND = true;
boolean IS_OBJECT_RA_FOUND = true;
boolean IS_OBJECT_DEC_FOUND = true;
boolean IS_MERIDIAN_PASSED = false;
boolean IS_POSIBLE_MERIDIAN_FLIP = true;
boolean IS_MERIDIAN_FLIP_AUTOMATIC = true;
boolean IS_BT_MODE_ON = false;
boolean IS_MANUAL_MOVE = false;
boolean IS_FAN1_ON = true;
boolean IS_FAN2_ON = true;
boolean IS_CUSTOM_MAP_SELECTED = false;
boolean IS_SOUND_ON = true;
int TFT_Brightness = 230;
int MAIN_SCREEN_MENU = 0;
int CURRENT_SCREEN = 0;
int LOAD_SELECTOR;   // selector to show which LOADING mechanism is used: 1 - Messier, 2 - File, 3 - NGCs
boolean TRACKING_MOON;
boolean sun_confirm = false;

String Fan1_State = "ON";
String Fan2_State = "ON";
String TFT_Time = "AL-ON";
String Sound_State = "ON";
String Stepper_State = "ON";
String Mer_Flip_State = "Auto";
String Tracking_Mode = "Celest";


int RA_microSteps, DEC_microSteps, rev_RA_microSteps, rev_DEC_microSteps;              // Current position of the motors in MicroSteps! - when movement occures, values are changed accordingly (manual, tracking or slew to);
int RA_mode_steps, DEC_mode_steps;
int SLEW_RA_microsteps, SLEW_DEC_microsteps;    // Where the mottors needs to go in order to point to the object
int RA_finish_last = 0;
int map_r = 0;    // Used to determine the StarMap Row ... image name (1-1.bmp; 1-2.bmp ....)
int map_c = 0;    // Ued to determine the StarMap Column .... image name


String old_t, old_d;
String Start_date;
int update_time, Tupdate_time, TFT_timeout;
unsigned long UPD_T, UPD_coord, DELAY_Slew, UPD_LST, TFT_Timer;
int RA_move_ending;
int w_DateTime[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // array to store date - as updated from updater screen - Wishing_Date
int dateEntryPos = 0;
int Summer_Time = 0;
int xPosition = 0;  // Joystick
int yPosition = 0;  // Joystick
float _temp, _Stemp;
float _humid, _Shumid;
int16_t texts, Button_State_ON, Button_State_OFF, Button_Title, l_text, d_text, btn_l_border, btn_d_border, btn_l_text, btn_d_text, btn_l_selection, title_bg, title_texts, messie_btn, file_btn, ngc_btn, MsgBox_bg, MsgBox_t; // defines string constants for the clor - Depending on the DAY/NIGHT modes
File roots;
File StarMaps;                    // bmp files

// Some variables used for Alignment procedure:
double Star_1_HA = 0;
double Star_1_DEC = 0;
double Star_2_HA = 0;
double Star_2_DEC = 0;
double err_RA = 0;
double err_DEC = 0;
double Eerr_RA = 0;
double Eerr_DEC = 0;
double err_ALT = 0;
double err_AZ = 0;
double det = 0;

// PIN selection
int speakerOut = DAC1;
int RA_STP = 8;
int RA_DIR = 5;
int DEC_STP = 6;
int DEC_DIR = 7;

// New version of the HW 1.4_c was with changed pins for RA_MODE2 and RA_MODE1
// I needed to switch them in the code!
// int RA_MODE1 = 13;
// int RA_MODE2 = 12;

int RA_MODE0 = 11;
int RA_MODE1 = 13;
int RA_MODE2 = 12;
int DEC_MODE0 = 10;
int DEC_MODE1 = 9;
int DEC_MODE2 = 8;

int yPin = A0;
int xPin = A1;
int FAN1 = 37;
int FAN2 = 39;
int TFTBright = DAC0;
int Joy_SW = A11;
int POWER_DRV8825 = A8;
int x_cal, y_cal = 0;
int calx = 8, caly = 11.59;


void setup(void)
{
  #ifdef serial_debug
    Serial.begin(57600);
    while(!Serial) {}
  #endif
  Serial2.begin(9600); // Initialize GPS communication on PINs: 17 (RX) and 16 (TX)
  Serial3.begin(9600); // Bluetooth communication on PINs:  15 (RX) and 14 (TX)
  pinMode(speakerOut, OUTPUT);

  // below variables are used to calculate the paramters where the drive works
  int ww = WORM * REDUCTOR;
  int www = DRIVE_STP * MICROSteps;

  MicroSteps_360 = ww * www;
  RA_90 = MicroSteps_360 / 4;  // How much in microSteps the RA motor have to turn in order to make 6h = 90 degrees;
  DEC_90 = RA_90;   // How mich in microSteps the DEC motor have to turn in order to make 6h = 90 degrees;
  HA_H_CONST = MicroSteps_360 / 360;
  DEC_D_CONST = HA_H_CONST;

  Clock_Sidereal = 1000000 / (MicroSteps_360 / SIDEREAL_DAY); // This way I make the interruption occuer 2wice faster than needed - REASON: allow max time for Pin-UP, Pin-DOWN action
  Clock_Solar = 1000000 / (MicroSteps_360 / (SIDEREAL_DAY - 235.9095));
  Clock_Lunar = 1000000 / (MicroSteps_360 / (SIDEREAL_DAY - 2089.2292));

  bool touch_init = myTouch.begin();  //ADS7873 begin();
  rtc.begin();
  dht.begin();
  tft.begin();

  tft.fillScreen2(BLACK);
  tft.setCursor(15, 10);

  // DRB8825 - drive mode pins (determine Steppping Modes 1/8, 1/16 and etc.
  pinMode(RA_MODE0, OUTPUT);
  pinMode(RA_MODE1, OUTPUT);
  pinMode(RA_MODE2, OUTPUT);

  pinMode(DEC_MODE0, OUTPUT);
  pinMode(DEC_MODE1, OUTPUT);
  pinMode(DEC_MODE2, OUTPUT);

  pinMode(RA_STP, OUTPUT); // RA Step
  pinMode(RA_DIR, OUTPUT); // RA Dir
  pinMode(DEC_STP, OUTPUT); // DEC Step
  pinMode(DEC_DIR, OUTPUT); // DEC Dir

  //digitalWrite(RA_DIR,HIGH); // Set Dir high
  //digitalWrite(RA_STP,LOW);
  //digitalWrite(DEC_DIR,HIGH); // Set Dir high
  //digitalWrite(DEC_STP,LOW);

  // Joystick
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);

  // Device 1 & 2 command Pins
  pinMode(FAN1, OUTPUT);
  pinMode(FAN2, OUTPUT);

  // Pin to define Power to the DRV8825 using MOSFET!
  pinMode(POWER_DRV8825, OUTPUT);

  // Set RA and DEC microstep position
  RA_microSteps = RA_90; //  --> point to North Sudereal Pole = -180 deg (-12h)
  DEC_microSteps = 0; //  --> Point to North Sudereal Pole = 90 deg

  Timer3.attachInterrupt(Sidereal_rate);
  //  Timer3.start(Clock_Sidereal); // executes the code every 62.329 ms.

  if (analogRead(DAY_NIGHT_PIN) > 800)
  {
    // Night Mode
    IS_NIGHTMODE = true;
    texts = Maroon;
    l_text = RED;
    d_text = Maroon;
    title_bg = RED;
    title_texts = BLACK;
    messie_btn = Maroon;
    btn_l_text = RED;
    btn_d_text = Maroon;
    btn_l_border = RED;
    btn_d_border = Maroon;
    btn_l_selection = RED;
    MsgBox_bg = RED;
    MsgBox_t = BLACK;
    Button_State_ON = BLACK;
    Button_State_OFF = BLACK;
    Button_Title = BLACK;
  }
  else
  {
    IS_NIGHTMODE = false;
    texts = LightGrey;
    l_text = WHITE;
    d_text = LightGrey;
    btn_l_text = GreenYellow;
    btn_d_text = DarkGreen;
    btn_l_border = GREEN;
    btn_d_border = DarkGreen;
    btn_l_selection = DarkGreen;
    title_bg = Orange;
    title_texts = BLACK;
    messie_btn = DarkGrey;
    MsgBox_bg = Purple;
    MsgBox_t = GreenYellow;
    Button_State_ON = DarkGreen;
    Button_State_OFF = BLACK;
    Button_Title = BLACK;
  }


  // Draw initial screen - INITIALIZE
  // The below part cannot be removed form the code
  // You can add messages, but not remove!
  tft.setTextColor(title_bg);
  tft.setTextSize(4);
  tft.println("rDUINO Scope");
  tft.setTextColor(l_text);
  tft.setTextSize(2);
  tft.setCursor(50, 45);
  tft.setTextColor(l_text);
  tft.print("coded by <dEskoG>");
  tft.setCursor(40, 70);
  tft.print("Dessislav Gouzgounov");
  tft.setCursor(4, 90);
  tft.setTextSize(2);
  tft.print("rduinoscope.byethost24.com");
  tft.setCursor(6, 130);
  tft.setTextColor(d_text);
  tft.print("GNU General Public License");
  tft.setCursor(10, 160);
  tft.setTextColor(d_text);
  tft.print("Version: " + FirmwareNumber);

  #ifdef serial_debug
    tft.setTextColor(title_bg);
    tft.println(" - DEBUG MODE");
    tft.setTextColor(d_text);
  #endif
  
  tft.setCursor(0, 220);
  tft.setTextSize(1);

  // see if the card is present and can be initialized:
  char in_char;
  String items = "";
  int j = 0;
  int k = 0;
  MESS_PAGER = 0;
  TREAS_PAGER = 0;
  STARS_PAGER = 0;
  CUSTOM_PAGER = 0;

  volatile bool check = true;
  tft.print("--> Initializing touchscreen... ");
  if (!touch_init)
  {
    tft.setTextColor(RED);
    tft.println("FAIL");
    tft.setTextColor(d_text);
    check = false;
  }
  else
  {
    tft.setTextColor(GREEN);
    tft.println("OK");
    tft.setTextColor(d_text);
  }
  
  tft.print("--> Initializing DHT sensor... ");
  if(isnan(dht.readTemperature()) || isnan(dht.readHumidity()))
  {
    tft.setTextColor(RED);
    tft.println("FAIL");
    tft.setTextColor(d_text);
  }
  else
  {
    tft.setTextColor(GREEN);
    tft.println("OK");
    tft.setTextColor(d_text);
  }

  tft.print("--> Initializing RTC... ");
  int prev_mil = millis(); 
  if(isnan(rtc.getTemp()))
  {
    tft.setTextColor(RED);
    tft.println("FAIL");
    tft.setTextColor(d_text);
    check = false;
  }
  else
  {
    tft.setTextColor(GREEN);
    tft.print("OK   ");
    #ifdef serial_debug
      tft.println(rtc.getTemp());
    #else
      tft.println("");  
    #endif
    tft.setTextColor(d_text);
  }
  
  tft.print("--> Initializing SD card... ");
  for (int i=0; i<10 && !SD.begin(SD_CS); i++)
  {
    if (i == 9)
    {
      tft.setTextColor(RED);
      tft.println("ERROR: Card failed, or not present\n");
      tft.println("Try formatting the SD card to FAT32 and replace the files.");
      tft.setTextColor(d_text);
      check = false;
    }
    delay(50);
  }
  //Debug or card initialized:
  tft.setTextColor(GREEN);
  tft.println("OK");
  tft.setTextColor(d_text);
  //loadOptions_SD();
  //delay(100);
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("messier.csv");
  // if the file is available, write to it:
  if (dataFile)
  {
    tft.print("--> loading Messier cathalog... ");
    delay(100);
    while (dataFile.available())
    {
      in_char = dataFile.read();
      items += in_char;
      k = k + 1;
      if (in_char == '\n') {
        Messier_Array[j] = items;
        j = j + 1;
        //          Serial.print(items);
        items = "";
      }
    }
    tft.setTextColor(GREEN);
    tft.println("OK");
    tft.setTextColor(d_text);
    delay(100);

  }
  else
  {
    tft.setTextColor(RED);
    tft.println("ERROR opening: messier.csv");
    tft.setTextColor(d_text);
  }

  dataFile.close();
  items = "";
  j = 0;
  k = 0;
  dataFile = SD.open("treasure.csv");
  // if the file is available, write to it:
  if (dataFile)
  {
    tft.print("--> loading Treasure cathalog... ");
    delay(100);
    while (dataFile.available())
    {
      in_char = dataFile.read();
      items += in_char;
      k = k + 1;
      if (in_char == '\n') {
        Treasure_Array[j] = items;
        j = j + 1;
        //          Serial.print(items);
        items = "";
      }
    }
    tft.setTextColor(GREEN);
    tft.println("OK");
    tft.setTextColor(d_text);
    delay(100);
  }
  else
  {
    tft.setTextColor(RED);
    tft.println("ERROR opening: treasure.csv");
    tft.setTextColor(d_text);
  }
  dataFile.close();
  last_button = 0;
  LOAD_SELECTOR = 0;

  items = "";
  j = 0;
  k = 0;
  dataFile = SD.open("custom.csv");


  // if the file is available, write to it:
  if (dataFile)
  {
    tft.print("--> loading custom.csv... ");
    delay(100);
    while (dataFile.available())
    {
      in_char = dataFile.read();
      items += in_char;
      k = k + 1;
      if (in_char == '\n')
      {
        custom_Array[j] = items;
        j = j + 1;
        //          Serial.print(items);
        items = "";
      }
    }
    tft.setTextColor(GREEN);
    tft.println("OK");
    tft.setTextColor(d_text);
    delay(100);
  }
  else
  {
    tft.setTextColor(RED);
    tft.println("ERROR opening: custom.csv");
    tft.setTextColor(d_text);
  }
  dataFile.close();
  last_button = 0;
  LOAD_SELECTOR = 0;

  tft.println("\n.................................\n");
  
  tft.print("--> loading options...");
  if(SD.exists("options.txt"))
  {
    loadOptions_SD();
    tft.setTextColor(GREEN);
    tft.println("OK");
    tft.setTextColor(d_text);
  }
  else
  {
    tft.setTextColor(RED);
    tft.println("FAIL");
    tft.setTextColor(d_text);
  }
  tft.println("--> initializing BlueTooth");
  delay(100);
  tft.println("--> initializing GPS");

  #ifndef serial_debug
    if (check == false)  while(1);  //don't do anything more
  #endif

  calibrateJoypad(&x_cal, &y_cal);

  // Draw Supporters Logos
  tft.setCursor(0, 380);
  tft.setTextColor(l_text);
  tft.println("SUPPORTERS:");
  
  char logo_n[50];
  String logo_name = "hackad24.bmp";
  logo_name.toCharArray(logo_n,50);
  bmpDraw(logo_n, 100, 390);
  delay(200);

  logo_name = "logogfe.bmp";
  logo_name.toCharArray(logo_n,50);
  bmpDraw(logo_n, 140, 420);
  delay(200);

  // EMPIRIAL MARCH - if sounds everything was initialized well   :)
  if (IS_SOUND_ON)
  {
    SoundOn(note_f, 48);
    delay(100);
    SoundOn(note_f, 48);
    delay(100);
    SoundOn(note_f, 48);
    delay(100);
    SoundOn(note_cb, 32);
    delay(140);
    SoundOn(note_gb, 8);
    delay(50);
    SoundOn(note_f, 48);
    delay(100);
    SoundOn(note_cb, 32);
    delay(140);
    SoundOn(note_gb, 8);
    delay(50);
    SoundOn(note_f, 48);
  }

  delay(500);
  CURRENT_SCREEN = 0;
  drawGPSScreen();
  UPD_T = millis();
  UPD_LST = millis();
  DELAY_Slew = millis();
  TFT_Timer = millis();
  //TFT_timeout = 0;
  RA_move_ending = 0;
  considerTempUpdates();

  digitalWrite(POWER_DRV8825, HIGH == !reverse_logic); // Switch on the Motor Driver Power!
}

void loop(void)
{
  // This is done in order to prevent multiple calculations of LST_HA per second (especially while SlewTo) and only
  // do it once the DEC SlewTo slows down, but before stopping OR once every 10 seconds (in order to do the Meridian Flip)
  if (RA_move_ending == 1) {
    calculateLST_HA();
  }

  if ((IS_MERIDIAN_FLIP_AUTOMATIC) && ((UPD_LST + 10000) <= millis()) && (IS_OBJ_FOUND == true)) {
    calculateLST_HA();
    UPD_LST = millis();
  }

  // Adding this delay to SLOW DOWN the Arduino so that the motors can catch up!
  // The delay is only needed when in full speed.... otherways the CalculateLST_HA() takes over and
  // slows down the arduino enought. CalculateLST_HA() when slewing only fires when the motors slows down
  // after they are very close to the Object Position.
  if ((DELAY_Slew + 1 <= millis()) && (IS_OBJ_FOUND == false)) {

    // If you wonder how I get to this delay - 800 uS
    // When I optimised the code for speed, the main delay was coming from calculateLST_HA() which back then was calculated on every Loop();
    // Once I optimized it to only calculate when the SlewTo stops (to fine tune after DEC stops) it turned out that
    // the code is too fast and the motors only "screemed" but not rotating - due to the low voltage/current.
    // This variable depends on How You Limit the Current to your motors and the Voltage you use!
    // I use 12V and 1.6A (70% in full step = 1.10A) to drive my NEMA 17 SY42STH47-1684B Motors.
    // Please note that Potentiometer does not really give consistent results for current on every restart (it drifted between 1.12A - 0.9A).

    // HINT: you can try to play with the Current/Voltage that powers the motors to get faster speeds.
    if (IS_STEPPERS_ON) {
      cosiderSlewTo();
    } else {
      IS_OBJECT_RA_FOUND = true;
      IS_OBJECT_DEC_FOUND = true;
      IS_OBJ_FOUND = true;
      RA_move_ending = 0;
    }
    DELAY_Slew = millis();
    // delayMicroseconds(800);
  }

  // The below part of the code makes sure that the system does NOT process any other inputs while SlweingTo!
  // Since both motors need every STEP to come from Arduino board, it needs it's entire power to run the motors in fastest possible way
  // The fastes possible from this board in the current state of the software is approx 3 turns/sec (600 steps/sec)
  // IS_OBJ_FOUND == true --> Means that SLEW command have completed
  //
  if (IS_OBJ_FOUND == true)
  {
    // BLUETOOTH Considerations ? ... if any
    if ((IS_BT_MODE_ON == true) && (Serial3.available() > 0) && (IS_MANUAL_MOVE == false))
    {
      BT_COMMAND_STR = Serial3.readStringUntil('#');
      #ifdef serial_debug
        Serial.println(BT_COMMAND_STR);
      #endif
      considerBTCommands();
    }

    // JOYSTICK Movements ? ... if any
    xPosition = analogRead(xPin);
    yPosition = analogRead(yPin);

    if ((xPosition < x_cal-100) || (xPosition > x_cal+100) || (yPosition < y_cal-100) || (yPosition > x_cal+100))
    {
      IS_MANUAL_MOVE = true;
      if (IS_STEPPERS_ON)
      {
        consider_Manual_Move(xPosition, yPosition);
      }
    }
    else
    {
      IS_MANUAL_MOVE = false;
    }

    // This will take care of turning OFF the TFT's background light if the device is not used
    // for XXX amont of seconds and IS_IN_OPERATION = TRUE
    if ((TFT_timeout > 0) && (millis() - TFT_Timer > TFT_timeout) && (IS_TFT_ON) && (IS_IN_OPERATION))
    {
      analogWrite(TFTBright, 0);
      IS_TFT_ON = false;
    }

    // TOUCH EVENTS:
    int tx = 0;
    int ty = 0;
    if (myTouch.touched())
    {
      p = myTouch.getPoint();
      while (p.z < 600)
      {
        p = myTouch.getPoint(); //to remove noise
        delay(100);
      }

      tx = (p.x - 257) / calx;
      ty = (p.y - 445) / caly;
      
      delay(100); // slow down touch identification

      //Useful to debug touch:
      #ifdef serial_debug
        Serial.print(" -> Touched: ");
        Serial.print(tx);
        Serial.print(", y = ");
        Serial.println(ty);
      #endif
      
    }
    considerTouchInput(ty, tx);

    // OTHER UPDATES ?  ... if any
    // Happens every 2 seconds
    if (((millis() - UPD_T) > 2000) && (IS_MANUAL_MOVE == false))
    {
      calculateLST_HA();  // Make sure it Updates the LST! used on Main Screen and When Calculating current Coords.
      considerTimeUpdates();
      considerDayNightMode();
      considerTempUpdates();
      // I need to make sure the Drives are not moved to track the stars,
      // if Object is below horizon ALT < 0 - Stop tracking.
      if ((ALT <= 0) && (IS_TRACKING == true) && (IS_IN_OPERATION == true))
      {
        IS_TRACKING = false;
        Timer3.stop();
        drawMainScreen();
      }
      UPD_T = millis();
    }
  }
}
