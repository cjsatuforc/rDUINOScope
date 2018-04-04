# rDUINOScope

### ILI9488 SPI version:

-  Uses ILI9488 480x320 px display in SPI, instead of HX8352B 400x240 px display, so bigger screen and less pin used. You can buy a breakout board with display, touch controller, SD card and RTC with battery [here](http://giacu92.bigcartel.com/product/ili9488-3-5-display-spi-breakout-board).
-  Using AD7873 touch IC. Also compatibile with TSC2046/XPT2046 and ADS7843/AD7843 ICs. The software is able to detect and display any communications error with the touch IC.
-  Small code optimization to maximize speed (the ILI9488 supports only 24bit/px so it's quite slow as screen compared to HX... ones).
-  Joypad calibration at startup. No more panic to find the right values for the joypad.
-  Empirial March sound function moved to confirm good initialization of the device. If no sound then you're having problems! :'(
-  Automatic data and time set on clockScreen() derived from GPS data: the software is able to calculate the location's time zone and to auto update from summer or winter time.
-  Custom line of text can be appended to any .csv file to display custom "Description" strings. Example: I added "rich cluster with more than 100 stars" to M11 in messier.csv.
-  Changed color for Name and Description when tracking objects.
-  Different pin arrangement.
-  Options page settings such as tft brightness or sound state, are now saved and restored at each boot.
-  Added custom.csv wich can be used to add more custom sky objects (still to be implemented).

![Board1](http://i68.tinypic.com/2qbdsuc.png)
![Board2](http://i63.tinypic.com/sgsgmd.png)

## THE WORLD'S FIRST STAND ALONE ARDUINO BASED TELESCOPE CONTROL GOTO (handheld controller). 

rDUINOScope is an Open Source, Arduino Due based Telescope Control System (GOTO). Drafted as stand alone system, rDUINOScope does not need PC, Tablet or Cell Phone, nor Internet connection in order to operate and deliver stunning views! It does however supports Bluetooth and LX200 protocol to connect and be controlled by smart devices!


rDUINOScope uses database with THE BEST ~250 stellar objects ( Messier and Hidden Treasures Catalogues) and 200 stars, calculates their position on the sky and points the telescope. Implemented Alignment procedures allow for "Always in the eyepiece" experience when slewing to selected objects and once the object is in the eyepiece, the system keeps tracking it.

The best part of rDUINOScope is that it is an OPEN SOURCE SOFTWARE and HARDWARE! Few custom telescope makers have announced that they are adding or about to add it to their product line. In addition, few months ago a new HW shield board was born as a spin-off project to spare you the hassle of building your own rDUINOScope!

Published under GNU General Public License

Instructions on how to build hardware can be found on the project's website: http://rduinoscope.byethost24.com/
HACKADAY.IO Project: https://hackaday.io/project/21024-rduinoscope

Facebook: https://www.facebook.com/rDUINOScope/


# rDUINOScope _2.3.1_Boiana_EQ_Giacomo
===========================================================
Release date: 03 August 2017
Status: Stable Version
Author: Dessislav Gouzgounov / Десислав Гузгнов /(deskog@gmail.com) 
Edited: Giacomo Mammarella (mammarella.g@gmail.com)

v2.3.1 aims to bring all existing hardware platforms into one code base. Additionally this version introduces new features to the rDUINOScope like Screensaver, TFT brightness, MosFET to control power to DRV8825 and etc.

Current release will also be used as a base to add the Alt/Az changes.

New features added in v2.3.1:
===========================================================
- Brings all existing flavours of the hardware to use same code!
- Hardware Changes!!! - Plese reffer to the HACKADAY instructions!
- Screen Brightness control;
- Screen Auto OFF function with timeout;
- Implemented FET transistor to control power to the stepper motors;
- More BlueTooth commands added to prepare for a screenless version (or operation);
- It is now possible to control rDUINOScope without the TFT screen - so called "BLIND" version.
- BUG fixes:
	- GPS fixes;
	- SlewTo speed issues for low amperage;



New features added in v2.3:
===========================================================
- Code Optimization for Performance (SlewTo now capable of speeds > 3 deg/sec);
- Added Celestial, Lunar and Solar tracking speeds;
- Added new Main Menu system for more functionality;
- Added control on 2 external devices (DEV 1 & 2 - you can hook your Heater, Fan and etc);
- Added support for SkySafari 5 (smartphone software);
- Added build in Star Atlas showing current telescope position;
- Shows current Firmware version on Loading screen and on Bluetooth request using LX200 protocol;
- Bug fixes:
	o Wrong DEC movement for negative DEC values passed over Bluetooth;
	o LST revised in the format XX:XX;




New features added in v2.1:
===========================================================
- Added “1 Star Alignment” Method;
- Added “Iterative Alignment” Method;
- Added 200 brightest stars to be used in alignment procedures;
- Added Bluetooth control from Stellarium (cloned LX200 communication protocols);
- Added Bluetooth connection with PC;
- Code has been refactored for easy reading/editing;
- Bug fixes:
	o Wrong DEC movement for negative DEC values;
	o Revised RA movement formula;
	o OnScreen messages have been fixed;
	o 1px. Buttons move after press event was fixed;


