/*
  ST7565R.h - Library for ST7565R LCD - Graphical LCD Display
  Created by Analog Devices Inc. - Circuits from the Lab, May 2015.
*/
#ifndef ST7565R_h
#define ST7565R_h

#include "Arduino.h"

//EVAL-ADXL362-ARDZ
#define ST7565R_SS      8	// value of the CS pin assignment
#define ST7565R_A0      3	// value of the A0 pin assignment
//#define ST7565R_RST     10	// value of the RST pin assignment
//#define ST7565R_BL      2	// value of the Backlight pin assignment

// WiFi demo pins ONLY!!
#define ST7565R_RST     6	// value of the RST pin assignment
#define ST7565R_BL      4	// value of the Backlight pin assignment


#define LCD_COLUMNS     128u
#define LCD_PAGES       4u
#define LCD_LINES       64u

//#define UP_X            112
//#define LEFT_X          104
//#define RIGHT_X         120
//#define DOWN_X          112
//#define CENTER_X        112

#define ACC_LIMIT       50


class ST7565Rclass
{
	public:
		void ST7565R_SPI_Configuration(void);
		void LcdINIT(void);
		void OutString(int ui8row, int ui8col, int *pi8str);
		void OutSymbol(int ui8row, int ui8col, int ui8width, const int *pui8symbol);
		void LcdFillPages(int ui8start, int ui8num, int ui8Data);
		void LcdSetLine(int ui8line);
		void writeCommand(int uicommand);
		void writeData(int uidata);
		void setLcdCursor(int uipage, int uicolumn);
	private:
	
};

extern ST7565Rclass ST7565R;

#endif
