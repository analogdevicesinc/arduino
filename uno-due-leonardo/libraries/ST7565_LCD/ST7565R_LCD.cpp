/*
  ST7565R_LCD.cpp - Library for ST7565R LCD - 
  Created by Analog Devices Inc. - Circuits from the Lab, May 2015.
*/

#include <Arduino.h>
#include <SPI.h>
#include "ST7565R_LCD.h"

/******************************************************************************/
/********************************* Definitions ********************************/
/******************************************************************************/
#define FONT_Y_SIZE  8

// ASCII offset
#define OFFS_ASCII   32

// ST7565R commands
#define CMD_DISPLAY_OFF          0xAE
#define CMD_DISPLAY_ON           0xAF
#define CMD_SET_DISP_START_LINE  0x40
#define CMD_SET_PAGE             0xB0
#define CMD_SET_COLUMN_UPPER     0x10
#define CMD_SET_COLUMN_LOWER     0x00
#define CMD_SET_ADC_NORMAL       0xA0
#define CMD_SET_ADC_REVERSE      0xA1
#define CMD_SET_DISP_NORMAL      0xA6
#define CMD_SET_DISP_REVERSE     0xA7
#define CMD_SET_ALLPTS_NORMAL    0xA4
#define CMD_SET_ALLPTS_ON        0xA5
#define CMD_SET_BIAS_9           0xA2
#define CMD_SET_BIAS_7           0xA3
#define CMD_RMW                  0xE0
#define CMD_RMW_CLEAR            0xEE
#define CMD_INTERNAL_RESET       0xE2
#define CMD_SET_COM_NORMAL       0xC0
#define CMD_SET_COM_REVERSE      0xC8
#define CMD_SET_POWER_CONTROL    0x28
#define CMD_SET_RESISTOR_RATIO   0x20
#define CMD_SET_VOLUME_FIRST     0x81
#define CMD_SET_VOLUME_SECOND    0
#define CMD_SET_STATIC_OFF       0xAC
#define CMD_SET_STATIC_ON        0xAD
#define CMD_SET_STATIC_REG       0x0
#define CMD_SET_BOOSTER_FIRST    0xF8
#define CMD_SET_BOOSTER_234      0
#define CMD_SET_BOOSTER_5        1
#define CMD_SET_BOOSTER_6        3
#define CMD_NOP                  0xE3
#define CMD_TEST                 0xF0

/******************************************************************************/
/****************************** Global Data ***********************************/
/******************************************************************************/
/*const int pui8Rec8x8[8] =
{
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

const int pui8RecInv8x8[8] =
{
      0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF
};

/******************************************************************************/
/******************** Symbol matrix structure: Font (8x14) ********************/
/******************************************************************************/
static const int pui8font5x7[96][5] =
{
      {0x00, 0x00, 0x00, 0x00, 0x00},  //     32  //
      {0x00, 0x00, 0x4F, 0x00, 0x00},  //  !  33  //
      {0x00, 0x07, 0x00, 0x07, 0x00},  //  "  34  //
      {0x14, 0x7F, 0x14, 0x7F, 0x14},  //  #  35  //
      {0x24, 0x2A, 0x7F, 0x2A, 0x12},  //  $  36  //
      {0x23, 0x13, 0x08, 0x64, 0x62},  //  %  37  //
      {0x36, 0x49, 0x55, 0x22, 0x50},  //  &  38  //
      {0x00, 0x05, 0x03, 0x00, 0x00},  //  '  39  //
      {0x00, 0x1C, 0x22, 0x41, 0x00},  //  (  40  //
      {0x00, 0x41, 0x22, 0x1C, 0x00},  //  )  41  //
      {0x14, 0x08, 0x3E, 0x08, 0x14},  //  *  42  //
      {0x08, 0x08, 0x3E, 0x08, 0x08},  //  +  43  //
      {0x00, 0x50, 0x30, 0x00, 0x00},  //  ,  44  //
      {0x08, 0x08, 0x08, 0x08, 0x08},  //  -  45  //
      {0x00, 0x60, 0x60, 0x00, 0x00},  //  .  46  //
      {0x20, 0x10, 0x08, 0x04, 0x02},  //  /  47  //
      {0x3E, 0x51, 0x49, 0x45, 0x3E},  //  0  48  //
      {0x00, 0x42, 0x7F, 0x40, 0x00},  //  1  49  //
      {0x42, 0x61, 0x51, 0x49, 0x46},  //  2  50  //
      {0x21, 0x41, 0x45, 0x4B, 0x31},  //  3  51  //
      {0x18, 0x14, 0x12, 0x7F, 0x10},  //  4  52  //
      {0x27, 0x45, 0x45, 0x45, 0x39},  //  5  53  //
      {0x3C, 0x4A, 0x49, 0x49, 0x30},  //  6  54  //
      {0x01, 0x71, 0x09, 0x05, 0x03},  //  7  55  //
      {0x36, 0x49, 0x49, 0x49, 0x36},  //  8  56  //
      {0x06, 0x49, 0x49, 0x29, 0x1E},  //  9  57  //
      {0x36, 0x36, 0x00, 0x00, 0x00},  //  :  58  //
      {0x56, 0x36, 0x00, 0x00, 0x00},  //  ;  59  //
      {0x08, 0x14, 0x22, 0x41, 0x00},  //  <  60  //
      {0x14, 0x14, 0x14, 0x14, 0x14},  //  =  61  //
      {0x00, 0x41, 0x22, 0x14, 0x08},  //  >  62  //
      {0x02, 0x01, 0x51, 0x09, 0x06},  //  ?  63  //
      {0x30, 0x49, 0x79, 0x41, 0x3E},  //  @  64  //
      {0x7E, 0x11, 0x11, 0x11, 0x7E},  //  A  65  //
      {0x7F, 0x49, 0x49, 0x49, 0x36},  //  B  66  //
      {0x3E, 0x41, 0x41, 0x41, 0x22},  //  C  67  //
      {0x7F, 0x41, 0x41, 0x22, 0x1C},  //  D  68  //
      {0x7F, 0x49, 0x49, 0x49, 0x41},  //  E  69  //
      {0x7F, 0x09, 0x09, 0x09, 0x01},  //  F  70  //
      {0x3E, 0x41, 0x49, 0x49, 0x7A},  //  G  71  //
      {0x7F, 0x08, 0x08, 0x08, 0x7F},  //  H  72  //
      {0x00, 0x41, 0x7F, 0x41, 0x00},  //  I  73  //
      {0x20, 0x40, 0x41, 0x3F, 0x01},  //  J  74  //
      {0x7F, 0x08, 0x14, 0x22, 0x41},  //  K  75  //
      {0x7F, 0x40, 0x40, 0x40, 0x40},  //  L  76  //
      {0x7F, 0x02, 0x0C, 0x02, 0x7F},  //  M  77  //
      {0x7F, 0x04, 0x08, 0x10, 0x7F},  //  N  78  //
      {0x3E, 0x41, 0x41, 0x41, 0x3E},  //  O  79  //
      {0x7F, 0x09, 0x09, 0x09, 0x06},  //  P  80  //
      {0x3E, 0x41, 0x51, 0x21, 0x5E},  //  Q  81  //
      {0x7F, 0x09, 0x19, 0x29, 0x46},  //  R  82  //
      {0x46, 0x49, 0x49, 0x49, 0x31},  //  S  83  //
      {0x01, 0x01, 0x7F, 0x01, 0x01},  //  T  84  //
      {0x3F, 0x40, 0x40, 0x40, 0x3F},  //  U  85  //
      {0x1F, 0x20, 0x40, 0x20, 0x1F},  //  V  86  //
      {0x3F, 0x40, 0x30, 0x40, 0x3F},  //  W  87  //
      {0x63, 0x14, 0x08, 0x14, 0x63},  //  X  88  //
      {0x07, 0x08, 0x70, 0x08, 0x07},  //  Y  89  //
      {0x61, 0x51, 0x49, 0x45, 0x43},  //  Z  90  //
      {0x00, 0x7F, 0x41, 0x41, 0x00},  //  [  91  //
      {0x02, 0x04, 0x08, 0x10, 0x20},  //  \  92  //
      {0x00, 0x41, 0x41, 0x7F, 0x00},  //  ]  93  //
      {0x04, 0x02, 0x01, 0x02, 0x04},  //  ^  94  //
      {0x40, 0x40, 0x40, 0x40, 0x40},  //  _  95  //
      {0x00, 0x01, 0x02, 0x04, 0x00},  //  `  96  //
      {0x20, 0x54, 0x54, 0x54, 0x78},  //  a  97  //
      {0x7F, 0x50, 0x48, 0x48, 0x30},  //  b  98  //
      {0x38, 0x44, 0x44, 0x44, 0x20},  //  c  99  //
      {0x38, 0x44, 0x44, 0x48, 0x7F},  //  d  100 //
      {0x38, 0x54, 0x54, 0x54, 0x18},  //  e  101 //
      {0x08, 0x7E, 0x09, 0x01, 0x02},  //  f  102 //
      {0x0C, 0x52, 0x52, 0x52, 0x3E},  //  g  103 //
      {0x7F, 0x08, 0x04, 0x04, 0x78},  //  h  104 //
      {0x00, 0x44, 0x7D, 0x40, 0x00},  //  i  105 //
      {0x20, 0x40, 0x44, 0x3D, 0x00},  //  j  106 //
      {0x7F, 0x10, 0x28, 0x44, 0x00},  //  k  107 //
      {0x00, 0x41, 0x7F, 0x40, 0x00},  //  l  108 //
      {0x78, 0x04, 0x18, 0x04, 0x78},  //  m  109 //
      {0x7C, 0x08, 0x04, 0x04, 0x78},  //  n  110 //
      {0x38, 0x44, 0x44, 0x44, 0x38},  //  o  111 //
      {0x7C, 0x14, 0x14, 0x14, 0x08},  //  p  112 //
      {0x08, 0x14, 0x14, 0x18, 0x7C},  //  q  113 //
      {0x7C, 0x08, 0x04, 0x04, 0x08},  //  r  114 //
      {0x48, 0x54, 0x54, 0x54, 0x20},  //  s  115 //
      {0x04, 0x3F, 0x44, 0x40, 0x20},  //  t  116 //
      {0x3C, 0x40, 0x40, 0x20, 0x7C},  //  u  117 //
      {0x1C, 0x20, 0x40, 0x20, 0x1C},  //  v  118 //
      {0x3C, 0x40, 0x30, 0x40, 0x3C},  //  w  119 //
      {0x44, 0x28, 0x10, 0x28, 0x44},  //  x  120 //
      {0x0C, 0x50, 0x50, 0x50, 0x3C},  //  y  121 //
      {0x44, 0x64, 0x54, 0x4C, 0x44},  //  z  122 //
      {0x00, 0x08, 0x36, 0x41, 0x00},  //  {  123 //
      {0x00, 0x00, 0x7F, 0x00, 0x00},  //  |  124 //
      {0x00, 0x41, 0x36, 0x08, 0x00},  //  }  125 //
      {0x0C, 0x02, 0x0C, 0x10, 0x0C},  //  ~  126 //
      {0x00, 0x00, 0x00, 0x00, 0x00}   //     127 //
};

ST7565Rclass ST7565R;

void ST7565Rclass::ST7565R_SPI_Configuration(void)
{
	SPI.setBitOrder(MSBFIRST);          		//  MSB to be sent first
	SPI.setDataMode(SPI_MODE3);         		//  Set for clock rising edge, clock idles low
	SPI.setClockDivider(SPI_CLOCK_DIV128);		//  Set clock divider (optional)
	delay(100);
}

void ST7565Rclass::LcdINIT(void)
{
	// Display OFF
	writeCommand(CMD_DISPLAY_OFF);
	// LCD bias select
	writeCommand(CMD_SET_BIAS_7);
	// (8) ADC select - normal
	writeCommand(CMD_SET_ADC_NORMAL);
	// (15) Common output mode select - reverse direction
	writeCommand(CMD_SET_COM_REVERSE);
	// (17) V0 voltage regulator internal resistor ratio set
	writeCommand(CMD_SET_RESISTOR_RATIO | 0x02);
	// (18) Electronic volume mode set
	writeCommand(CMD_SET_VOLUME_FIRST);
	writeCommand(CMD_SET_VOLUME_SECOND | 0x04);
	// (16) Power control set - Booster circuit: ON, Voltage regulator circuit: ON, Voltage follower circuit: ON
	writeCommand(CMD_SET_POWER_CONTROL | 0x07);
   // Clear screen
   LcdFillPages(0, 8, 0x00);	
}


/***************************************************************************//**
 * @brief The function displays a string at the specified position for with 5x7 font size.
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::OutString(int ui8row, int ui8col, int *pi8str)
{
   int ui8x;
   int ui8i;
   int ui8ch;
   int ui8data;

   ui8ch = 0;
   ui8x = ui8col;

   while((pi8str[ui8ch] != 0) && (ui8col < LCD_COLUMNS))
   {
      setLcdCursor(ui8row, ui8x);

      // Symbol matrix column loop

      for (ui8i = 0; ui8i < 5; ui8i++)
      {
         ui8data = pui8font5x7[pi8str[ui8ch] - OFFS_ASCII][ui8i];

         writeData(ui8data);
      }

      // Increase column counter with 6 pixels
      ui8x += 6;

      ui8ch++;
   }

   // Display ON
   writeCommand(CMD_DISPLAY_ON);
}

/***************************************************************************//**
 * @brief Displays a symbol (8 x width) at the specified position on the LCD.
 *
 * @pqrqm ui8row -
 * @pqrqm ui8col -
 * @pqrqm ui8width -
 * @pqrqm pui8symbol -
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::OutSymbol(int ui8row, int ui8col, int ui8width, const int *pui8symbol)
{
   int ui8i;
   int ui8data;

   setLcdCursor(ui8row, ui8col);

   // Symbol matrix column loop
   for (ui8i = 0; ui8i < ui8width; ui8i++)
   {
      ui8data = pui8symbol[ui8i];

      writeData(ui8data);
   }

   // Display ON
   writeCommand(CMD_DISPLAY_ON);
}

/***************************************************************************//**
 * @brief The function fills the selected LCD pages with the data specified.
 *
 * @param ui8start -
 * @param ui8num -
 * @param ui8Data -
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::LcdFillPages(int ui8start, int ui8num, int ui8Data)
{
   int ui8p;
   int ui8c;

   for (ui8p = ui8start; ui8p < (ui8start + ui8num); ui8p++)
   {
      setLcdCursor(ui8p, 0);

      for (ui8c = 0; ui8c < LCD_COLUMNS; ui8c++)
      {
         writeData(ui8Data);
      }
   }

   // Display ON
   writeCommand(CMD_DISPLAY_ON);
}

/***************************************************************************//**
 * @brief The function sets the start line of the LCD.
 *
 * @param ui8line -
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::LcdSetLine(int ui8line)
{
   int ui8Cmd;

   // Set start line
   ui8Cmd = CMD_SET_DISP_START_LINE | (ui8line & 0x3F);
   writeCommand(ui8Cmd);
}

/******************************************************************************/
/*************************** Static Functions *********************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The function writes a command byte to the LCD.
 *
 * @param uicommand - the command byte
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::writeCommand(int uicommand)
{
	digitalWrite(ST7565R_A0,LOW);	
	digitalWrite(ST7565R_SS,LOW);
	SPI.transfer(uicommand);
	digitalWrite(ST7565R_SS,HIGH);
	
}
/***************************************************************************//**
 * @brief The function writes a data byte to the LCD.
 *
 * @param uidata - the data byte
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::writeData(int uidata)
{
	digitalWrite(ST7565R_A0,HIGH);
	digitalWrite(ST7565R_SS,LOW);
	SPI.transfer(uidata);
	digitalWrite(ST7565R_SS,HIGH);
}
/***************************************************************************//**
 * @brief The function sets the cursor position at which data will be written.
 *
 * @param uipage - sets the page number
 * @param uicolumn -  sets the column
 *
 * @return none.
*******************************************************************************/
void ST7565Rclass::setLcdCursor(int uipage, int uicolumn)
{
	int uicursorCommand = 0;
	
	// Set page address
   uicursorCommand = 0xB0 | (uipage & 0x0F);
   writeCommand(uicursorCommand);

   // Set column address LSB CA[3:0]
   uicursorCommand = uicolumn & 0x0F;
   writeCommand(uicursorCommand);

   // Set column address MSB CA[7:4]
   uicursorCommand = 0x10 | (uicolumn >> 4);
   writeCommand(uicursorCommand);
}