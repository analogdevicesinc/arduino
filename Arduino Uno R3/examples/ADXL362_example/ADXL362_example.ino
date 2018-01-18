#include "ADXL362.h"
#include "Lcd.h"
#include "Communication.h"
#include <SPI.h>

byte ADXL_CS_PIN = 10;
byte LCD_CS_PIN = 11;

uint8_t ui8s[22];
uint8_t ui8xu;
uint8_t ui8xd;
uint8_t ui8yu;
uint8_t ui8yd;
uint8_t ui8all;
uint8_t ui8awake;

char temp[10]; 

#if TEMP_ADC == 0
   float f32temp;
#endif


void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); //CPHA = CPOL = 1    MODE = 3
  delay(1000);

  /* Initialize LCD */
   Lcd_Init();

   /* Initialize accelerometer */
   Sensor_Init();

   /* Start accelerometer measurement mode */
   Sensor_Start();

}

void loop() {
if (digitalRead(INTACC_PIN)) {
         if (ui8awake == 0) {
            ui8awake = 1;

            /* Set BLLCD pin - turn on LCD backlight */
            digitalWrite(BLLCD_PIN,HIGH);

            Lcd_DisplayString(0, 60, (int8_t *)"[mG]");
            Lcd_DisplayString(1, 60, (int8_t *)"[mG]");
            Lcd_DisplayString(2, 60, (int8_t *)"[mG]");

#if TEMP_ADC == 1
            Lcd_DisplayString(3, 60, (int8_t *)"[ADC]");
#else
            Lcd_DisplayString(3, 60, (int8_t *)" [C]");
#endif
            ui8xu = 0;
            ui8xd = 0;
            ui8yu = 0;
            ui8yd = 0;
            ui8all = 0;

            Lcd_DisplaySymbol(0, UP_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(1, LEFT_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(1, RIGHT_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(2, DOWN_X, 8, pui8RecInv8x8);
            Lcd_DisplaySymbol(1, CENTER_X, 8, pui8RecInv8x8);
         }

      } else {
         if (ui8awake == 1) {
            ui8awake = 0;

            /* Clear BLLCD pin - turn off LCD backlight */
            digitalWrite(BLLCD_PIN,LOW);

            /* Clear screen */
            Lcd_FillPages(0, 4, 0x00);
         }
      }

      if (ui8awake == 1) {
         Sensor_Scan();
        
         sprintf((char *)ui8s, "x = % 5d", i16SensorX);
         Lcd_DisplayString(0, 0, (int8_t *)ui8s);

         sprintf((char *)ui8s, "y = % 5d", i16SensorY);
         Lcd_DisplayString(1, 0, (int8_t *)ui8s);

         sprintf((char *)ui8s, "z = % 5d", i16SensorZ);
         Lcd_DisplayString(2, 0, (int8_t *)ui8s);

#if TEMP_ADC == 1
         sprintf((char *)ui8s, "t = % 5d", i16SensorT);
         Lcd_DisplayString(3, 0, (int8_t *)ui8s);
#else
         f32temp = ((float)i16SensorT + ACC_TEMP_BIAS) / (1 / ACC_TEMP_SENSITIVITY);
         Lcd_DisplayString(3, 60, (int8_t *)" [C]");
         dtostrf(f32temp, 2, 2, temp);  
         sprintf((char *)ui8s, "t = %s  ", temp);
         Lcd_DisplayString(3, 0, (int8_t *)ui8s); 
#endif

         if (i16SensorY > ACC_LIMIT) {
            if (ui8xu == 0) {
               ui8xu = 1;
               Lcd_DisplaySymbol(0, UP_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8xu == 1) {
               ui8xu = 0;
               Lcd_DisplaySymbol(0, UP_X, 8, pui8RecInv8x8);
            }
         }

         if (i16SensorY < -ACC_LIMIT) {
            if (ui8xd == 0) {
               ui8xd = 1;
               Lcd_DisplaySymbol(2, DOWN_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8xd == 1) {
               ui8xd = 0;
               Lcd_DisplaySymbol(2, DOWN_X, 8, pui8RecInv8x8);
            }
         }

         if (i16SensorX > ACC_LIMIT) {
            if (ui8yu == 0) {
               ui8yu = 1;
               Lcd_DisplaySymbol(1, RIGHT_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8yu == 1) {
               ui8yu = 0;
               Lcd_DisplaySymbol(1, RIGHT_X, 8, pui8RecInv8x8);
            }
         }

         if (i16SensorX < -ACC_LIMIT) {
            if (ui8yd == 0) {
               ui8yd = 1;
               Lcd_DisplaySymbol(1, LEFT_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8yd == 1) {
               ui8yd = 0;
               Lcd_DisplaySymbol(1, LEFT_X, 8, pui8RecInv8x8);
            }
         }

         if ((ui8xu == 0) && (ui8xd == 0) && (ui8yu == 0) && (ui8yd == 0)) {
            if (ui8all == 0) {
               ui8all = 1;
               Lcd_DisplaySymbol(1, CENTER_X, 8, pui8Rec8x8);
            }

         } else {
            if (ui8all == 1) {
               ui8all = 0;
               Lcd_DisplaySymbol(1, CENTER_X, 8, pui8RecInv8x8);
            }
         }
      }
}
