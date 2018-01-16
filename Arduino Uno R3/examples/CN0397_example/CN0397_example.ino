#include <SPI.h>
#include "CN0397.h"
#include <Arduino.h>

byte CS_PIN = 10;

void setup() {

  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); //CPHA = CPOL = 1    MODE = 3
  delay(1000);

  pinMode(CS_PIN, OUTPUT);

  CN0397_Init();
}

void loop() {
  
         delay(DISPLAY_REFRESH);

         CN0397_SetAppData();

         CN0397_DisplayData();

}
