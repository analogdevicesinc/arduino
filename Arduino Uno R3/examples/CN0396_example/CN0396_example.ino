#include "SPI.h"
#include "Communication.h"
#include "CN0396.h"

void setup() {
  Serial.begin(9600);
  delay(1000);

  CN0396_Init();

}

void loop() {
  
  delay(DISPLAY_REFRESH);
  
  CN0396_SetAppData();
  
  CN0396_DisplayData();
}
