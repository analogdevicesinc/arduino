#include "SPI.h"
#include "Communication.h"
#include "adxl372.h"

struct adxl372_device adxl372;
unsigned char devId;
AccelTriplet_t accel_data;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0); //CPHA = CPOL = 0    MODE = 0
  delay(1000);

  pinMode(CS_PIN, OUTPUT);

  adxl372_Get_DevID(&adxl372, &devId);

  Serial.print("Device id: ");
  Serial.println(devId, HEX);

  adxl372_Set_Op_mode(&adxl372, FULL_BW_MEASUREMENT);
}

void loop() {
  
  adxl372_Get_Accel_data(&adxl372, &accel_data);

  Serial.print("X accel = "); Serial.print(accel_data.x);
  Serial.print(" Y accel = "); Serial.print(accel_data.y);
  Serial.print(" Z accel = "); Serial.println(accel_data.x);

  delay(500);

}
