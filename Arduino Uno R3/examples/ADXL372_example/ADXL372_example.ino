#include "SPI.h"
#include "Communication.h"
#include "adxl372.h"

struct adxl372_device adxl372;
unsigned char devId;
AccelTriplet_t accel_data;

typedef struct {
  float x;
  float y;
  float z;
}acceleration_G_t;

acceleration_G_t data_G;

void Set_Impact_Detection(void)
{
  adxl372_Set_Op_mode(&adxl372, STAND_BY);
  
  adxl372_Set_Autosleep(&adxl372, false);
  
  adxl372_Set_BandWidth(&adxl372, BW_3200Hz);
  
  adxl372_Set_ODR(&adxl372, ODR_6400Hz);
  
  adxl372_Set_WakeUp_Rate(&adxl372, WUR_52ms);
  
  adxl372_Set_Act_Proc_Mode(&adxl372, LOOPED);
  
  /* Set Instant On threshold */
  adxl372_Set_InstaOn_Thresh(&adxl372, ADXL_INSTAON_LOW_THRESH); //Low threshold 10-15 G
  
  /*Put fifo in Peak Detect and Stream Mode */
  adxl372_Configure_FIFO(&adxl372, 512, STREAMED, XYZ_PEAK_FIFO);
  
  /* Set activity/inactivity threshold */
  adxl372_Set_Activity_Threshold(&adxl372, ACT_VALUE, true, true);
  adxl372_Set_Inactivity_Threshold(&adxl372, INACT_VALUE, true, true);
  
  /* Set activity/inactivity time settings */
  adxl372_Set_Activity_Time(&adxl372, ACT_TIMER);
  adxl372_Set_Inactivity_Time(&adxl372, INACT_TIMER);
  
  /* Set instant-on interrupts and activity interrupts */
   adxl372_Set_Interrupts(&adxl372);
  
  /* Set filter settle time */
  adxl372_Set_Filter_Settle(&adxl372, FILTER_SETTLE_16);
  
  /* Set operation mode to Instant-On */
  adxl372_Set_Op_mode(&adxl372, INSTANT_ON);
}

void setup() {
  Serial.begin(9600);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0); //CPHA = CPOL = 0    MODE = 0
  delay(1000);

  pinMode(CS_PIN, OUTPUT);
  pinMode(ADXL_INT1_PIN, INPUT);

  adxl372_Get_DevID(&adxl372, &devId);

  Serial.print("Device id: ");
  Serial.println(devId, HEX);

  adxl372_Set_Op_mode(&adxl372, FULL_BW_MEASUREMENT);
  Set_Impact_Detection();
}

void loop() {

  if (digitalRead(ADXL_INT1_PIN)) {
    delay(500);
    adxl372_Get_Highest_Peak_Accel_data(&adxl372, &accel_data);
  
    /*Transform in G values*/
    data_G.x = (float)accel_data.x * 100 / 1000;
    data_G.y = (float)accel_data.y * 100 / 1000;
    data_G.z = (float)accel_data.z * 100 / 1000;
  
    Serial.print("X accel = "); Serial.print(data_G.x); Serial.println(" G");
    Serial.print("Y accel = "); Serial.print(data_G.y); Serial.println(" G");
    Serial.print("Z accel = "); Serial.print(data_G.z); Serial.println(" G");
    Serial.println("");
    
  }

}
