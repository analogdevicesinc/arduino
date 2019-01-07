#include <Arduino.h>
#include "ADuCM3029_demo_cn0428_cn0429.h"
#include "adi_m355_gas_sensor.h"
#include <stdio.h>
#include <string.h>
#include "cli.h"
#include "RingBuffer.h"

using namespace adi_sensor_swpack;

#define ADI_CFG_I2C_DEFAULT_ADDR (0x4Cu)

/* ================================================== */
/*   SYSTEM VARIABLES - handles, memory, error flags   */
/* ================================================== */
int       eSensorResult;

/* ================================================== */
/*            VARIABLES           */
/* ================================================== */
const uint8_t CS_Pin [4] = {7, 6, 5, 4};   // Store Chip Select GPIO Pin address
const uint8_t INT_Pin [4] = {2, 3, 8, 9}; // Store Interrupt GPIO Pin address

/* Ringbuffer structure */
struct RingBuf RX, TX;

char       cmdInString[64] = ""; // holds incoming user command
uint8_t       cmdInCnt = 0;     // counts incoming bytes over UART

uint8_t Slave_Rx_Buffer[200] = {0};

M355_GAS      m355_gas_sensor;
M355_GAS     *pGasSensor = &m355_gas_sensor;

uint8_t Slave_Rx_Index = 0;
uint8_t initialcycle = 1;

bool S1_detected = false;
bool S2_detected = false;
bool S3_detected = false;
bool S4_detected = false;

eFSM_State FSM_State = INIT;

uint8_t tempError;
uint8_t numbytes;
uint8_t cmdLen;
uint16_t timeout;
char    tempChar = 0;   // temporary variable for char
bool    exitloop = false;

void flushBuff(uint8_t *buff, uint16_t len) {
  memset(buff, 0, len);
}

/*********************************************************************

    Function:       check_string

    Description:    Function looks for string (string_to_search) with
            length string_len within the ring buffer. If this
            string is found, function returns Success and
            deletes this string from ring buffer. Otherwise
            returns Failure.

*********************************************************************/
int check_string(uint8_t *string_to_search, uint16_t string_len)
{
  uint8_t read_char = 0;
  uint16_t  match_success = 0, loop = 0;

  // Find first character of search string in read buffer
  do
  {
    read_char = GetChar();
    loop++;
  } while ((read_char != string_to_search[0]) && (loop <= MAX_BUFLEN));                  // 35 is the size of the RX buffer, if it's all read out and no match then reset

  if (loop >= MAX_BUFLEN)
  {
    return 0;                                                              // no match
  }
  else                                                                          // Got a match, check if its as expected
  {
    match_success += 1;                                                         // if we got here then there was a match in the first character

    for (loop = 1; loop < string_len; loop++)                                  // Loop through the match string and start at 1 as already found first character
    {
      read_char = GetChar();
      if (read_char == string_to_search[loop])
      {
        match_success += 1;
      }
    }
    if (match_success == string_len)                                           // have all chars matched ?
    {
      DeleteChar(string_len);                       // delete command from the ring buffer
      return 1;
    }
    else
    {
      Empty_Ring();
      return 0;
    }
  }
}

/* UART Callback handler */
void serialEvent()
{
  char c;

  while (Serial.available() > 0) {
    cmdInString[cmdInCnt++] = Serial.read();
    if (cmdInCnt >= 64) {
      cmdInCnt = 0;
      cmdReceived = 1;
      break;
    } else if (cmdInString[cmdInCnt - 1] == _CR)   //check for carriage return
    {
      cmdInString[cmdInCnt - 1] = '\0'; // end of string indicator
      cmdInCnt = 0;
      cmdReceived = 1;
    }

  }
}

/*!
   @brief      Initializes gas sensor and slave I2C address

   @details    Called for each site, so the I2C address is set at run-time based on the site
          the board is plugged into. Slave has a function that sets its I2C address to
          new_sensor_address only if the /SS GPIO is set low. This function sets the
          gpios and then calls that function.
*/
int SensorInitwithI2CAddr(uint8_t new_sensor_address, uint8_t site)
{
  uint8_t i = 0;

  //  Set target site /CS low and all other sites high
  for (i = 0; i < 4; i++) {
    if (site == i + 1) {
      digitalWrite(CS_Pin[i], LOW);       // Set Sensor CS Low if correct site
    }
    else {
      digitalWrite(CS_Pin[i], HIGH);        // Otherwise set Sensor CS High
    }
    delay(50);
  }

  eSensorResult = pGasSensor->SetI2CAddr(new_sensor_address);
  if (eSensorResult == SENSOR_ERROR_NONE)
  {
    Serial.print(F("Gas Sensor in site ")); Serial.print(site);
    Serial.print(F(" initialized successfully with address ")); Serial.println(new_sensor_address, HEX);
  }
  else if (eSensorResult == SENSOR_ERROR_PH) //Used to identify that a water quality probe was found
  {
    Serial.print(F("Water Quality Sensor in site ")); Serial.print(site);
    Serial.print(F(" initialized successfully with address ")); Serial.println(new_sensor_address, HEX);
  }
  else
  {
    Serial.print(F("Sensor in site ")); Serial.print(site); Serial.print(F(" ")); Serial.print((unsigned int) eSensorResult, HEX);
    Serial.print(F(" initialization error. Address ")); Serial.println(new_sensor_address, HEX);
  }

  return eSensorResult;
}

/*!
   @brief      Read sensor config

   @details    Reads sensor config of the sensor with address specified in input parameter
*/
int GetSensorCfg(uint8_t sensor_address)
{
  uint8_t pBuff[8] = {0};
  int16_t sigVal = 0;
  uint16_t unsigVal = 0;
  uint32_t unsiglVal = 0;

  eSensorResult = pGasSensor->openWithAddr(sensor_address);
  if (eSensorResult != SENSOR_ERROR_NONE) return eSensorResult;
  delay(5);

  if (sensor_address == 0x0A) Serial.println(F("Config of sensor on site 1:"));
  if (sensor_address == 0x0B) Serial.println(F("Config of sensor on site 2:"));
  if (sensor_address == 0x0C) Serial.println(F("Config of sensor on site 3:"));
  if (sensor_address == 0x0D) Serial.println(F("Config of sensor on site 4:"));
  delay(5);

  eSensorResult = pGasSensor->ReadSensorBias(&sigVal);
  if (eSensorResult != SENSOR_ERROR_NONE) return eSensorResult;
  Serial.print(F("Vbias = "));
  Serial.print(sigVal, DEC); Serial.println(F(" mV"));
  delay(5);

  eSensorResult = pGasSensor->ReadSensorSensitivity(&unsiglVal);
  if (eSensorResult != SENSOR_ERROR_NONE) return eSensorResult;
  Serial.print(F("Sensitivity = "));
  Serial.print((float)(unsiglVal / 100.0), 2); Serial.println(F(" nA/ppm"));
  delay(5);

  eSensorResult = pGasSensor->ReadMeasurementTime(&unsigVal);
  if (eSensorResult != SENSOR_ERROR_NONE) return eSensorResult;
  Serial.print(F("Measurement Time = "));
  Serial.print(unsigVal, DEC); Serial.println(F(" msec"));
  delay(5);

  delay(5);

  return eSensorResult;
}

/*!
   @brief      Reads averaged ppb value from gas sensor

   @details    Reads sensor data utilizing custom gas sensor I2C library and sends this data over UART.
*/
int32_t SensorReadoutPPB(uint8_t sensor_address)
{
  int32_t PPB_Gas_Reading = 0;
  eSensorResult = pGasSensor->openWithAddr(sensor_address);
  eSensorResult = pGasSensor->ReadDataPPB(&PPB_Gas_Reading);
  delay(10);
  return PPB_Gas_Reading;
}

int32_t ExtractCfgVal()
{
  char *ret;

  ret = strstr(cmdInString, " ");
  ret++;

  return strtol(ret, NULL, 10);
}

uint8_t *int2binString(int32_t a, uint8_t *buff, uint8_t bufSize) {
  buff += (bufSize - 1);

  for (int i = 31; i >= 0; i--) {
    *buff-- = (a & 1) + '0';

    a >>= 1;
  }

  return buff;
}

double Ieee754ConvertToDouble(uint8_t s[32])
{
  double f;
  int16_t sign, exponent;
  uint32_t mantissa;
  int16_t i;

  sign = s[0] - '0';

  exponent = 0;
  for (i = 1; i <= 8; i++)
    exponent = exponent * 2 + (s[i] - '0');

  exponent -= 127;

  if (exponent > -127)
  {
    mantissa = 1;
    exponent -= 23;
  }
  else
  {
    mantissa = 0;
    exponent = -126;
    exponent -= 23;
  }

  for (i = 9; i <= 31; i++)
    mantissa = mantissa * 2 + (s[i] - '0');

  f = mantissa;

  while (exponent > 0)
    f *= 2, exponent--;

  while (exponent < 0)
    f /= 2, exponent++;

  if (sign)
    f = -f;

  return f;
}

/* ================================================== */
/*           State Machine            */
/* ================================================== */
void DataDisplayFSM(void)
{
  uint8_t   readChar = 0; // variable holding address of selected sensor
  uint8_t   cmdResponseBuff[256] = {0}; // buffer holding sensor response to commands
  uint8_t   tempBuff[40] = {0};     // buffer for EIS results parsing
  int16_t   sigVal = 0;   // temporary variable for signed value
  uint16_t  unsigVal = 0; // temporary variable for unsigned value
  uint32_t  unsiglVal = 0;  // temporary variable for long unsigned value

  tempError = SensorInitwithI2CAddr(ADI_CFG_I2C_SENSOR1_ADDR, 1);
  if (tempError == SENSOR_ERROR_NONE) {
    S1_detected = true;
  }
  else if (tempError == SENSOR_ERROR_PH) { //Water Quality Board installed
    S1_detected = true;
    FSM_State = WATER;
  }

  tempError = SensorInitwithI2CAddr(ADI_CFG_I2C_SENSOR2_ADDR, 2);
  if (tempError == SENSOR_ERROR_NONE) {
    S2_detected = true;
  }
  else if (tempError == SENSOR_ERROR_PH) { //Water Quality Board installed
    S2_detected = true;
    FSM_State = WATER;
  }

  tempError = SensorInitwithI2CAddr(ADI_CFG_I2C_SENSOR3_ADDR, 3);
  if (tempError == SENSOR_ERROR_NONE) {
    S3_detected = true;
  }
  else if (tempError == SENSOR_ERROR_PH) { //Water Quality Board installed
    S3_detected = true;
    FSM_State = WATER;
  }

  tempError = SensorInitwithI2CAddr(ADI_CFG_I2C_SENSOR4_ADDR, 4);
  if (tempError == SENSOR_ERROR_NONE) {
    S4_detected = true;
  }
  else if (tempError == SENSOR_ERROR_PH) { //Water Quality Board installed
    S4_detected = true;
    FSM_State = WATER;
  }

  if (FSM_State == WATER)
  {
    if (initialcycle) {
      //Do initial read of water quality board transmit buffer
      if (S1_detected) {
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR1_ADDR);
        if (eSensorResult != SENSOR_ERROR_NONE) {
          Serial.println(F("Error opening sensor 1!"));
        }
      }
      else if (S2_detected) {
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR2_ADDR);
        if (eSensorResult != SENSOR_ERROR_NONE) {
          Serial.println(F("Error opening sensor 2!"));
        }
      }
      else if (S3_detected) {
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR3_ADDR);
        if (eSensorResult != SENSOR_ERROR_NONE) {
          Serial.println(F("Error opening sensor 3!"));
        }
      }
      else if (S4_detected) {
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR4_ADDR);
        if (eSensorResult != SENSOR_ERROR_NONE) {
          Serial.println(F("Error opening sensor 4!"));
        }
      }
      else {
        Serial.println(F("Error: No board found!"));
      }
      numbytes = 0;
      pGasSensor->I2CReadWrite(READ, BYTES_TO_READ, (uint8_t *)&numbytes, 1);//numbytes can be 0 to 255, but there can be more than 255 bytes in slave buffer
      delay(5);
      while (numbytes > 0) {
        pGasSensor->I2CReadWrite(READ, 0x63, &Slave_Rx_Buffer[0], numbytes);
        delay(10);
        for (Slave_Rx_Index = 0; Slave_Rx_Index < numbytes; Slave_Rx_Index++) {
          tempChar = Slave_Rx_Buffer[Slave_Rx_Index];
          Slave_Rx_Buffer[Slave_Rx_Index] = '\0';
          if (tempChar != '~' && tempChar != '\0') {
            Serial.write((const char *)&tempChar);
          }
        }
        pGasSensor->I2CReadWrite(READ, BYTES_TO_READ, (uint8_t *)&numbytes, 1);
        delay(5);
      } //end while data available loop
      initialcycle = 0;
    }
  }
}

void setup() {
  /* Initialize ring buffer */
  Rb_Init();
  Serial.begin(9600);
  Wire.begin();

  for (int i = 0; i < 4; i++)
    pinMode(CS_Pin[i], OUTPUT);          // sets the digital pin 13 as output

  DataDisplayFSM();
}

void loop() {

  if (cmdReceived) {
    //Command Received. Check if we need to switch sensor, otherwise send command to sensor board.
    cmdReceived = 0;
    cmdLen = strlen((const char*)cmdInString);
    cmdInString[cmdLen] = _CR; //add carriage return back in to send to slave
    if (strncmp((const char *)cmdInString, "switchsensor", 11) == 0) {
      tempChar = cmdInString[cmdLen - 1];//character before the enter key
      Serial.print(F("Switching to site "));
      Serial.print((const char *)&tempChar);
      if (tempChar == '1' && S1_detected) {
        pGasSensor->close();
        delayMicroseconds(100);
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR1_ADDR);
      }
      else if (tempChar == '2' && S2_detected) {
        pGasSensor->close();
        delayMicroseconds(100);
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR2_ADDR);
      }
      else if (tempChar == '3' && S3_detected) {
        pGasSensor->close();
        delayMicroseconds(100);
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR3_ADDR);
      }
      else if (tempChar == '4' && S4_detected) {
        pGasSensor->close();
        delayMicroseconds(100);
        eSensorResult = pGasSensor->openWithAddr(ADI_CFG_I2C_SENSOR4_ADDR);
      }
      else {
        Serial.println(F("Invalid Site. Not switched."));
      }
      memset(cmdInString, 0, cmdLen + 1); //reset all array elements to char(0)
    }
    else {
      //Send UART command to I2C, including return key.
      tempError = pGasSensor->I2CReadWrite(WRITE, 0x63, cmdInString, cmdLen + 1);
      if (tempError != SENSOR_ERROR_NONE) {
        Serial.println(F("Error writing to sensor"));
      }
      memset(cmdInString, 0, cmdLen + 1); //reset all array elements to char(0)
      //poll sensor until end of message character is read.
      timeout = 600; //EIS result can take 45s depending on chosen frequencies
      exitloop = false;
      do {        //0.2s loop
        delay(185);
        numbytes = 0;
        pGasSensor->I2CReadWrite(READ, BYTES_TO_READ, (uint8_t *)&numbytes, 1); //numbytes can be 0 to 255, but there can be more than 255 bytes from slave
        delay(5);
        while (numbytes > 0) { //data available
          tempError = pGasSensor->I2CReadWrite(READ, 0x63, &Slave_Rx_Buffer[0], numbytes);
          if (tempError != SENSOR_ERROR_NONE) {
            Serial.println(F("Error reading from sensor"));
          }
          delay(10);
          for (Slave_Rx_Index = 0; Slave_Rx_Index < numbytes; Slave_Rx_Index++) {
            tempChar = Slave_Rx_Buffer[Slave_Rx_Index];
            Slave_Rx_Buffer[Slave_Rx_Index] = '\0'; //clean buffer
            if (tempChar == '~') {
              exitloop = true;
            }
            else { //end message character received
                Serial.print((const char *)&tempChar);
            }
          }
          pGasSensor->I2CReadWrite(READ, BYTES_TO_READ, (uint8_t *)&numbytes, 1);
          delay(5);
        }// end data available while loop
        delayMicroseconds(100);
        timeout--;
      } while (timeout > 0 && !exitloop);
      if (timeout == 0) {
        Serial.println(F("Slave timeout."));
      }
    } //end slave command loop
  } //end command received loop
  delay(100); //wait to check the ring buffer for new characters
}
