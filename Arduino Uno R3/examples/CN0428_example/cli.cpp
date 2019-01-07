/*
   cli.c

        Author: SHunt
*/


/***************************** Include Files **********************************/
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "cli.h"
#include "ADuCM3029_demo_cn0428_cn0429.h"
#include "Ringbuffer.h"

/********************************* Definitions ********************************/



/************************** Variable Definitions ******************************/
/* Available commands */
char *CmdCommands[] = {
  "",
  "help",
  "defaultsensor",
  "sensorsconnected",
  "readconfigs",
  "readtemp",
  "readhum",
  "setmeastime",
  "startmeas",
  "stopmeas",
  "setsensitivity",
  "readsensors",
  "stopread",
  "setupdaterate",
  ""
};

/* Functions for available commands */
cmdFunc CmdFun[] = {
  DoNothing,
  CmdHelp,
  SetDefaultSns,
  SnsConnected,
  RdCfgs,
  RdTemp,
  RdHum,
  CfgMeasTime,
  StartMeas,
  StopMeas,
  CfgSens,
  RdSensors,
  StopRd,
  NULL
};



/*Local Variables */
uint8_t uiDefaultSite = 0;
uint8_t uiDefaultAddress = 0;
uint8_t uiPulseAmpl = 0;
uint8_t uiPulseDur = 0;
char strCmd[20] = "";

uint8_t		cmdReceived = 0;			// flag indicating user command has been received (ENTER pressed)

/****************************** Global functions ******************************/

//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
   @brief Display info for <help> command

   @param args - pointer to the arguments on the command line.

   @return none
**/
void CmdHelp(uint8_t *args)
{
  Serial.println(F("help                           -Print commands"));
  Serial.println(F("defaultsensor <site>           -Set default sensor site"));
  Serial.println(F("                                  <site> = 1, 2, 3, 4"));
  Serial.println(F("sensorsconnected               -Print which sensors are connected"));
  Serial.println(F("readconfigs                    -Read sensor configurations"));
  Serial.println(F("readtemp                       -Read temperature from on-board sensor"));
  Serial.println(F("readhum                        -Read humidity from on-board sensor"));
  Serial.println(F("                                  ADC performs avg of 10 samples at this interval"));
  Serial.println(F("startmeas                      -Start ADC sampling at configured interval"));
  Serial.println(F("stopmeas                       -Stop ADC sampling the sensor"));
  Serial.println(F("setsensitivity <sens>          -Configure sensitivity"));
  Serial.println(F("                                  <sens> = sensitivity in nA/ppm"));
  Serial.println(F("readsensors                    -Read the ppb value of all sensors"));
  Serial.println(F("stopread                       -Stop sensor data reading"));
  Serial.println(F("\r\n"));
}

/**
   @brief Finds the next command line argument

   @param args - pointer to the arguments on the command line.

   @return pointer to the next argument.

**/
uint8_t *FindArgv(uint8_t *args)
{
  uint8_t *p = args;
  int     fl = 0;

  while (*p != 0) {
    if ((*p == _SPC))
      fl = 1;
    else if (fl)
      break;
    p++;
  }

  return p;
}

/**
   @brief Separates a command line argument

    @param dst - pointer to a buffer where the argument will be copied
    @param args - pointer to the current position of the command line .

   @return none

**/
void GetArgv(char *dst, uint8_t *args)
{
  uint8_t  *s = args;
  char     *d = dst;

  while (*s) {
    if (*s == _SPC)
      break;
    *d++ = *s++;
  }

  *d = '\0';
}

/**
   @brief Puts command for FSM to the ring buffer

   @param cmd - pointer to the command

   @return none
**/
void CmdToRB(const char *cmd)
{
  uint8_t cmdLen = strlen(cmd);
  for (int i = 0; i < cmdLen; i++)
  {
    *RingBuf_Putvalue(&RX) = *cmd++;
    RingBuf_Write_Increment(&RX);
  }
  Empty_Ring();
}

/**
   @brief Command line process function

   @return none
**/
void CmdProcess(void)
{
  cmdFunc   func;

  Serial.print(cmdInString);
  /* <ENTER> key was pressed */
  /* Find needed function based on typed command */
  func = FindCommand((char *)cmdInString);

  /* Check if there is a valid command */
  if (func) {
    /* Call the desired function */
    (*func)(&cmdInString[2]);
    cmdInString[0] = '\0';
    /* Check if there is no match for typed command */
  } else if (strlen((const char *)cmdInString) != 0) {
    /* Display a message for unknown command */
    Serial.println(F("Unknown command!"));
  }
  /* Prepare for next <ENTER> */
  // uart_cmd = UART_FALSE;
  CmdPrompt();
}

/**
   @brief Command line prompt. Caller must verify that only the enabled board can run this.

   @return int - UART status: UART_SUCCESS or error status
**/
void CmdPrompt(void)
{
  static uint8_t count = 0;

  /* Check first <ENTER> is pressed after reset */
  if (count == 0) {
    Serial.println(F("Welcome to CN0429!"));
    Serial.println(F("Type <help> to see available commands..."));
    count++;
  }
}

/**
   @brief Find available commands

   @param cmd - command to search

   @return cmdFunc - return the specific function for available command or NULL
					 for invalid command
**/
cmdFunc FindCommand(char *cmd)
{
  cmdFunc func = NULL;
  char *p = cmd;
  int i = 0;
  int f1 = 0;
  int cmdlength = 0;

  while (*p != 0)
  {
    if (*p == _SPC)
      f1 = 1;
    else if (!f1)
      cmdlength++;
    p++;
  }

  while (CmdFun[i] != NULL) {
    if (strncmp(cmd, CmdCommands[i], cmdlength) == 0) {
      func = CmdFun[i];
      break;
    }
    i++;
  }

  return func;
}

void DoNothing()
{
  // nothing to do here
  // this is to avoid help printing each time enter is pressed
}

/**
   @brief Set sensor to use for single sensor commands

   @param args - pointer to the arguments on the command line.
   	   site: 1, 2, 3, or 4

   @return none
**/
void SetDefaultSns(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  /*DEFAULT CONFIGURATION*/
  if (strncmp(arg, "1", 2) == 0)
  {
    uiDefaultSite = 1;
    uiDefaultAddress = ADI_CFG_I2C_SENSOR1_ADDR;
  }
  else if (strncmp(arg, "2", 2) == 0)
  {
    uiDefaultSite = 2;
    uiDefaultAddress = ADI_CFG_I2C_SENSOR2_ADDR;
  }
  else if (strncmp(arg, "3", 2) == 0)
  {
    uiDefaultSite = 3;
    uiDefaultAddress = ADI_CFG_I2C_SENSOR3_ADDR;
  }
  else if (strncmp(arg, "4", 2) == 0)
  {
    uiDefaultSite = 4;
    uiDefaultAddress = ADI_CFG_I2C_SENSOR4_ADDR;
  }
  else {
    Serial.println(F("Invalid site"));
  }
  Serial.print(F("Selected site: ")); Serial.println(uiDefaultSite);
}

/**
   @brief Print which sensors are connected

   @param args - pointer to the arguments on the command line.

   @return none
**/
void SnsConnected(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "PRESENCE_CHECK");
  CmdToRB(strCmd);
}

/**
   @brief Get configuration of sensors

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdCfgs(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_CFGS");
  CmdToRB(strCmd);
}

/**
   @brief Read temperature from the daughter board's on-board T&RH sensor

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdTemp(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_TEMP%c", uiDefaultAddress);
  CmdToRB(strCmd);
}

/**
   @brief Read humidity from the daughter board's on-board T&RH sensor

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdHum(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_HUM%c", uiDefaultAddress);
  CmdToRB(strCmd);
}

/**
   @brief Set measurement time (ms)

   @param args - pointer to the arguments on the command line.
					time: [msec] 50 - 32000

   @return none
**/
void CfgMeasTime(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;
  uint32_t value = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;
  }
  if (digitcount == i) //All characters are numeric
    value = strtol(arg, (char **)NULL, 10); //convert string to number
  else
    Serial.println(F("Invalid entry!"));

  if (value > 32000 || value < 50)
  {
    value = 500;
    Serial.println(F("Out of range! Set to default = 500 ms"));
  }

  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "CFG_MEASTIME%c%d", uiDefaultAddress, (uint16_t)value);
  CmdToRB(strCmd);
}

/**
   @brief Start ADC sampling the sensor

   @param args - pointer to the arguments on the command line.

   @return none
**/
void StartMeas(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "STARTMEAS%c", uiDefaultAddress);
  CmdToRB(strCmd);
}

/**
   @brief Stop ADC sampling the sensor

   @param args - pointer to the arguments on the command line.

   @return none
**/
void StopMeas(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "STOPMEAS%c", uiDefaultAddress);
  CmdToRB(strCmd);
}

/**
   @brief Program the TIA with the requested gain resistor

   @param args - pointer to the arguments on the command line.
					Rtia: from 0 to 26 equals 0R - 512k , see user guide for details

   @return none
**/
void CfgRtia(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;
  uint32_t value = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;
  }
  if (digitcount == i) //All characters are numeric
    value = (uint8_t) strtol(arg, (char **)NULL, 10); //convert string to number
  else
    Serial.println(F("Invalid entry!"));

  if (value > 26 || value < 0)
  {
    value = 1;
    Serial.println(F("Out of range! Set to default = 200 ohm"));
  }

  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "CFG_RTIA%c%d", uiDefaultAddress, (uint16_t)value);
  CmdToRB(strCmd);
}

/**
   @brief Set sensor bias voltage

   @param args - pointer to the arguments on the command line.
					sensor bias: [mV] from -2200 to 2200

   @return none
**/
void CfgVbias(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;
  int32_t value = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  if (arg[0] == '-') digitcount++;
  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;
  }
  if (digitcount == i) //All characters are numeric
    value = (int32_t) strtol(arg, (char **)NULL, 10); //convert string to number
  else
    Serial.println(F("Invalid entry!"));

  if (value > 2200 || value < -2200)
  {
    value = 0;
    Serial.println(F("Out of range! Set to default = 0 mV"));
  }

  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "CFG_VBIAS%c%ld", uiDefaultAddress, (int32_t)value);
  CmdToRB(strCmd);
}

/**
   @brief Configure sensor sensitivity

   @param args - pointer to the arguments on the command line.
					sensitivity: [nA/ppm]

   @return none
**/
void CfgSens(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;
  uint8_t  dotcount = 0;
  float 	 value = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;

    if (arg[i] == '.')
    {
      digitcount++;
      dotcount++;
    }
  }
  if (digitcount == i) //All characters are numeric
    value = strtod(arg, (char **)NULL); //convert string to number
  else
  {
    Serial.println(F("Invalid entry!"));
    return;
  }
  
  if (dotcount > 1)
  {
    Serial.println(F("Invalid entry!"));
    return;
  }

  if (value >= 32768 || value < 0)
  {
    Serial.println(F("Out of range!"));
    return;
  }

  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "CFG_SENS%c%ld", uiDefaultAddress, (uint32_t)(value * 100));
  CmdToRB(strCmd);
}

/**
   @brief Configure temperature compensation of sensor reading

   @param args - pointer to the arguments on the command line.
					temperature compensation: enabled/disabled

   @return none
**/
void CfgTempComp(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  if ((arg[0] == '0' || arg[0] == '1') && arg[1] == '\0')
  {
    memset(strCmd, '\0', sizeof(strCmd));
    sprintf(strCmd, "CFG_TEMPCOMP%c%c", uiDefaultAddress, arg[0]);
    CmdToRB(strCmd);

  }
  else
    Serial.println(F("Invalid entry! 0 or 1 allowed"));
}

/**
   @brief Start electrochemical impedance spectroscopy test

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RunEIS(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "RUN_EIS%c", uiDefaultAddress);
  CmdToRB(strCmd);
}

/**
   @brief Read electrochemical impedance spectroscopy results

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdEIS(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_EIS%c%s", uiDefaultAddress, _EOS);
  CmdToRB(strCmd);
}

/**
   @brief Read electrochemical impedance spectroscopy FULL results

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdEISfull(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_FULL_EIS%c%s", uiDefaultAddress, _EOS);
  CmdToRB(strCmd);
}

/**
   @brief Read the value of the 200R calibration resistor

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdRcal(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_RCAL%c%s", uiDefaultAddress, _EOS);
  CmdToRB(strCmd);
}

/**
   @brief Start chronoamperometry pulse test

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RunPulse(uint8_t *args)
{
  if (uiPulseAmpl != 0 && uiPulseDur != 0)
  {
    memset(strCmd, '\0', sizeof(strCmd));
    sprintf(strCmd, "RUN_PULSE%c%c%c", uiDefaultAddress, uiPulseAmpl, uiPulseDur);
    CmdToRB(strCmd);
  }
  else
  {
    Serial.println(F("Pulse and/or Duration not set!"));
  }
}

/**
   @brief Read results of chronoamperometry pulse test

   @param args - pointer to the arguments on the command line.

   @return none
**/
void ReadPulse(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "PULSE_RESULT%c%c%c", uiDefaultAddress, uiPulseAmpl, uiPulseDur);
  CmdToRB(strCmd);
}

/**
   @brief Set step amplitude for pulse test

   @param args - pointer to the arguments on the command line.
   	   amplitude: [mV] - Amplitude of the pulse in mV (typically 1mV)

   @return none
**/
void SetPulseAmpl(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;
  }
  if (digitcount == i) //All characters are numeric
    uiPulseAmpl = (uint8_t) strtol(arg, (char **)NULL, 10); //convert string to number
  else
  {
    Serial.println(F("Invalid entry!"));
    return;
  }

  if (uiPulseAmpl >= 3 || uiPulseAmpl < 1)
  {
    uiPulseAmpl = 1;
    Serial.println(F("Out of range! Set to default = 1 mV"));
    return;
  }

  memset(strCmd, '\0', sizeof(strCmd));
  Serial.print(F("Pulse amplitude set to "));
  Serial.print(uiPulseAmpl, DEC); Serial.println(F(" mV"));
}

/**
   @brief Set duration for CAPA test

   @param args - pointer to the arguments on the command line.
   	   duration: [milliseconds] - Duration of the pulse (keep <200 ms)

   @return none
**/
void SetPulseDuration(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;
  }
  if (digitcount == i) //All characters are numeric
    uiPulseDur = (uint8_t) strtol(arg, (char **)NULL, 10); //convert string to number
  else
    Serial.println(F("Invalid entry!"));

  if (uiPulseDur > 200 || uiPulseDur < 1)
  {
    uiPulseDur = 50;
    Serial.println(F("Out of range! Set to default = 50 msec"));
  }

  memset(strCmd, '\0', sizeof(strCmd));
  Serial.print(F("Pulse duration set to "));
  Serial.print(uiPulseDur, DEC); Serial.println(F(" msec"));
}

/**
   @brief Select sensor load resistor

   @param args - pointer to the arguments on the command line.
					Rtia: from 0 to 7 equals 0R - 3k6 , see user guide for details

   @return none
**/
void CfgRload(uint8_t *args)
{
  uint8_t  *p = args;
  char     arg[17];
  uint8_t  i;
  uint8_t  digitcount = 0;
  uint32_t value = 0;

  /* Check if this function gets an argument */
  while (*(p = FindArgv(p)) != '\0') {
    /* Save channel parameter */
    GetArgv(arg, p);
  }

  //Check if arg is numeric
  for (i = 0; arg[i] != '\0'; i++)
  {
    // check for decimal digits
    if (isdigit(arg[i]) != 0)
      digitcount++;
  }
  if (digitcount == i) //All characters are numeric
    value = (uint8_t) strtol(arg, (char **)NULL, 10); //convert string to number
  else
    Serial.println(F("Invalid entry!"));

  if (value > 7 || value < 0)
  {
    value = 1;
    Serial.println(F("Out of range! Set to default = 10 ohm"));
  }

  memset(strCmd, '\0', sizeof(strCmd));
  Serial.print(F("CFG_RLOAD"));
  Serial.print(uiDefaultAddress); Serial.println((uint16_t)value, DEC);
  CmdToRB(strCmd);
}

/**
   @brief Read the value of the configured load resistor

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdRload(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READ_RLOAD%c", uiDefaultAddress, _EOS);
  CmdToRB(strCmd);
}

/**
   @brief Read ppb value of all sensors

   @param args - pointer to the arguments on the command line.

   @return none
**/
void RdSensors(uint8_t *args)
{
  memset(strCmd, '\0', sizeof(strCmd));
  sprintf(strCmd, "READALLSENSORS");
  CmdToRB(strCmd);
}

/**
   @brief Stop reading sensor data

   @param args - pointer to the arguments on the command line.

   @return none
**/
void StopRd(uint8_t *args)
{
  FSM_State = COMMAND;
  Serial.println(F("Data reading interrupted!"));
}
