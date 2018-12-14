#include <Arduino.h>
#include <SPI.h>

#include "adi_cn0410.h"

char Buffer[12];

uint8_t u8Index;
bool boCommandReceived = false;
uint8_t u8Error;
char u8Channel;
uint16_t u16DacValue, u16ChannelValue;

CN0410 cn0410;

void setup() {

  Serial.begin(9600);

  cn0410.Init(); //init spi and pins

  cn0410.SendCommand(AD5686_ITERNAL_REFERENCE,
                     AD5686_DAC_NONE, 0x00); /*enable internal reference*/
  cn0410.SendCommand(AD5686_POWER,
                     AD5686_DAC_NONE, 0x00); /*normal power mode for all ch*/

  delay(10); //10 ms delay

  cn0410.SendCommand(AD5686_RESET,
                     AD5686_DAC_NONE, 0x00); /*soft reset to zero scale*/

  Serial.print(F("\n\n\rCN0410 Demo Software\n\r"));
  Serial.print(F("Enter command in the following format set_x value,\n\r \
          where x is the channel (a,b,c) and hit enter \n\r \
          To reset channels input command set_zero \n\r> "));
}

void loop() {

  if (boCommandReceived) {
    boCommandReceived = false; /*reset command received*/

    u8Error = ProcessCommand(&u8Channel, &u16DacValue);
    
    if ( u8Error == 0 ) {
      Serial.print("Command received.\n\r");

      switch ((char)u8Channel) {
        case 'a':
          cn0410.SendCommand(AD5686_WRITE_UPDATE,
                             AD5686_DAC_A, u16DacValue);
          u16ChannelValue = cn0410.ReadBack(AD5686_DAC_A);
          Serial.print("Channel A set to: "); Serial.println(u16ChannelValue);
          break;
        case 'b':
          cn0410.SendCommand(AD5686_WRITE_UPDATE,
                             AD5686_DAC_B, u16DacValue);
          u16ChannelValue = cn0410.ReadBack(AD5686_DAC_B);
          Serial.print("Channel B set to: "); Serial.println(u16ChannelValue);
          break;
        case 'c':
          cn0410.SendCommand(AD5686_WRITE_UPDATE,
                             AD5686_DAC_C, u16DacValue);
          u16ChannelValue = cn0410.ReadBack(AD5686_DAC_C);
          Serial.print("Channel C set to: "); Serial.println(u16ChannelValue);
          break;
        case '0':
          cn0410.Reset();
          Serial.print("All channels reset to 0\n\r");
          break;
        default:
          Serial.print("Invalid value\n\r");
          break;
      }
    } else {
      Serial.print("Invalid command\n\r");
    }

    u16ChannelValue = cn0410.ReadBack(AD5686_DAC_A);
    Serial.print("\n\rChannel A set to: "); Serial.println(u16ChannelValue);
    u16ChannelValue = cn0410.ReadBack(AD5686_DAC_B);
    Serial.print("Channel B set to: "); Serial.println(u16ChannelValue);
    u16ChannelValue = cn0410.ReadBack(AD5686_DAC_C);
    Serial.print("Channel C set to: "); Serial.println(u16ChannelValue);

    delay(1000); //1 second sleep
    Serial.print("\n\n\rEnter new command\n\r> ");
  }
}

void serialEvent() {

  while (Serial.available() > 0) {
    Buffer[u8Index] = Serial.read();
    u8Index++;
  }

  if ((Buffer[u8Index - 1] == '\n') | (u8Index == 12)) {
    boCommandReceived = true;
    u8Index = 0;
  }
}

uint8_t ProcessCommand(uint8_t * u8Channel, uint16_t * u16DacValue)
{
  uint16_t u16RawValue;

  if ((((char)Buffer[4] != (char)'a') &  ((char)Buffer[4] != (char)'b') &
       ((char)Buffer[4] != (char)'c') & ((char)Buffer[4] != (char)'z')))
    return -1;

  if ((Buffer[4] != 'z'))
    *u8Channel = (char)Buffer[4]; /*channel letter*/
  else
    *u8Channel = (char)'0';

  u16RawValue = (uint16_t)atoi((char *)&Buffer[6]);
  *u16DacValue = u16RawValue;

  return 0;
}
