/*
  AD5270.h 
 */
#ifndef AD5270_h
#define AD5270_h

#include <Arduino.h>

/*****************************************************************************/
/********************************  Definitions *******************************/
/*****************************************************************************/

//Commands
#define WRITE_RDAC				0x1					// Write to the RDAC Register
#define READ_RDAC				0x2					// Read from the RDAC Register
#define STORE_50TP				0x3					// Write to the RDAC to memory
#define SW_RST					0x4					// Software reset to last memory location
#define READ_50TP_CONTENTS		0x5					// Read the last memory contents
#define READ_50TP_ADDRESS		0x6					// Read the last memory address
#define WRITE_CTRL_REG			0x7					// Write to the control Register
#define READ_CTRL_REG			0x8					// Read from the control Register
#define SW_SHUTDOWN				0x9					// Software shutdown (0) - Normal, (1) - Shutdown

#define HI_Z_PREP				0xF0				// Get the SDO line ready for High Z
#define HI_Z					0x0F				// Puts AD5270 into High Z mode

//Pins
#define AD5270_SS				6					// AD5270 SPI chip select

class AD5270class
{
	public:
		void writeAd5270 (uint8_t ui8command, uint16_t ui16value);
		uint16_t readAd5270 (uint8_t ui8address);
		void AD5270_SPI_Configuration(void);
	private:

};

extern AD5270class AD5270;

#endif
