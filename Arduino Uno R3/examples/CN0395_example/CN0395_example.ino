#include <SPI.h>
#include <Wire.h>

#include "AD7988.h"
#include "CN0395.h"
#include "ADN8810.h"
#include "SHT30.h"
#include "Communication.h"

static uint8_t count = 0;
sMeasurementVariables *sMeasVar;
extern uint8_t ui8ContinousRsMeasurement;

void setup() {
  Serial.begin(9600);
  /* Initialize SPI */
  SPI.begin();
  SPI.setDataMode(SPI_MODE0); //CPHA = CPOL = 0    MODE = 0

  pinMode(CSAD7988_PIN, OUTPUT);
  pinMode(CSADN8810_PIN, OUTPUT);
  digitalWrite(CSAD7988_PIN, HIGH);
  digitalWrite(CSADN8810_PIN, HIGH);

  /* Initialize I2C */
  Wire.begin();

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  sMeasVar = (sMeasurementVariables *)malloc(sizeof(*sMeasVar));

  AD7988_Init(); // Initialize ADC

  CN0395_PowerOn(sMeasVar);
}

void loop() {
  unsigned short  status;

  if (uart_cmd == UART_TRUE) {  /* condition becomes true when the system receives as Carriage Return(CR) command from the UART */
    if (count == 0) { // Check first <ENTER> is pressed after reset
      Serial.print("\tWelcome to CN0395 application!\n");
      Serial.print("\r\nDefault at power up: \n");
      Serial.print("\r\n* RS operation mode\n");
      Serial.print("\r* IH = 8 mA (Constant current)\n");
      Serial.print("\r* Ro = "); Serial.print(sMeasVar->fSensorResCleanAir); Serial.print(" [Ohms] (Sensor resistance in clean air)\n");
      Serial.print("\r\n>>Type in <help> to see available commands\n");

      count++;
    }
    else { // At a second <ENTER> press, do the processing
      CN0395_CmdProcess(sMeasVar);
    }
    uart_cmd = UART_FALSE;
    CN0395_CmdPrompt();
  }
  if (ui8ContinousRsMeasurement) {

    uint16_t ui16AdcData = 0;
    float    fHeaterVoltage = 0;

    AD7988_SetOperationMode(AD7988_RH_MODE);
    ui16AdcData = CN0395_ReadAdc(sMeasVar);
    fHeaterVoltage = AD7988_DataToVoltage(ui16AdcData);
    delay(50); // delay 50ms

    sMeasVar->fHeaterVoltage = fHeaterVoltage;
    CN0395_ComputeHeaterRPT(sMeasVar); // Compute RH, PH and TH

    CN0395_MeasureSensorResistance(sMeasVar);
    delay(998);
    CN0395_DisplayData(sMeasVar);
  }
}

void serialEvent() {
  char c;

  while (Serial.available() > 0) {
    c = Serial.read();                          /* Read character from UART */
    switch (c) {
      case _BS:
        if (uart_rcnt) {
          uart_rcnt--;
          uart_rx_buffer[uart_rcnt] = 0;
          Serial.write(c);
          Serial.write(' ');
          Serial.write(c);
        }
        break;
      case _CR: /* Check if read character is ENTER */
        uart_cmd = UART_TRUE;                    /* Set flag */
        break;

      default:
        uart_rx_buffer[uart_rcnt++] = c;

        if (uart_rcnt == UART_RX_BUFFER_SIZE) {
          uart_rcnt--;
        }
    }

    uart_rx_buffer[uart_rcnt] = '\0';
  }
}
