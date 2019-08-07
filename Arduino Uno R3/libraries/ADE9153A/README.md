# ADE9153A Library

[ADE9153A](https://www.analog.com/en/products/ade9153a.html) is the latest single phase energy measurement IC with mSure self-calibration. This library specifically targets the [EV-ADE9153ASHIELDZ](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADE9153A.html) shield evaluation board.

There is example code available in the [examples](https://github.com/analogdevicesinc/arduino/tree/master/Arduino%20Uno%20R3/examples) section.

## Read Before Use
To make the evaluation board easy to use, the library has some registers defined with non-default values.

When using this ADE9153A library be sure to include both ADE9153A.h and ADE9153AAPI.h.

### Defaults
Register definitions are found in the ADE9153AAPI.h. Be sure that these definitions are set up correctly for the applications, especially ADE9153A_ACCMODE which must be setup for 50Hz or 60Hz systems.

### Hardware
The hardware has flexibility to use both 3.3V and 5V with the proper modifications as described in the [EV-ADE9153ASHIELDZ User Guide](https://www.analog.com/media/en/technical-documentation/user-guides/ev-ade9153ashieldz-ug-1253.pdf).

For the simplest layout the IAN and IAP were reversed compared to the test circuit in the [ADE9153A Data Sheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ade9153a.pdf). This means that a negative gain must be applied to AIGAIN, which can be seen in the [example code](https://github.com/analogdevicesinc/arduino/tree/master/Arduino%20Uno%20R3/examples).

## Support
Feelfree to ask any questions on [EngineerZone](https://ez.analog.com/energy-metering)
