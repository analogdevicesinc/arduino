/*!
 *******************************************************************************
 * @file:    Timer.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *------------------------------------------------------------------------------
 *
Copyright (c) 2014 Liviu Ionescu.

Portions Copyright (c) 2017 Analog Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

#include "CN0411.h"
#include "Timer.h"
#include <Arduino.h>

// -----------------------------------------------------------------------------

void
timer_start (void)
{
	cli(); /* stop interrupts */

	/* set timer0 interrupt at 2kHz */
	TCCR2A = 0; /* set entire TCCR0A register to 0 */
	TCCR2B = 0; /* same for TCCR0B */
	TCNT2  = 0; /* initialize counter value to 0 */
	/* set compare match register for 2khz increments */
	OCR2A = 20; /* = (16*10^6) / (2000*64) - 1 (must be <256) */
	/* turn on CTC mode */
	TCCR2A |= (1 << WGM01);
	/* Set CS01 and CS00 bits for 64 prescaler */
	TCCR2B |= (1 << CS01) | (0 << CS00);
	/* enable timer compare interrupt */
	TIMSK2 |= (1 << OCIE0A);

	sei(); /* start interrupts */
}

void
timer_sleep (timer_ticks_t ticks)
{
	delay(ticks);
}

void timer_sleep_5uS(timer_ticks_t ticks)
{
	timer_ticks_t i;

	if(ticks > (ARDUINO_MAX_MICROSECONDS_DELAY / 5))
		return;

	for(i = 0; i < ticks; i++) {
		CN0411_pwm_gen();
		delayMicroseconds(5);
	}
}

// ----- Timer2 Interrupt Handler ----------------------------------------------

ISR(TIMER2_COMPA_vect)
{
	CN0411_pwm_gen();
}

