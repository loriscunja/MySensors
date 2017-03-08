/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifdef ARDUINO_ARCH_XI 

#include "MyHwLG8F.h"
#include <PMU.h>
#define INVALID_INTERRUPT_NUM	(0xFFu)

volatile uint8_t _wokeUpByInterrupt =
    INVALID_INTERRUPT_NUM;    // Interrupt number that woke the mcu.
volatile uint8_t _wakeUp1Interrupt  =
    INVALID_INTERRUPT_NUM;    // Interrupt number for wakeUp1-callback.
volatile uint8_t _wakeUp2Interrupt  =
    INVALID_INTERRUPT_NUM;    // Interrupt number for wakeUp2-callback.

void wakeUp1()	 //place to send the interrupts
{
	detachInterrupt(_wakeUp1Interrupt);
	if (_wakeUp2Interrupt != INVALID_INTERRUPT_NUM) {
		detachInterrupt(_wakeUp2Interrupt);
	}
	_wokeUpByInterrupt = _wakeUp1Interrupt;
}
void wakeUp2()	 //place to send the second interrupts
{
	detachInterrupt(_wakeUp2Interrupt);
	if (_wakeUp1Interrupt != INVALID_INTERRUPT_NUM) {
		detachInterrupt(_wakeUp1Interrupt);
	}
	_wokeUpByInterrupt = _wakeUp2Interrupt;
}

bool interruptWakeUp()
{
	return _wokeUpByInterrupt != INVALID_INTERRUPT_NUM;
}

// Watchdog Timer interrupt service routine. This routine is required
// to allow automatic WDIF and WDIE bit clearance in hardware.
/*
ISR (WDT_vect)
{
}
*/
void hwPowerDown(period_t period)
{
	PMU.sleep(PM_POFFS0, period);  
}

void hwInternalSleep(unsigned long ms)
{
	// Let serial prints finish (debug, log etc)
#ifndef MY_DISABLED_SERIAL
	MY_SERIALDEVICE.flush();
#endif
	while (!interruptWakeUp() && ms >= 32000) {
		hwPowerDown(SLEEP_32S);
		ms -= 8000;
	}
    if (!interruptWakeUp() && ms >= 16000)    {
		hwPowerDown(SLEEP_16S);
		ms -= 4000;
	}
    if (!interruptWakeUp() && ms >= 8000)    {
		hwPowerDown(SLEEP_8S);
		ms -= 4000;
	}
	if (!interruptWakeUp() && ms >= 4000)    {
		hwPowerDown(SLEEP_4S);
		ms -= 4000;
	}
	if (!interruptWakeUp() && ms >= 2000)    {
		hwPowerDown(SLEEP_2S);
		ms -= 2000;
	}
	if (!interruptWakeUp() && ms >= 1000)    {
		hwPowerDown(SLEEP_1S);
		ms -= 1000;
	}
	if (!interruptWakeUp() && ms >= 512)     {
		hwPowerDown(SLEEP_512MS);
		ms -= 512;
	}
	if (!interruptWakeUp() && ms >= 256)     {
		hwPowerDown(SLEEP_256MS);
		ms -= 256;
	}
	if (!interruptWakeUp() && ms >= 128)     {
		hwPowerDown(SLEEP_128MS);
		ms -= 128;
	}
	if (!interruptWakeUp() && ms >= 64)      {
		hwPowerDown(SLEEP_64MS );
		ms -= 64;
	}	
}

int8_t hwSleep(unsigned long ms)
{
	hwInternalSleep(ms);
	return MY_WAKE_UP_BY_TIMER;
}

int8_t hwSleep(uint8_t interrupt, uint8_t mode, unsigned long ms)
{
	return hwSleep(interrupt,mode,INVALID_INTERRUPT_NUM,0u,ms);
}

int8_t hwSleep(uint8_t interrupt1, uint8_t mode1, uint8_t interrupt2, uint8_t mode2,
               unsigned long ms)
{
	// Disable interrupts until going to sleep, otherwise interrupts occurring between attachInterrupt()
	// and sleep might cause the ATMega to not wakeup from sleep as interrupt has already be handled!
	cli();
	// attach interrupts
	_wakeUp1Interrupt  = interrupt1;
	_wakeUp2Interrupt  = interrupt2;
	if (interrupt1 != INVALID_INTERRUPT_NUM) {
		attachInterrupt(interrupt1, wakeUp1, mode1);
	}
	if (interrupt2 != INVALID_INTERRUPT_NUM) {
		attachInterrupt(interrupt2, wakeUp2, mode2);
	}

	if (ms>0) {
		// sleep for defined time
		hwInternalSleep(ms);
	} else {
		// sleep until ext interrupt triggered
		hwPowerDown(SLEEP_FOREVER);
	}

	// Assure any interrupts attached, will get detached when they did not occur.
	if (interrupt1 != INVALID_INTERRUPT_NUM) {
		detachInterrupt(interrupt1);
	}
	if (interrupt2 != INVALID_INTERRUPT_NUM) {
		detachInterrupt(interrupt2);
	}

	// Return what woke the mcu.
	int8_t ret = MY_WAKE_UP_BY_TIMER;       // default: no interrupt triggered, timer wake up
	if (interruptWakeUp()) {
		ret = static_cast<int8_t>(_wokeUpByInterrupt);
	}
	// Clear woke-up-by-interrupt flag, so next sleeps won't return immediately.
	_wokeUpByInterrupt = INVALID_INTERRUPT_NUM;

	return ret;
}

#if defined(MY_DEBUG) || defined(MY_SPECIAL_DEBUG)
uint16_t hwCPUVoltage()
{

	return (1125300UL) / 1;
}

uint16_t hwCPUFrequency()
{
	// return frequency in 1/10MHz (accuracy +- 10%)
	return 2048UL / 100000UL;
}

uint16_t hwFreeMem()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif

#ifdef MY_DEBUG
void hwDebugPrint(const char *fmt, ... )
{
	char fmtBuffer[MY_SERIAL_OUTPUT_SIZE];
	
#ifdef MY_GATEWAY_FEATURE
	// prepend debug message to be handled correctly by controller (C_INTERNAL, I_LOG_MESSAGE)
	snprintf_P(fmtBuffer, sizeof(fmtBuffer), PSTR("0;255;%d;0;%d;"), C_INTERNAL, I_LOG_MESSAGE);
	MY_SERIALDEVICE.print(fmtBuffer);
#else
	// prepend timestamp (AVR nodes)
	//MY_SERIALDEVICE.print(hwMillis());
	MY_SERIALDEVICE.print(".");
#endif
	va_list args;
	va_start (args, fmt );
#ifdef MY_GATEWAY_FEATURE
	// Truncate message if this is gateway node
	vsnprintf_P(fmtBuffer, sizeof(fmtBuffer), fmt, args);
	fmtBuffer[sizeof(fmtBuffer) - 2] = '\n';
	fmtBuffer[sizeof(fmtBuffer) - 1] = '\0';
#else
	vsnprintf_P(fmtBuffer, sizeof(fmtBuffer), fmt, args);
#endif
	va_end (args);
	MY_SERIALDEVICE.print(fmtBuffer);
	MY_SERIALDEVICE.flush();

	//MY_SERIALDEVICE.write(freeRam());
}
#endif

#endif // #ifdef ARDUINO_ARCH_XI 
