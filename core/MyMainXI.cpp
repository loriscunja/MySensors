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

// Initialize library and handle sketch functions like we want to

//extern "C" void __libc_init_array(void);

#include <stdint.h>
#include <avr/wdt.h>
#include <Arduino.h>

#if defined(__LGT8FX8E__)
void __patch_wdt(void) \
	     __attribute__((naked)) \
	     __attribute__((section(".init3")));
void __patch_wdt(void)
{
	MCUSR = 0;
	wdt_disable();
}
#endif

void lgt8fx8x_init()
{
	volatile unsigned char btmp = 0;

#if defined(__LGT8F88ES__)
	TKCSR = 0x00;
	PCMSK0 |= 0x40;
#endif

#if defined(__LGT8FX8E__)
// store ivref calibration 
	GPIOR1 = VCAL1;
	GPIOR2 = VCAL2;

// disable SWD/IF to release PE0/2
	MCUSR = 0x80;
	MCUSR = 0x80;

#if defined(__LGT8F_SSOP20__)
	btmp = PMXCR | 0x07;
	PMXCR = 0x80;
	PMXCR = btmp;
// for A0/1 and A4/5
	DIDR0 = 0x33;
#else
// for A0 ~ A5
	DIDR0 = 0x3F;
#endif

// enable 1KB E2PROM 
	ECCR = 0x80;
	ECCR = 0x40;

// clock source settings
	if((VDTCR & 0x0C) == 0x0C)
	{
		// switch to external crystal
		btmp = PMCR | 0x04;
		PMCR = 0x80;
		PMCR = btmp;

		for(btmp=0xf; btmp > 0; --btmp);
		btmp = PMCR | 0x20;
		PMCR = 0x80;
		PMCR = btmp;

		CLKPR = 0x80;
		CLKPR = 0x00;	
	} else {
		CLKPR = 0x80;
		CLKPR = 0x01;	
	}
#else
	CLKPR = 0x80;
	CLKPR = 0x01;
#endif
}

int main(void)
{
    
#if defined(__LGT8F88A__) || defined(__LGT8F88ES__) || defined(__LGT8FX8E__)
	lgt8fx8x_init();
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); 
#endif

    init();

#if defined(USBCON)
	USBDevice.attach();
#endif

    _begin(); // Startup MySensors library  

	for(;;) {
		_process();  // Process incoming data
		if (loop) {
			loop(); // Call sketch loop
		}
		if (serialEventRun) {
			serialEventRun();
		}
	}

    return 0;
}    