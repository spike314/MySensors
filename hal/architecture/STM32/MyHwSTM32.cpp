/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2020 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * STM32 architecture support added by Alexander KooLru <kool@kool.ru>
 * Copyright (C) 2022 Alexander KooLru
 * STM32 sleep mode and EEPROM support added by WhiskyDelta <arne.schwarz@d2a.de>
 * Copyright (C) 2022 Arne Schwarz
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include "MyHwSTM32.h"

/*
* Pinout STM32F103C8 dev board:
* http://wiki.stm32duino.com/images/a/ae/Bluepillpinout.gif
*
* Wiring
* --------------------------------------------------
RFM69 	CLK		MISO		MOSI	CSN		CE		IRQ
SPI1		PA5		PA6			PA7		PA4		NA		PA3 (default)

RF24 		CLK		MISO		MOSI	CSN		CE		IRQ
SPI1		PA5		PA6			PA7		PA4		PB0		NA

*/

volatile uint8_t _wokeUpByInterrupt =
    INVALID_INTERRUPT_NUM;    // Interrupt number that woke the mcu.
volatile uint8_t _wakeUp1Interrupt  =
    INVALID_INTERRUPT_NUM;    // Interrupt number for wakeUp1-callback.
volatile uint8_t _wakeUp2Interrupt  =
    INVALID_INTERRUPT_NUM;    // Interrupt number for wakeUp2-callback.

static uint32_t sleepRemainingMs = 0ul;

void wakeUp1(void)
{
	// First interrupt occurred will be reported only
	if (INVALID_INTERRUPT_NUM == _wokeUpByInterrupt) {
		_wokeUpByInterrupt = _wakeUp1Interrupt;
	}
}
void wakeUp2(void)
{
	// First interrupt occurred will be reported only
	if (INVALID_INTERRUPT_NUM == _wokeUpByInterrupt) {
		_wokeUpByInterrupt = _wakeUp2Interrupt;
	}
}

inline bool interruptWakeUp(void)
{
	return _wokeUpByInterrupt != INVALID_INTERRUPT_NUM;
}

bool hwInit(void)
{
#if !defined(MY_DISABLED_SERIAL)
	MY_SERIALDEVICE.begin(MY_BAUD_RATE);
#if defined(MY_GATEWAY_SERIAL)
	while (!MY_SERIALDEVICE) {}
#endif
#endif
	LowPower.begin();
	return true;
}

void hwReadConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *dst = static_cast<uint8_t *>(buf);
	int offs = reinterpret_cast<int>(addr);
#if !defined(DATA_EEPROM_BASE)
	eeprom_buffer_fill();
	while (length-- > 0) {
		*dst++ = eeprom_buffered_read_byte(offs++);
	}
#else
	while (length-- > 0) {
		*dst++ = eeprom_read_byte(offs++);
	}
#endif
}

void hwWriteConfigBlock(void *buf, void *addr, size_t length)
{
	uint8_t *src = static_cast<uint8_t *>(buf);
	int offs = reinterpret_cast<int>(addr);
#if !defined(DATA_EEPROM_BASE)
	while (length-- > 0) {
		eeprom_buffered_write_byte(offs++, *src++);
	}
	eeprom_buffer_flush();
#else
	while (length-- > 0) {
		eeprom_write_byte(offs++, *src++);
	}
#endif
}

uint8_t hwReadConfig(const int addr)
{
	uint8_t value;
	hwReadConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
	return value;
}

void hwWriteConfig(const int addr, uint8_t value)
{
	if (hwReadConfig(addr) != value) {
		hwWriteConfigBlock(&value, reinterpret_cast<void *>(addr), 1);
	}
}

int8_t hwSleep(uint32_t ms)
{
	// Return what woke the mcu.
	// Default: no interrupt triggered, timer wake up
	int8_t ret = MY_WAKE_UP_BY_TIMER;

	if (ms > 0u) {
		// sleep for defined time
		LowPower.deepSleep(ms);
	} else {
		// sleep until ext interrupt triggered
		LowPower.deepSleep();
	}
	if (interruptWakeUp()) {
		ret = static_cast<int8_t>(_wokeUpByInterrupt);
	}
	// Clear woke-up-by-interrupt flag, so next sleeps won't return immediately.
	_wokeUpByInterrupt = INVALID_INTERRUPT_NUM;

	return ret;
}

int8_t hwSleep(const uint8_t interrupt, const uint8_t mode, uint32_t ms)
{
	return hwSleep(interrupt, mode, INVALID_INTERRUPT_NUM, 0u, ms);
}

int8_t hwSleep(const uint8_t interrupt1, const uint8_t mode1, const uint8_t interrupt2,
               const uint8_t mode2,
               uint32_t ms)
{
	// According to STM32LowPower API following modes to wake from sleep are supported: HIGH, LOW, RISING, FALLING or CHANGE
	// Ref: https://github.com/stm32duino/STM32LowPower

	// attach interrupts
	_wakeUp1Interrupt  = interrupt1;
	_wakeUp2Interrupt  = interrupt2;

	if (interrupt1 != INVALID_INTERRUPT_NUM) {
		LowPower.attachInterruptWakeup(interrupt1, wakeUp1, mode1, DEEP_SLEEP_MODE);
	}
	if (interrupt2 != INVALID_INTERRUPT_NUM) {
		LowPower.attachInterruptWakeup(interrupt2, wakeUp2, mode2, DEEP_SLEEP_MODE);
	}

	if (ms > 0u) {
		// sleep for defined time
		return hwSleep(ms);
	} else {
		// sleep until ext interrupt triggered
		return hwSleep(0);
	}
}



bool hwUniqueID(unique_id_t *uniqueID)
{
	// Fill ID with FF
	(void)memset((uint8_t *)uniqueID,  0xFF, 16);
	// Read device ID
	(void)memcpy((uint8_t *)uniqueID, (uint32_t *)UID_BASE, 12);

	return true;
}

uint16_t hwCPUVoltage(void)
{
	//Not yet implemented
	return FUNCTION_NOT_SUPPORTED;
}

uint16_t hwCPUFrequency(void)
{
	return HAL_RCC_GetSysClockFreq()/1000000UL;
}

int8_t hwCPUTemperature(void)
{
	return FUNCTION_NOT_SUPPORTED;
}

uint16_t hwFreeMem(void)
{
	//Not yet implemented
	return FUNCTION_NOT_SUPPORTED;
}


void hwWatchdogReset(void)
{
	IWatchdog.reload();
}

void hwReboot(void)
{
	NVIC_SystemReset();
	while (true)
		;
}
