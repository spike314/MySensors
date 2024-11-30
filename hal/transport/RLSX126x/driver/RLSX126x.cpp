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
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * Based on Olivier Mauti's SX126x driver
 *
 * SX126x driver for MySensors, Copyright (C) 2020 Eduard Iten <eduard@iten.pro>
 *
 */

#include "RLSX126x.h"
#include "drivers/CircularBuffer/CircularBuffer.h"

// debug
#if defined(MY_DEBUG_VERBOSE_SX126x)
#define SX126x_DEBUG(x, ...) DEBUG_OUTPUT(x, ##__VA_ARGS__) //!< Debug print
#else
#define SX126x_DEBUG(x, ...) //!< DEBUG null
#endif

//Global status variable
static sx126x_internal_t SX126x;

// RX Buffer
static sx126x_packet_t rx_circular_buffer_buffer[MY_SX126X_RX_BUFFER_SIZE];
// use current packet as Pointer to rx circular buffer
// Circular buffer
static CircularBuffer<sx126x_packet_t> rx_circular_buffer(rx_circular_buffer_buffer,
        MY_SX126X_RX_BUFFER_SIZE);


//helper funcions
#define SX126x_internalToSNR(internalSNR) internalSNR/4

static sx126x_RSSI_t SX126x_RSSItoInternal(const int16_t externalRSSI)
{
	return static_cast<sx126x_RSSI_t>(constrain(externalRSSI + SX126x_RSSI_OFFSET, 0, 255));
}

static int16_t SX126x_internalToRSSI(const sx126x_RSSI_t internalRSSI)
{
	return static_cast<int16_t>(internalRSSI - SX126x_RSSI_OFFSET);
}

static bool SX126x_initialise()
{
	// setting pin modes
	SX126x_DEBUG(PSTR("SX126x:INIT\n"));
#ifdef STM32WLxx // Radio connections are internal (no pins) on STM32WL series
	SX126x_DEBUG(PSTR("SX126x:INIT:STM32WLxx defined. Pin assignment not applicable.\n"),
	             MY_SX126x_BUSY_PIN);
#endif

#if defined(MY_SX126x_POWER_PIN)
	hwPinMode(MY_SX_126x_POWER_PIN, OUTPUT);
	SX126x_powerUp();
	SX126x_DEBUG(PSTR("SX126x:INIT:PWRPIN=%u\n"), MY_SX126x_POWER_PIN);
#endif


#if defined(MY_SX126x_BUSY_PIN)
	// hwPinMode(MY_SX126x_BUSY_PIN, INPUT); // RadioLib will set up the busy pin.  Part of SX126x Module
	SX126x_DEBUG(PSTR("SX126x:INIT:BSYPIN=%u\n"), MY_SX126x_BUSY_PIN);
#endif


#if defined(MY_SX126x_IRQ_PIN)
	// hwPinMode(MY_SX126x_IRQ_PIN, INPUT); // RadioLib will set up the IRQ pin.  Part of SX126x Module
	SX126x_DEBUG(PSTR("SX126x:INIT:IRQPIN=%u\n"), MY_SX126x_IRQ_PIN);
#endif

#if defined(MY_SX126x_RESET_PIN)
	// RadioLib will set up the reset pin.  Part of SX126x Module
	SX126x_DEBUG(PSTR("SX126x:INIT:RSTPIN=%u\n"), MY_SX126x_RESET_PIN);
#endif


	SX126x.address = SX126x_BROADCAST_ADDRESS;
	SX126x.ackReceived = false;
	SX126x.dataReceived = false;
	SX126x.txSequenceNumber = 0;
	SX126x.powerLevel = 0;
	SX126x.targetRSSI = MY_SX126x_ATC_TARGET_DBM;
	SX126x.ATCenabled = false;

	// Are we using a TCXO and if so, is it controlled by the SX126x
#if defined(MY_SX126x_USE_TCXO)
#if defined(MY_SX126x_TCXO_VOLTAGE)
	SX126x_DEBUG(PSTR("SX126x:INIT:DIO3TCXO,VCONF:%02X,DELAY:%ums\n"), MY_SX126x_TCXO_VOLTAGE,
	             MY_SX126c_TCXO_STARTUP_DELAY);
#else
	SX126x_DEBUG(PSTR("SX126x:INIT:TCXO,EXT\n"));
#endif //MY_SX126x_TCXO_VOLTAGE
#else
#define MY_SX126x_TCXO_VOLTAGE 0.0 // RadioLib uses 0.0 if not using TCXO
#endif // MY_SX126x_USE_TCXO



	// Antenna RX/TX switch logic and TXCO for STM32WL
#if defined(STM32WLxx)
	// For STM32WL
	// set RF switch control configuration
	// this has to be done prior to calling begin()
#define MY_SX126x_TCXO_VOLTAGE (1.7)  //This is the TCXO voltage for E5 module

	radio1.setRfSwitchTable(rfswitch_pins, rfswitch_table);
#else



#if defined(MY_SX126x_USE_DIO2_ANT_SWITCH) && defined(MY_SX126x_ANT_SWITCH_PIN)
#error MY_SX126x_USE_DIO2_ANT_SWITCH and MY_SX126x_ANT_SWITCH_PIN both defined which makes no sense
#endif
#if !defined(MY_SX126x_USE_DIO2_ANT_SWITCH) && !defined(MY_SX126x_ANT_SWITCH_PIN)
#error Either MY_SX126x_USE_DIO2_ANT_SWITCH or MY_SX126x_ANT_SWITCH_PIN has to be defined
#endif
#ifdef MY_SX126x_ANT_SWITCH_PIN // Switch Pin High = Transmit (this seems to be the norm for DIO2)
	radio1.setRfSwitchPins(RADIOLIB_NC,
	                       MY_SX126x_ANT_SWITCH_PIN); // RX enable not connected, TX enable high on transmit.
	SX126x_DEBUG(PSTR("SX126x:INIT:ASWPIN=%u\n"), MY_SX126x_ANT_SWITCH_PIN);
#endif
#endif // else


	// start radio
	int16_t status = radio1.begin(MY_SX126x_FREQUENCY, MY_SX126x_LORA_BW, MY_SX126x_LORA_SF,
	                              MY_SX126x_LORA_CR, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, MY_SX126x_TX_POWER_DBM, 8,
	                              MY_SX126x_TCXO_VOLTAGE, false);
	// 8 is the preamble length
	// false sets it to use DC/DC regulator
	if(status) { // there was an error
		SX126x_handleError( status); // Sends sanity check messages without calling sanitycheck()
		return false;
	}
#if defined(MY_SX126x_USE_DIO2_ANT_SWITCH)
	radio1.setDio2AsRfSwitch();
	SX126x_DEBUG(PSTR("SX126x:INIT:DIO2AntSw\n"));
#endif


	// disable and clear all interrupts
	radio1.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
	radio1.setDio1Action(SX126x_interruptHandler);
	SX126x_sleep();
	SX126x_standBy();
	return true;
}

static void SX126x_interruptHandler()
{
	noInterrupts();
	SX126x.irqFired = true;
	SX126x.radioMode = SX126x_MODE_STDBY_RC;
	interrupts();
}

static void SX126x_handle()
{
#ifdef MY_SX126x_IRQ_PIN 
	if (SX126x.irqFired) {
#endif
		uint32_t irqStatus = RADIOLIB_SX126X_IRQ_NONE;
		//get the interrupt fired
		irqStatus = radio1.getIrqFlags();

		//Transmission done
		if (irqStatus & RADIOLIB_SX126X_IRQ_TX_DONE) {
			SX126x.txComplete = true;
		}

		//Reception done
		if (irqStatus & RADIOLIB_SX126X_IRQ_RX_DONE ) {
			sx126x_rxBufferStatus_t bufferStatus;
			bufferStatus.fields.payloadLength = min(radio1.getPacketLength(true), SX126x_MAX_PACKET_LEN);
			SX126x.currentPacket.payloadLen = bufferStatus.fields.payloadLength - SX126x_HEADER_LEN;
			radio1.readData(SX126x.currentPacket.data,
			                bufferStatus.fields.payloadLength); // If packet length is 0, it will be retrived automatically, but MySensors needs to know the length.
			SX126x.currentPacket.RSSI = SX126x_RSSItoInternal((int16_t)radio1.getRSSI());
			SX126x.currentPacket.SNR = radio1.getSNR();
			if ((SX126x.currentPacket.header.version >= SX126x_MIN_PACKET_HEADER_VERSION) &&
			(SX126x_PROMISCUOUS || SX126x.currentPacket.header.recipient == SX126x.address ||
			SX126x.currentPacket.header.recipient == SX126x_BROADCAST_ADDRESS)) {
				// Message for us
				SX126x.ackReceived = SX126x.currentPacket.header.controlFlags.fields.ackReceived &&
				                     !SX126x.currentPacket.header.controlFlags.fields.ackRequested;
				SX126x.dataReceived = !SX126x.ackReceived;
				// if data was received, push it to the circular buffer
				if(SX126x.dataReceived) {
					rx_circular_buffer.pushFront(&SX126x.currentPacket);
				}
			}
		}

		//CAD done
		if (irqStatus & RADIOLIB_SX126X_IRQ_CAD_DONE) {
			SX126x.channelFree = true;
			SX126x.radioMode = SX126x_MODE_STDBY_RC;

		}

		//CAD channel active
		if (irqStatus & RADIOLIB_SX126X_IRQ_CAD_DETECTED) {
			SX126x.channelActive = true;
			SX126x.radioMode = SX126x_MODE_RX;
		}

		//			// Clear the IRQ flags
		radio1.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
		SX126x.irqFired = false;
#ifdef MY_SX126x_IRQ_PIN
	}
#endif
}


static void SX126x_standBy()
{
	int16_t status = radio1.standby(); // this uses 13 MHz RC oscilator by default
	if(status) {
		SX126x_DEBUG(PSTR("SX126x:ERR:STBY:status=%d\n"), status);
	}
}

static void SX126x_sleep(void)
{
	radio1.sleep();
}

static bool SX126x_txPower(sx126x_powerLevel_t power)
{
	power = constrain(power, MY_SX126x_MIN_POWER_LEVEL_DBM,
	                  MY_SX126x_MAX_POWER_LEVEL_DBM); // constrain to user power settings
	int8_t clippedPower;
	radio1.checkOutputPower(power, &clippedPower); // Let RadioLib constrain to radio variant capability
	radio1.setOutputPower(clippedPower);
	SX126x.powerLevel = clippedPower;
	SX126x_DEBUG(PSTR("SX126x:PWR:LEVEL=%d db\n"), SX126x.powerLevel);
	return true;
}

static bool SX126x_sanityCheck()
{

#ifdef STM32WLxx
	return true;   // No errors since the radio wiring is internal to the chip
#else
	int16_t status = radio1.standby(); // For non-STM32LS set to standby and check if there are errors
	// TODO:   This isn't a very good sanity check except at init.  Find a method that doesn't require standby.
	if(status) {
		SX126x_handleError(status);
		return false;
	} else {
		SX126x_rx();
		return true;
	}
#endif
}

static void SX126x_setAddress(uint8_t address)
{
	SX126x.address = address;
}

static uint8_t SX126x_getAddress(void)
{
	return SX126x.address;
}

static bool SX126x_sendWithRetry(const uint8_t recipient, const void *buffer,
                                 const uint8_t bufferSize, const bool noACK)
{
	sx126x_controlFlags_t flags{ 0 };
	flags.fields.ackRequested = !noACK;
	SX126x.txSequenceNumber++;
	for (uint8_t retry = 0; retry <  SX126x_RETRIES; retry++) {
		SX126x_DEBUG(PSTR("SX126x:SWR:SEND,TO=%u,SEQ=%u,RETRY=%u\n"),
		             recipient,
		             SX126x.txSequenceNumber,
		             retry);
		bool sendResult = SX126x_send(recipient, (uint8_t *)buffer, bufferSize, flags);
		if (!sendResult) {
			SX126x.radioMode = SX126x_MODE_STDBY_RC;
			return false;
		}
		SX126x_rx();
		if (noACK) {
			return true;
		}
		uint32_t start = hwMillis(); // We need to wait for an ack
		while (hwMillis() < start + SX126x_RETRY_TIMEOUT_MS) {
			SX126x_handle();
			if (SX126x.ackReceived) {
				SX126x_rx();  // Send was successful so go back to receive
				//is it the ACK for our current packet?
				if (SX126x.currentPacket.header.sender == recipient &&
				SX126x.currentPacket.ACK.sequenceNumber == SX126x.txSequenceNumber) {
					SX126x_DEBUG(PSTR("SX126x:SWR:ACK FROM=%u,SEQ=%u,RSSI=%d,SNR=%d\n"),
					             SX126x.currentPacket.header.sender,
					             SX126x.currentPacket.ACK.sequenceNumber,
					             SX126x_internalToRSSI(SX126x.currentPacket.ACK.RSSI),
					             SX126x_internalToSNR(SX126x.currentPacket.ACK.SNR));
					if (SX126x.ATCenabled) {
						SX126x_ATC();
					}
					// Ack received.
					return true;
				}
			}
			doYield();
		}
		SX126x_DEBUG(PSTR("!SX126x:SWR:NACK\n"));  // we didn't receive an ack to our current packet
		const uint32_t enterCSMAMS =
		    hwMillis();   // SMVMVS = Carrier Sense Multiple Access, essentially wait for a clear channel.
		const uint16_t randDelayCSMA = start % 100;
		while (hwMillis() - enterCSMAMS < randDelayCSMA) {
			doYield();
		}
		if (SX126x.ATCenabled) {
			SX126x_txPower(SX126x.powerLevel + 2); //increase power, maybe we are far away from gateway
		}
	}
	return false;
}

static bool SX126x_send(const uint8_t recipient, uint8_t *data, const uint8_t len,
                        const sx126x_controlFlags_t flags)
{
	sx126x_packet_t packet;
	packet.header.version = SX126x_PACKET_HEADER_VERSION;
	packet.header.sender = SX126x.address;
	packet.header.recipient = recipient;
	packet.payloadLen = min(len, (uint8_t)SX126x_MAX_PAYLOAD_LEN);
	packet.header.controlFlags = flags;
	memcpy((void *)&packet.payload, (void *)data, packet.payloadLen);
	return SX126x_sendPacket(&packet); 
}

static bool SX126x_sendPacket(sx126x_packet_t *packet)
{
	if (!SX126x_cad()) {
		return false;
	}
	packet->header.sequenceNumber = SX126x.txSequenceNumber;
	uint8_t finalLength = packet->payloadLen + SX126x_HEADER_LEN;
	int16_t transmissionState = RADIOLIB_ERR_NONE;
	SX126x.txComplete = false;
	transmissionState = radio1.startTransmit(packet->data, finalLength);
	//	SX126x_tx();
	SX126x.radioMode = SX126x_MODE_TX;
	uint32_t txStart = hwMillis();
	while (!SX126x.txComplete) {
		SX126x_handle();
		if (millis() > txStart + MY_SX126x_TX_TIMEOUT_MS) {
			return false;
		}
		doYield();
	}
	if (transmissionState == RADIOLIB_ERR_NONE) {
		// packet was successfully sent
		radio1.finishTransmit();  //  Switch to RX for Ack is in SendWithRetry()
		return true;
	} else {
		SX126x_DEBUG(PSTR("SX126x:SWR:TX ERR:status=%d\n"), transmissionState);
		// clean up after transmission is finished
		radio1.finishTransmit();
		SX126x.txComplete = false;
		return false;
	}
}

static void SX126x_rx()
{
		SX126x.ackReceived = false;
		SX126x.dataReceived = false;
		radio1.startReceive(); 
		SX126x.radioMode = SX126x_MODE_RX;
}

static bool SX126x_cad()
{
	SX126x.channelActive = false;
	SX126x.channelFree = false;

	radio1.startChannelScan();
	while (!SX126x.channelActive && !SX126x.channelFree) {
		SX126x_handle();
	}
	if (SX126x.channelFree) {
		return true;
	} else {
		SX126x_DEBUG(PSTR("!SX126x:CAD\n"));
		return false;
	}
}

static bool SX126x_packetAvailable()
{
	uint8_t packetcnt = rx_circular_buffer.available();
	if (packetcnt > 0) {
		return true;
	} else {
		return false;
	}
}

static uint8_t SX126x_getData(uint8_t *buffer, const uint8_t bufferSize)
{
	// get content from rx buffer
	sx126x_packet_t *first_message = rx_circular_buffer.getBack();

	// clear data flag
	SX126x.dataReceived = false;

	const uint8_t payloadSize = min(first_message->payloadLen, bufferSize);
	if (buffer != NULL) {
		(void)memcpy((void *)buffer, (void *)&first_message->payload, payloadSize);
	}

	// ACK handling
	if (first_message->header.controlFlags.fields.ackRequested &&
	        !first_message->header.controlFlags.fields.ackReceived) {
#if defined(MY_GATEWAY_FEATURE) && (F_CPU>16*1000000ul)
		// delay for fast GW and slow nodes
		delay(50);
#endif
		SX126x_DEBUG(PSTR("SX126x:RCV:SEND ACK\n"));
		SX126x_sendAck(
		    first_message->header.sender,
		    first_message->header.sequenceNumber,
		    first_message->RSSI,
		    first_message->SNR);
	}
	// release buffer
	rx_circular_buffer.popBack();
	// Go back to receive 
	SX126x_rx(); 
	return payloadSize;
}

static void SX126x_sendAck(const uint8_t recipient, const sx126x_sequenceNumber_t sequenceNumber,
                           const sx126x_RSSI_t RSSI, const sx126x_SNR_t SNR)
{
	SX126x_DEBUG(PSTR("SX126x:SAC:SEND ACK,TO=%u,SEQ=%u,RSSI=%d,SNR=%u\n"),
	             recipient,
	             sequenceNumber,
	             SX126x_internalToRSSI(RSSI),
	             SX126x_internalToSNR(SNR));
	sx126x_ack_t ACK;
	ACK.sequenceNumber = sequenceNumber;
	ACK.RSSI = RSSI;
	ACK.SNR = SNR;
	sx126x_controlFlags_t flags = { 0 };
	flags.fields.ackReceived = true;
	flags.fields.ackRssiReport = true;
	(void)SX126x_send(recipient, (uint8_t *)&ACK, sizeof(sx126x_ack_t), flags);
}

static void SX126x_ATC()
{
#if !defined(MY_GATEWAY_FEATURE) && !defined(MY_SX126x_DISABLE_ATC)  // Save a little memory

	int8_t delta;
	sx126x_powerLevel_t newPowerLevel;
	delta = SX126x.targetRSSI - SX126x_internalToRSSI(SX126x.currentPacket.ACK.RSSI);
	newPowerLevel = SX126x.powerLevel + delta / 2;
	// newPowerLevel = constrain(newPowerLevel, MY_SX126x_MIN_POWER_LEVEL_DBM, // This constraint is in SX126x_txPower()
	//                           MY_SX126x_MAX_POWER_LEVEL_DBM);
	SX126x_DEBUG(PSTR("SX126x:ATC:ADJ TXL, cR=%d, tR=%d, rTXL=%d\n"),
	             SX126x_internalToRSSI(SX126x.currentPacket.ACK.RSSI),
	             SX126x.targetRSSI,
	             newPowerLevel
	            );
	if (newPowerLevel != SX126x.powerLevel) {
		SX126x_txPower(newPowerLevel);
	}
#endif
}

static void SX126x_setATC(bool onOff, int8_t targetRSSI)
{
	SX126x.ATCenabled = onOff;
	SX126x.targetRSSI = targetRSSI;
}


void SX126x_powerUp()
{
#ifdef MY_SX126x_POWER_PIN
	hwDigitalWrite(SX126x_POWER_PIN, HIGH);
	SX126x_DEBUG(PSTR("SX126x:PWU\n");
#endif
}

void SX126x_powerDown()
{
#ifdef MY_SX126x_POWER_PIN
	hwDigitalWrite(SX126x_POWER_PIN, LOW);
	SX126x_DEBUG(PSTR("SX126x:PWD\n");
#endif
}

static int16_t SX126x_getSendingRSSI(void)
{
	// own RSSI, as measured by the recipient - ACK part
	if (SX126x.currentPacket.header.controlFlags.fields.ackRssiReport) {
		return SX126x_internalToRSSI(SX126x.currentPacket.ACK.RSSI);
	} else {
		// not possible
		return INVALID_RSSI;
	}
}

static int16_t SX126x_getSendingSNR(void)
{
	// own SNR, as measured by the recipient - ACK part
	if (SX126x.currentPacket.header.controlFlags.fields.ackRssiReport) {
		return static_cast<int16_t>(SX126x_internalToSNR(SX126x.currentPacket.ACK.SNR));
	} else {
		// not possible
		return INVALID_SNR;
	}
}

static int16_t SX126x_getReceivingRSSI(void)
{
	// RSSI from last received packet
	return static_cast<int16_t>(radio1.getRSSI(
	                                true)); // true = last read packet RSSI, false = current packet (I think).  getRSSI returns float
}

static int16_t SX126x_getReceivingSNR(void)
{
	// SNR from last received packet
	return static_cast<int16_t>
	       (radio1.getSNR()); // SNR of last received packet.  No options.  getSNR retruns float
}

static int8_t SX126x_getTxPowerLevel(void)
{
	return SX126x.powerLevel;
}

static uint8_t SX126x_getTxPowerPercent(void)
{
	// report TX level in %
	const uint8_t result = static_cast<uint8_t>(100.0f * (SX126x.powerLevel -
	                       MY_SX126x_MIN_POWER_LEVEL_DBM) /
	                       (MY_SX126x_MAX_POWER_LEVEL_DBM
	                        - MY_SX126x_MIN_POWER_LEVEL_DBM));
	return result;
}
static bool SX126x_setTxPowerPercent(const uint8_t newPowerPercent)
{
	const sx126x_powerLevel_t newPowerLevel = static_cast<sx126x_powerLevel_t>
	    (MY_SX126x_MIN_POWER_LEVEL_DBM + (MY_SX126x_MAX_POWER_LEVEL_DBM
	                                      - MY_SX126x_MIN_POWER_LEVEL_DBM) * (newPowerPercent / 100.0f));
	SX126x_DEBUG(PSTR("SX126x:SPP:PCT=%u,TX LEVEL=%d\n"), newPowerPercent, newPowerLevel);
	return SX126x_txPower(newPowerLevel);
}

static void SX126x_handleError(int16_t statusCode)
{

	switch (statusCode) {
	case RADIOLIB_ERR_UNKNOWN:
	// fall thru
	case RADIOLIB_ERR_SPI_CMD_TIMEOUT:
	// fall thru
	case RADIOLIB_ERR_SPI_CMD_INVALID:
		SX126x_DEBUG(PSTR("!SX126x:INIT:SANCHK FAIL\n"));
		break;

	case RADIOLIB_ERR_SPI_CMD_FAILED:
		SX126x_DEBUG(PSTR("!SX126x:INIT:SANCHK FAIL\n"));
		SX126x_DEBUG(PSTR("!SX126x:INIT:CHECK TCXO\n"));
		SX126x_DEBUG(PSTR("!SX126x:INIT:ERR:status=%d\n"), statusCode);
		break;

	case RADIOLIB_ERR_INVALID_CRC_CONFIGURATION:
	// fall thru
	case RADIOLIB_ERR_INVALID_TCXO_VOLTAGE:
	// fall thru
	case RADIOLIB_ERR_INVALID_MODULATION_PARAMETERS:
	// fall thru
	case RADIOLIB_ERR_INVALID_SLEEP_PERIOD:
	// fall thru
	case RADIOLIB_ERR_INVALID_RX_PERIOD:
		SX126x_DEBUG(PSTR("!SX126x:INIT:ERR:status=%d\n"), statusCode);
		break;

	case RADIOLIB_ERR_INVALID_OUTPUT_POWER:
		SX126x_DEBUG(PSTR("SX126x:ATC:ERR:Power constrained to level valid for this module\n"));
		break;

	default:
		// Just print the RadioLib error number
		SX126x_DEBUG(PSTR("!SX126x:INIT:ERR:status=%d\n"), statusCode);
		break;
	}

}
