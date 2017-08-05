// rf95_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W 

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>


#define SERVER_ADDRESS 0

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);


uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void setup()
{
	Serial.begin(115200);
	while (!Serial); // Wait for serial port to be available
	if (!manager.init())
		Serial.println("init failed");

	driver.setFrequency(915.0);
	driver.setTxPower(20, false);  // Modtronix inAir9 power transmitter power for -1 to 14 dBm and with useRFO true
	//driver.setCADTimeout(10000); // wait until Channel Activity Detection shows no activity on the channel before transmitting
	Serial.println(F("Begin..."));
}

void loop()
{
	if (manager.available())
	{
		// Wait for a message addressed to us from the client
		uint8_t len = sizeof(buf);
		uint8_t from;
		if (manager.recvfromAck(buf, &len, &from))
		{
			Serial.write(len);
			Serial.println();
			for (uint8_t i = 0; i < len-2; i++) {
				Serial.write(buf[i]);
				//Serial.print(F(" "));
			}
			//Serial.print(driver.lastRssi(), HEX);
			//Serial.print(driver.lastSNR()), HEX;
			Serial.println();

			//for (uint8_t i = 0; i < len; i++) {
			//	if (buf[i] < 16) {
			//		Serial.print(F("0"));
			//	}
			//	else {
			//		//Serial.print(F(""));
			//	}
			//	Serial.print(buf[i], HEX);
			//	Serial.print(F(" "));
			//}

			//Serial.print(F(" "));
			//Serial.print(driver.lastRssi());
			//Serial.print(F(" "));
			//Serial.print(driver.lastSNR());
			//Serial.print(F(" "));
			//Serial.print(len);
			//Serial.println();
		}
	}
}
