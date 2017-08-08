/*
Name:		Bineye_Bin_Master.ino
Created:	7/20/2017 12:17:14 AM
Author:	tpeterson
*/

#include <EEPROM.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <LowPower.h>
#include <Arduino.h>
#include <OneWire.h>


//Radio parameters
#define SERVER_ADDRESS			0x00
#define CLIENT_ADDRESS			0xFE

// 1-wire commands
#define HT_Power	            0x09 //PIN to turn on sensor string
#define Write_Data_Stop         0x4B
#define Write_Data_No_Stop      0x5A
#define Write_Data_Only         0x69
#define Write_Data_Only_Stop    0x78
#define Read_Data_Stop          0x87
#define Write_Read_Data_Stop    0x2D
#define Write_Config            0xD2
#define Read_Config             0xE1
#define Enable_Sleep            0x1E
#define Read_Device_Rev         0xC3

// HDC1080 commands
#define i2c_Address             0x80 // Address of the i2c temp and humidity sensor
#define get_humidity            0x01 // Command to take a Measurement
#define get_temperature         0x00 // Command to take a Measurement

#define EEPROM_VERSION			0x00 // EEPROM address for version
#define EEPROM_UUID				0x01 // EEPROM address for UUID
#define EEPROM_RADIOID			0x09 // EEPROM address for radioid
#define EEPROM_SENSORS			0x10 // Start address for sensor 
#define VERSION					0x06 // Version of software. Increment to refresh EEPROM
#define HT_SEND_PACKET			0x01 // Identifier for packet type, unique to each client product
#define SENSORS_PER_STRING		0x06 // Number of sensors per string TODO dynamic?
#define BYTES_FOR_SENSORS		0x0C // 2xSENSORS_PER_STRING
#define ONEWIRE_ADDRESS_LENGTH  0x08 // bytes in a onewire address
//#define STRINGS					0x01 // Number of strings TODO

// Variables for 1-wire side and data
OneWire oneWire(8);
uint8_t config_packet[8];
uint8_t request_packet[6];
uint8_t read_packet[5];
uint8_t hdata[2];
uint8_t tdata[2];
uint16_t CRC16;

struct packet {
	char packet_type[8] = { 'H', 'T', 'S', '0', '0', '0', '0', '1' }; // [0]
	uint8_t UUID[8];												// [8]
	uint8_t sensor_count = BYTES_FOR_SENSORS;						// [16]
	uint8_t humidity[BYTES_FOR_SENSORS];							// [17]
	uint8_t temperature[BYTES_FOR_SENSORS];							// [29]
	int8_t sensor_RSSI;												// [41]
	int8_t sensor_SNR;												// [42]
} payload;


uint8_t UUID[8];
uint8_t radio_address = 0xFE;

// Variables for Radio
RH_RF95 driver;
RHReliableDatagram manager(driver, SERVER_ADDRESS);
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];


// Initial device setup. burns a UUID, Radioid, and version into EEPROM
void init_device() {
	if (EEPROM.read(EEPROM_VERSION) == VERSION) {
		Serial.print(F("Version: "));
		Serial.print(EEPROM.read(EEPROM_VERSION));
		Serial.println();
	}
	else {
		Serial.println(F("NEW DEVICE SETUP"));
		delay(5000);
		uint8_t sensor_count = 0;
		Serial.println(F("Adding sensors to EEPROM"));
		while (oneWire.search(UUID)) {
			for (uint8_t i = 0; i < 8; i++) {
				if (UUID[i] < 16) {
					Serial.print(F("0x0"));
				}
				else {
					Serial.print(F("0x"));
				}
				EEPROM.write(EEPROM_SENSORS + sensor_count*ONEWIRE_ADDRESS_LENGTH + i, UUID[i]);
				Serial.print(EEPROM.read(EEPROM_SENSORS + sensor_count*ONEWIRE_ADDRESS_LENGTH + i), HEX);
				Serial.print(F(" "));
			}
			Serial.println();
			sensor_count++;
		}
		Serial.print(F("Total sensor count: "));
		Serial.println(sensor_count);

		oneWire.reset_search();
		if (oneWire.search(UUID)) {
			Serial.println(F("Setting UUID: "));

			for (uint8_t i = 0; i < 8; i++) {
				EEPROM.write(EEPROM_UUID + i, UUID[i]);
				if (EEPROM.read(EEPROM_UUID + i) < 16) {
					Serial.print(F("0x0"));
				}
				else {
					Serial.print(F("0x"));
				}
				Serial.print(EEPROM.read(EEPROM_UUID + i), HEX);
				Serial.print(F(" "));
			}
		}
		Serial.println();
		Serial.print(F("Setting RadioID: "));
		EEPROM.write(EEPROM_RADIOID, CLIENT_ADDRESS);
		Serial.println(EEPROM.read(EEPROM_RADIOID));
		Serial.print(F("Setting version: "));
		EEPROM.write(EEPROM_VERSION, VERSION);
		Serial.println(EEPROM.read(EEPROM_VERSION));
	}
}

void setup() {
	ADCSRA &= ~(1 << 7); //disable ADC for power savings
	pinMode(HT_Power, OUTPUT);
	digitalWrite(HT_Power, HIGH); // Power up the sensor string
	Serial.begin(115200);
	while (!Serial);
	init_device();

	if (!manager.init())
		Serial.println(F("Radio init failed"));
	manager.setThisAddress(EEPROM.read(EEPROM_RADIOID));
	driver.setFrequency(915.0);
	driver.setTxPower(19, false);  //
	driver.setCADTimeout(10000); // wait until Channel Activity Detection shows no activity on the channel before transmitting
	Serial.println(F("Begin Slave..."));

	for (uint8_t i = 0; i < 8; i++) {
		payload.UUID[i] = EEPROM.read(EEPROM_UUID + i);
	}
}

void loop() {

	for (uint8_t i = 0; i < SENSORS_PER_STRING; i++) {
		//read address from EEPROM
		for (uint8_t j = 0; j < ONEWIRE_ADDRESS_LENGTH; j++) {
			UUID[j] = EEPROM.read(EEPROM_SENSORS + i*ONEWIRE_ADDRESS_LENGTH + j);
		}

		//debug
		for (uint8_t i = 0; i < 8; i++) {

			if (UUID[i] < 16) {
				Serial.print(F("0x0"));
			}
			else {
				Serial.print(F("0x"));
			}
			Serial.print(UUID[i], HEX);
			Serial.print(F(" "));
		}

		//Step 1/5 - setup parameters
		config_packet[0] = Write_Data_Stop;                // Command ("Write Data With Stop")
		config_packet[1] = i2c_Address;                    // I2C slave to address
		config_packet[2] = 0x03;                           // Number of data bytes to write
		config_packet[3] = 0x02;                           // configuration register
		config_packet[4] = 0x50;                           // 14bit&14bit resolution
		config_packet[5] = 0x00;
		CRC16 = ~oneWire.crc16(&config_packet[0], 6);      // Generate CRC & Invert
		config_packet[7] = CRC16 >> 8;                     // Most significant byte of 16 bit CRC
		config_packet[6] = CRC16 & 0xFF;                   // Least significant byte of 16 bit CRC
		oneWire.reset();
		oneWire.select(UUID);

		oneWire.write_bytes(config_packet, sizeof(config_packet), 1);

		while (oneWire.read_bit() == true)                 // Wait for not busy
			delay(1);

		// Step 2/5 - Send Humidity read packet
		request_packet[0] = Write_Data_Stop;               // Command ("Write Data With Stop")
		request_packet[1] = i2c_Address;                   // I2C slave to address
		request_packet[2] = 0x01;                          // Number of data bytes to write
		request_packet[3] = get_humidity;                  // The data to write
		CRC16 = ~oneWire.crc16(&request_packet[0], 4);
		request_packet[5] = CRC16 >> 8;                    // Most significant byte of 16 bit CRC
		request_packet[4] = CRC16 & 0xFF;                  // Least significant byte of 16 bit CRC
		oneWire.reset();
		oneWire.select(UUID);
		oneWire.write_bytes(request_packet, sizeof(request_packet), 1);

		while (oneWire.read_bit() == true)                 // Wait for not busy
			delay(1);

		// Step 3/5 - read Humidity
		delay(13);                                         // Delay between Messages 14-bit = 6.5ms
		read_packet[0] = Read_Data_Stop;                   // Command ("read Data With Stop")
		read_packet[1] = 0x81;                             // I2C slave to address
		read_packet[2] = 0x02;                             // Number of data bytes to read
		CRC16 = ~oneWire.crc16(&read_packet[0], 3);	       // Generate CRC & Invert
		read_packet[4] = CRC16 >> 8;                       // Most significant byte of 16 bit CRC
		read_packet[3] = CRC16 & 0xFF;                     // Least significant byte of 16 bit CRC
		oneWire.reset();
		oneWire.select(UUID);
		oneWire.write_bytes(read_packet, sizeof(read_packet), 1);

		while (oneWire.read_bit() == true)                 // Wait for not busy
			delay(1);

		if (oneWire.read() == 0) {						   // store and convert the reply
			oneWire.read_bytes(hdata, sizeof(hdata));
			payload.humidity[2 * i] = hdata[0];
			payload.humidity[2 * i + 1] = hdata[1];
		}
		else {
			payload.humidity[2 * i] = 0xFF;
			payload.humidity[2 * i + 1] = 0xFF;
		}

		// Step 4/5 - Send Temperature read packet
		request_packet[0] = Write_Data_Stop;               // Command ("Write Data With Stop")
		request_packet[1] = i2c_Address;                   // I2C slave to address
		request_packet[2] = 0x01;                          // Number of data bytes to write
		request_packet[3] = get_temperature;               // The data to write
		CRC16 = ~oneWire.crc16(&request_packet[0], 4);     // Generate CRC & Invert
		request_packet[5] = CRC16 >> 8;                    // Most significant byte of 16 bit CRC
		request_packet[4] = CRC16 & 0xFF;                  // Least significant byte of 16 bit CRC
		oneWire.reset();
		oneWire.select(UUID);
		oneWire.write_bytes(request_packet, sizeof(request_packet), 1);

		while (oneWire.read_bit() == true)                 // Wait for not busy
			delay(1);

		// Step 5/5 - Get Temperature
		delay(13);                                         // Delay between Messages 14-bit = 6.35ms
		read_packet[0] = Read_Data_Stop;                   // Command ("read Data With Stop")
		read_packet[1] = 0x81;                             // I2C slave to address
		read_packet[2] = 0x02;                             // Number of data bytes to read
		CRC16 = ~oneWire.crc16(&read_packet[0], 3);		   // Generate CRC & Invert
		read_packet[4] = CRC16 >> 8;                       // Most significant byte of 16 bit CRC
		read_packet[3] = CRC16 & 0xFF;                     // Least significant byte of 16 bit CRC
		oneWire.reset();
		oneWire.select(UUID);
		oneWire.write_bytes(read_packet, sizeof(read_packet), 1);

		while (oneWire.read_bit() == true)                 // Wait for not busy
			delay(1);

		if (oneWire.read() == 0) {
			oneWire.read_bytes(tdata, sizeof(tdata));
			payload.temperature[2 * i] = tdata[0];
			payload.temperature[2 * i + 1] = tdata[1];

			//debug
			uint16_t temp = ((payload.temperature[2 * i] & 0xff) << 8) | (payload.temperature[2 * i + 1]);
			uint16_t humi = ((payload.humidity[2 * i] & 0xff) << 8) | (payload.humidity[2 * i + 1]);
			float convh = (humi);// / 0xFFFF)*100;
			float convt = (temp);// / 0xFFFF)*165-40;
			convh = (convh / 0xFFFF) * 100;
			convt = (convt / 0xFFFF) * 165 - 40;
			Serial.print("RH: ");
			Serial.print(convh);
			Serial.print(", T: ");
			Serial.print(convt);
			Serial.println();

		}
		else {
			payload.temperature[2 * i] = 0xff;
			payload.temperature[2 * i + 1] = 0xff;
		}
	}
	digitalWrite(HT_Power, LOW);  // Shut down sensors
	payload.sensor_RSSI = driver.lastRssi();
	payload.sensor_SNR = driver.lastSNR();
	if (!manager.sendtoWait((const void*)(&payload), sizeof(payload), SERVER_ADDRESS)) {
		Serial.println(F("sendtoWait failed"));
	}
	else {
		Serial.print(F("Sent OK, Last RSSI: "));
		Serial.print(payload.sensor_RSSI);
		Serial.print(F(", Last SNR: "));
		Serial.println(payload.sensor_SNR);
	}
	Serial.flush();// delay(8); // Allow serial to finish writing TODO serial.flush()?
	driver.sleep();

	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

	digitalWrite(HT_Power, HIGH); // Power up the sensor string
}