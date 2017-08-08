/*
 Name:    Bineye_Bin_Master.ino
 Created: 7/20/2017 12:17:14 AM
 Author:  tpeterson
*/


#include <SPI.h>
#include <Arduino.h>
#include <OneWire.h>


// 1-wire commands
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
#define i2c_Address             0x80        // Address of the i2c temp and humidity sensor
#define get_humidity            0x01      // Command to take a Measurement
#define get_temperature         0x00

// Variables for 1-wire side and data

OneWire oneWire(8);
uint8_t config_packet[8];
uint8_t request_packet[6];
uint8_t read_packet[5];
uint8_t hdata[2];
uint8_t tdata[2];
uint16_t CRC16;

struct packet {
  uint8_t address[8];
  uint16_t humidity;
  uint16_t temperature;
  int8_t radio_temperature;
  int8_t sensor_RSSI;
  int8_t sensor_count;
} payload;

void setup() {
  Serial.begin(115200);
  while (!Serial);
}

void loop() {

  while (oneWire.search(payload.address)) {

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
    oneWire.select(payload.address);
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
    oneWire.select(payload.address);
    oneWire.write_bytes(request_packet, sizeof(request_packet), 1);

    while (oneWire.read_bit() == true)                 // Wait for not busy
      delay(1);

    // Step 3/5 - read Humidity
    delay(13);                                         // Delay between Messages 14-bit = 6.5ms
    read_packet[0] = Read_Data_Stop;                   // Command ("read Data With Stop")
    read_packet[1] = 0x81;                             // I2C slave to address
    read_packet[2] = 0x02;                             // Number of data bytes to read
    CRC16 = ~oneWire.crc16(&read_packet[0], 3);        // Generate CRC & Invert
    read_packet[4] = CRC16 >> 8;                       // Most significant byte of 16 bit CRC
    read_packet[3] = CRC16 & 0xFF;                     // Least significant byte of 16 bit CRC
    oneWire.reset();
    oneWire.select(payload.address);
    oneWire.write_bytes(read_packet, sizeof(read_packet), 1);

    while (oneWire.read_bit() == true)                 // Wait for not busy
      delay(1);

    if (oneWire.read() == 0) {               // store and convert the reply
      oneWire.read_bytes(hdata, sizeof(hdata));
      payload.humidity = hdata[0];
      payload.humidity <<= 8;
      payload.humidity |= hdata[1];
    }
    else {
      payload.humidity = 0xFFFF;
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
    oneWire.select(payload.address);
    oneWire.write_bytes(request_packet, sizeof(request_packet), 1);

    while (oneWire.read_bit() == true)                 // Wait for not busy
      delay(1);

    // Step 5/5 - Get Temperature
    delay(13);                                         // Delay between Messages 14-bit = 6.35ms
    read_packet[0] = Read_Data_Stop;                   // Command ("read Data With Stop")
    read_packet[1] = 0x81;                             // I2C slave to address
    read_packet[2] = 0x02;                             // Number of data bytes to read
    CRC16 = ~oneWire.crc16(&read_packet[0], 3);      // Generate CRC & Invert
    read_packet[4] = CRC16 >> 8;                       // Most significant byte of 16 bit CRC
    read_packet[3] = CRC16 & 0xFF;                     // Least significant byte of 16 bit CRC
    oneWire.reset();
    oneWire.select(payload.address);
    oneWire.write_bytes(read_packet, sizeof(read_packet), 1);

    while (oneWire.read_bit() == true)                 // Wait for not busy
      delay(1);

    if (oneWire.read() == 0) {
      oneWire.read_bytes(tdata, sizeof(tdata));
      payload.temperature = tdata[0];
      payload.temperature <<= 8;
      payload.temperature |= tdata[1];
    }
    else {
      payload.temperature = 0xFFFF;
    }
      float convh = (payload.humidity);// / 0xFFFF)*100;
      float convt = (payload.temperature);// / 0xFFFF)*165-40;
      convh = (convh / 0xFFFF) * 100;
      convt = (convt / 0xFFFF) * 165 - 40;
      Serial.print("Found UUID = ");
      for(int i = 0; i < 8; i++){
        if(payload.address[i] < 16) {
          Serial.print("0x0");
        } else {
          Serial.print("0x");
        }
        
        Serial.print(payload.address[i], HEX);
        Serial.print(" ");
      }
      
      Serial.print(", RH = ");
      Serial.print(convh);
      Serial.print("%, T = ");
      Serial.println(convt);
  }
  delay(500);
}
