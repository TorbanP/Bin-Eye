/*
 * Project TP_Serial_Gateway
 * Description:
 * Author:
 * Date:
 */
 #include <RHReliableDatagram.h>
 #include <RH_RF95.h>
 #include <SPI.h>

 #define SERVER_ADDRESS   0
 #define EVENT_LEN        8
 // Singleton instance of the radio driver
 RH_RF95 driver;

 // Class to manage message delivery and receipt, using the driver declared above
 RHReliableDatagram manager(driver, SERVER_ADDRESS);

//temporary for testing
double temperature[6];
double humidity[6];


 void setup() {
   Particle.variable("temp_1", &temperature[0], DOUBLE);
   Particle.variable("temp_2", &temperature[1], DOUBLE);
   Particle.variable("temp_3", &temperature[2], DOUBLE);
   Particle.variable("temp_4", &temperature[3], DOUBLE);
   Particle.variable("temp_5", &temperature[4], DOUBLE);
   Particle.variable("temp_6", &temperature[5], DOUBLE);
   Particle.variable("humid_1", &humidity[0], DOUBLE);
   Particle.variable("humid_2", &humidity[1], DOUBLE);
   Particle.variable("humid_3", &humidity[2], DOUBLE);
   Particle.variable("humid_4", &humidity[3], DOUBLE);
   Particle.variable("humid_5", &humidity[4], DOUBLE);
   Particle.variable("humid_6", &humidity[5], DOUBLE);

   Serial.begin(115200);
   while (!Serial); // Wait for serial port to be available
   if (!manager.init()){
     Serial.println("init failed");
   }
     driver.setTxPower(20, false);
     driver.setFrequency(915.0);
 }

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void loop() {
   if (manager.available()) {
     // Wait for a message addressed to us from the client
     uint8_t len = sizeof(buf);
     uint8_t from;
     char data[255];
     data[0] = '\0';

     if (manager.recvfromAck(buf, &len, &from)) {
       Serial.print("Message from: 0x");
       Serial.print(strlen(data));
       Serial.print(", Event: ");
       // pull out the event name
       char event[EVENT_LEN+1]; event[EVENT_LEN] = '\0';
       for (uint8_t i = 0; i < EVENT_LEN; i++){
          event[i] = buf[i];
          Serial.print(event[i]);
         }
      data[0] = '\0';
      Serial.print(data);
      //Print data
      Serial.print(", Data: ");
       for(uint8_t i = EVENT_LEN;i < len; i++){
        sprintf(data + strlen(data),"%02X", buf[i]);
       }

       Serial.print(data);

       Serial.print(", Last RSSI: ");
       int last_rssi = driver.lastRssi();
       Serial.print(last_rssi);
       // doesnt like the function. shitty particle ide cant tell me why
       //Serial.print(", SNR: ");
       //int last_snr = driver.lastSNR();
       //Serial.print(last_snr):

       Serial.println();

        if(Particle.publish(event, data, 60, PRIVATE)){
          Serial.println("Publish Success");
        } else {
          Serial.println("Publish Failed");
        }
        parse_message();
        Serial.println();
     }
   }
 }

//for testing
void parse_message(){
  // update humidity
  int offset = 17;
  uint16_t temp;
  for(int i = 0; i < 6; i++){
    temp = ((buf[offset] & 0xff) << 8) | (buf[offset + 1]);
    humidity[i] = temp;
    humidity[i] = (humidity[i]/0xFFFF)* 100;
    offset += 2;
  }
  //update temperature
  for(int i = 0; i < 6; i++){
    temp = ((buf[offset] & 0xff) << 8) | (buf[offset + 1]);
    temperature[i] = temp;
    temperature[i] = (temperature[i]/0xFFFF)* 165 - 40;
    offset += 2;
    Serial.print(", RH: ");
    Serial.print(humidity[i]);
    Serial.print(", T: ");
    Serial.print(temperature[i]);
  }
  Serial.println();
}
