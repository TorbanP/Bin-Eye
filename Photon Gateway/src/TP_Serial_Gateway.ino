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

 void setup() {

   Serial.begin(115200);
   while (!Serial) ; // Wait for serial port to be available
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

        if(Particle.publish(event, data, 60, PRIVATE)){
          Serial.println("Publish Success");
        } else {
          Serial.println("Publish Failed");
        }
     }
   }
 }
