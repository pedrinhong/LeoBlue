#include <Wire.h>

#define SLAVE_ADDRESS 0x4a
#define REQUEST_PIN  1

String incoming_string = "";
const String READY = "ready";

uint8_t Tx_bytes[2] = {8,5};
//uint8_t received_data[2];

void requestEvent(){
    Wire.write(Tx_bytes, sizeof(Tx_bytes));
    Serial.println("Data sent to master.");
    Tx_bytes[0]++;
    Tx_bytes[1]++;
    digitalWrite(REQUEST_PIN, LOW);
}

void setup(){
    pinMode(REQUEST_PIN, OUTPUT);
    digitalWrite(REQUEST_PIN, LOW);
    Wire.begin(0x4a);
    Wire.onRequest(requestEvent);
    Serial.begin(9600);
    while (!Serial); // Wait for Serial Monitor to open
    Serial.println("I2C Slave Initialized.");
}

void loop(){
    if(Serial.available() > 0){
      incoming_string = Serial.readString();
      incoming_string.trim();
      Serial.println(incoming_string);
      if(incoming_string == READY){
        digitalWrite(REQUEST_PIN, HIGH);
        //Serial.println("Reading request done."); 
      }
    }
}

//void receiveEvent(int howMany) {
//    for (int i = 0; i < howMany; i++) {
//        received_data[i] = Wire.read();
//    }
//    Serial.print("Received: ");
//    for (int i = 0; i < howMany; i++) {
//        Serial.print(received_data[i], HEX);
//        Serial.print(" ");
//    }
//    Serial.println();
//}
