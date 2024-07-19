#include <SPI.h>              // include libraries
#include <LoRa.h>            
#include <Wire.h>

// CONSTANTS FOR I2C LINK
#define SLAVE_ADDRESS 0x4a
#define REQUEST_PIN  1

// VARIABLES FOR LORA
const int csPin = 9;          // LoRa radio chip select
const int resetPin = 2;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 30000;          // interval between sends

// VARIABLES FOR I2C LINK
String incoming_string = "";
const String DISASTER = "Disaster", COORDINATES = "Coordinates";
uint8_t Tx_bytes[2] = {8,5};

// Variable for temperature sensor
const int inputPin = A2;

// Setting up serial terminal to manage debugging.
void setupSerial(){
  // Init serial.
  Serial.begin(9600);                  
  while (!Serial);
  Serial.println("Flying station initializing.");
}

// Setting up LoRa parameters.
void setupLora(){
  // Init LoRa.
  // override the default CS, reset, and IRQ pins (optional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  Serial.println("LoRa init succeeded.");
}

void setupTempSensor(){
    pinMode(inputPin,INPUT);
}

// Function called when master send reading request. Put down flag pin.
void requestEventI2C(){
    Wire.write(Tx_bytes, sizeof(Tx_bytes));
    Serial.println("Data sent to master.");
//    Tx_bytes[0]++;
//    Tx_bytes[1]++;
    digitalWrite(REQUEST_PIN, LOW);
}

// Setting up I2C parameters. Including the flag PIN. The flag pin is used to signalize the master that the slave has data to send.
void setupI2C(){
    pinMode(REQUEST_PIN, OUTPUT);
    digitalWrite(REQUEST_PIN, LOW);
    Wire.begin(0x4a);
    Wire.onRequest(requestEventI2C);
    Serial.println("I2C init succeded.");
}

void setup() {
  setupSerial();
  setupLora();
  setupTempSensor();
  setupI2C();
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

// Checking if the message sent by the ground station contains "Disaster". If so, transmit it to the Argon.
void checkforCommand(String incoming_string){
    incoming_string.trim();
    if(incoming_string.length() < DISASTER.length()) return;
    String aux = "";
    for(int i = 0; i < DISASTER.length(); i++){
      aux += incoming_string[i];
    }
    if(aux == DISASTER){ // "Disaster" is the prefix of the incoming string.
      Serial.println("Reading request done. ");
      digitalWrite(REQUEST_PIN, HIGH);
      aux = "";
      for(int i = 8; i < incoming_string.length(); i++){
        if(incoming_string[i] == '\n' || incoming_string[i] == '\r') continue;
        aux += incoming_string[i];
      }
      if(aux == "Tsunami"){
        Tx_bytes[0] = 0;
        Tx_bytes[1] = 0;
      }
      else if(aux == "Fire"){
        Tx_bytes[0] = 1;
        Tx_bytes[1] = 1;        
      }
      else if(aux == "Earthquake"){
        Tx_bytes[0] = 2;
        Tx_bytes[1] = 2;
      }
      else{
        Tx_bytes[0] = 3;
        Tx_bytes[1] = 3;
      }
    }
    else if(incoming_string == COORDINATES){
        sendCoordinates();
    }
    return;
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.print("Received from 0x" + String(sender, HEX));
  //Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  //Serial.println("Message length: " + String(incomingLength));
  Serial.println(" the message: " + incoming);
  //Serial.println("RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  Serial.println();
  checkforCommand(incoming);
}

void sendCoordinates(){
    float bitsVal = analogRead(inputPin); // Analog value read as voltage is automatically converted to bits. 10 bits -> 0b to 1023b.
    float tempVal = bitsVal*330/1023;
    String string_tempVal = String(tempVal);
    String message = "GPS: LAT = 44.8064619672657, LONG = -0.605359040369402 \n Ambient Temperature = " + string_tempVal + "°C";
    sendMessage(message);
}

void loop() {
  // Being a Tx every 30 seconds.
  if ( (millis() - lastSendTime) > interval) {
    String message = "GPS coordinates";   // send a message
    sendCoordinates();
    Serial.println("Sending " + message + " at " + millis());
    lastSendTime = millis();            // timestamp the message
  }

  // Being a Rx all the time.
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}
