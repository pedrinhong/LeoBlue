/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>

const int csPin = 9;          // LoRa radio chip select
const int resetPin = 2;       // LoRa radio reset
const int irqPin = 1;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 30000;          // interval between sends

// Variables to manage input.
int commandType = 0;
String incomingString = "", command = "", typeDisaster = "", aux;
bool firstWord = true, validCommand = false;

#define DISASTER 1
#define COORDINATES 2

void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Duplex");

  // override the default CS, reset, and IRQ pins (optional)
  //LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(868E6)) {             // initialize ratio at 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  // Being Tx when demanded on serial monitor.
  if (Serial.available() > 0) {
    manageInput();
    if(validCommand){
      if(commandType == DISASTER){
        sendMessage(command + typeDisaster); 
      }
      else if(commandType == COORDINATES){
        sendMessage(command);
      }
    }
  }
  
  // Being a Rx all the time.
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
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
}

void manageInput(void){
    incomingString = Serial.readString();
    aux = "";
    firstWord = true;
    validCommand = false;
    for(int i = 0; i < incomingString.length(); i++){
        char carac = incomingString[i];
        if(carac == ' ' || carac == '\n'){
          if(firstWord){
            if(aux == "Disaster"){
              validCommand = true;
              command = aux;
              commandType = DISASTER;
            }
            else if(aux == "Coordinates"){
              validCommand = true;
              command = aux;
              commandType = COORDINATES;
              Serial.println("Command valid");
            }
            else{
              Serial.println("Commmand not valid.");
              break;
            }
            firstWord = false;
          }
          else{
            typeDisaster = aux;
            Serial.println("Command valid. Disaster type: " + typeDisaster);
            break;
          }
          aux = "";
        }
        else{
          //if(firstWord && aux.length() > strlen("Disaster")) break;
          aux += incomingString[i];
        }
    }
}
