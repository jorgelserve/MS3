// Adapte from LibAPRS to SimpleRadioV2 By:
// Julian Galvez Serna, 

// -----------------------------Librerias-----------------------------------
// Include LibAPRS
#include <LibAPRS.h>
#define ADC_REFERENCE REF_5V
// You can also define whether your modem will be running with an open squelch radio:
#define OPEN_SQUELCH false



// -----------------------------APRS Packet Decoder-----------------------------------

// called from within an interrupt.  FAST
boolean gotPacket = false;
AX25Msg incomingPacket;
uint8_t *packetData;
void aprs_msg_callback(struct AX25Msg *msg) {
  // If we already have a packet waiting to be
  // processed, we must drop the new one.
  if (!gotPacket) {
    // Set flag to indicate we got a packet
    gotPacket = true;

    // The memory referenced as *msg is volatile
    // and we need to copy all the data to a
    // local variable for later processing.
    memcpy(&incomingPacket, msg, sizeof(AX25Msg));

    // We need to allocate a new buffer for the
    // data payload of the packet. First we check
    // if there is enough free RAM.
    if (freeMemory() > msg->len) {
      packetData = (uint8_t*)malloc(msg->len);
      memcpy(packetData, msg->info, msg->len);
      incomingPacket.info = packetData;
    } else {
      // We did not have enough free RAM to receive
      // this packet, so we drop it.
      gotPacket = false;
    }
  }
}

//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz Setup zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz

void setup() {
  // Set up serial port
  Serial.begin(115200);

  // Radio Ports Configurations
  pinMode(RxSelection,INPUT);
  pinMode(TxSelection,INPUT);
  pinMode(RxD,INPUT);
  pinMode(A6,INPUT);


  // APRS lib Setup
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);     // Initialise APRS library - This starts the modem
  APRS_setCallsign("5K4MS3", 1);              // Minimum configure, callsign and SSID
  // Others configurations
  APRS_setDestination("APZMDM", 0);           // Destination identifier
  APRS_setPath1("WIDE1", 1);                  // Path parameters are set to sensible values by
  APRS_setPath2("WIDE2", 2);                  //  default, but this is how you can configure them:
  APRS_setPreamble(350);                      // Preamble
  APRS_setTail(50);                           // Tail
  APRS_useAlternateSymbolTable(false);        // Normal or alternate symbol table
  APRS_setSymbol('n');                        // Symbol you want to use
  
  APRS_printSettings(); // We can print out all the settings
  
  Serial.print(F("Free RAM:     ")); 
  Serial.println(freeMemory());
}

void locationUpdateExample() {
  // Let's first set our latitude and longtitude.
  // These should be in NMEA format!
  APRS_setLat("5530.80N");
  APRS_setLon("01143.89E");
  
  // We can optionally set power/height/gain/directivity
  // information. These functions accept ranges
  // from 0 to 10, directivity 0 to 9.
  // See this site for a calculator:
  // http://www.aprsfl.net/phgr.php
  // LibAPRS will only add PHG info if all four variables
  // are defined!
  APRS_setPower(2);
  APRS_setHeight(4);
  APRS_setGain(7);
  APRS_setDirectivity(0);
  
  // We'll define a comment string
  char *comment = "LibAPRS location update";
    
  // And send the update
  APRS_sendLoc(comment, strlen(comment));
  
}

void messageExample() {
  // We first need to set the message recipient
  APRS_setMessageDestination("AA3BBB", 0);
  
  // And define a string to send
  char *message = "Hi there! This is a message.";
  APRS_sendMsg(message, strlen(message));
  
}

// Here's a function to process incoming packets
// Remember to call this function often, so you
// won't miss any packets due to one already
// waiting to be processed
void processPacket() {
  if (gotPacket) {
    gotPacket = false;
    
    Serial.print(F("Received APRS packet. SRC: "));
    Serial.print(incomingPacket.src.call);
    Serial.print(F("-"));
    Serial.print(incomingPacket.src.ssid);
    Serial.print(F(". DST: "));
    Serial.print(incomingPacket.dst.call);
    Serial.print(F("-"));
    Serial.print(incomingPacket.dst.ssid);
    Serial.print(F(". Data: "));

    for (int i = 0; i < incomingPacket.len; i++) {
      Serial.write(incomingPacket.info[i]);
    }
    Serial.println("");

    // Remeber to free memory for our buffer!
    free(packetData);

    // You can print out the amount of free
    // RAM to check you don't have any memory
    // leaks
    // Serial.print(F("Free RAM: ")); Serial.println(freeMemory());
  }
}

boolean whichExample = false;
void loop() {
  
  delay(1000);
  if (whichExample) {
    locationUpdateExample();
  } else {
    messageExample();
  }
  whichExample ^= true;

  delay(500);
  processPacket();
}
