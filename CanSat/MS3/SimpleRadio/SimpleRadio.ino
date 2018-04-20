// Adapte from LibAPRS to SimpleRadioV2 By:
// Julian Galvez Serna,

// ----------------------------- Librerias -----------------------------------
// Include LibAPRS
#include <LibAPRS.h>
#define ADC_REFERENCE REF_5V
// You can also define whether your modem will be running with an open squelch radio:
#define OPEN_SQUELCH false

// ----------------------------- APRS Packet Decoder -----------------------------------
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


// ----------------------------- Ports Definitions -----------------------------------
/*#define RxS_pin 12   // Pin RX Selection (MISO) PB4
  #define TxS_pin 11   // Pin TX Selection (MOSI) PB3
  #define RxD_pin 8   // Pin Digital RX PB0
  #define A6_pin A6   // Pin RSSI Radio
  (RAD-TX-LED: PB1, RAD-RX-LED: PB2) */

// ----------------------------- Global variables -----------------------------------
char *message = "";

// ----------------------------- Definitions -----------------------------------
#define DebugOn 0    //On(1)/Off(0) Serial Debuggin Data

//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz Setup zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz

void setup() {
  // Set up serial port
  Serial.begin(115200);

  // --------------------- To Do
  /* Add Rssi to control de Rx led to Turn On only whit strong signals
  */

  // --------------------- APRS lib Setup
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);     // Initialise APRS library - This starts the modem
  APRS_setCallsign("5K4MS3", 11);              // Minimum configure, callsign and SSID
  // Others configurations
  APRS_setDestination("APZMDM", 0);           // Destination identifier
  APRS_setPath1("WIDE1", 1);                  // Path parameters are set to sensible values by
  APRS_setPath2("WIDE2", 2);                  //  default, but this is how you can configure them:
  APRS_setPreamble(350);                      // Preamble
  APRS_setTail(50);                           // Tail
  APRS_useAlternateSymbolTable(false);        // Normal or alternate symbol table
  APRS_setSymbol('n');                        // Symbol you want to use

#if DebugOn
  APRS_printSettings(); // We can print out all the settings
  Serial.print(F("Free RAM:     "));
  Serial.println(freeMemory());
#endif

  // SimpleRadio V2 Ready message to vital
  Serial.println("RR");
} // End Setup

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
  char *comment = " - LibAPRS location update";

  // And send the update
  APRS_sendLoc(comment, strlen(comment));
}

void messageExample() {
  APRS_sendSMsg(message, strlen(message)); // Send Message
}

// Here's a function to process incoming packets
// Remember to call this function often, so you
// won't miss any packets due to one already
// waiting to be processed

void processPacket() {
  if (gotPacket) {
    gotPacket = false;
    /*
      Serial.print(F("Received APRS packet. SRC: "));
      Serial.print(incomingPacket.src.call);
      Serial.print(F("-"));
      Serial.print(incomingPacket.src.ssid);
      Serial.print(F(". DST: "));
      Serial.print(incomingPacket.dst.call);
      Serial.print(F("-"));
      Serial.print(incomingPacket.dst.ssid);
      Serial.print(F(". Data: "));
    */
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

//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz Loop (Main code) zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
unsigned long TimeR = 0;

void loop() {

  //delay(1000);
  /*if (TimeR + 5000 < millis()) {
    TimeR = millis();
    // Trasmision
    if (whichExample) {
      locationUpdateExample();
    } else {
      messageExample();
    }
    whichExample ^= true;
    }
  */
  processPacket(); // Print in console the incoming data

  //enviarDatosSeriales();
  procesarTramasLargas();
} // fin loop

//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz Funciones Principales zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz

inline void enviarDatosSeriales() {
  // Check for Serial Comunication
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    //readString += c; //makes the string readString
    if (c == 'M') {
      //lectura recursiva de los datos seriales llegan de a 63
      String data2Send = Serial.readString();
      /*if (data2Send.length() == 63) {
        delay 10;
        while (Serial.available() > 0) {
          c = Serial.readString();
          data2Send += c; //makes the string readString
        }
        }*/
      // A message will be send, capture de string data
      /*while (Serial.available() > 0){
        c = Serial.readString();
        data2Send += c; //makes the string readString
        }
        //Serial.println(data2Send);
        // /225151h0611.91N/07534.72WO073/000A1526B417C4115D27E27636F13189G-7520H2228I3091J3422K5599L3350M9366
        // /225201h0611.91N/07534.72WO073/000A1526T323P19210A1481V0

        //data2Send.toCharArray(message,80);
        //message = "hola";.c_str()*/
      message = data2Send.c_str();
      APRS_sendSMsg(message, strlen(message)); // Send Message
    }
  }
}


inline void procesarTramasLargas() {
  String data2Send = "";
  bool badPacket = true;
  // Check for Serial Comunication
  if (Serial.available() > 0) {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == 'M') {
      Serial.print('n'); // se le indica a la vital que envie el primer paquete

      unsigned long tiempoFinMensaje = millis() + 5000;
      // Recibimos los la trama a enviar por paquetes de a 63 o menos
      while (tiempoFinMensaje > millis()) {

        if (Serial.available() > 0) {
          // lectura recursiva de los datos seriales, llegan de a 63  o menos
          String dataR = Serial.readString(); // Se lee el primer paquete

          data2Send += dataR;   // Agrupamos Paquetes

          if (dataR.length() == 63) {
            //delay(500);
            Serial.print('n'); // Enviamos de nuevo una n, preguntando por el posible siguiente paquete
          } else {
            badPacket = false;
            // el paquete es mas pequeño, no hay mas paquetes, se procede a enviar el mensaje
            break;
          }

        } // Hay datos seriales

      } // Fin while

      // Se envia mensaje
      if (!badPacket) {
        message = data2Send.c_str();
        APRS_sendSMsg(message, strlen(message)); // Send Message
      }

    } // Fin M detectada

  } // Fin datos seriales disponibles

} // fin funcion procesar


