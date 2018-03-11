
// we include libraries
#include "ConfiRadio.h"

char band_transmission = 0;

unsigned int gps_secondss = 0;

void setup() {
  //////////////////////////////////////////// Comunication
  // Serial Port Configuration
  Serial.begin(DEBUG_SERIAL_SPEED);

  //////////////////////////////////////////// Radio Ports Configuration
  // analog
  pinMode(AF_PIN, INPUT);
  pinMode(RSSI_PIN, INPUT);
  // digital 
  pinMode(RXD_PIN, INPUT);
  //pinMode(ETX_PIN, INPUT); // RX Disabled

  //////////////////////////////////////////// Sensors Inicialization
  //pinMode(LED_PIN, OUTPUT);
  //pin_write(LED_PIN, LOW);
  next_aprs = millis();

#if DEBUG < 2
  Serial.print("N-APRS:");
  Serial.println(next_aprs);
#endif

  // TODO:
  // - 
  
}

void loop() {
  ///////////////////////////////////////////////////// Init of SimpleRadioV2 Main Code ///////////////////////////////////////////////////// 

  ///////////////////////////////////////////////////// Check Radio Ports
  int audioImput = analogRead(AF_PIN);
  float voltAudioImput = (0.0048828125 * audioImput);
  Serial.print(voltAudioImput);
  Serial.print("\t");
  
  int signalImput = analogRead(RSSI_PIN);
  float voltSignalImput = (0.0048828125 * signalImput);
  Serial.print(voltSignalImput);
  Serial.print("\t");

  int RXBitcode = digitalRead(RXD_PIN);
  Serial.print(RXBitcode);
  Serial.print("\t");
  
  ///////////////////////////////////////////////////// Check Int/Ext the Temperature Sensors
  int tempExt = analogRead(pinTempExt);
  float externalTemp = (5.0 * tempExt / 1024 * 1000 - 500) / 10;
  load_volt(externalTemp);

  

  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {
    Serial.print("Tx...");

    aprs_send();


    /*
    if (band_transmission == 0) {
    aprs_send();
      band_transmission = 1;
    } else {
      aprs_send_variables();
      band_transmission = 0;
    }
    
    //digitalWrite(LedTX_PIN, LOW);

    if (APRS_SLOT >= 0) {
      //next_aprs = millis() + 1000 * (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
    } else {
      next_aprs += APRS_PERIOD * 5000L;
    }
    */
    
    while (afsk_flush()) {
      power_save();
    }
    
    next_aprs += APRS_PERIOD * 2000L;
    Serial.print("Rx.");

#ifdef DEBUG_MODEM
    // Show modem ISR stats from the previous transmission
    afsk_debug();
#endif

  } else {
    // Discard GPS data received during sleep window
    //while (Serial.available()) {
    //  Serial3.read();
    //}
  }

  // modificamos el tiempo de transmision pin21
  //qAPRS_PERIOD = 2 ;
  /*
    if(digitalRead(pinPeriodo)){
     Serial.print(" 10s ");
     APRS_PERIOD = 10;
    }else{
     Serial.print(" 2s ");
     APRS_PERIOD = 2;
    }
  */

#ifdef DEBUG <1
  Serial.println();
  delay(500);
#endif

  //power_save(); // Incoming serial data or interrupts will wake us up

  //*/

}


