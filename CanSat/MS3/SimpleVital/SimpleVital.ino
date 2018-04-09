//////////////////////////////////////////////////////////////// Codigo Simple Vital ////////////////////////////////////////////////////////////////
/* Pendientes
  - Organizar codigo
  - Agregar rutina de chequeo inicial
  - Agregar sensor analogo radio trama
  - Sensor analogo pcb trama
  - vector puertos mediciones analogas
*/
//////////////////////////////////////////////////////////////// Librerias ////////////////////////////////////////////////////////////////
// se incluyen librerias
#include "ConfiVital.h"

//////////////////////////////////////////////////////////////// Setup ////////////////////////////////////////////////////////////////
void setup() {
  //////////////////////////////////////////// Pitido inicial
  pinMode(buzzer_PIN, OUTPUT);
  pitar(500);

  ////////////////////////////////////////////
#if MS2Compatible
  pinMode(8, INPUT); // Importante Configurar pin 8 como entrada(va al radio, pero no tiene el timer adecuado)
  pinMode(7, INPUT); // Bug de pcb vital MS2 fabricado.
#endif

  //////////////////////////////////////////// Comunication
  Serial.begin(DEBUG_SERIAL_SPEED);

#if RadioSerial
  Serial2.begin(115200);
#endif

  Serial3.begin(GPS_BAUDRATE);

  //////////////////////////////////////////// Serial init
#if MS2Compatible
  Serial.println("Serial Debuggin Started - Hi from SimpleVital 2");
#else
  Serial.println("Serial Debuggin Started - Hi from SimpleVital 3");
#endif

#if RadioSerial
  pinMode(4, OUTPUT);
  // reset Radio
  digitalWrite(4, LOW);
  delay(50);
  pinMode(4, INPUT);
#endif

  // Se configura sistema despligue paneles
  pinMode(pinPanel0, OUTPUT);
  digitalWrite(pinPanel0, LOW);
  pinMode(pinPanel1, OUTPUT);
  digitalWrite(pinPanel1, LOW);

  // se configura pin voltaje bateria
  pinMode(pinVolBat1, INPUT);
  // se configura pin voltaje bateria
  pinMode(pinTempRad, INPUT);
  pinMode(pinTempPCB, INPUT);

  // se configuran los leds indicadores
  pinMode(LedSD_PIN, OUTPUT);
  //pinMode(LedRUN_PIN, OUTPUT);
  //pinMode(LedTX_PIN, OUTPUT);



#if DEBUG < 2
  Serial.print("Initializing I2C Bus...");
#endif
  Wire.begin();

#if DEBUG < 2
  Serial.println(" OK.");
#endif

  //////////////////////////////////////////// Sensors Inicialization
  ///////////////////// initialize IMU
  Serial.print("Initializing IMU...");
  Serial.println(accelgyro.testConnection() ? "OK" : "MPU9255 connection failed");

  ///////////////////// initialize Barometers
  Serial.println("Initializing Barometers");
#if not MS2Compatible
  Serial.print("BMP280B test... ");
  if (!baro_BMP280B.begin()) { // default dir 0x77
    Serial.println("Could not find a valid BMP280B sensor, check wiring!");
  } else {
    Serial.println(" OK.");
  }


  Serial.print("BMP280T test... ");
  if (!baro_BMP280T.begin(0x76)) {
    Serial.println("Could not find a valid BMP280T sensor, check wiring!");
  } else {
    Serial.println(" OK.");
  }
#else
  Serial.print("BMP180 test... ");
  Barometer.init();
  Serial.println(" OK.");
#endif

  ///////////////////// Setup AFSK - Radio analalogo
#if not RadioSerial
  Serial.print("Setup Afsk...");
  afsk_setup();
  Serial.println(" OK.");
#endif

  ///////////////////// Setup GPS
  Serial.print("Setup Gps...");
  gps_setup();
  Serial.println(" OK.");
  // Se reseteo del GPS
  pinMode(gpsRESET_PIN, OUTPUT);
  digitalWrite(gpsRESET_PIN, HIGH);
  delay(50);
  digitalWrite(gpsRESET_PIN, LOW);


  // Do not start until we get a valid time reference for slotted transmissions.
  if (APRS_SLOT >= 0) {
#if DEBUG < 2
    Serial.println("Getting time from Gps, for slotted transmission...");
#endif

    uint32_t endMillis1 = millis() + GPS_timeout;
    do {
      power_save(); // se duerme hasta que llege un dato por serial
      read_gps();
    } while (!gps_seconds && endMillis1 > millis());

    next_aprs = millis() + 1000 * (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
  } else {
    next_aprs = millis();
  }

#if DEBUG < 2
  Serial.print("N-APRS:");
  Serial.println(next_aprs);
#endif

#if DEBUG < 2
  Serial.print("Initializing SD card...");
  bool sd_estado = false;
  sd_estado = SD.begin(chipSelect);
  //Serial.println(sd_estado);
  Serial.println(sd_estado ? "OK" : "Card failed, or not present");
  if (sd_estado) {
    sd_ok = true;
  }

#else
  if (SD.begin(chipSelect)) {
    sd_ok = true;
  }
#endif

#if RadioSerial
  Serial.println("Initializing Radio...");
  Serial2.println("MSimple3-ON");
  delay(1000);
  revisarRadio();
#endif

  // Pitido final
  pitar(100);

  // Se verifica si esta en modo carga esperando una c por serial durante 5 segundos
  Serial.println("Presione c para iniciar modo carga");
  unsigned long tiempoConfCarga = millis() + 5000;
  while (tiempoConfCarga > millis()) {
    char comando = Serial.read();
    if (comando == 'c') {
      Serial.println("Modo Carga activado.... Presione una tecla para iniciar modulo.");
      while (Serial.available() < 1) {
        //delay(500);
        power_save();
        revisarRadio();
      }
    }
  }


}

//////////////////////////////////////////////////////////////// Loop ////////////////////////////////////////////////////////////////
void loop() {
  // verificamos mensajes seriales del radio y los mostramos en pantalla:
  revisarRadio();

  ////////////////////////////// Apogeo del sistema //////////////////////////////////////
  apogeoSistema();

  /////////////////////////////// Liberación de paneles /////////////////////////////////////
  //liberarPaneles();

  ////////////////////////////////////////////////////////Temperatura y Humedad //////////////////////////////////////////////
  ////// Sensor i2C /////////////
  float tempi2cV = PCBTemperature(0); // lectura Temp PCB Vital
  tempi2cSDV = (float)tempi2cV * 100;

  float tempi2cG = PCBTemperature(1); // lectura Temp PCB Gases
  tempi2cSDG = (float)tempi2cG * 100;

  // se carga temp I2C a Trama radio
  tempPromPCB = (tempi2cSDV + tempi2cSDG) / 2;

#if not RadioSerial
  load_tempi2c(tempi2cV);
#endif

  ///// Sensor SHT11 //////////
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();

#if not RadioSerial
  load_tempC(tempC);
  load_humidity(humidity);
#endif

  tempSD = (int)(tempC * 100);
  humSD = (int)(humidity * 100);

  ////////////////////////////////////////////////////////Sensor de Gases //////////////////////////////////////////////
  medirGases04();
#if not MS2Compatible
  medirGases05();
#endif
  ///////////////////////////////////////////////////// Se lee el Barometro /////////////////////////////////////////////////
  medirBarometroB();
#if not MS2Compatible
  medirBarometroT();
#endif
  ///////////////////////////////////////////////////// Se lee la IMU /////////////////////////////////////////////////
#if DEBUG <1
  Serial.print("RI");
#endif

  getAccel_Data();
  getGyro_Data();
  getCompassData_calibrated(); // compass data has been calibrated here
  getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();

  ///////////////////////////////////////////////////// Se lee Voltaje Bateria /////////////////////////////////////////////////
  int volBat1 = analogRead(pinVolBat1);
  voltajeBateria0 = 5.0 * volBat1 / 1024 * 1000;
  //Serial.print("VoltBat: ");Serial.print(voltajeBateria0);
#if not RadioSerial
  load_volt(voltajeBateria0);
#endif
  ///////////////////////////////////////////////////// Se lee Temperatura Radio /////////////////////////////////////////////////
  int volTR = analogRead(pinTempRad);
  float tempRad = (volTR * 5.0) / 1024.0;
  tempRad = tempRad - 0.5;
  tempRad = tempRad / 0.01;
  //load_tempADC(tempRad);
  //Serial.print("TempR: ");Serial.print(tempRad);load_volt(voltajeBateria0);

  ///////////////////////////////////////////////////// Se lee Temperatura PCB /////////////////////////////////////////////////
  int volTP = analogRead(pinTempPCB);
  tempPCB = (5.0 * volTP / 1024 * 1000 - 500) * 10;
  //Serial.print("TempPCB: ");Serial.print(tempPCB);
  //load_volt(voltajeBateria0);

  //////////////////////////////////////////////////// Sensores de temperatura analogos externos ///////////////////////////////////

  int tempADC1 = analogRead(pinTemp1);
  int tempADC2 = analogRead(pinTemp2);
  int tempADC3 = analogRead(pinTemp3);

  float tempext1 = tempADC1 * 5.0 / 1024.0;
  tempext1 = tempext1 - 0.5;
  tempext1 = tempext1 / 0.01;
  float tempext2 = (tempADC2 * 5.0) / 1024.0;
  tempext2 = tempext2 - 0.5;
  tempext2 = tempext2 / 0.01;
  float tempext3 = (tempADC3 * 5.0) / 1024.0;
  tempext3 = tempext3 - 0.5;
  tempext3 = tempext3 / 0.01;

  tempext1SD = tempext1;
  tempext2SD = tempext2;
  tempext3SD = tempext3;

  float prom = tempext1SD + tempext2SD + tempext3SD;

  prom = prom / 3;
#if not RadioSerial
  load_tempADC(prom);
#endif
  ///////////////////////////////////////////////////// Se lee el GPS /////////////////////////////////////////////////
#if DEBUG <1
  Serial.print("RG");
#endif

  read_gps();

#if DEBUG <1
  Serial.print("T");
  Serial.print(gps_time);
  Serial.print("La");
  Serial.print(gps_lat * 10000);
  Serial.print("Lo");
  Serial.print(gps_lon * 10000);
  Serial.print("C");
  Serial.print(gps_course);
  Serial.print("S");
  Serial.print(gps_speed);
  Serial.print("A");
  Serial.print(gps_altitude);
#endif



  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {
    //digitalWrite(LedTX_PIN, HIGH);

#if not RadioSerial
    if (band_transmission == 0) {
      aprs_send();
      band_transmission = 1;
    } else {
      aprs_send_variables();
      band_transmission = 0;
    }
#else
    enviarTramaIRadio();
#endif

    //digitalWrite(LedTX_PIN, LOW);

    if (APRS_SLOT >= 0) {
      next_aprs = millis() + 1000 * (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
    } else {
      next_aprs += APRS_PERIOD * 1000L;
    }

#if not RadioSerial

    while (afsk_flush()) {
      power_save();
    }
#ifdef DEBUG_MODEM
    // Show modem ISR stats from the previous transmission
    afsk_debug();
#endif

#endif

  } else {
    // Discard GPS data received during sleep window
    //while (Serial.available()) {
    //  Serial3.read();
    //}
  }

#if DEBUG <1
  Serial.print("2SD.");
#endif
  guardarDatosSD();
  //Serial.print(datos);

#ifdef DEBUG <1
  Serial.println();
#endif

  //power_save(); // Incoming GPS data or interrupts will wake us up

  // Enviamos Paquete a al radio:

  mostrarMediciones();

} // fin loop

