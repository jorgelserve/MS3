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
  //////////////////////////////////////////// Comunication
  Serial.begin(DEBUG_SERIAL_SPEED);
#if RadioSerial
  Serial2.begin(115200);
#endif
  Serial3.begin(GPS_BAUDRATE);
  
  //////////////////////////////////////////// Serial init
#if DEBUG < 2
  Serial.println("Serial Debuggin Started - Hi from SimpleVital 3");
#endif

#if RadioSerial
  pinMode(4, OUTPUT);
  // reset Radio
  digitalWrite(4, LOW);
  delay(50);
  pinMode(4, INPUT);
#endif

  pinMode(7, INPUT);

  // Pitido inicial
  pinMode(buzzer_PIN, OUTPUT);
  pitar(500);

  // se configura sistema despligue paneles
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
#if DEBUG < 2
  Serial.print("Initializing IMU...");
#endif

  accelgyro.initialize();

#if DEBUG < 2
  Serial.println(" OK.");
#endif


#if DEBUG < 2
  Serial.print("Initializing Barometer...");
#endif

  //Barometer.init();
  Serial.println("BMP280 test");

  if (!baro_BMP280.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

#if DEBUG < 2
  Serial.println(" OK.");
#endif

#if DEBUG < 2
  Serial.print("Testing IMU Conection...");
  Serial.println(accelgyro.testConnection() ? "MPU9255 connection successful" : "MPU9255 connection failed");
#endif

#if DEBUG < 2
  Serial.println(" OK.");
#endif

#if DEBUG < 2
  Serial.print("Setup Afsk...");
#endif

  afsk_setup();

#if DEBUG < 2
  Serial.println(" OK.");
#endif

#if DEBUG < 2
  Serial.print("Setup Gps...");
#endif

  gps_setup();

#if DEBUG < 2
  Serial.println(" OK.");
#endif

  ///////////////////////////////////////////// Se realiza

  // Se resetea el GPS
  pinMode(gpsRESET_PIN, OUTPUT);
  digitalWrite(gpsRESET_PIN, HIGH);
  delay(50);
  digitalWrite(gpsRESET_PIN, LOW);
  delay(50);



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
  Serial.println(sd_estado);
  Serial.println(sd_estado ? "card initialized." : "Card failed, or not present");
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
#endif

  // Pitido final
  pitar(100);

  // Se verifica si esta en modo carga esperando una c por serial durante 5 segundos
  Serial.println("Presione c para iniciar modo carga");
  unsigned long tiempoConfCarga = millis()+ 5000;
  while(tiempoConfCarga > millis()){
    char comando = Serial.read();
    if (comando == 'c'){
      Serial.println("Modo Carga activado.... Presione una tecla para iniciar modulo.");
      while(Serial.available()<1){
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
  /*if (gps_altitude > alt_apogeo && altitud > alt_apogeo) {
    if (band_apogeo == 0) {
      fall_time = millis();
      band_apogeo = 1;
    }
    }
    if (band_apogeo == 1) {
    if ((millis() - fall_time) > apogeo_time) {
      if (gps_altitude < alt_apogeo && altitud < alt_apogeo) {
        ban_apogeo_active = 1;
      }
    }
    }
    if (ban_apogeo_active == 1) {
    if (band_buzzer == 1) {
      digitalWrite(buzzer_PIN, HIGH);
      band_buzzer = 0;
    } else {
      digitalWrite(buzzer_PIN, LOW);
      band_buzzer = 1;
    }
    }*/

  /////////////////////////////// Liberación de paneles /////////////////////////////////////
  if (band_paneles == 0) {

    weight = ((millis() / 1000) > panel_time) ? weight + 1 : weight;
    weight = (gps_altitude > alt_paneles) ? weight + 1 : weight;
    weight = (altitud > alt_paneles) ? weight + 1 : weight;

    if (weight == 2) {
      Serial.println("Cuenta Regresiva para Despliegue de Paneles, Iniciada...");
      int conteo = 10;
      for (int i = conteo; i > 0; i--) {
        delay(1000);
        Serial.print("T-0");
        Serial.println(i);
      }
      Serial.print("Desplegando...");
      digitalWrite(pinPanel0, HIGH);
      digitalWrite(pinPanel1, HIGH);
      delay(5000);
      digitalWrite(pinPanel0, LOW);
      digitalWrite(pinPanel1, LOW);
      Serial.print("OK");
      //Serial.println("Simple Pregunta: Desplego?");
      band_paneles = 1;
      pitar(5000);
    } else {
      weight = 0;
    }
    // si, que chimba, no bueno, intentemos de nuevo

  }//*/

  ////////////////////////////////////////////////////////Temperatura y Humedad //////////////////////////////////////////////
  ////// Sensor i2C /////////////
  float tempi2c = PCBTemperature();
  load_tempi2c(tempi2c);
  tempi2cSD = (float)tempi2c * 100;


  ///// Sensor SHT11 //////////
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  load_tempC(tempC);
  load_humidity(humidity);
  tempSD = (int)(tempC * 100);
  humSD = (int)(humidity * 100);

  ////////////////////////////////////////////////////////Sensor de Gases //////////////////////////////////////////////
  if (error_gas == 0) {
    error_gas = iniciarGases();
  }

  c[0] = gas.measure_NH3();
  load_NH3(c[0]);
  c[1] = gas.measure_CO();
  load_CO(c[1]);
  c[2] = gas.measure_NO2();
  load_NO2(c[2]);
  c[3] = gas.measure_C3H8();
  load_C3H8(c[3]);
  c[4] = gas.measure_C4H10();
  load_C4H10(c[4]);
  c[5] = gas.measure_CH4();
  load_CH4(c[5]);
  c[6] = gas.measure_H2();
  load_H2(c[6]);
  c[7] = gas.measure_C2H5OH();
  load_C2H5OH(c[7]);


  dim = (sizeof(c) / sizeof(float));

  for (int i = 0; i < (dim); i++) {
    cSD[i] = (int) (c[i] * 100);
  }

  ///////////////////////////////////////////////////// Se lee el Barometro /////////////////////////////////////////////////

  //temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); // Get the temperature, bmp180ReadUT MUST be called first
  temperature = baro_BMP280.readTemperature(); // Get the temperature, bmp180ReadUT MUST be called first

  //pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());// Get the presure
  pressure = baro_BMP280.readPressure();

  //altitud = Barometer.calcAltitude(pressure); // Uncompensated caculation - in Meters
  altitud = baro_BMP280.readAltitude(1013.25);

  load_temp(temperature);
  load_pres(pressure);
  load_alti(altitud);

#if DEBUG <1
  Serial.print("T");
  Serial.print(temperature);
  Serial.print("P");
  Serial.print(pressure);
  Serial.print("A");
  Serial.print(altitud);
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

#if DEBUG <1
  Serial.print("A");
  Serial.print(Axyz[0]);
  Serial.print(",");
  Serial.print(Axyz[1]);
  Serial.print(",");
  Serial.print(Axyz[2]);
  //Serial.println("Gyro(degress/s) of X,Y,Z:");
  Serial.print("G");
  Serial.print(Gxyz[0]);
  Serial.print(",");
  Serial.print(Gxyz[1]);
  Serial.print(",");
  Serial.print(Gxyz[2]);
  //Serial.println("Compass Value of X,Y,Z:");
  Serial.print("M");
  Serial.print(Mxyz[0]);
  Serial.print(",");
  Serial.print(Mxyz[1]);
  Serial.print(",");
  Serial.print(Mxyz[2]);

  //Serial.println("The clockwise angle between the magnetic north and X-Axis:");
  Serial.print("B");
  Serial.print(heading);
  //Serial.print(" ");
  //Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
  Serial.print("D");
  Serial.print(tiltheading);
#endif

  ///////////////////////////////////////////////////// Se lee Voltaje Bateria /////////////////////////////////////////////////
  int volBat1 = analogRead(pinVolBat1);
  voltajeBateria0 = 5.0 * volBat1 / 1024 * 1000;
  //Serial.print("VoltBat: ");Serial.print(voltajeBateria0);
  load_volt(voltajeBateria0);

  ///////////////////////////////////////////////////// Se lee Temperatura Radio /////////////////////////////////////////////////
  int volTR = analogRead(pinTempRad);
  float tempRad = (volTR * 5.0) / 1024.0;
  tempRad = tempRad - 0.5;
  tempRad = tempRad / 0.01;
  //load_tempADC(tempRad);
  //Serial.print("TempR: ");Serial.print(tempRad);load_volt(voltajeBateria0);

  ///////////////////////////////////////////////////// Se lee Temperatura PCB /////////////////////////////////////////////////
  int volTP = analogRead(pinTempPCB);
  int tempPCB = (5.0 * volTP / 1024 * 1000 - 500) * 10;
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

  load_tempADC(prom);

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
    enviarTramaCRadio();
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



}



