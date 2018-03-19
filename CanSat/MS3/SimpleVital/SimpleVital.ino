//////////////////////////////////////////////////////////////// Codigo Simple Vital ////////////////////////////////////////////////////////////////

/* Pendientes
  - Agregar sensor analogo radio trama
  - Sensor analogo pcb trama
  - vector puertos mediciones analogas
*/

//////////////////////////////////////////////////////////////// Librerias ////////////////////////////////////////////////////////////////

// se incluyen lbrerias
#include "ConfiVital.h"
#include "MutichannelGasSensor.h"
#include "SHT1x.h"


//////////////////////////////////////////////////////////////// Declaraciones ////////////////////////////////////////////////////////////////

// Gases
char band_gas = 0;
int dim;
float c[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //
int cSD[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte error;
byte error_gas = 0;

// Humedad y Temperatura (SHT11)
#define dataPin 6                 // SHT11 serial data
#define clockPin 7                // SHT11 serial clock
//---
int tempSD;
int humSD;
//---
SHT1x sht1x(dataPin, clockPin);

// Tempertura PCB (I2C)
#define i2cAddress 0x18
#define add_reg 0x05
//---
int tempi2cSD;

// Temperatura Externa
int tempext1SD;
int tempext2SD;
int tempext3SD;

// Apogeo
#define alt_apogeo 4000    // altura por encima de la que inicia el conteo
#define apogeo_time 360000  // tiempo en mili-segundos desde que pasa altura
//---
byte band_buzzer = 0;
long int fall_time = 0;
byte band_apogeo = 0;
byte ban_apogeo_active = 0;

// liberacion Paneles
#define panel_time 24      // Tiempo en segundos
#define alt_paneles 80      // Altura en metros antes de despliegue 
//---
float weight = 0;
byte band_paneles = 0;
char band_transmission = 0;

//Configuracion Simple 144/434
//#define AnalogosExt // 144

int voltajeBateria0 = 0;


#define clockPin 7                // SHT11 serial clock

#define Led_RAD 8
#define Simple 3          // Seleccionamos la simple con la estamos trabajando
#define RadioSerial 1     // Radio Serial/analogo
#define Silencio  1       // Cambia el buzzer(0) por led(1)

//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz Setup zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
void setup() {
  //# importante Configurar pin 8 como entrada(va al radio, pero no tiene el timer adecuado), bug de pcb vital MS2 fabricado.

#if Simple == 2
  pinMode(8, INPUT);
#else if Simple == 3
  pinMode(8, OUTPUT);
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
  pinMode(pinbuzzer, OUTPUT);
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

  //////////////////////////////////////////// Comunication
  // Serial Port Configuration
  Serial.begin(DEBUG_SERIAL_SPEED);
  Serial3.begin(GPS_BAUDRATE);

#if RadioSerial
  Serial2.begin(115200);
#endif

#if DEBUG < 2
  Serial.println("Serial Debuggin Started - Hi from SimpleVital");
#endif

#if DEBUG < 2
  Serial.print("Initializing I2C Bus...");
#endif

  //join I2C bus (I2Cdev library doesn't do this automatically)
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

  Barometer.init();

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

  // Se congura pin PPS en el GPS
  pinMode(gpsPPS_PIN, INPUT);


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
  Serial.print("Initializing Radio...");
  Serial2.println("MSimple3-ON");
#endif

  // Pitido final
  pitar(100);
}

//////////////////////////////////////////////////////////////// Loop ////////////////////////////////////////////////////////////////

void loop() {
  // verificamos mensajes seriales del radio y los mostramos en pantalla:
  if (Serial2.available() > 0) {
    digitalWrite(Led_RAD, HIGH);
    String data2Send = Serial2.readString();
    Serial.println(data2Send);
    Serial2.print("M");
    Serial2.print(data2Send);
    digitalWrite(Led_RAD, LOW);
  }


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
      digitalWrite(pinbuzzer, HIGH);
      band_buzzer = 0;
    } else {
      digitalWrite(pinbuzzer, LOW);
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

  temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); // Get the temperature, bmp180ReadUT MUST be called first

  pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());// Get the presure

  altitud = Barometer.calcAltitude(pressure); // Uncompensated caculation - in Meters

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

//////////////////////////////////////////////////////////////// Sub Funciones ////////////////////////////////////////////////////////////////

void pitar(int tiempo) {
#if Silencio
  digitalWrite(Led_RAD, HIGH);
  delay(tiempo);
  digitalWrite(Led_RAD, LOW);
#else
  digitalWrite(pinbuzzer, HIGH);
  delay(tiempo);
  digitalWrite(pinbuzzer, LOW);
#endif
}

void guardarDatosSD() {
  // A - GPS - Tiempo
  // B - GPS - Latitud
  // C - GPS - Longitud
  // D - GPS - Altitud
  // E - GPS - Curso
  // F - GPS - Velocidad

  char sep [] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i'};
  // datos GPS
  String datos = String(sep[0]);
  datos += String(gps_time);
  datos += sep[1];
  datos += String(gps_lat * 10000);
  datos += sep[2];
  datos += String(gps_lon * 10000);
  datos += sep[3];
  datos += String(gps_altitude);
  datos += sep[4];
  datos += String(gps_course);
  datos += sep[5];
  datos += String(gps_speed);

  // datos IMU
  // ACC
  datos += sep[6];
  datos += String(Axyz[0]);
  datos += sep[7];
  datos += String(Axyz[1]);
  datos += sep[8];
  datos += String(Axyz[2]);
  //Serial.println("Gyro(degress/s) of X,Y,Z:");
  datos += sep[9];
  datos += String(Gxyz[0]);
  datos += sep[10];
  datos += String(Gxyz[1]);
  datos += sep[11];
  datos += String(Gxyz[2]);
  //Serial.println("Compass Value of X,Y,Z:");
  datos += sep[12];
  datos += String(Mxyz[0]);
  datos += sep[13];
  datos += String(Mxyz[1]);
  datos += sep[14];
  datos += String(Mxyz[2]);
  // datos Barometro
  datos += sep[15];
  datos += String(temperature);
  datos += sep[16];
  datos += String(pressure);
  datos += sep[17];
  datos += String(altitud);
  // datos tiempo
  datos += sep[18];
  datos += String(millis());
  //Gas NH3
  datos += sep[19];
  datos += String(cSD[0]);
  //Gas CO
  datos += sep[20];
  datos += String(cSD[1]);
  //Gas NO2
  datos += sep[21];
  datos += String(cSD[2]);
  //Gas C3H8
  datos += sep[22];
  datos += String(cSD[3]);
  //Gas C4H10
  datos += sep[23];
  datos += String(cSD[4]);
  //Gas CH4
  datos += sep[24];
  datos += String(cSD[5]);
  //GAS H2
  datos += sep[25];
  datos += String(cSD[6]);
  //Gas C2H5OH
  datos += sep[26];
  datos += String(cSD[7]);
  //Temperatura
  datos += sep[27];
  datos += String(tempSD);
  //Humedad
  datos += sep[28];
  datos += String(humSD);
  //Tempi2c
  datos += sep[29];
  datos += String(tempi2cSD);
  //Temp exterior
  datos += sep[30];
  datos += String(tempext1SD);
  datos += sep[31];
  datos += String(tempext2SD);
  datos += sep[32];
  datos += String(tempext3SD);
  datos += sep[33];
  datos += String(voltajeBateria0);
  datos += sep[34];
  // fin trama

  if (sd_ok) {
    digitalWrite(LedSD_PIN, HIGH);
    // se guarda en la SD
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(datos);
      dataFile.flush();
      dataFile.close();
      // led indicador guardado en SD

      // print to the serial port too:
      //Serial.print(datos);
      Serial.print("ON SD .. ");

      if (altitud < alt_apogeo && gps_altitude < alt_apogeo) {
        pitar(50);
      }
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
      dataFile.close();
      sd_ok =  false;
    }
  } else {
    iniciarSd();
  }
  //*/
  digitalWrite(LedSD_PIN, LOW);
  Serial.print(datos);
}

void enviarTramaCRadio() {
  // A - GPS - Tiempo
  // B - GPS - Latitud
  // C - GPS - Longitud
  // D - GPS - Altitud
  // E - GPS - Curso
  // F - GPS - Velocidad

  char sep [] = {'/', 'h', '/', 'O', '/', 'A', 'T', 'P', 'A', 'V', ' '};
  // datos GPS
  String datos = String(sep[0]);
  datos += String(gps_time);
  datos += sep[1];
  datos += String(gps_aprs_lat);
  datos += sep[2];
  datos += String(gps_aprs_lon);
  datos += sep[3];
  char text[] ="";
  snprintf(text, 4, "%03d", (int)(gps_course + 0.5));
  datos += text;
  datos += sep[4];
  snprintf(text, 4, "%03d", (int)(gps_speed + 0.5));
  datos += text;
  datos += sep[5];
  snprintf(text, 7, "%d", (int)(gps_altitude));
  datos += text;
  datos += sep[6];
  snprintf(text, 6, "%d", (int)(tempSD));
  datos += text;
  datos += sep[7];
  snprintf(text, 6, "%d", (int)pressure);
  datos += text;
  datos += sep[8];
  snprintf(text, 6, "%d", (int)altitud);
  datos += text;
  datos += sep[9];
  snprintf(text, 6, "%d", (int)(voltajeBateria0));
  datos += text;
  //datos += sep[10];
  // fin trama

  digitalWrite(Led_RAD, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(Led_RAD, LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void read_gps2() {
  // Get a valid position from the GPS
  int valid_pos = 0;
  //uint32_t timeout = millis();

#ifdef DEBUG_GPS
  Serial.println("RGPS");
#endif
  // si hay datos en el puerto se procesan todos
  if (Serial3.available()) {
    while (Serial3.available()) {
      valid_pos = gps_decode(Serial3.read());
    }
  }
  //if ((millis() > VALID_POS_TIMEOUT + timeout) && !valid_pos) {
  //  gps_reset_parser();
  //}

  if (valid_pos) {
    gps_reset_parser();
    if (gps_altitude > BUZZER_ALTITUDE) {
      //buzzer_off();   // In space, no one can hear you buzz
    } else {
      //buzzer_on();
    }
  }
}

void read_gps() {
  // Get a valid position from the GPS
  int valid_pos = 0;
  uint32_t timeout = millis();
  char lecturagps;
#ifdef DEBUG_GPS
  Serial.println("RGPS");
#endif

  gps_reset_parser();

  do {
    if (Serial3.available()) {
      lecturagps = Serial3.read();
#ifdef DEBUG_GPS
      Serial.print(lecturagps);
#endif
      valid_pos = gps_decode(lecturagps);
    }
  } while ( (millis() - timeout < VALID_POS_TIMEOUT) && ! valid_pos) ;



  if (valid_pos) {
    if (gps_altitude > BUZZER_ALTITUDE) {
      //buzzer_off();   // In space, no one can hear you buzz
    } else {
      //buzzer_on();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void getHeading(void)
{
  heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
  if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
  float pitch = asin(-Axyz[0]);
  float roll = asin(Axyz[1] / cos(pitch));

  float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
  float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
  float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
  tiltheading = 180 * atan2(yh, xh) / PI;
  if (yh < 0)    tiltheading += 360;
}

void Mxyz_init_calibrated ()
{

  Serial.println(F("Before using 9DOF,we need to calibrate the compass frist,It will takes about 2 minutes."));
  Serial.print("  ");
  Serial.println(F("During  calibratting ,you should rotate and turn the 9DOF all the time within 2 minutes."));
  Serial.print("  ");
  Serial.println(F("If you are ready ,please sent a command data 'ready' to start sample and calibrate."));
  while (!Serial.find("ready"));
  Serial.println("  ");
  Serial.println("ready");
  Serial.println("Sample starting......");
  Serial.println("waiting ......");

  get_calibration_Data ();

  Serial.println("     ");
  Serial.println("compass calibration parameter ");
  Serial.print(mx_centre);
  Serial.print("     ");
  Serial.print(my_centre);
  Serial.print("     ");
  Serial.println(mz_centre);
  Serial.println("    ");
}

void get_calibration_Data ()
{
  for (int i = 0; i < sample_num_mdate; i++)
  {
    get_one_sample_date_mxyz();
    /*
      Serial.print(mx_sample[2]);
      Serial.print(" ");
      Serial.print(my_sample[2]);                            //you can see the sample data here .
      Serial.print(" ");
      Serial.println(mz_sample[2]);
    */



    if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
    if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
    if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

    if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
    if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
    if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

  }

  mx_max = mx_sample[1];
  my_max = my_sample[1];
  mz_max = mz_sample[1];

  mx_min = mx_sample[0];
  my_min = my_sample[0];
  mz_min = mz_sample[0];



  mx_centre = (mx_max + mx_min) / 2;
  my_centre = (my_max + my_min) / 2;
  mz_centre = (mz_max + mz_min) / 2;

}

void get_one_sample_date_mxyz()
{
  getCompass_Data();
  mx_sample[2] = Mxyz[0];
  my_sample[2] = Mxyz[1];
  mz_sample[2] = Mxyz[2];
}

void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = (double) ax / 16384;
  Axyz[1] = (double) ay / 16384;
  Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Gxyz[0] = (double) gx * 250 / 32768;
  Gxyz[1] = (double) gy * 250 / 32768;
  Gxyz[2] = (double) gz * 250 / 32768;
}

void getCompass_Data(void)
{
  I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
  delay(10);
  I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

  mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
  my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
  mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

  Mxyz[0] = (double) mx * 1200 / 4096;
  Mxyz[1] = (double) my * 1200 / 4096;
  Mxyz[2] = (double) mz * 1200 / 4096;
}

void getCompassData_calibrated ()
{
  getCompass_Data();
  Mxyz[0] = Mxyz[0] - mx_centre;
  Mxyz[1] = Mxyz[1] - my_centre;
  Mxyz[2] = Mxyz[2] - mz_centre;
}

void iniciarSd() {
  pinMode(chipSelect, OUTPUT);
  Serial.print(" ini.. ");
  if (SD.begin(chipSelect)) {
    Serial.println("OK ");
    sd_ok = true;

  } else {
    Serial.println("Err ");
  }
}

byte iniciarGases() {
  Wire.beginTransmission(0x04);
  error = Wire.endTransmission();
  if (error == 0) {
    gas.begin(0x04);//the default I2C address of the slave is 0x04

    band_gas = gas.powerOn();

    if (band_gas == 1) {
      band_gas = 0;
      Serial.println("Gas Sensor Initialized");
      error_gas = 1;
      return error_gas;

    }
  } else {
    Serial.println("Gas Sensor not found");
    error_gas = 0;
    return error_gas;
  }
}

float PCBTemperature() {

  char raw[2] = {0, 0};
  int count = 0;
  unsigned int total = 0;
  int bitre = 0;
  float temp_vector[12] = {0.125, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64, 128, -1};
  int count_data = 0;
  int count_temp = 0;
  float temp = 0;

  Wire.beginTransmission(i2cAddress);

  Wire.write(add_reg);

  Wire.endTransmission();
  Wire.requestFrom(i2cAddress, 2);

  while (Wire.available())
  {

    raw[count] = Wire.read();
    count++;


  }
  count = 0;
  while (count_data < 16) {
    if (count_data > 7 && count_data < 12) {

      bitre = bitRead(raw[0], (count_data - 8));
      if (bitre == 1) {
        temp = temp + temp_vector[count_temp];
      }
      count_temp++;
    }
    if (count_data == 12) {
      bitre = bitRead(raw[0], (count_data - 8));
      if (bitre == 1) {
        temp = temp * temp_vector[count_temp];
      }
      count_temp++;
    }
    if (count_data > 0 && count_data < 8) {

      bitre = bitRead(raw[1], count_data);
      if (bitre == 1) {
        temp = temp + temp_vector[count_temp];
      }
      count_temp++;
    }
    count_data++;
  }
  count_data = 0;
  count_temp = 0;
  return temp;
}




