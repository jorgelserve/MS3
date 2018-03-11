
// se incluyen lbrerias
#include "confiVital.h"


void setup() {
  //# importante Configurar pin 8 como entrada(va al radio, pero no tiene el timer adecuado), bug de pcb vital fabricado.
  pinMode(8,INPUT);
  
  // se configura sistema despligue paneles
  pinMode(pinbuzzer,OUTPUT);
  digitalWrite(pinbuzzer,HIGH);
  delay(100);
  digitalWrite(pinbuzzer,LOW);

  
  // se configura sistema despligue paneles
  pinMode(pinPanel1,OUTPUT);
  digitalWrite(pinPanel1,LOW);

  // se configura pin voltaje bateria
  pinMode(pinVolBat1,INPUT);
  
  // se configuran los leds indicadores
  pinMode(LedSD_PIN, OUTPUT);
  //pinMode(LedRUN_PIN, OUTPUT);
  //pinMode(LedTX_PIN, OUTPUT);

  digitalWrite(LedSD_PIN, HIGH);
  delay(100);
  digitalWrite(LedSD_PIN, LOW);
  
  //digitalWrite(LedRUN_PIN, HIGH);
  //delay(100);
  //digitalWrite(LedRUN_PIN, LOW);
  //digitalWrite(LedTX_PIN, HIGH);

  //setupVital();
  //////////////////////////////////////////// Comunication
  // Serial Port Configuration
  Serial.begin(DEBUG_SERIAL_SPEED);
  Serial3.begin(GPS_BAUDRATE);

    //GPS
    pinMode(gpsRESET_PIN,OUTPUT);
    pinMode(gpsPPS_PIN,INPUT);
    digitalWrite(gpsRESET_PIN,HIGH); // se resetea el GPS
    delay(20);
    digitalWrite(gpsRESET_PIN,LOW); // se resetea el GPS
  

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

  //////////////////////////////////////////// Sensors Inicialization
  //  Mxyz_init_calibrated ();
  // used to show sleep mode -> indication of the CPU activity.
  pinMode(LED_PIN, OUTPUT);
  pin_write(LED_PIN, LOW);

#if DEBUG < 2
  //Serial.print("Setup Buzzer...");
#endif

  //buzzer_setup();

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
  bool sd_estado = SD.begin(chipSelect);
  Serial.println(sd_estado ? "card initialized." : "Card failed, or not present");
  if (sd_estado) {
    sd_ok = true;
  }
#else
  if (SD.begin(chipSelect)) {
    sd_ok = true;
  }
#endif

  /*
    #if DEBUG < 2
    Serial.println("Setup Sensors...");
    #endif

    sensors_setup();  // sensores analogos
  */
  // TODO:
  // - Completar Captura de datos GPS y guardar en un archivo de texto en la SD
  // - Optimizar libreria buzzer
  pinMode(pinPeriodo, INPUT);


  // apagamos los leds indicadores
  digitalWrite(LedSD_PIN, LOW);
  //digitalWrite(LedRUN_PIN, LOW);
  //digitalWrite(LedTX_PIN, LOW);
}

void loop() {
  //

  // secuencia disparo paneles
  if (Serial.available() > 0) {
    char lectura = Serial.read();
    byte conteo = byte(lectura);
    if (conteo > byte('0') && conteo < byte('9')) {
      Serial.println("Cuenta Regresiva para Despliegue de Paneles, Iniciada...");
      //Serial.print("cont: ");
      //Serial.println(conteo);
      //Serial.print("lec: ");
      //Serial.println(lectura);
      conteo -= 48;
      for (int i = conteo; i > 0; i--) {
        delay(1000);
        Serial.print("T-0");
        Serial.println(i);
      }
      Serial.print("Despliege...");
      digitalWrite(pinPanel1, HIGH);
      delay(2000);
      digitalWrite(pinPanel1, LOW);
      Serial.println("Simple Pregunta: Desplego?");
      // si, que chimba, no bueno, intentemos de nuevo
    }
  }

  //digitalWrite(LedRUN_PIN, HIGH);
  // cadena en la que se guardara la informacion de los sensores

  ///////////////////////////////////////////////////// Se lee el Barometro /////////////////////////////////////////////////

#if DEBUG <1
  Serial.print("RB");
#endif

  temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); // Get the temperature, bmp180ReadUT MUST be called first

  pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());// Get the presure

  altitud = Barometer.calcAltitude(pressure); // Uncompensated caculation - in Meters

#if DEBUG <1
  Serial.print("T");
  Serial.print(temperature);
  Serial.print("P");
  Serial.print(pressure);
  Serial.print("A");
  Serial.print(altitud);
#endif
  load_temp(temperature);
  load_pres(pressure);
  load_alti(altitud);

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
  //Serial.print("C");
  //Serial.print(mx_centre);
  //Serial.print("/");
  //Serial.print(my_centre);
  //Serial.print("/");
  //Serial.print(mz_centre);

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
  float voltajeBateria1 = 5.0*volBat1/1024;
  load_volt(voltajeBateria1);

  // TEMP RADIO
  //int volBat1 = analogRead(pinVolBat1);
  //float voltajeBateria1 = (5.0*volBat1/1024*1000-500)/10;
  //load_volt(voltajeBateria1);

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

  //digitalWrite(LedRUN_PIN, LOW);  

  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {
    //digitalWrite(LedTX_PIN, HIGH);
    aprs_send();
    //digitalWrite(LedTX_PIN, LOW);

    if (APRS_SLOT >= 0) {
      next_aprs = millis() + 1000 * (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
    } else {
      next_aprs += APRS_PERIOD * 1000L;
    }

    while (afsk_flush()) {
      power_save();
    }

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

#if DEBUG <1
  Serial.print("2SD.");
#endif
  guardarDatosSD();
  //Serial.print(datos);


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
#endif

  power_save(); // Incoming GPS data or interrupts will wake us up

  //*/

}

void guardarDatosSD() {
  // A - GPS - Tiempo
  // B - GPS - Latitud
  // C - GPS - Longitud
  // D - GPS - Altitud
  // E - GPS - Curso
  // F - GPS - Velocidad

  char sep [] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V'};
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
  datos += sep[19];
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

