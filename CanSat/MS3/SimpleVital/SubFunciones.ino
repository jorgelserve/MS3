//////////////////////////////////////////////////////////////// Sub Funciones ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void pitar(int tiempo) {
#if Silencio
  digitalWrite(LedRAD_PIN, HIGH);
  delay(tiempo);
  digitalWrite(LedRAD_PIN, LOW);
#else
  digitalWrite(buzzer_PIN, HIGH);
  delay(tiempo);
  digitalWrite(buzzer_PIN, LOW);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
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
  char text[] = "";
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

  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
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

