//////////////////////////////////////////////////////////////// Sub Funciones ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void revisarRadio() {
  if (Serial2.available() > 0) {
    digitalWrite(LedRAD_PIN, HIGH);
    String data2Send = Serial2.readString();
    Serial.println(data2Send);
    Serial2.print("M");
    Serial2.print(data2Send);
    digitalWrite(LedRAD_PIN, LOW);
  }
}

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

  /* Trama SD:
    A TiempoGPS
    B Latitud*10000
    C Longitud*10000
    D AltitudGPS
    E CursoGPS
    F VelocidadGPS
    G ACC-X
    H ACC-Y
    I ACC-Z
    J GYR-X
    K GYR-Y
    L GYR-Z
    M MAG-X
    O MAG-Y
    P MAG-Z
    Q BAR-TEMP
    R BAR-PRES
    S BAR-ALTI
    T TiempoMsDesdeInicio
    U NH3
    V CO
    W NO2
    X C3H8
    Y C4H10
    Z CH4
    a H2
    b C2H5OH
    c SHT11-TEMP
    d SHT11-HUME
    e TempPcbI2C
    f TempExt1
    g TempExt2
    h TempExt3
    i VoltBat
  */
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
  datos += String(BarB_temp);
  datos += sep[16];
  datos += String(BarB_pres);
  datos += sep[17];
  datos += String(BarB_alti);
  // datos tiempo
  datos += sep[18];
  datos += String(millis());
  //Gas NH3
  datos += sep[19];
  datos += String(cSD04[0]);
  //Gas CO
  datos += sep[20];
  datos += String(cSD04[1]);
  //Gas NO2
  datos += sep[21];
  datos += String(cSD04[2]);
  //Gas C3H8
  datos += sep[22];
  datos += String(cSD04[3]);
  //Gas C4H10
  datos += sep[23];
  datos += String(cSD04[4]);
  //Gas CH4
  datos += sep[24];
  datos += String(cSD04[5]);
  //GAS H2
  datos += sep[25];
  datos += String(cSD04[6]);
  //Gas C2H5OH
  datos += sep[26];
  datos += String(cSD04[7]);

  /*
  //Gas NH3
  datos += sep[19];
  datos += String(cSD05[0]);
  //Gas CO
  datos += sep[20];
  datos += String(cSD05[1]);
  //Gas NO2
  datos += sep[21];
  datos += String(cSD05[2]);
  //Gas C3H8
  datos += sep[22];
  datos += String(cSD05[3]);
  //Gas C4H10
  datos += sep[23];
  datos += String(cSD05[4]);
  //Gas CH4
  datos += sep[24];
  datos += String(cSD05[5]);
  //GAS H2
  datos += sep[25];
  datos += String(cSD05[6]);
  //Gas C2H5OH
  datos += sep[26];
  datos += String(cSD05[7]);
   */
  
  //Temperatura
  datos += sep[27];
  datos += String(tempSD);
  //Humedad
  datos += sep[28];
  datos += String(humSD);
  //Tempi2c
  datos += sep[29];
  datos += String(tempi2cSDV);
  //datos += sep[30];
  //datos += String(tempi2cSDG);
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
      Serial.print(datos);
      pitar(50);
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
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void mostrarMediciones() {
  Serial.println();
  Serial.println("---------------------------------------------------------------------------------");
  Serial.print("Tiempo en milisegundos desde inicio: ");
  Serial.println(millis());

  Serial.print("Datos GPS       : Tiemp: ");
  Serial.print(gps_time);
  Serial.print("\t Lat: ");
  Serial.print(gps_lat * 1000);
  Serial.print("\t Lon: ");
  Serial.print(gps_lon * 1000);
  Serial.print("\t Alt: ");
  Serial.print(gps_altitude);
  Serial.print("\t Curso: ");
  Serial.print(gps_course);
  Serial.print("\t Fecha: ");
  Serial.print(gps_date);
  Serial.print("\t Vel: ");
  Serial.println(gps_speed);

  Serial.print("Aceleracion(s^2): X: ");
  Serial.print(Axyz[0]);
  Serial.print("\t Y: ");
  Serial.print(Axyz[1]);
  Serial.print("\t Z: ");
  Serial.println(Axyz[2]);

  Serial.print("Giroscopo(mm/s) : X: ");
  Serial.print(Gxyz[0]);
  Serial.print("\t Y: ");
  Serial.print(Gxyz[1]);
  Serial.print("\t Z: ");
  Serial.println(Gxyz[2]);

  Serial.print("Magnetometro()  : X: ");
  Serial.print(Mxyz[0]);
  Serial.print("\t Y: ");
  Serial.print(Mxyz[1]);
  Serial.print("\t Z: ");
  Serial.println(Mxyz[2]);

  Serial.print("Calculos IMU    : Azm: ");
  Serial.print(heading); // The clockwise angle between the magnetic north and X-Axis
  //Serial.println("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:");
  Serial.print("\t til: ");
  Serial.println(tiltheading); // The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane

  Serial.print("Datos BarometroB: Temp: ");
  Serial.print(BarB_temp);
  Serial.print("\t Pre: ");
  Serial.print(BarB_pres);
  Serial.print("\t Alt: ");
  Serial.println(BarB_alti);

  Serial.print("Datos BarometroT: Temp: ");
  Serial.print(BarT_temp);
  Serial.print("\t Pre: ");
  Serial.print(BarT_pres);
  Serial.print("\t Alt: ");
  Serial.println(BarT_alti);

  Serial.print("Sensor Gases(04): NH3: ");    //
  Serial.print(cSD04[0]);
  Serial.print("\t CO: ");
  Serial.print(cSD04[1]);
  Serial.print("\t NO2: ");
  Serial.print(cSD04[2]);
  Serial.print("\t C3H8: ");
  Serial.print(cSD04[3]);
  Serial.print("\t C4H10: ");
  Serial.print(cSD04[4]);
  Serial.print("\t CH4: ");
  Serial.print(cSD04[5]);
  Serial.print("\t H2: ");
  Serial.print(cSD04[6]);
  Serial.print("\t C2H5OH: ");
  Serial.println(cSD04[7]);

  Serial.print("Sensor Gases(05): NH3: ");    //
  Serial.print(cSD05[0]);
  Serial.print("\t CO: ");
  Serial.print(cSD05[1]);
  Serial.print("\t NO2: ");
  Serial.print(cSD05[2]);
  Serial.print("\t C3H8: ");
  Serial.print(cSD05[3]);
  Serial.print("\t C4H10: ");
  Serial.print(cSD05[4]);
  Serial.print("\t CH4: ");
  Serial.print(cSD05[5]);
  Serial.print("\t H2: ");
  Serial.print(cSD05[6]);
  Serial.print("\t C2H5OH: ");
  Serial.println(cSD05[7]);

  Serial.print("Sensor SHT11    : Temp: ");
  Serial.print(tempSD); // Temperatura
  Serial.print("\t Hum: ");
  Serial.println(humSD); // Humedad

  Serial.print("Temperatura PCBs: Vital: ");
  Serial.print(tempi2cSDV); // Temp i2c Vital
  Serial.print("\t Gases: ");
  Serial.println(tempi2cSDG); // Temp i2c Gases

  Serial.print("Temperatura Ext : T1: ");
  Serial.print(tempext1SD); //Temp exterior
  Serial.print("\t T2: ");
  Serial.print(tempext2SD);
  Serial.print("\t T3: ");
  Serial.println(tempext3SD);

  Serial.print("Voltaje Bateria : ");
  Serial.println(voltajeBateria0);

  Serial.println("---------------------------------------------------------------------------------");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
String GenerarTramaCorta(){
  // A - GPS - Tiempo
  // B - GPS - Latitud
  // C - GPS - Longitud
  // D - GPS - Altitud
  // E - GPS - Curso
  // F - GPS - Velocidad

  char sep [] = {'/', 'h', '/', 'O', '/', 'A', 'B', 'C', 'D', 'E', 'F'};
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
  snprintf(text, 6, "%d", (int)BarB_alti);
  datos += text;
  datos += sep[7];
  snprintf(text, 6, "%d", (int)BarB_temp);
  datos += text;
  datos += sep[8];
  snprintf(text, 6, "%d", (int)(tempSD)); // Temp SHT11
  datos += text;
  datos += sep[9];
  snprintf(text, 6, "%d", (int)(voltajeBateria0));
  datos += text;
  //datos += sep[10];
  // fin trama
  return datos;
}

void enviarTramaCRadio() {
  String datos = GenerarTramaCorta();
  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
}

void enviarTramaGRadio() {
  String datos = GenerarTramaCorta();
  char sep [] = {'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P'};
  // datos GPS
  datos += sep[0];
  datos += String(humSD); // humedad DHT11
  datos += sep[1];
  datos += String(cSD04[0]); //NH3
  datos += sep[2];
  datos += String(cSD04[1]); //CO
  datos += sep[3];
  datos += String(cSD04[2]); //NO2
  datos += sep[4];
  datos += String(cSD04[6]); //H2
  datos += sep[5];
  datos += String(cSD04[7]); //C2H5OH
  datos += sep[6];

  datos += String(tempPromPCB); //tempI2c Promedio
  datos += sep[7];
  datos += String(tempPCB); // tempADC Promedio
  datos += sep[8];
  char text[] = "";
  snprintf(text, 6, "%d", (unsigned int)BarB_pres);
  datos += text;
  
  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
}

void enviarTramaIRadio() {
  String datos = GenerarTramaCorta();
  char sep [] = {'f', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P'};
  // datos GPS
  datos += sep[0];
  char text[] = "";
  snprintf(text, 6, "%d", (int)Axyz[0]*100);
  datos += text;
  datos += sep[1];
  snprintf(text, 6, "%d", (int)Axyz[1]*100);
  datos += text;
  datos += sep[2];
  snprintf(text, 6, "%d", (int)Axyz[2]*100);
  datos += text;

  datos += sep[3];
  snprintf(text, 6, "%d", (int)Gxyz[0]*100);
  datos += text;
  datos += sep[4];
  snprintf(text, 6, "%d", (int)Gxyz[1]*100);
  datos += text;
  datos += sep[5];
  snprintf(text, 6, "%d", (int)Gxyz[2]*100);
  datos += text;

  datos += sep[6];
  snprintf(text, 6, "%d", (int)Mxyz[0]*100);
  datos += text;
  datos += sep[7];
  snprintf(text, 6, "%d", (int)Mxyz[1]*100);
  datos += text;
  datos += sep[8];
  snprintf(text, 6, "%d", (int)Mxyz[2]*100);
  datos += text;
  
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

byte iniciarGases04() {
  Wire.beginTransmission(0x04);
  error04 = Wire.endTransmission();
  if (error04 == 0) {
    gas04.begin(0x04);//the default I2C address of the slave is 0x04

    band_gas04 = gas04.powerOn();

    if (band_gas04 == 1) {
      band_gas04 = 0;
      Serial.println("Gas Sensor Initialized");
      error_gas04 = 1;
      return error_gas04;

    }
  } else {
    Serial.println("Gas Sensor not found");
    error_gas04 = 0;
    return error_gas04;
  }
}

byte iniciarGases05() {
  Wire.beginTransmission(0x05);
  error05 = Wire.endTransmission();
  if (error05 == 0) {
    gas05.begin(0x05);//the default I2C address of the slave is 0x05

    band_gas05 = gas05.powerOn();

    if (band_gas05 == 1) {
      band_gas05 = 0;
      Serial.println("Gas Sensor Initialized");
      error_gas05 = 1;
      return error_gas05;

    }
  } else {
    Serial.println("Gas Sensor not found");
    error_gas05 = 0;
    return error_gas05;
  }
}


float PCBTemperature(byte PCB) {

  char raw[2] = {0, 0};
  float temp = 0;
  float temp_vector[12] = {0.125, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64, 128, -1};
  int count = 0;
  int bitre = 0;
  int count_data = 0;
  int count_temp = 0;
  unsigned int total = 0;


  if (PCB == 0) {
    Wire.beginTransmission(i2cAddressV);
  }
  else if (PCB == 1) {
    Wire.beginTransmission(i2cAddressG);
  }
  else {
    return 0;
  }

  Wire.write(add_reg);
  Wire.endTransmission();

  if (PCB == 0) {
    Wire.requestFrom(i2cAddressV, 2);
  }
  else if (PCB == 1) {
    Wire.requestFrom(i2cAddressG, 2);
  }
  else {
    return 0;
  }

  while (Wire.available()) {
    raw[count] = Wire.read();
    count++;
  } // fin while (datos disponibles en puerto I2C)

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
  } // fin while (revisar en detalle posible paro por while infinito?)

  return temp;
}

inline void liberarPaneles() {
  if (band_paneles == 0) {

    weight = ((millis() / 1000) > panel_time) ? weight + 1 : weight;
    weight = (gps_altitude > alt_paneles) ? weight + 1 : weight;
    weight = (BarB_alti > alt_paneles) ? weight + 1 : weight;

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
  }
}

inline void apogeoSistema() {
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
}


inline void medirGases04() {
  if (error_gas04 == 0) {
    error_gas04 = iniciarGases04();
  }

  c04[0] = gas04.measure_NH3();
  load_NH3(c04[0]);
  c04[1] = gas04.measure_CO();
  load_CO(c04[1]);
  c04[2] = gas04.measure_NO2();
  load_NO2(c04[2]);
  c04[3] = gas04.measure_C3H8();
  load_C3H8(c04[3]);
  c04[4] = gas04.measure_C4H10();
  load_C4H10(c04[4]);
  c04[5] = gas04.measure_CH4();
  load_CH4(c04[5]);
  c04[6] = gas04.measure_H2();
  load_H2(c04[6]);
  c04[7] = gas04.measure_C2H5OH();
  load_C2H5OH(c04[7]);


  dim04 = (sizeof(c04) / sizeof(float));

  for (int i = 0; i < (dim04); i++) {
    cSD04[i] = (int) (c04[i] * 100);
  }
}

inline void medirGases05() {
  if (error_gas05 == 0) {
    error_gas05 = iniciarGases05();
  }

  c05[0] = gas05.measure_NH3();
  load_NH3(c05[0]);
  c05[1] = gas05.measure_CO();
  load_CO(c05[1]);
  c05[2] = gas05.measure_NO2();
  load_NO2(c05[2]);
  c05[3] = gas05.measure_C3H8();
  load_C3H8(c05[3]);
  c05[4] = gas05.measure_C4H10();
  load_C4H10(c05[4]);
  c05[5] = gas05.measure_CH4();
  load_CH4(c05[5]);
  c05[6] = gas05.measure_H2();
  load_H2(c05[6]);
  c05[7] = gas05.measure_C2H5OH();
  load_C2H5OH(c05[7]);


  dim05 = (sizeof(c05) / sizeof(float));

  for (int i = 0; i < (dim05); i++) {
    cSD05[i] = (int) (c05[i] * 100);
  }
}

inline void medirBarometroB(){
  //temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); // Get the temperature, bmp180ReadUT MUST be called first
  BarB_temp = baro_BMP280T.readTemperature()*100; // Get the temperature, bmp180ReadUT MUST be called first

  //pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());// Get the presure
  BarB_pres = baro_BMP280T.readPressure();

  //altitud = Barometer.calcAltitude(pressure); // Uncompensated caculation - in Meters
  BarB_alti = baro_BMP280T.readAltitude(1013.25);

  load_temp(BarB_temp);
  load_pres(BarB_pres);
  load_alti(BarB_alti);
}

inline void medirBarometroT(){
  //temperature = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()); // Get the temperature, bmp180ReadUT MUST be called first
  BarT_temp = baro_BMP280T.readTemperature(); // Get the temperature, bmp180ReadUT MUST be called first

  //pressure = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());// Get the presure
  BarT_pres = baro_BMP280T.readPressure();

  //altitud = Barometer.calcAltitude(pressure); // Uncompensated caculation - in Meters
  BarT_alti = baro_BMP280T.readAltitude(1013.25);
  /*
  load_temp(BarT_temp);
  load_pres(BarT_pres);
  load_alti(BarT_alti);
  */
}

