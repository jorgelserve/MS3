//////////////////////////////////////////////////////////////// SD ////////////////////////////////////////////////////////////////

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

