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
    // MS3
    
  */
  char sep [] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z'};
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
  
  // datos BarometroB
  datos += sep[15];
  datos += String(BarB_temp);
  datos += sep[16];
  datos += String(BarB_pres);
  datos += sep[17];
  datos += String(BarB_alti);
  
  // datos tiempo
  datos += sep[18];
  datos += String(millis());

  //Sensor de gases (04)
  //Gas NH3
  datos += sep[19];
  datos += String(c04[0]);
  //Gas CO
  datos += sep[20];
  datos += String(c04[1]);
  //Gas NO2
  datos += sep[21];
  datos += String(c04[2]);
  //Gas C3H8
  datos += sep[22];
  datos += String(c04[3]);
  //Gas C4H10
  datos += sep[23];
  datos += String(c04[4]);
  //Gas CH4
  datos += sep[24];
  datos += String(c04[5]);
  //GAS H2
  datos += sep[25];
  datos += String(c04[6]);
  //Gas C2H5OH
  datos += sep[26];
  datos += String(c04[7]);
  
  //Temperatura SHT11
  datos += sep[27];
  datos += String(tempSD);
  //Humedad SHT11
  datos += sep[28];
  datos += String(humSD);
  
  //Tempi2c Vital
  datos += sep[29];
  datos += String(tempi2cSDV);
  
  //Temp exterior
  datos += sep[30];
  datos += String(tempext1SD);
  datos += sep[31];
  datos += String(tempext2SD);
  datos += sep[32];
  datos += String(tempext3SD);

  // Voltaje Bateria
  datos += sep[33];
  datos += String(voltajeBateria0);

  // FechaGPS
  datos += sep[34];
  datos += String(gps_date);
  
  #if not MS2Compatible
  // datos BarometroT
  datos += sep[35];
  datos += String(BarT_temp);
  datos += sep[36];
  datos += String(BarT_pres);
  datos += sep[37];
  datos += String(BarT_alti);
  
  //Sensor de gases (05)
  //Gas NH3
  datos += sep[38];
  datos += String(c05[0]);
  //Gas CO
  datos += sep[39];
  datos += String(c05[1]);
  //Gas NO2
  datos += sep[40];
  datos += String(c05[2]);
  //Gas C3H8
  datos += sep[41];
  datos += String(c05[3]);
  //Gas C4H10
  datos += sep[42];
  datos += String(c05[4]);
  //Gas CH4
  datos += sep[43];
  datos += String(c05[5]);
  //GAS H2
  datos += sep[44];
  datos += String(c05[6]);
  //Gas C2H5OH
  datos += sep[45];
  datos += String(c05[7]);

  //Tempi2c Vital
  datos += sep[46];
  datos += String(tempi2cSDG);
  
  // Datos potencia Gondola
  datos += sep[47];
  datos += String(voltajePaneles);
  
  datos += sep[48];
  datos += String(corrientePaneles);
  
  // fin trama MS3
  datos += sep[49];
  #else
  // fin trama MS2
  datos += sep[35];
  #endif
  
  guardarStringSD(datos,"d");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void guardarStringSD(String Datos, String NombreA) {
  String nombre = NombreA;
  nombre += String(gps_date);
  nombre += ".txt";
  if (sd_ok) {
    digitalWrite(LedSD_PIN, HIGH);
    // se guarda en la SD
    dataFile = SD.open(nombre, FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(Datos);
      dataFile.flush();
      dataFile.close();
      // led indicador guardado en SD

      // print to the serial port too:
      //Serial.print(datos);
      Serial.print("On: ");
      Serial.print(nombre);
      Serial.print(": ");
      Serial.print(Datos);
      pitar(50);   //Pitido indicando que se guardo correctamente la informacion
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.print("error opening:");
      Serial.println(nombre);
      dataFile.close();
      sd_ok =  false;
    }
    
  } else {
    iniciarSd(); // si sd no esta ok (sd_ok) tratamos de iniciarla nuevamente
  }
  digitalWrite(LedSD_PIN, LOW); // gurdado SD finalizado
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

