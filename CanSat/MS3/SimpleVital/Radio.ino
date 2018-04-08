//////////////////////////////////////////////////////////////// Radio ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void revisarRadio() {
  if (Serial2.available() > 0) {
    digitalWrite(LedRAD_PIN, HIGH);
    String data2Send = Serial2.readString();
    // data2Send.length() max lenght = 63
    //Serial.println(data2Send.length());
    guardarStringSD(data2Send, "r");
    Serial.println("Radio: " + data2Send);
    if (data2Send.substring(0,2).equals("DP")) {
      Serial2.print("M");
      Serial2.print("Desplegando...");
      Serial.print("Comando Despliegue detectado, ");  
      liberarPaneles();
      Serial2.print("M");
      Serial2.print("Paneles Desplegados");
      digitalWrite(LedRAD_PIN, LOW);
      return;
    }
    // Repetimos informacion por Radio
    Serial2.print("M");
    Serial2.print(data2Send);
    digitalWrite(LedRAD_PIN, LOW);
  }
}

///////////////////////////////////////// Trama Ubicacion /////////////////////////////////////////////////////////////
String GenerarTramaCorta() {
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

///////////////////////////////////////// Trama Corta Ubicacion /////////////////////////////////////////////////////////////
void enviarTramaCRadio() {
  String datos = GenerarTramaCorta();
  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
}

///////////////////////////////////////// Trama Larga Gases /////////////////////////////////////////////////////////////
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

///////////////////////////////////////// Trama Larga IMU /////////////////////////////////////////////////////////////
void enviarTramaIRadio() {
  String datos = GenerarTramaCorta();
  char sep [] = {'f', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P'};
  // datos GPS
  datos += sep[0];
  char text[] = "";
  snprintf(text, 6, "%d", (int)Axyz[0] * 100);
  datos += text;
  datos += sep[1];
  snprintf(text, 6, "%d", (int)Axyz[1] * 100);
  datos += text;
  datos += sep[2];
  snprintf(text, 6, "%d", (int)Axyz[2] * 100);
  datos += text;

  datos += sep[3];
  snprintf(text, 6, "%d", (int)Gxyz[0] * 100);
  datos += text;
  datos += sep[4];
  snprintf(text, 6, "%d", (int)Gxyz[1] * 100);
  datos += text;
  datos += sep[5];
  snprintf(text, 6, "%d", (int)Gxyz[2] * 100);
  datos += text;

  datos += sep[6];
  snprintf(text, 6, "%d", (int)Mxyz[0] * 100);
  datos += text;
  datos += sep[7];
  snprintf(text, 6, "%d", (int)Mxyz[1] * 100);
  datos += text;
  datos += sep[8];
  snprintf(text, 6, "%d", (int)Mxyz[2] * 100);
  datos += text;

  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
}

