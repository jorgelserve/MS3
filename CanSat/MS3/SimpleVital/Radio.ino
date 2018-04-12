//////////////////////////////////////////////////////////////// Radio ////////////////////////////////////////////////////////////////
///////////////////////////////////////// Trasmitir por Radio /////////////////////////////////////////////////////////////
inline void trasmitirMediciones() {
  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {
    //digitalWrite(LedTX_PIN, HIGH);

    // Se envian las tramas por radio
#if not RadioSerial
    switch (band_transmission) {
      case 2:
        enviarTramaIRadioA();
        break;
      case 1:
        enviarTramaGRadioA();
        break;
      default:
        enviarTramaCRadioA();
        break;
    }
    band_transmission = (band_transmission + 1) % 3;
#else
    switch (band_transmission) {
      case 2:
        enviarTramaIRadio();
        break;
      case 1:
        enviarTramaGRadio();
        break;
      default:
        enviarTramaCRadio();
        break;
    }
    band_transmission = (band_transmission + 1) % 3;
#endif

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
}

///////////////////////////////////////// Revisar Paquetes Radio /////////////////////////////////////////////////////////////
inline void revisarRadio() {
  if (Serial2.available() > 0) {
    digitalWrite(LedRAD_PIN, HIGH);
    String data2Send = Serial2.readString();
    // data2Send.length() max lenght = 63
    //Serial.println(data2Send.length());
    guardarStringSD(data2Send, "r");
    Serial.println("Radio: " + data2Send);
    if (data2Send.substring(0, 2).equals("DP")) {
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
inline String GenerarTramaCorta() {
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
inline void enviarTramaCRadio() {
  String datos = GenerarTramaCorta();
  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
}

///////////////////////////////////////// Trama Larga Gases /////////////////////////////////////////////////////////////
inline void enviarTramaGRadio() {
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
  snprintf(text, 6, "%d", (long)BarB_pres);
  datos += text;

  digitalWrite(LedRAD_PIN, HIGH);
  Serial.print("Enviando... ");
  Serial.println(datos);
  Serial2.print("M");
  Serial2.print(datos);
  digitalWrite(LedRAD_PIN, LOW);
}

///////////////////////////////////////// Trama Larga IMU /////////////////////////////////////////////////////////////
inline void enviarTramaIRadio() {
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

///////////////////////////////////////// Cargar Datos Trama analoga /////////////////////////////////////////////////////////////
#if not RadioSerial
inline void cargarMediciones() {
  // Trama corta
  load_alti(BarB_alti);
  load_temp(BarB_temp);
  load_tempC(tempSD);           // Temperatura SHT11
  load_volt(voltajeBateria0);   // Voltaje Bateria

  // Trama Larga Gases
  load_humidity(humSD);

  load_NH3(cSD04[0]);
  load_CO(cSD04[1]);
  load_NO2(cSD04[2]);
  load_C3H8(cSD04[3]);
  load_C4H10(cSD04[4]);
  load_CH4(cSD04[5]);
  load_H2(cSD04[6]);
  load_C2H5OH(cSD04[7]);

  load_tempi2c(tempPromPCB);
  load_tempADC(promTADC);
  load_pres(BarB_pres);
  //load_tempADC(tempRad);

  // Trama Larga IMU
  load_Accx ((long)Axyz[0] * 100);
  load_Accy ((long)Axyz[1] * 100);
  load_Accz ((long)Axyz[2] * 100);
  load_Gyrx ((long)Gxyz[0] * 100);
  load_Gyry ((long)Gxyz[1] * 100);
  load_Gyrz ((long)Gxyz[2] * 100);
  load_Magx ((long)Mxyz[0] * 100);
  load_Magy ((long)Mxyz[1] * 100);
  load_Magz ((long)Mxyz[2] * 100);
}
#endif

