//////////////////////////////////////////////////////////////// Radio ////////////////////////////////////////////////////////////////
///////////////////////////////////////// Trasmitir por Radio /////////////////////////////////////////////////////////////
inline void trasmitirMediciones() {
  // Time for another APRS frame
  if (millis() >= next_aprs) {
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
#else
    switch (band_transmission) {
      case 2:
        generarTramaCorta();
        generarTramaIRadio();
        enviarTramaIRadio();
        break;
      case 1:
        generarTramaCorta();
        generarTramaGRadio();
        enviarTramaGRadio();
        break;
      default:
        generarTramaCorta();
        enviarTramaCRadio();
        break;
    }
#endif
    band_transmission = (band_transmission + 1) % 3;

    if (APRS_SLOT >= 0) {
      next_aprs = millis() + (APRS_PERIOD - (gps_seconds * 1000 + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
    } else {
      next_aprs += APRS_PERIOD;
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
    ultimoTiempoTX = millis();    // reportamos respuesta del radio
    digitalWrite(LedRAD_PIN, HIGH);
    String data2Send = Serial2.readString();
    /*String datosR = data2Send;
      // Verificamos el Tamaño el de la trama recibida, si es igual a 63, es posible que falte informacion por recibir
      if (data2Send.length() == 63) {
      //preguntamos por mas datos
      Serial2.println("D");
      delay(10);
      String data2Send2 = Serial2.readString();
      }*/
    // data2Send.length() max lenght = 63
    //Serial.println(data2Send.length());

    guardarStringSD(data2Send, "r");
    Serial.println("Radio: " + data2Send);

    // Verificamos Recepcion Comando de Despliegue
    if (data2Send.substring(0, 2).equals("DP")) {
      Serial2.print("M");
      Serial2.print("Desplegando...");
      Serial.print("Comando Despliegue detectado, ");
      liberarPaneles(0);
      liberarPaneles(1);
      Serial2.print("M");
      Serial2.print("Paneles Desplegados");
      digitalWrite(LedRAD_PIN, LOW);
      return; // No repetimos mensaje de despligue
    }

    // Repetimos informacion por Radio
    Serial2.print("M");
    Serial2.print(data2Send);
    digitalWrite(LedRAD_PIN, LOW);
  } else {
    // si no hay respuesta del radio en 20 segundos, lo reiniciamos
    if ((ultimoTiempoTX + 20000) < millis()) {
      resetRadio();
    }

  }

} // fin revisar Radio

///////////////////////////////////////// Generar Tramas /////////////////////////////////////////////////////////////
/*inline void generarTramas() {
  generarTramaCorta();
  generarTramaGRadio();
  generarTramaIRadio();

  //enviarTramasRadio(DatosRC + DatosRG + "----Alargado intencionalTrama");
  //enviarTramasRadio(DatosRC + DatosRG);
  }*/
///////////////////////////////////////// Trama Corta Ubicacion /////////////////////////////////////////////////////////////
inline void generarTramaCorta() {
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
  DatosRC = datos; // cargamos trama a variable global
}

///////////////////////////////////////// Envio Trama Ubicacion
inline void enviarTramaCRadio() {
  Serial.print("Enviando... ");
  Serial.println(DatosRC);
  // Envio datos por Radio
  enviarTramasRadio(DatosRC);
}

///////////////////////////////////////// Trama Larga Gases /////////////////////////////////////////////////////////////
inline void generarTramaGRadio() {
  char sep [] = {'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q'};
  // datos GPS
  String datos = String(sep[0]);
  datos += String(humSD); // humedad DHT11
  datos += sep[1];
  datos += String(cSD04[0]); //NH3
  datos += sep[2];
  datos += String(cSD04[1]); //CO
  datos += sep[3];
  datos += String(cSD04[2]); //NO2
  datos += sep[4];
  datos += String(cSD04[3]); //C3H8
  datos += sep[5];
  datos += String(cSD04[4]); //C4H10
  datos += sep[6];
  datos += String(cSD04[5]); //CH4
  datos += sep[7];
  datos += String(cSD04[6]); //H2
  datos += sep[8];
  datos += String(cSD04[7]); //C2H5OH
  datos += sep[9];

  datos += String(tempPromPCB); //tempI2c Promedio
  datos += sep[10];
  datos += String(tempPCB); // tempADC Promedio
  datos += sep[11];
  datos += String((long)BarB_pres);

  DatosRG = datos; // cargamos trama a variable global
}

///////////////////////////////////////// Envio Trama Larga Gases
inline void enviarTramaGRadio() {
  String datos = DatosRC;
  datos += DatosRG;
  Serial.print("Enviando... ");
  Serial.println(datos);
  // Envio datos por Radio
  enviarTramasRadio(datos);
}

///////////////////////////////////////// Trama Larga IMU /////////////////////////////////////////////////////////////
inline void generarTramaIRadio() {
  char sep [] = {'f', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P'};
  // datos GPS
  String datos = String(sep[0]);
  char text[] = "";
  snprintf(text, 6, "%d", (long)(Axyz[0] * 100));
  datos += text;
  datos += sep[1];
  snprintf(text, 6, "%d", (long)(Axyz[1] * 100));
  datos += text;
  datos += sep[2];
  snprintf(text, 6, "%d", (long)(Axyz[2] * 100));
  datos += text;

  datos += sep[3];
  snprintf(text, 6, "%d", (long)(Gxyz[0] * 100));
  datos += text;
  datos += sep[4];
  snprintf(text, 6, "%d", (long)(Gxyz[1] * 100));
  datos += text;
  datos += sep[5];
  snprintf(text, 6, "%d", (long)(Gxyz[2] * 100));
  datos += text;

  datos += sep[6];
  snprintf(text, 6, "%d", (long)(Mxyz[0] * 100));
  datos += text;
  datos += sep[7];
  snprintf(text, 6, "%d", (long)(Mxyz[1] * 100));
  datos += text;
  datos += sep[8];
  snprintf(text, 6, "%d", (long)(Mxyz[2] * 100));
  datos += text;

  DatosRI = datos; // cargamos trama a variable global
}

///////////////////////////////////////// Envio Trama Larga IMU
inline void enviarTramaIRadio() {
  String datos = DatosRC;
  datos += DatosRI;
  Serial.print("Enviando... ");
  Serial.println(datos);
  // Envio datos por Radio
  enviarTramasRadio(datos);
}

///////////////////////////////////////// Envio de Trama Largas
inline void enviarTramasRadio(String datosR) {
  digitalWrite(LedRAD_PIN, HIGH);
  byte largoTrama = datosR.length();
#define punto1  63
#define punto2  126  // Punto final para la segunda trama
  byte cont = 0;
  String Pack1 = "";
  String Pack2 = "";
  String Pack3 = "";

  // Convertimos la Trama en paquetes de maximo 63 caracteres
  if (largoTrama <= punto1) {
    Pack1 = datosR;
  } else if ( largoTrama > punto1 && largoTrama <= punto2) {
    Pack1 = datosR.substring(0, punto1);
    Pack2 = datosR.substring(punto1, largoTrama);
    cont = 1; // 2 paquetes
  } else if (largoTrama > punto2) {
    Pack1 = datosR.substring(0, punto1);
    Pack2 = datosR.substring(punto1, punto2);
    Pack3 = datosR.substring(punto2, largoTrama);
    cont = 2; // 3 paquetes
  }

  /*Serial.println("");
    Serial.println("Prueba Enviando... ");
    Serial.println("M");
    Serial.println(Pack1);
    Serial.println(Pack2);
    Serial.println(Pack3);
    Serial.println("Debug Radio...");*/
  // Enviamos M para indiciarle al radio que va un mensaje, y luego esperamos
  Serial2.print("M");

  // Esperamos una n, indicando que el radio esta listo para recibir el mensaje
  unsigned long tiempoFinMensaje = millis() + 3000;
  byte contador = 0;

  while (tiempoFinMensaje > millis()) {
    if (Serial2.available() > 0) {
      char c = Serial2.read();
      ultimoTiempoTX = millis();    // reportamos respuesta del radio
      //Serial.println(c);
      // Si Pack 2 y 3 tienen contenido los enviamos luego de recivir nuevamente N
      if (c == 'n') {
        switch (contador) {
          case 0:
            Serial2.print(Pack1); contador++; // Enviamos mensaje 1
            //Serial.println(Pack1);
            break;
          case 1:
            Serial2.print(Pack2); contador++; // Enviamos mensaje 2
            //Serial.println(Pack2);
            break;
          case 2:
            Serial2.print(Pack3); contador++; // Enviamos mensaje 3
            //Serial.println(Pack3);
            break;
        }
      }
      // si fueron enviados todos los paquetes, terminamos la espera
      if (contador > cont) {
        //Serial.println("Todos Enviados");
        break;  // Si se evio todo se continua con el codigo
      }
    }
  }// Fin envio tramai

  //Serial.println("Fin..");
  // Envio datos por Radio
  //
  //Serial2.print("M");
  //Serial2.print(datos);
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
  load_Accx ((long)(Axyz[0] * 100));
  load_Accy ((long)(Axyz[1] * 100));
  load_Accz ((long)(Axyz[2] * 100));
  load_Gyrx ((long)(Gxyz[0] * 100));
  load_Gyry ((long)(Gxyz[1] * 100));
  load_Gyrz ((long)(Gxyz[2] * 100));
  load_Magx ((long)(Mxyz[0] * 100));
  load_Magy ((long)(Mxyz[1] * 100));
  load_Magz ((long)(Mxyz[2] * 100));
}
#endif

///////////////////////////////////////// Resetear Radio
inline void resetRadio() {
  pinMode(4, OUTPUT);
  // reset Radio
  digitalWrite(4, LOW);
  delay(100);
  pinMode(4, INPUT);
}

