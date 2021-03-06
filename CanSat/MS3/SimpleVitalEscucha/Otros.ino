//////////////////////////////////////////////////////////////// Otras Sub Funciones ////////////////////////////////////////////////////////////////
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

#if not MS2Compatible
  Serial.print("Datos BarometroT: Temp: ");
  Serial.print(BarT_temp);
  Serial.print("\t Pre: ");
  Serial.print(BarT_pres);
  Serial.print("\t Alt: ");
  Serial.println(BarT_alti);
#endif

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

#if not MS2Compatible
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
#endif

  Serial.print("Sensor SHT11    : Temp: ");
  Serial.print(tempSD); // Temperatura
  Serial.print("\t Hum: ");
  Serial.println(humSD); // Humedad

  Serial.print("Temperatura PCBs: Vital: ");
  Serial.print(tempi2cSDV); // Temp i2c Vital
#if not MS2Compatible
  Serial.print("\t Gases: ");
  Serial.println(tempi2cSDG); // Temp i2c Gases
#else
  Serial.println();
#endif

  Serial.print("Temperatura Ext : T1: ");
  Serial.print(tempext1SD); //Temp exterior
  Serial.print("\t T2: ");
  Serial.print(tempext2SD);
  Serial.print("\t T3: ");
  Serial.println(tempext3SD);

  Serial.print("Voltaje Bateria : ");
  Serial.println(voltajeBateria0);

  Serial.print("Voltaje Paneles Gondola: ");
  Serial.print(voltajePaneles);
  Serial.print("\t Corriente Paneles Gondola: ");
  Serial.print(corrientePaneles);
  Serial.println("\n ---------------------------------------------------------------------------------");
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void revisarDesPaneles() {
  if (panelesDesplegados == 0) {

    weight = (millis() > panel_time) ? weight + 1 : weight;         // Condicion Temporal
    weight = (gps_altitude > alt_paneles) ? weight + 1 : weight;    // Condicion GPS
    weight = (BarB_alti > alt_paneles) ? weight + 1 : weight;       // Condicion Barometro

    if (weight >= 2) {
      liberarPaneles(1);
      revisarDespliegue();   // Solo poner en uno si realmente desplego (Deteccion de aumento del voltaje de la bateria?)
    } else {
      weight = 0;
    }
  }
}

inline void liberarPaneles(bool panel) {
  Serial.println("Cuenta Regresiva para Despliegue de Paneles, Iniciada...");
  pitar(1000);
  for (int i = 9; i > 0; i--) {
    delay(1000);
    Serial.print("T-0");
    Serial.println(i);
  }
  desplegarPaneles(panel, 15000);
}

inline void desplegarPaneles(bool panel, unsigned int tiempoD) {
  Serial.print(" Desplegando ");
  if (panel) {
    digitalWrite(pinPanel0, HIGH);  // toca elegir uno a la vez
    Serial.print("1...");
  } else {
    digitalWrite(pinPanel1, HIGH);
    Serial.print("0...");
  }
  pitar(tiempoD);
  if (panel) {
    digitalWrite(pinPanel0, LOW); // toca alegir uno a la vez
  } else {
    digitalWrite(pinPanel1, LOW);
  }
  Serial.print(" OK");
}

inline void revisarDespliegue() {
  // si se percive un cambio del voltaje de la bateria que indique que los paneles reciven sol y estan cargando la bateria, las probalidades de que desplegaran es alta

  // panelesDesplegados = 1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void medirBarometroB() {
#if not MS2Compatible
  BarB_temp = baro_BMP280T.readTemperature() * 100; // Get the temperature, bmp180ReadUT MUST be called first
  BarB_pres = baro_BMP280T.readPressure();
  BarB_alti = baro_BMP280T.readAltitude(1013.25);

#else
  BarB_temp = Barometer.bmp180GetTemperature(Barometer.bmp180ReadUT()) * 100; // Get the temperature, bmp180ReadUT MUST be called first
  BarB_pres = Barometer.bmp180GetPressure(Barometer.bmp180ReadUP());// Get the presure
  BarB_alti = Barometer.calcAltitude(BarB_pres); // Uncompensated caculation - in Meters
#endif

}

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void medirBarometroT() {
#if not MS2Compatible
  BarT_temp = baro_BMP280T.readTemperature() * 100; // Get the temperature, bmp180ReadUT MUST be called first
  BarT_pres = baro_BMP280T.readPressure();
  BarT_alti = baro_BMP280T.readAltitude(1013.25);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void promedioBarometros() {
#if not MS2Compatible
  BarT_temp = baro_BMP280T.readTemperature() * 100; // Get the temperature, bmp180ReadUT MUST be called first
  BarT_pres = baro_BMP280T.readPressure();
  BarT_alti = baro_BMP280T.readAltitude(1013.25);
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////////
float revisarPotencia(char comando) {
  // si comando = r revisamos estado ultimas peticiones, si comando es B, C o D enviamos el comando y cambiamos el estado
  if (comando == 'r') {
    Serial.print("R...");
    if (ultimaSolicitud == 'B') { // Iluminacion leds
      // Veificamos que tengamos comando de respuesta de despligue panales
      if (Serial1.available() > 0) {
        char dato = Serial1.read();
        if (dato == 'B') {
          ultimaConexion = millis();
          Serial.print("Leds OK... ");
          Serial.println(dato);
        }
      }
    } else if (ultimaSolicitud == 'C') { // Voltaje Paneles
     
      if (Serial1.available() > 2) {
         
        float dato = Serial1.parseFloat();
        if (dato > -1) {
          Serial.print("Voltaje: ");
          Serial.println(dato);
          ultimaConexion = millis();
          voltajePaneles = dato;      // se actualiza el ultimo valor de voltaje medido
        }
      }
    } else if (ultimaSolicitud == 'D') { // Corrientes Panele
      if (Serial1.available() > 2) {
        float dato = Serial1.parseFloat();
        if (dato > -1) {
          Serial.print("Corriente: ");
          Serial.println(dato);
          ultimaConexion = millis();
          corrientePaneles = dato;      // se actualiza el ultimo valor de corriente medido
        }
      }
    }
  } else {
    Serial1.print(comando);
    Serial.print("Solicitando comando: ");
    Serial.println(comando);
    ultimaSolicitud = comando;
    maximaEsperaComando = millis();
  }
  

}
//////////////////////////////////////////////////////////////////////////////////////////////////////
float readAmp(){
  if(band_serial == false){
    Serial1.print('D');
    band_serial = true;
    comandoInicio = millis();
  }
  if(band_serial == true){  
    maximaEsperaComando = millis();
    totalComando = maximaEsperaComando - comandoInicio;
    if (Serial1.available() > 2) {
      float dato = Serial1.parseFloat();
      if (dato > -1) {
        band_serial = false;
        ultimaConexion = millis();
        band_lectura = 1;
        Serial1.flush();
        return dato;       // se actualiza el ultimo valor de corriente medido
      }
    }
    if(totalComando > 3000){
      totalComando = 0;
      band_serial = false;
      band_lectura = 1;
    }
  } 
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
float readVolt(){
  if(band_serial == false){
    Serial1.print('C');
    band_serial = true;
    comandoInicio = millis();
  }
  if(band_serial == true){  
    maximaEsperaComando = millis();
    totalComando = maximaEsperaComando - comandoInicio;
    if (Serial1.available() > 2) {
      float dato1 = Serial1.parseFloat();
      if (dato1 > -1) {
        band_serial = false;
        ultimaConexion = millis();
        band_lectura = 0;
        Serial1.flush();
        return dato1;       // se actualiza el ultimo valor de voltaje medido
      }
    }
    if(totalComando > 3000){
      totalComando = 0;
      band_serial = false;
      band_lectura = 0;
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
char LedOn(){
  time_mission = millis() - fall_time;
  //Serial.print(time_mission);
  if ((BarT_alti > alt_apogeo && BarB_alti > alt_apogeo) || (gps_altitude > alt_apogeo && BarB_alti > alt_apogeo) || (gps_altitude > alt_apogeo && BarT_alti > alt_apogeo) || (time_mission > apogeo_time)) {
    if (band_apogeo == 0) {
      fall_time = millis();
      band_apogeo = 1;
    }
  }
  if (band_apogeo == 1) {
    if ((millis() - fall_time) > apogeo_time) {
      if ((BarT_alti < alt_apogeo && BarB_alti < alt_apogeo) || (gps_altitude < alt_apogeo && BarB_alti < alt_apogeo) || (gps_altitude < alt_apogeo && BarT_alti < alt_apogeo) || (time_mission > apogeo_time)) {
        ban_apogeo_active = 1;
      }
    }
  }
  if (ban_apogeo_active == 1) {
    if(band_serial_led == false){
        Serial1.print('B');
        pitar(1000);
        band_serial_led = true;
        comandoInicioLed = millis();
    }
    if(band_serial_led == true){
      maximaEsperaComandoLed = millis();
      totalComandoLed = maximaEsperaComandoLed - comandoInicioLed;  
      if (Serial1.available() > 2) {
        float dato2 = Serial1.read();
        if (dato2 == 'B') {
          band_serial_led = false;
          ultimaConexion = millis();
          Serial1.flush();
          return dato2;       // se actualiza el ultimo valor de corriente medido
        }
      }
    }
    if(totalComandoLed > 3000){
      totalComandoLed = 0;
      band_serial_led = false;
    }
  }
}

