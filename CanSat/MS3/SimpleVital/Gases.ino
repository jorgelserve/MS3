//////////////////////////////////////////////////////////////// Gases ////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void medirGases04() {
  if (error_gas04 == 0) {
    error_gas04 = iniciarGases04();
  }

  c04[0] = gas04.measure_NH3();
  c04[1] = gas04.measure_CO();
  c04[2] = gas04.measure_NO2();
  c04[3] = gas04.measure_C3H8();
  c04[4] = gas04.measure_C4H10();
  c04[5] = gas04.measure_CH4();
  c04[6] = gas04.measure_H2();
  c04[7] = gas04.measure_C2H5OH();

  dim04 = (sizeof(c04) / sizeof(float));

  for (int i = 0; i < (dim04); i++) {
    cSD04[i] = (long) (c04[i] * 100);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void medirGases05() {
  if (error_gas05 == 0) {
    error_gas05 = iniciarGases05();
  }

  c05[0] = gas05.measure_NH3();
  c05[1] = gas05.measure_CO();
  c05[2] = gas05.measure_NO2();
  c05[3] = gas05.measure_C3H8();
  c05[4] = gas05.measure_C4H10();
  c05[5] = gas05.measure_CH4();
  c05[6] = gas05.measure_H2();
  c05[7] = gas05.measure_C2H5OH();


  dim05 = (sizeof(c05) / sizeof(float));

  for (int i = 0; i < (dim05); i++) {
    cSD05[i] = (long) (c05[i] * 100);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void promedioGases() {
  dim04 = (sizeof(c04) / sizeof(float));  // estraccion Tamaño
  // Generacion promedio, validando funcionamiento sensor
  for (int i = 0; i < (dim04); i++) { 
    if (error_gas04 != 0) {
      if (error_gas05 != 0) {
        gases[i] = (cSD04[i] + cSD05[i]) / 2; // si ningun sensor tiene error se promedian sus valores
      } else {
        gases[i] = cSD04[i];  // si solo 5 esta fallando
      }
    } else if (error_gas05 != 0) {
      gases[i] = cSD05[i];    // si solo 4 esta fallando
    } else {
      gases[i] = 0;           // si ambos fallan
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void calibrarGases04() {
  if (error_gas04 == 0) {
    error_gas04 = iniciarGases04();
  }
  Serial.print("Begin to calibrate 04...: ");
  Serial.println(millis());
  gas04.doCalibrate();
  Serial.print("04... Calibration ok: ");
  Serial.println(millis());
  gas04.display_eeprom();

}

inline void calibrarGases05() {
  if (error_gas05 == 0) {
    error_gas05 = iniciarGases05();
  }
  Serial.print("Begin to calibrate 05...: ");
  Serial.println(millis());
  gas05.doCalibrate();
  Serial.print("05... Calibration ok: ");
  Serial.println(millis());

  gas05.display_eeprom();
}

