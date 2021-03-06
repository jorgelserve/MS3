//////////////////////////////////////////////////////////////// Codigo Simple Vital ////////////////////////////////////////////////////////////////
/* Pendientes
  - Organizar codigo
  - Agregar rutina de chequeo inicial
  - Agregar sensor analogo radio trama
  - Sensor analogo pcb trama
  - vector puertos mediciones analogas
*/
//////////////////////////////////////////////////////////////// Librerias ////////////////////////////////////////////////////////////////
// se incluyen librerias
#include "ConfiVital.h"

//////////////////////////////////////////////////////////////// Setup ////////////////////////////////////////////////////////////////
void setup() {
  //////////////////////////////////////////// Pitido inicial
  pinMode(buzzer_PIN, OUTPUT);
  pitar(500);

  //////// Pruebas Mosfet (Problema retorno de corriente a pin de desactivacion del elevador de 5V)

  pinMode(potUSB, OUTPUT);
  digitalWrite(potUSB, HIGH);  // Mosfet Tipo P- Se apaga durante el funcionamiento con bateria
  //digitalWrite(potUSB, LOW);  // Se enciende con el uso del puerto USB

  ////////////////////////////////////////////
#if MS2Compatible
  pinMode(8, INPUT); // Importante Configurar pin 8 como entrada(va al radio, pero no tiene el timer adecuado)
  pinMode(7, INPUT); // Bug de pcb vital MS2 fabricado.
#endif

  //////////////////////////////////////////// Comunication
  Serial.begin(DEBUG_SERIAL_SPEED);
  
  Serial1.begin(9600);      // Serial potencia gondola
  Serial1.setTimeout(500);
  Serial1.flush();
  
  Serial3.begin(GPS_BAUDRATE);

  //////////////////////////////////////////// Serial init
#if MS2Compatible
  Serial.println("Serial Debuggin Started - Hi from SimpleVital 2");
#else
  Serial.println("Serial Debuggin Started - Hi from SimpleVital 3");
#endif

  //////////////////////////////////////////// Se configura sistema despligue paneles
  Serial.print("Iniciando Sistema Despliegue... ");
  pinMode(pinPanel0, OUTPUT); // Se configuran puertos como salida
  pinMode(pinPanel1, OUTPUT);
  digitalWrite(pinPanel0, LOW); // Se Inician Apagados
  digitalWrite(pinPanel1, LOW);
  //probamos despliegue
  desplegarPaneles(0, 100);
  desplegarPaneles(1, 100);
  Serial.println("");

  // se configura pin voltaje bateria
  pinMode(pinVolBat1, INPUT);
  // se configura pin voltaje bateria
  pinMode(pinTempRad, INPUT); // usado en la vital MS3 para monitorear salida Radio (No usado)
  pinMode(pinTempPCB, INPUT);

  // se configuran los leds indicadores
  pinMode(LedSD_PIN, OUTPUT);
  //pinMode(LedRUN_PIN, OUTPUT);
  //pinMode(LedTX_PIN, OUTPUT);



#if DEBUG < 2
  Serial.print("Initializing I2C Bus...");
#endif
  Wire.begin();

#if DEBUG < 2
  Serial.println(" OK.");
#endif

  //////////////////////////////////////////// Sensors Inicialization
  ///////////////////// initialize IMU
  Serial.print("Initializing IMU...");
  Serial.println(accelgyro.testConnection() ? "OK" : "MPU9255 connection failed");

  ///////////////////// initialize Barometers
  Serial.println("Initializing Barometers");
#if not MS2Compatible
  Serial.print("BMP280B test... ");
  if (!baro_BMP280B.begin()) { // default dir 0x77
    Serial.println("Could not find a valid BMP280B sensor, check wiring!");
  } else {
    Serial.println(" OK.");
  }


  Serial.print("BMP280T test... ");
  if (!baro_BMP280T.begin(0x76)) {
    Serial.println("Could not find a valid BMP280T sensor, check wiring!");
  } else {
    Serial.println(" OK.");
  }
#else
  Serial.print("BMP180 test... ");
  Barometer.init();
  Serial.println(" OK.");
#endif

  ///////////////////// Setup AFSK - Radio analalogo
#if not RadioSerial
  Serial.print("Setup Afsk...");
  afsk_setup();
  Serial.println(" OK.");
#endif

  ///////////////////// Setup GPS
  Serial.print("Setup Gps...");
  gps_setup();
  Serial.println(" OK.");
  // Se reseteo del GPS
  pinMode(gpsRESET_PIN, OUTPUT);
  digitalWrite(gpsRESET_PIN, HIGH);
  delay(50);
  digitalWrite(gpsRESET_PIN, LOW);


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

    next_aprs = millis() + 1000 * (APRS_PERIOD / 1000 - (gps_seconds + APRS_PERIOD / 1000 - APRS_SLOT) % APRS_PERIOD);
  } else {
    next_aprs = millis();
  }

#if DEBUG < 2
  Serial.print("N-APRS:");
  Serial.println(next_aprs);
#endif

#if DEBUG < 2
  Serial.print("Initializing SD card...");
  bool sd_estado = false;
  sd_estado = SD.begin(chipSelect);
  //Serial.println(sd_estado);
  Serial.println(sd_estado ? "OK" : "Card failed, or not present");
  if (sd_estado) {
    sd_ok = true;
  }

#else
  if (SD.begin(chipSelect)) {
    sd_ok = true;
  }
#endif

  ///////////////////// Pruebas Leds Gondola
  Serial.println("Prueba Leds gondola... ");
  revisarPotencia('B');
  delay(200);
  revisarPotencia('B');
  // Pitido final
  pitar(100);

  // Se verifica si esta en modo carga esperando una c por serial durante 5 segundos
  Serial.println("");
  Serial.println("------------ Menu configuraciones iniciales -----------------");
  Serial.println("Presione:");
  Serial.println(" i para iniciar imediatamente");
  Serial.println(" c para iniciar modo carga rapida");
  Serial.println(" g para calibrar sensores de gases");
  Serial.println(" p para probar despliegue de panales");
  Serial.println(" r para cargar con seguridad codigo en la radio");
  Serial.println(" s probar potencia gondola");
  Serial.println("-------------------------------------------------------------");

  // Configuraciones
  unsigned long tiempoConfCarga = millis() + 5000;
  while (tiempoConfCarga > millis()) {
    char comando = Serial.read();
    if (comando == 'i') {
      break;

    } else if (comando == 'c') {
      Serial.println("Modo Carga activado.... Presione una tecla para iniciar modulo.");
      digitalWrite(potUSB, LOW);  // Se enciende el flujo de potencia desde el USB
      while (Serial.available() < 1) {
        //delay(500);
        power_save();
        revisarRadio();
      }
      digitalWrite(potUSB, HIGH);  // Se apaga el flujo de potencia del puerto USB, para evitar que se apague el elevador

    } else if (comando == 'g') {
      Serial.println("Iniciando calibracion de sensores de gases... Precalentando.");
      calibrarGases04();
      calibrarGases05();

    } else if (comando == 'p') {
      Serial.println("Iniciando Pruebas de sistemas de despligue 0 y 1");
      liberarPaneles(0);
      liberarPaneles(1);

    } else if (comando == 'r') {
      pinMode(16, INPUT);
      pinMode(17, INPUT);
      Serial.println("Pines comunicacion Radio listos para evitar colision de datos");
      while (Serial.available() < 1) {
        power_save();
      }
    } else if (comando == 's') {
      Serial.println("Iniciando Pruebas de sistemas de potencia gondola... presione una tecla para finalizar");
      byte prueba = 0;
      Serial1.flush();
      while (Serial.available() < 1) {
        delay(1000);
        revisarPotencia('r');
        delay(1000);
        revisarPotencia('r');
        delay(1000);
        revisarPotencia('r');
        switch (prueba) {
          case 2:
            revisarPotencia('D');
            break;
          case 1:
            revisarPotencia('C');
            break;
          default:
            revisarPotencia('B');
            break;
        }
        prueba = (prueba + 1) % 3;
        revisarPotencia('r');

      }
    }
  }

  //////////////////////////////////////////// Inicio Radio Serial
#if RadioSerial
  Serial2.begin(115200);
  Serial.print("Iniciando Radio...");
  resetRadio();
  Serial.println("OK");
#endif


  //////////////////////////////////////////// Se reinician los tiempos de referencia
  next_aprs = millis();

  // fin menu configuraciones iniciales.
}

//////////////////////////////////////////////////////////////// Loop ////////////////////////////////////////////////////////////////
void loop() {
  // verificamos mensajes seriales del radio y los mostramos en pantalla:
  revisarRadio();

  /////////////////////////////// Liberación de paneles /////////////////////////////////////
  revisarDesPaneles();

  ////////////////////////////////////////////////////////Temperatura y Humedad //////////////////////////////////////////////
  ////// Sensor i2C /////////////
  float tempi2cV = PCBTemperature(0); // lectura Temp PCB Vital
  tempi2cSDV = (float)tempi2cV * 100;

  float tempi2cG = PCBTemperature(1); // lectura Temp PCB Gases
  tempi2cSDG = (float)tempi2cG * 100;

  // se carga temp I2C a Trama radio
  tempPromPCB = (tempi2cSDV + tempi2cSDG) / 2;


  ///// Sensor SHT11 //////////
  float tempC = sht1x.readTemperatureC();
  float humidity = sht1x.readHumidity();
  tempSD = (int)(tempC * 100);
  humSD = (int)(humidity * 100);

  ////////////////////////////////////////////////////////Sensor de Gases //////////////////////////////////////////////
  medirGases04();
#if not MS2Compatible
  medirGases05();
  promedioGases();    // se calcula el promedio de ambos sensores para enviarlos por radio
#endif
  ///////////////////////////////////////////////////// Se lee el Barometro /////////////////////////////////////////////////
  medirBarometroB();
#if not MS2Compatible
  medirBarometroT();
#endif
  ///////////////////////////////////////////////////// Se lee la IMU /////////////////////////////////////////////////
  getAccel_Data();
  getGyro_Data();
  getCompassData_calibrated(); // compass data has been calibrated here

  getHeading();               //before we use this function we should run 'getCompassDate_calibrated()' frist, so that we can get calibrated data ,then we can get correct angle .
  getTiltHeading();

  ///////////////////////////////////////////////////// Se lee Voltaje Bateria /////////////////////////////////////////////////
  voltajeBateria0 = ADC2Mil * analogRead(pinVolBat1);
  //voltajeBateria0 = 5.0 * volBat1 / 1024 * 1000;

  ///////////////////////////////////////////////////// Se lee Temperatura Radio /////////////////////////////////////////////////
  tempRad = ADC2Mil * analogRead(pinTempRad) - 500;

  ///////////////////////////////////////////////////// Se lee Temperatura PCB /////////////////////////////////////////////////
  tempPCB = ADC2Mil * analogRead(pinTempPCB) - 500;

  //////////////////////////////////////////////////// Sensores de temperatura analogos externos ///////////////////////////////////
  tempext1SD = ADC2Mil * analogRead(pinTemp1) - 500;
  tempext2SD = ADC2Mil * analogRead(pinTemp2) - 500;
  tempext3SD = ADC2Mil * analogRead(pinTemp3) - 500;

  promTADC = (tempext1SD + tempext2SD + tempext3SD) / 3;

  ///////////////////////////////////////////////////// Se lee el GPS
  read_gps();

  ///////////////////////////////////////////////////// Se envian por Radio los datos
#if not RadioSerial
  cargarMediciones();
#else if
  //generarTramas();
#endif
  trasmitirMediciones();

  //////////////////////////////////////////////////// Medir corriente
  if(band_lectura == 0){
    corrientePaneles = readAmp();
  }
  ////////////////////////////////////////////////// Medir voltaje
  if(band_lectura == 1){
    voltajePaneles = readVolt();
  }
  ////////////////////////////////////////////////////Leds
  char ok = LedOn();

  ///////////////////////////////////////////////////// Se guardan en uSD
  guardarDatosSD();

  ///////////////////////////////////////////////////// Se muetran por serial las medidas
  mostrarMediciones();

  //power_save(); // Incoming GPS data or interrupts will wake us up
} // fin loop

