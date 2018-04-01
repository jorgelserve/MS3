/*PYTHON ENTREGA EL VECTOR QUE DEBERIA TENER LA ANTENA EN CADA
 *INSTANTE DE TIEMPO, ARDUINO SE ENCARGA DE MEDIR LA DIFERENCIA 
 *ENTRE EL VECTOR ACTUAL Y EL DESEADO PARA CORREGIR EL ERROR
 */

//#include <I2Cdev.h>
#include <stdlib.h>
#include <SD.h>

//#include "Wire.h"
//#include "I2Cdev.h"
//#include <HMC5883L.h>
//#define address 0x1E


#define datoEntendido 12
//#define SDDmal 12
#define manualLed 11 //es 11
#define bateriaLed 10
#define autonomoFuncionando 8 // es 8

/*#define IN1_MH  12
#define IN2_MH  11
#define IN3_MH  10
#define IN4_MH  9

#define IN1_MV  8
#define IN2_MV  7
#define IN3_MV  6
#define IN4_MV  5*/

#define SENTIDO_MV 7
#define PASOS_MV 6
#define ENABLE_MV 5
#define SENTIDO_MH 4
#define PASOS_MH 3
#define ENABLE_MH 2 

#define bateriaVolt A3
#define pinJoyX  A0
#define pinJoyY  A1
#define pinJoyButton  9

//#define setHorizontal A0
#define setVertical A2

//#define brujula1 A4
//#define brujula2 A5

//HMC5883L compass;
//File logger;

/*int Paso_MH [ 8 ][ 4 ] =
   {  {1, 0, 0, 0},
      {1, 1, 0, 0},
      {0, 1, 0, 0},
      {0, 1, 1, 0},
      {0, 0, 1, 0},
      {0, 0, 1, 1},
      {0, 0, 0, 1},
      {1, 0, 0, 1}
   };
   
int Paso_MV [ 8 ][ 4 ] =
   {  {1, 0, 0, 0},
      {1, 1, 0, 0},
      {0, 1, 0, 0},
      {0, 1, 1, 0},
      {0, 0, 1, 0},
      {0, 0, 1, 1},
      {0, 0, 0, 1},
      {1, 0, 0, 1}
   };*/
int divCero = 0;   
int steps_MH = 0;
int steps_MV = 0;
boolean stopp;
double theta_prima;
double omega_prima;
double theta;
double omega;
double theta_error;
double omega_error;
double theta_bru;
double decliMAG = -6.23;
boolean iniciar = false;
int pasosPorVuelta = 12000;
int repeticiones = 1;
int numPasos_MH = 0;
int numPasos_MV = 0;
int16_t mx, my, mz;
float declinacionGrados;
char incomingData;
String receivedCommand;
String signoTheta;
long moduloTheta;
long moduloOmega;
long theta_prima_int;
long omega_prima_int;
unsigned long suma = 0;
unsigned long suma_recibida;
boolean datoRecibido = false;
float calibrated_values[3];
boolean modoManual = true; 
int pasosManual = 0;
int numLecBut = 0;
int Xvalue;
int Yvalue;
String loggerBATERIA = "";
String loggerOMEGAPRIMA = "";
String loggerTHETAPRIMA = "";
int periodo = 1000;  //frecuencia de tiempo guardar en SD
unsigned long tiempoAnterior = 0;  

void setup() {
  /*Wire.begin();
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();*/
  pinMode(SENTIDO_MH, OUTPUT);
  pinMode(SENTIDO_MV, OUTPUT);
  pinMode(PASOS_MV, OUTPUT);
  pinMode(PASOS_MH, OUTPUT);
  /*pinMode(IN1_MH, OUTPUT);
  pinMode(IN2_MH, OUTPUT);
  pinMode(IN3_MH, OUTPUT);
  pinMode(IN4_MH, OUTPUT); 
  pinMode(IN1_MV, OUTPUT);
  pinMode(IN2_MV, OUTPUT);
  pinMode(IN3_MV, OUTPUT);
  pinMode(IN4_MV, OUTPUT);*/
  pinMode(ENABLE_MV, OUTPUT);
  pinMode(ENABLE_MH, OUTPUT);
  pinMode(autonomoFuncionando, OUTPUT);
  pinMode(manualLed, OUTPUT);
  //pinMode(SDDmal, OUTPUT);
  pinMode(datoEntendido, OUTPUT);
  pinMode(pinJoyButton, INPUT);
    digitalWrite(pinJoyButton, HIGH);
  pinMode(bateriaVolt, INPUT);
    analogWrite(bateriaVolt, LOW);
  Serial.begin(9600);
  /*if (!SD.begin(9)){
    digitalWrite(SDDmal, HIGH);
    return;
  }*/
  //buscarSetVertical();    //garantia de omega = 0
  omega = 0;                //vector actual antena
  theta = 0;                //vector actual antena
}

//T-36000O-3600072253
//T+09536O-0324513032
//T-09000O+0000009251
void loop() {
  //conteoTiempoGuardar();
  //medirBateria();
  lecturaPuertoProcesa();
  autonomo_O_manual();
}

void lecturaPuertoProcesa(){
  while (Serial.available () > 0){
     incomingData = Serial.read();
     receivedCommand += incomingData;
     if (receivedCommand.length() == 19 ){
        break;
     }
     datoRecibido = true;
     delay(2);
  }
    if (receivedCommand.length() == 19 ){
      datoRecibido = false;
      if (receivedCommand.substring(0,1) == "T"){
        suma += 84;
        moduloTheta = receivedCommand.substring(2,7).toInt();
        if (receivedCommand.substring(1,2) == "-"){
          suma += 45;
          theta_prima_int = - moduloTheta;
          loggerTHETAPRIMA = - moduloTheta;
          suma += moduloTheta;
        }
        if (receivedCommand.substring(1,2) == "+"){
          suma += 43;
          theta_prima_int = moduloTheta;
          loggerTHETAPRIMA = moduloTheta;
          suma += moduloTheta;
        }
     }
     if (receivedCommand.substring(7,8) == "O"){
        suma += 79;
        moduloOmega = receivedCommand.substring(9,14).toInt();
        if (receivedCommand.substring(8,9) == "-"){
          suma += 45;
          omega_prima_int = - moduloOmega;
          loggerOMEGAPRIMA = - moduloOmega;
          suma += moduloOmega;
        }
        if (receivedCommand.substring(8,9) == "+"){
          suma += 43;
          omega_prima_int = moduloOmega;
          loggerOMEGAPRIMA = moduloOmega;
          suma += moduloOmega;
        }
     }
     suma_recibida = receivedCommand.substring(14,19).toInt();
     //Serial.println(suma_recibida);
     //Serial.println(suma);
     if (suma_recibida == 99999){
        Serial.println("SET");
        omega = ((double)omega_prima_int)/100; 
        theta = ((double)theta_prima_int)/100;
     }else if (suma_recibida == 90909){
        Serial.println("listo");
     }else if (suma == suma_recibida){
        Serial.println("entendido");
        digitalWrite(datoEntendido, HIGH);
        iniciar = true;
     }else{
        digitalWrite(datoEntendido, LOW);
        Serial.println("no se entiende");
     }
  }else if (datoRecibido == true && receivedCommand.length() != 19){
      datoRecibido = false;
      Serial.println("no es el tamaño");
  }
  suma = 0;
  receivedCommand = "";
}

void autonomo_O_manual(){
  if (digitalRead(pinJoyButton) == LOW){
    numLecBut++;
    if (numLecBut == 5){
      if (modoManual == true){
        modoManual = false;
      }else{
        modoManual = true;
      }
    }
  }else{
    numLecBut = 0;
  } 
  if (modoManual == true){
    digitalWrite(ENABLE_MH, HIGH);
    digitalWrite(ENABLE_MV, HIGH);
    digitalWrite(manualLed, HIGH);
    Xvalue = analogRead(pinJoyX);
    delayMicroseconds(100);                 //es necesaria una pequeña pausa entre lecturas analógicas
    Yvalue = analogRead(pinJoyY);
    
    if (Xvalue < 480){
      //girar izquierda
      
      digitalWrite(SENTIDO_MH, HIGH); //verificar
      for (int i=0; i<5; i++){
        //motorHorizontal(Xvalue+1);
        motorHorizontal(2);
        
      }
      //Serial.print("X+ ");
      //Serial.println(pasosManual++);
    }
    if (Xvalue > 520){
      //girar derecha
      digitalWrite(SENTIDO_MH, LOW); //verificar
      for (int i=0; i<5; i++){
        //motorHorizontal((1023-Xvalue)+1);
        motorHorizontal(2);
      }
      //Serial.print("X- ");
      //Serial.println(pasosManual++);
    }
    if (Yvalue < 490){
      //girar abajo
      digitalWrite(SENTIDO_MV, HIGH); //verificar
      for (int i=0; i<5; i++){
        //motorVertical(Yvalue+1);
        motorVertical(2);
        //Serial.println(pasosManual++);
      }
      /*Serial.print("Y+ ");
      Serial.println(pasosManual++);*/
    }
    if (Yvalue > 530){
      //girar arriba
      digitalWrite(SENTIDO_MV, LOW); //verificar
      for (int i=0; i<5; i++){
        //motorVertical((1023-Yvalue)+1);
        motorVertical(2);
      }
      /*Serial.print("Y- ");
      Serial.println(pasosManual++);*/
    }
  }else{
    digitalWrite(ENABLE_MH, LOW);
    digitalWrite(ENABLE_MV, LOW);
    digitalWrite(manualLed, LOW);
  }  

  if (iniciar == true && modoManual == false){
    digitalWrite(autonomoFuncionando, HIGH);
    //Serial.println("IC");
    omega_prima = ((double)omega_prima_int)/100; 
    theta_prima = ((double)theta_prima_int)/100; 
    controlReal();
    iniciar = false;
    //Serial.println("FC");
    digitalWrite(autonomoFuncionando, LOW);
  }
}

void controlReal(){
  theta_error = theta_prima - (theta - decliMAG);
  if (theta_error > 180){
    theta_error = -360 - (theta - decliMAG) + theta_prima;
  }
  if (theta_error < -180){
    theta_error = 360 - (theta - decliMAG) + theta_prima;
  }
  omega_error = omega_prima - omega;
  //Serial.println(theta_error);
  //Serial.println(omega_error);
  int pasosHoriz = int(((pasosPorVuelta/360) * theta_error)/repeticiones);
  int pasosVerti = int(((pasosPorVuelta/360) * omega_error)/repeticiones);
  boolean romper_MH = false;
  boolean romper_MV = false;
  digitalWrite(ENABLE_MV, HIGH);
  digitalWrite(ENABLE_MH, HIGH);
  while(1){
    if (pasosHoriz > 0){
      if (numPasos_MH < abs(pasosHoriz)){
        digitalWrite(SENTIDO_MH, LOW); //verificar
        motorHorizontal(2);
        numPasos_MH++;
      }else{romper_MH = true;}
    }else{
      if (numPasos_MH < abs(pasosHoriz)){
        digitalWrite(SENTIDO_MH, HIGH); //verificar
        motorHorizontal(2);
        numPasos_MH++;
      }else{romper_MH = true;}
    }
    
    if (pasosVerti > 0){
      if (numPasos_MV < abs(pasosVerti)){
        digitalWrite(SENTIDO_MV, LOW); //verificar
        motorVertical(2);
        numPasos_MV++;
      }else{romper_MV = true;}
    }else{
      if (numPasos_MV < abs(pasosVerti)){
        digitalWrite(SENTIDO_MV, HIGH); //verificar
        motorVertical(2);
        numPasos_MV++;
      }else{romper_MV = true;}
    }
    
    if (romper_MH == true && romper_MV == true){
      numPasos_MH = 0;
      numPasos_MV = 0;
      break;
    }
    delay(1);
  }
  digitalWrite(ENABLE_MV, LOW);
  digitalWrite(ENABLE_MH, LOW);
  //control lazo abierto
  omega = omega_prima; 
  theta = theta_prima + decliMAG;
}



int motorHorizontal(int velocidad){
  for (int i=0;i<repeticiones;i++){
    digitalWrite(PASOS_MH, HIGH);          
    digitalWrite(PASOS_MH, LOW); 
    delay(velocidad);  
  }
}

void motorVertical(int velocidad){
  for(int i=0;i<repeticiones;i++){
    digitalWrite(PASOS_MV, HIGH);        
    digitalWrite(PASOS_MV, LOW); 
    delay(velocidad); 
  }
}



/*void buscarSetHorizontal(){
  while(analogRead(setHorizontal) < 900){
    digitalWrite(SENTIDO_MH, HIGH); //verificar debe girar a la derecha (vista lateral)
    motorHorizontal(5);
  }
}
*/
void buscarSetVertical(){
  while(analogRead(setVertical) < 900){
    digitalWrite(SENTIDO_MH, HIGH); //verificar debe girar a la derecha (vista lateral)
    motorVertical(2);
  }  
}

/*void medirAngulo(){ 
  float xx, yy, zz;
  long x,y,z;
  float mxNew = 0;
  float myNew = 0;
  int numFiltro = 4;
  for (int i=0; i < numFiltro; i++){
    Wire.beginTransmission(address);
    Wire.write(0x03); //select register 3, X MSB register
    Wire.endTransmission();
    Wire.requestFrom(address, 6);
    if(6<=Wire.available()){
        x = Wire.read()<<8; //X msb
        x |= Wire.read(); //X lsb
        z = Wire.read()<<8; //Z msb
        z |= Wire.read(); //Z lsb
        y = Wire.read()<<8; //Y msb
        y |= Wire.read(); //Y lsb
    }
    xx = (float)x;
    yy = (float)y;
    zz = (float)z;
    float data[3] = {xx, yy, zz};
    transformation(data);
    xx = calibrated_values[0];
    yy = calibrated_values[1];
    zz = calibrated_values[2];
    mxNew = mxNew + xx;
    myNew = myNew + yy;
    delayMicroseconds(100);
  }
  mx = (int)(mxNew/(numFiltro+1));
  my = (int)(myNew/(numFiltro+1));
  float declinacion = atan2(my, mx);
  //if(declinacion < 0){declinacion += 2*PI;}
  //if(declinacion > 2*PI){declinacion -= 2*PI;}
  declinacionGrados = declinacion * 180/M_PI;
  Serial.println(declinacionGrados);
}
*/

/*void buscarDesviacionCero(){
  int velocidad;
  while (1){
    medirAngulo();
    if (declinacionGrados < 0.3){
      //desviacionCero = true;
      break;
    }
    if (declinacionGrados < 360){velocidad = 1;}
    if (declinacionGrados < 200){velocidad = 10;}
    if (declinacionGrados < 40){velocidad = 50;}
    digitalWrite(SENTIDO_MH, HIGH); //verificar debe girar a la derecha (vista lateral)
    motorHorizontal(velocidad);
  }
}*/

/*void buscarDesviacionCero_prueba(){
  long velocidad;
  while(1){
    medirAngulo();
    Serial.println(declinacionGrados);
    if (declinacionGrados < 360){velocidad = 1;}
    if (declinacionGrados < 200){velocidad = 10;}
    if (declinacionGrados < 40){velocidad = 50;}
    if (declinacionGrados < 0.3){
      break;
    }
    MotorHorizontal_prueba();
    steps_MH--;
    if (steps_MH < 0){steps_MH = 7 ;}
    delay(velocidad);
  }
}*/

/*int stringToNumber(String thisString){
   int i, value, length;
   length = thisString.length();
   char blah[(length+1)];
   for(i=0; i<length; i++){
    blah[i] = thisString.charAt(i);    
   }    
   blah[i]=0;    
   value = atoi(blah);    
   return value; 
} */

/*void transformation(float uncalibrated_values[3]){
  double calibration_matrix[3][3] = 
  {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}  
  };
  double bias[3] = 
  {
    10,
    -220,
    -20
  };
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}*/

/*void medirBateria(){
  int voltBateria = analogRead(bateriaVolt);
  loggerBATERIA = voltBateria;
  if (voltBateria < 654){
    digitalWrite(bateriaLed, LOW);
  }else{
    digitalWrite(bateriaLed, HIGH);
  }
}*/

/*void loggerWrite(){
  logger = SD.open("loggerEstacionS3.txt", FILE_WRITE);
  if (logger){
    Serial.println("escribiendo");
    logger.print("bateria: ");
    logger.print(loggerBATERIA);
    logger.print("omega_prima: ");
    logger.print(loggerOMEGAPRIMA);
    logger.print("theta_prima: ");
    logger.println(loggerTHETAPRIMA);
    logger.close();
  }else{
    //digitalWrite(SDDmal, HIGH);
  }
  delay(40);
}*/



/*void conteoTiempoGuardar(){
  if(millis() > tiempoAnterior + periodo){  //si ha transcurrido el periodo programado
    loggerWrite();
    tiempoAnterior = millis();  //guarda el tiempo actual como referencia
  }
}*/

