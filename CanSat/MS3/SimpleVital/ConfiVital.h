/* trackuino copyright (C) 2010  EA5HAV Javi

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

// SimpleVital by SimpleSpace

// -> based on trackuino

////////////////////////////////////////////// Check Compiler //////////////////////////////////////////////
#if (ARDUINO + 0) == 0
#error "Oops! We need the real Arduino IDE (version 22 or 23) for Arduino builds."
#error "See trackuino.pde for details on this"

// Refuse to compile on arduino version 21 or lower. 22 includes an
// optimization of the USART code that is critical for real-time operation
// of the AVR code.
#elif (ARDUINO + 0) < 22
#error "Oops! We need Arduino 22 or 23"
#error "See trackuino.pde for details on this"

#endif

// Arduino/AVR libs
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif

////////////////////////////////////////////// Trackduino Libraries //////////////////////////////////////////////

// Trackuino custom libs
#include "config.h"

#include "aprs.h"
#include "afsk_avr.h"

#include "I2Cdev.h"
#include "gps.h"
#include "pin.h"
#include "power.h"

////////////////////////////////////////////// SimpleVital Libraries //////////////////////////////////////////////
//////////////////////////// Imu libraries
#include "MPU9250.h"

//////////////////////////// Gas libraries
#include "MutichannelGasSensor.h"
MutichannelGasSensor gas04;
MutichannelGasSensor gas05;

//////////////////////////// SHT11 libraries
#include "SHT1x.h"

//////////////////////////// Barometro libraries
#include <Adafruit_BMP280.h>
Adafruit_BMP280 baro_BMP280B;
Adafruit_BMP280 baro_BMP280T;

//////////////////////////// SD librarie
#include <SD.h>


////////////////////////////////////////////// Declaraciones /////////////////////////////////////////////////////////////
#define Simple 3          // Seleccionamos la simple con la estamos trabajando
#define RadioSerial 1     // Radio Serial/analogo
#define Silencio  1       // Cambia el buzzer(0) por led(1)


////////////////////////////////////////////// SimpleVital Pin Distribution //////////////////////////////////////////////
//////////////////////////// SD
#define chipSelect 38

//////////////////////////// GPS
#define gpsForceOn_PIN 25
#define gpsRESET_PIN 24

//////////////////////////// Buzzer
#define buzzer_PIN 26

//////////////////////////// SHT11
#define dataPin 6     // SHT11 serial data
#define clockPin 7    // SHT11 serial clock

//////////////////////////// Info Leds
#define LedSD_PIN 13 // PH7(T4)
//#define Led13 13
#define LedRUN_PIN 12
#define LedRAD_PIN 8

//////////////////////////// Despliegue paneles
#define pinPanel0 30
#define pinPanel1 32

//////////////////////////// Analogos //////////////////////////// 
//////////////////////////// Temperatura Radio
#define pinTempRad 0    //analogo
//////////////////////////// Temperatura PCB
#define pinTempPCB 1    //analogo
//////////////////////////// Voltaje Bateria
#define pinVolBat1 3    //analogo

///////////////////////// Pines Temperaturas Externas
#define pinTemp1 13
#define pinTemp2 12
#define pinTemp3 15

////////////////////////////////////////////// SimpleVital Variables //////////////////////////////////////////////
// ----------------------------- Gases 04
char band_gas04 = 0;
int dim04;
float c04[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int cSD04[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte error04;
byte error_gas04 = 0;
// ----------------------------- Gases 05
char band_gas05 = 0;
int dim05;
float c05[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int cSD05[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte error05;
byte error_gas05 = 0;

// ----------------------------- Humedad y Temperatura (SHT11)
int tempSD;
int humSD;
//---
SHT1x sht1x(dataPin, clockPin);


// ----------------------------- Tempertura PCB (I2C)
// Sensor PCB Vital
#define i2cAddressV 0x18
#define add_reg 0x05
//---
int tempi2cSDV;

// Sensor PCB Gases
#define i2cAddressG 0x19
//#define add_regG 0x05
//---
int tempi2cSDG;

int tempPromPCB = 0;
int tempPCB = 0; // Temperatura analoga PCBs

// ----------------------------- Temperatura Externa
int tempext1SD;
int tempext2SD;
int tempext3SD;
//#define ana2tem 0.0048828125

// ----------------------------- Apogeo
#define alt_apogeo 4000    // altura por encima de la que inicia el conteo
#define apogeo_time 360000  // tiempo en mili-segundos desde que pasa altura
//---
byte band_buzzer = 0;
long int fall_time = 0;
byte band_apogeo = 0;
byte ban_apogeo_active = 0;

// ----------------------------- liberacion Paneles
#define panel_time 24      // Tiempo en segundos
#define alt_paneles 80      // Altura en metros antes de despliegue 
//---
float weight = 0;
byte band_paneles = 0;
char band_transmission = 0;

//Configuracion Simple 144/434
//#define AnalogosExt // 144

int voltajeBateria0 = 0;

//////////////////////////// SD
bool sd_ok = false;
File dataFile;

//////////////////////////// Buzzer
boolean buzzer_est = false;

//////////////////////////// IMU
// Imu Inicialization
MPU9250 accelgyro;
//BMP180 Barometer;

// Puerto I2C
I2Cdev   I2C_M;

// Imu raw Variables
uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

// barometer Variables
float BarB_temp;
float BarB_pres;
float BarB_atmo;
float BarB_alti;

float BarT_temp;
float BarT_pres;
float BarT_atmo;
float BarT_alti;

////////////////////////////////////////////// SimpleVital Parameters //////////////////////////////////////////////

#define sample_num_mdate  5000

// Module constants
static const uint32_t VALID_POS_TIMEOUT = 2000;  // ms

// Module variables
static int32_t next_aprs = 0;


////////////////////////////////////////////// Debuggin //////////////////////////////////////////////
//////////////////////// Preprocesor
# define DEBUG_SERIAL_SPEED 115200

# define DEBUG 1
// levels of debuggin
// 0 - All processes
// 1 - Only Inicializations
// Less important task

#define GAS_SENSOR  0

//////////////////////// Runtime
// time outs (milliseconds)
uint32_t GPS_timeout = 5000;
int APRS_PERIOD = 10;  // tiempo en segundos entre transmision
//#define pinPeriodo 23


