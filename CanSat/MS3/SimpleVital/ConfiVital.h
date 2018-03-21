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
#include "BMP180.h"

//////////////////////////// SD librarie
#include <SD.h>


////////////////////////////////////////////// SimpleVital Pin Distribution //////////////////////////////////////////////
////////////////////////////////////////////// Trackduino PinOut

////////////////////////////////////////////// SimpleVital PinOut
//////////////////////////// SD
const int chipSelect = 38;

//////////////////////////// GPS
const int gpsPPS_PIN = 25;
const int gpsRESET_PIN = 24;

//////////////////////////// Buzzer
const int pinbuzzer = 26;

//////////////////////////// Info Leds
const int LedSD_PIN = 13;
//const int LedRUN_PIN = 12;
//const int LedTX_PIN = 9;  // el pin 9 es usado por el radio

//////////////////////////// Despliegue paneles
const byte pinPanel0 = 30;
const byte pinPanel1 = 32;

//////////////////////////// Voltaje Bateria
const byte pinVolBat1 = 3;    //analogo
//////////////////////////// Temperatura Radio
const byte pinTempRad = 0;    //analogo
//////////////////////////// Temperatura PCB
const byte pinTempPCB = 1;    //analogo

///////////////////////// pin analogo temperatura
const byte pinTemp1 = 13;
const byte pinTemp2 = 12;
const byte pinTemp3 = 15;



////////////////////////////////////////////// SimpleVital Variables //////////////////////////////////////////////
//////////////////////////// FreeRTOS
// Declare a semaphore handle.
//SemaphoreHandle_t sem;

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
float temperature;
float pressure;
float atm;
float altitud;

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
const int pinPeriodo = 23;

//////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Setup /////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
