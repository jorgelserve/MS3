// SimpleRadio by SimpleSpace
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
#include "pin.h"
#include "power.h"

////////////////////////////////////////////// SimpleRadioV2 Libraries //////////////////////////////////////////////
// Radio Usado: NiM2-434.650-10
////////////////////////////////////////////// SimpleRadioV2 Pin Distribution //////////////////////////////////////////////
////////// Usados por Trackuino
// pin Salida audio -> 9
// pin PTT TX -> 11
////////// used by SimpleRadioV2
#define AF_PIN     0   // Analog Audio Imput
#define RSSI_PIN   1   // Rx Signal intensity
#define RXD_PIN    11  // Bicode Rx Data
//#define ERX_PIN    13  // Radio RX Enable Pin 


//////////////////////////// temperaturas Radio
// externas
const byte pinTempExt = 3;    //analogo
// internas
const byte pinTempInt = 4;    //analogo

////////////////////////////////////////////// SimpleRadioV2 Variables //////////////////////////////////////////////


////////////////////////////////////////////// SimpleRadioV2 Parameters //////////////////////////////////////////////

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

//////////////////////// Runtime
// time outs (milliseconds)
uint32_t TXtimeout = 5000;
int APRS_PERIOD = 5;  // tiempo en segundos entre transmision

