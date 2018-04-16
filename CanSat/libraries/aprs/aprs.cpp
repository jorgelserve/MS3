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

#include "config.h"
#include "ax25.h"
#include "gps.h"
#include "aprs.h"
//#include "sensors_avr.h"
//#include "sensors_pic32.h"
#include <stdio.h>
#include <stdlib.h>
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif

// Trama Corta
int altitu = 100;
int temperatura = 20.5 ;
int tempC = 0;
int voltajeBateria1 = 5;

// Trama Larga G
int humidity = 0;
long NH3 = 0;
long CO = 0;
long NO2 = 0;
long C3H8 = 0;
long C4H10 = 0;
long CH4 = 0;
long H2 = 0;
long C2H5OH = 0;
int tempi2c = 0;
int tempADC = 0;
long pressur = 10;

// Trama Larga IMU
long Accx = 0;
long Accy= 0;
long Accz = 0;
long Gyrx = 0;
long Gyry= 0;
long Gyrz = 0;
long Magx = 0;
long Magy= 0;
long Magz = 0;


const struct s_address addresses[] = {
    {D_CALLSIGN, D_CALLSIGN_ID},  // Destination callsign
    {S_CALLSIGN, S_CALLSIGN_ID},  // Source callsign (-11 = balloon, -9 = car)
#ifdef DIGI_PATH1
    {DIGI_PATH1, DIGI_PATH1_TTL}, // Digi1 (first digi in the chain)
#endif
#ifdef DIGI_PATH2
    {DIGI_PATH2, DIGI_PATH2_TTL}, // Digi2 (second digi in the chain)
#endif
};

//////////////////////////////////////////////////////////////////////////////////////////////////////
// Trama corta
void load_alti(int alti) {altitu = alti;}
void load_temp(int temp){temperatura =  temp;} //Temperatura Barometro
void load_tempC(int tempC_){tempC = tempC_;}
void load_volt(int volt) {voltajeBateria1 = volt;}

// Trama Larga Gases
void load_humidity(int humidity_){humidity = humidity_;}

void load_NH3(long NH3_){NH3 = NH3_;}
void load_CO(long CO_){CO = CO_;}
void load_NO2(long NO2_){NO2 = NO2_;}
void load_C3H8(long C3H8_){C3H8 = C3H8_;}
void load_C4H10(long C4H10_){C4H10 = C4H10_;}
void load_CH4(long CH4_){CH4 = CH4_;}
void load_H2(long H2_){H2 = H2_;}
void load_C2H5OH(long C2H5OH_){C2H5OH = C2H5OH_;}

void load_tempi2c(int tempi2c_){tempi2c = tempi2c_;}
void load_tempADC(int tempADC_){tempADC = tempADC_;}
void load_pres(long pres) {pressur = pres;}

// Trama Larga IMU
void load_Accx (long Accxe){Accx = Accxe;}
void load_Accy (long Accye){Accy = Accye;}
void load_Accz (long Accze){Accz = Accze;}
void load_Gyrx (long Gyrxe){Gyrx = Gyrxe;}
void load_Gyry (long Gyrye){Gyry = Gyrye;}
void load_Gyrz (long Gyrze){Gyrz = Gyrze;}
void load_Magx (long Magxe){Magx = Magxe;}
void load_Magy (long Magye){Magy = Magye;}
void load_Magz (long Magze){Magz = Magze;}


// Module functions
int meters_to_feet(int m){return m / 0.3048;} // 10000 ft = 3048 m


////////////////////////////////////////// Funciones para el envio de las tramas ////////////////////////////////////////////////////////////
// Trama Corta
inline void TramaCRadioA(){
  char dato[12];                   // datoerature (int/ext)

  ax25_send_header(addresses, sizeof(addresses) / sizeof(s_address));
  ax25_send_byte('/');                // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
  // ax25_send_string("021709z");     // 021709z = 2nd day of the month, 17:09 zulu (UTC/GMT)
  ax25_send_string(gps_time);         // 170915 = 17h:09m:15s zulu (not allowed in Status Reports)
  ax25_send_byte('h');
  ax25_send_string(gps_aprs_lat);     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
  ax25_send_byte('/');                // Symbol table
  ax25_send_string(gps_aprs_lon);     // Lon: 000deg and 25.80 min
  ax25_send_byte('O');                // Symbol: O=balloon, -=QTH

  snprintf(dato, 4, "%03d", (int)(gps_course + 0.5));
  ax25_send_string(dato);
  ax25_send_byte('/');                // and
  snprintf(dato, 4, "%03d", (int)(gps_speed + 0.5));
  ax25_send_string(dato);
  ax25_send_string("A");            // Altitude (feet). Goes anywhere in the comment area
  snprintf(dato, 7, "%d", (int)gps_altitude);
  ax25_send_string(dato);
  ax25_send_string("B");
  snprintf(dato, 6, "%d", altitu); // Altura Barometrica
  ax25_send_string(dato);
  ax25_send_string("C");
  snprintf(dato, 7, "%d", temperatura); // Temp Barometro
  ax25_send_string(dato);
  ax25_send_string("D");
  snprintf(dato, 7, "%d", tempC); // Temp SHT11
  ax25_send_string(dato);
  ax25_send_string("E");
  snprintf(dato, 6, "%d", voltajeBateria1);
  ax25_send_string(dato);
}

// Trama Corta
void enviarTramaCRadioA(){
  TramaCRadioA();
  finTramaRadioA();
}

// Trama larga Gases
void enviarTramaGRadioA(){
  TramaCRadioA();

  char data[12];                   // dataerature (int/ext)
  // Datos Trama larga1
  ax25_send_string("F");
  snprintf(data, 7, "%d", humidity);
  ax25_send_string(data);

  // Sensor Gases (Promedio)
  ax25_send_string("G");
  snprintf(data, 7, "%d", NH3);
  ax25_send_string(data);
  ax25_send_string("H");
  snprintf(data, 7, "%d", CO);
  ax25_send_string(data);
  ax25_send_string("I");
  snprintf(data, 7, "%d", NO2);
  ax25_send_string(data);
  ax25_send_string("J");
  snprintf(data, 7, "%d", C3H8); // extraña
  ax25_send_string(data);
  ax25_send_string("K");
  snprintf(data, 7, "%d", C4H10); // extraña
  ax25_send_string(data);
  ax25_send_string("L");
  snprintf(data, 7, "%d", CH4); // extraña
  ax25_send_string(data);
  ax25_send_string("M");
  snprintf(data, 7, "%d", H2);
  ax25_send_string(data);
  ax25_send_string("N");
  snprintf(data, 7, "%d", C2H5OH);
  ax25_send_string(data);

  // Temp
  ax25_send_string("O");
  snprintf(data, 7, "%d", tempi2c);
  ax25_send_string(data);
  ax25_send_string("P");
  snprintf(data, 7, "%d", tempADC);
  ax25_send_string(data);
  ax25_send_string("Q");
  snprintf(data, 7, "%d", pressur);
  ax25_send_string(data);
  
  finTramaRadioA();
}

// Trama larga IMU
void enviarTramaIRadioA(){
  TramaCRadioA();

  char data[12];
  // Aceleraciones
  ax25_send_string("f");
  snprintf(data, 7, "%d", Accx);
  ax25_send_string(data);
  ax25_send_string("G");
  snprintf(data, 7, "%d", Accy);
  ax25_send_string(data);
  ax25_send_string("H");
  snprintf(data, 7, "%d", Accz);
  ax25_send_string(data);
  ax25_send_string("I");
  // Giroscopo
  snprintf(data, 7, "%d", Gyrx);
  ax25_send_string(data);
  ax25_send_string("J");
  snprintf(data, 7, "%d", Gyry);
  ax25_send_string(data);
  ax25_send_string("K");
  snprintf(data, 7, "%d", Gyrz);
  ax25_send_string(data);
  ax25_send_string("L");
  // Magnetometro
  snprintf(data, 7, "%d", Magx);
  ax25_send_string(data);
  ax25_send_string("M");
  snprintf(data, 7, "%d", Magy);
  ax25_send_string(data);
  ax25_send_string("N");
  snprintf(data, 7, "%d", Magz);
  ax25_send_string(data);

  finTramaRadioA();
}

// fin tramas
inline void finTramaRadioA(){
  ax25_send_footer();
  ax25_flush_frame();                 // Tell the modem to go
}