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

float temperatura = 20.5 ;
float pressur = 10;
float altitu = 100;
float voltajeBateria1 = 5;
float NH3 = 0;
float CO = 0;
float NO2 = 0;
float C3H8 = 0;
float C4H10 = 0;
float CH4 = 0;
float H2 = 0;
float C2H5OH = 0;
float tempC = 0;
float humidity = 0;
float tempi2c = 0;
float tempADC = 0;

void load_tempi2c(float tempi2c_){
  tempi2c = tempi2c_;
}

void load_CO(float CO_){
  CO = CO_;
}
void load_NO2(float NO2_){
  NO2 = NO2_;
}
void load_C3H8(float C3H8_){
  C3H8 = C3H8_;
}
void load_C4H10(float C4H10_){
  C4H10 = C4H10_;
}
void load_CH4(float CH4_){
  CH4 = CH4_;
}
void load_H2(float H2_){
  H2 = H2_;
}
void load_C2H5OH(float C2H5OH_){
  C2H5OH = C2H5OH_;
}
void load_NH3(float NH3_)
{
  NH3 = NH3_;
}
void load_tempC(float tempC_){
  tempC = tempC_;
}
void load_humidity(float humidity_){
  humidity = humidity_;
}
void load_temp(float temp)
{
  // 10000 ft = 3048 m
  temperatura =  temp;
}

void load_pres(float pres) {
  pressur = pres;
}
void load_alti(float alti) {
  altitu = alti;
}

void load_volt(float volt) {
  voltajeBateria1 = volt;
}
void load_tempADC(float tempADC_){
  tempADC = tempADC_;
}

// Module functions
float meters_to_feet(float m)
{
  // 10000 ft = 3048 m
  return m / 0.3048;
}

// Exported functions
void aprs_send()
{
  char temp[12];                   // Temperature (int/ext)
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

  ax25_send_header(addresses, sizeof(addresses) / sizeof(s_address));
  ax25_send_byte('/');                // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
  // ax25_send_string("021709z");     // 021709z = 2nd day of the month, 17:09 zulu (UTC/GMT)
  ax25_send_string(gps_time);         // 170915 = 17h:09m:15s zulu (not allowed in Status Reports)
  ax25_send_byte('h');
  ax25_send_string(gps_aprs_lat);     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
  ax25_send_byte('/');                // Symbol table
  ax25_send_string(gps_aprs_lon);     // Lon: 000deg and 25.80 min
  ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
  // cambiar temp por dato
  snprintf(temp, 4, "%03d", (int)(gps_course + 0.5));
  ax25_send_string(temp);
  ax25_send_byte('/');                // and
  snprintf(temp, 4, "%03d", (int)(gps_speed + 0.5));
  ax25_send_string(temp);
  ax25_send_string("A");            // Altitude (feet). Goes anywhere in the comment area
  snprintf(temp, 7, "%d", (int)(gps_altitude));
  ax25_send_string(temp);
  ax25_send_string("T");
  snprintf(temp, 6, "%d", (int)(temperatura * 10));
  ax25_send_string(temp);
  ax25_send_string("P");
  snprintf(temp, 6, "%d", (int)pressur);
  ax25_send_string(temp);
  ax25_send_string("A");
  snprintf(temp, 6, "%d", (int)altitu);
  ax25_send_string(temp);
  ax25_send_string("V");
  snprintf(temp, 6, "%d", (int)(voltajeBateria1 * 10));
  ax25_send_string(temp);
  ax25_send_byte(' ');
  ax25_send_string(APRS_COMMENT);     // Comment
  ax25_send_footer();

  ax25_flush_frame();                 // Tell the modem to go
}

// Exported functions
void aprs_send_variables()
{
  char temp[12];                   // Temperature (int/ext)
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

  ax25_send_header(addresses, sizeof(addresses) / sizeof(s_address));
  ax25_send_byte('/');                // Report w/ timestamp, no APRS messaging. $ = NMEA raw data
  // ax25_send_string("021709z");     // 021709z = 2nd day of the month, 17:09 zulu (UTC/GMT)
  //ax25_send_byte('t');
  ax25_send_string(gps_time);         // 170915 = 17h:09m:15s zulu (not allowed in Status Reports)
  ax25_send_byte('h');
  ax25_send_string(gps_aprs_lat);     // Lat: 38deg and 22.20 min (.20 are NOT seconds, but 1/100th of minutes)
  ax25_send_byte('/');                // Symbol table
  ax25_send_string(gps_aprs_lon);     // Lon: 000deg and 25.80 min
  ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
  // cambiar temp por dato
  snprintf(temp, 4, "%03d", (int)(gps_course + 0.5));
  ax25_send_string(temp);
  ax25_send_byte('/');                // and
  snprintf(temp, 4, "%03d", (int)(gps_speed + 0.5));
  ax25_send_string(temp);
  ax25_send_string("A");            // Altitude (feet). Goes anywhere in the comment area
  snprintf(temp, 7, "%d", (int)(gps_altitude));
  ax25_send_string(temp);
  ax25_send_string("B");
  snprintf(temp, 7, "%d", (int)(NH3 * 100));
  ax25_send_string(temp);
  ax25_send_string("C");
  snprintf(temp, 7, "%d", (int)(CO * 100));
  ax25_send_string(temp);
  ax25_send_string("D");
  snprintf(temp, 7, "%d", (int)(NO2 * 100));
  ax25_send_string(temp);
  ax25_send_string("E");
  snprintf(temp, 7, "%d", (int)(C3H8 * 100));
  ax25_send_string(temp);
  ax25_send_string("F");
  snprintf(temp, 7, "%d", (int)(C4H10 * 100));
  ax25_send_string(temp);
  ax25_send_string("G");
  snprintf(temp, 7, "%d", (int)(CH4 * 100));
  ax25_send_string(temp);
  ax25_send_string("H");
  snprintf(temp, 7, "%d", (int)(H2 * 100));
  ax25_send_string(temp);
  ax25_send_string("I");
  snprintf(temp, 7, "%d", (int)(C2H5OH * 100));
  ax25_send_string(temp);
  ax25_send_string("J");
  snprintf(temp, 7, "%d", (int)(tempC * 100));
  ax25_send_string(temp);
  ax25_send_string("K");
  snprintf(temp, 7, "%d", (int)(humidity * 100));
  ax25_send_string(temp);
  ax25_send_string("L");
  snprintf(temp, 7, "%d", (int)(tempi2c * 100));
  ax25_send_string(temp);
  ax25_send_string("M");
  snprintf(temp, 7, "%d", (int)(tempADC * 100));
  ax25_send_string(temp);
  ax25_send_byte(' ');
  ax25_send_string(APRS_COMMENT);     // Comment
  ax25_send_footer();

  ax25_flush_frame();                 // Tell the modem to go
}
