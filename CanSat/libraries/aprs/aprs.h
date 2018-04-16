/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __APRS_H__
#define __APRS_H__

// Trama corta
void load_alti(int alti);
void load_temp(int temp);
void load_tempC(int tempC);
void load_volt(int volt);

// Trama Larga Gases
void load_humidity(int humidity);
void load_NH3(long NH3);
void load_CO(long CO);
void load_NO2(long NO2);
void load_C3H8(long C3H8);
void load_C4H10(long C4H10);
void load_CH4(long CH4);
void load_H2(long H2);
void load_C2H5OH(long C2H5OH);
void load_tempi2c(int tempi2c);
void load_tempADC(int tempADC);
void load_pres(long pres);

// Trama Larga IMU
void load_Accx (long Accx);
void load_Accy (long Accy);
void load_Accz (long Accz);
void load_Gyrx (long Gyrx);
void load_Gyry (long Gyry);
void load_Gyrz (long Gyrz);
void load_Magx (long Magx);
void load_Magy (long Magy);
void load_Magz (long Magz);

// Envio Tramas
void TramaCRadioA();
void enviarTramaCRadioA();
void enviarTramaGRadioA();
void enviarTramaIRadioA();
void finTramaRadioA();

#endif
