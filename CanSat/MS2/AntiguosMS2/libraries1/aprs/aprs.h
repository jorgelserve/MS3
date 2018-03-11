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

void aprs_send();
void aprs_send_variables();
void load_temp(float temp);
void load_pres(float pres);
void load_alti(float alti);
void load_volt(float volt);
void load_NH3(float NH3);
void load_CO(float CO);
void load_NO2(float NO2);
void load_C3H8(float C3H8);
void load_C4H10(float C4H10);
void load_CH4(float CH4);
void load_H2(float H2);
void load_C2H5OH(float C2H5OH);
void load_tempC(float tempC);
void load_humidity(float humidity);

#endif
