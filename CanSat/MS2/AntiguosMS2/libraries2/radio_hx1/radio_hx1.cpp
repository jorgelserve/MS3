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

#include "config.h"
#include "pin.h"
#include "radio_hx1.h"
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif
//#include "confiRadio.h"


void RadioHx1::setup(){
  // Configure pins
  pinMode(PTT_PIN, INPUT); 
  pinMode(ERX_PIN, OUTPUT); 
  pinMode(AUDIO_PIN, OUTPUT);

  pin_write(ERX_PIN, LOW); // Init with the radio RX enabled and tx disabled
  
}

void RadioHx1::ptt_on(){	
	pinMode(ERX_PIN, INPUT);
	pinMode(PTT_PIN, OUTPUT);
 	pin_write(PTT_PIN, LOW);
 	delay(50);   // The HX1 takes 5 ms from PTT to full RF, (give it 10)
}

void RadioHx1::ptt_off(){
	pinMode(PTT_PIN, INPUT);
	pinMode(ERX_PIN, OUTPUT); 
	pin_write(ERX_PIN, LOW); // When not in Tx mode, switch to listening, means RX enable
}
