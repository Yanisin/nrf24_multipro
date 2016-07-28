/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef DISPLAY_IFACE

#include <Wire.h>
#include <ACROBOTIC_SSD1306.h>

void display_init(void)
{
	Wire.begin();	
	oled.init();                      // Initialze SSD1306 OLED display
	oled.clearDisplay();              // Clear screen

	oled.setTextXY(0, 0);
	oled.putString("multipro alive");

	oled.setTextXY(2, 0);
	oled.putString("Init... Proto or");
	oled.setTextXY(3, 0);
	oled.putString("throttle down");
}

void display_update(void)
{
	oled.setTextXY(0, 0);
	oled.putString(current_protocol_str);
	oled.setTextXY(1, 0);
	oled.putString("Mode");
	oled.setTextXY(1, 5);
	oled.putNumber(aux_mode);

	oled.setTextXY(2, 0);
	oled.putString("Ail____ Ele____");

	oled.setTextXY(2, 3);
	oled.putNumber(trim[AILERON]);
	oled.setTextXY(2, 11);
	oled.putNumber(trim[ELEVATOR]);

	oled.setTextXY(3, 0);
	oled.putString("Measured bias");
	oled.setTextXY(4, 0);
	oled.putString("Ail____ Ele____");
	oled.setTextXY(4, 3);
	oled.putNumber(ppm_bias[AILERON]);
	oled.setTextXY(4, 11);
	oled.putNumber(ppm_bias[ELEVATOR]);

	oled.setTextXY(5, 0);
	oled.putString("Rudder div 2/");
	oled.setTextXY(5, 14);
	oled.putNumber(rudder_div);
}

#endif
