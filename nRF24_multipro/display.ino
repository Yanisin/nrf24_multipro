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
	oled.setInverseDisplay();
	oled.setTextXY(0,0);              // Set cursor position, start of line 0
	oled.putString("Hello there");
	delay(200);
}

#endif
