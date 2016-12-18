uint8_t aux_mode = 0;
uint8_t rudder_div = 2;
uint8_t processButtons = 1;
uint8_t reinit_proto = 0;
uint8_t save_trims = 0;

#define PPM_SCALE ((PPM_MAX-PPM_MIN)/(PPM_MAX_A - PPM_MIN_A))
#define PPM_RANGE (PPM_MAX-PPM_MIN)
#define PPM_RANGE_A (PPM_MAX_A-PPM_MIN_A)

#define PPM_BIAS_READ_COUNT (10)
#define PPM_BIAS_READ_DELAY (10)



enum {
	BUTTON_NONE = 0,

	BUTTON_R_UP,
	BUTTON_R_DOWN,
	BUTTON_R_LEFT,
	BUTTON_R_RIGHT,

	BUTTON_L_UP,
	BUTTON_L_DOWN,
	BUTTON_L_LEFT,
	BUTTON_L_RIGHT,

	BUTTON_F_LEFT,
	BUTTON_F_RIGHT,

	BUTTON_COUNT
};
const uint16_t btns_range = 20;

const uint16_t btns[BUTTON_COUNT] = {
	[BUTTON_NONE] = 1023,

	[BUTTON_R_UP] = btns_range, // ~= 0
	[BUTTON_R_DOWN] = 92,
	[BUTTON_R_LEFT] = 342,
	[BUTTON_R_RIGHT] = 514,

	[BUTTON_L_UP] = 170,
	[BUTTON_L_DOWN] = 236,
	[BUTTON_L_LEFT] = 385,
	[BUTTON_L_RIGHT] = 293,

	[BUTTON_F_LEFT] = 457,
	[BUTTON_F_RIGHT] = 683,
};


enum {
	BUTTON_FLIP1 = BUTTON_F_RIGHT,
	BUTTON_FLIP2 = BUTTON_F_LEFT,
	BUTTON_MODE = BUTTON_L_LEFT,
	BUTTON_RUDDER_DIV = BUTTON_L_RIGHT,
	BUTTON_CALIBRATE_BIAS = BUTTON_L_DOWN,
	
	BUTTON_TRIM_AILERON_MINUS = BUTTON_R_LEFT,
	BUTTON_TRIM_AILERON_PLUS = BUTTON_R_RIGHT,
	BUTTON_TRIM_ELEVATOR_MINUS = BUTTON_R_DOWN,
	BUTTON_TRIM_ELEVATOR_PLUS = BUTTON_R_UP,

#ifdef DISPLAY_IFACE
	BUTTON_MENU = BUTTON_L_UP,
	BUTTON_MENU_UP = BUTTON_R_UP,
	BUTTON_MENU_DOWN = BUTTON_R_DOWN,
	BUTTON_MENU_PLUS = BUTTON_R_RIGHT,
	BUTTON_MENU_MINUS = BUTTON_R_LEFT,
#endif
};
static uint8_t btn, btn_last = BUTTON_NONE;

static int8_t ppm_bias[4];

void analogPPM_init(void)
{
	// try loading previous settings from EEPROM
	aux_mode = constrain(EEPROM.read(ee_PPM_MODE), 0, 2);
	rudder_div = constrain(EEPROM.read(ee_PPM_RUDDER_DIV), 2, 6);
	ppm_bias[ELEVATOR] = constrain((signed)EEPROM.read(ee_PPM_BIAS_ELEVATOR) - 100, -100, 100);
	ppm_bias[AILERON] = constrain((signed)EEPROM.read(ee_PPM_BIAS_AILERON) - 100, -100, 100);

#ifdef DISPLAY_IFACE
	reinit_proto = constrain(EEPROM.read(ee_REINIT_PROTO), 0, 1);
#endif
	save_trims = constrain(EEPROM.read(ee_SAVE_TRIMS), 0, 1);
	if (save_trims) {
		trim[ELEVATOR] = constrain((signed)EEPROM.read(ee_TRIM_ELEVATOR) - 100, -100, 100);
		trim[AILERON] = constrain((signed)EEPROM.read(ee_TRIM_AILERON) - 100, -100, 100);
		if (trim[ELEVATOR] % 5)
			trim[ELEVATOR] = 0;
		if (trim[AILERON] % 5)
			trim[AILERON] = 0;
	}
}

void savePPM(void)
{
	EEPROM.update(ee_PPM_MODE, aux_mode);
	EEPROM.update(ee_PPM_RUDDER_DIV, rudder_div);
	EEPROM.update(ee_PPM_BIAS_ELEVATOR, ppm_bias[ELEVATOR] + 100);
	EEPROM.update(ee_PPM_BIAS_AILERON, ppm_bias[AILERON] + 100);
}

#ifdef DISPLAY_IFACE
enum {
	MENU_ITEM_BOOL = 0,
	MENU_ITEM_UINT8,
	MENU_ITEM_INT8
};
struct {
	const char *name;
	uint8_t type;
	void *data;
} menuItems[] = {
	{ "Bayang/DTRIM", MENU_ITEM_BOOL, &Bayang_dyntrim },
	{ "Default proto", MENU_ITEM_UINT8, &current_protocol },
	{ "Reinit proto", MENU_ITEM_BOOL, &reinit_proto },
	{ "Save trims", MENU_ITEM_BOOL, &save_trims },
};
const uint8_t noItems = sizeof(menuItems) / sizeof(menuItems[0]);
uint8_t curItem = 0;
const uint8_t itemsPerPage = 6;

void showMenu(void)
{
	uint8_t i;
	const uint8_t first = (curItem / itemsPerPage) * itemsPerPage;
	const uint8_t last = min(noItems, (curItem / itemsPerPage) * itemsPerPage + itemsPerPage);

	oled.clearDisplay();
	for (i = first; i < last; i++) {
		oled.setTextXY(i, 0);
		oled.putString(menuItems[i].name);
		oled.setTextXY(i, strlen(menuItems[i].name) + 1);
		if (menuItems[i].type == MENU_ITEM_INT8)
			oled.putNumber(*(int8_t*)menuItems[i].data);
		else
			oled.putNumber(*(uint8_t*)menuItems[i].data);
	}

	oled.setTextXY(curItem - first, 15);
	oled.putString("*");
}

void doMenu(void)
{
	showMenu();

	processButtons = 0;
	while (1) {
		loop();

		if (btn != BUTTON_NONE) {
			if (btn_last != BUTTON_NONE) {
				btn_last = btn;
				continue;
			}
			tone(BUZ_PIN, 1000, 10);
			btn_last = btn;
		} else {
			btn_last = BUTTON_NONE;
			continue;
		}

		switch (btn) {
		case BUTTON_MENU:
			goto exit;
		case BUTTON_MENU_UP:
			if (curItem > 0)
				curItem--;
			else
				curItem = noItems - 1;
			break;
		case BUTTON_MENU_DOWN:
			if (curItem + 1 < noItems)
				curItem++;
			else
				curItem = 0;
			break;
		case BUTTON_MENU_PLUS:
			switch (menuItems[curItem].type) {
			case MENU_ITEM_BOOL:
				*(uint8_t*)menuItems[curItem].data = !*(uint8_t*)menuItems[curItem].data;
				break;
			case MENU_ITEM_UINT8:
				*((uint8_t*)menuItems[curItem].data) += 1;
				break;
			case MENU_ITEM_INT8:
				*((int8_t*)menuItems[curItem].data) += 1;
				break;
			}
			break;
		case BUTTON_MENU_MINUS:
			switch (menuItems[curItem].type) {
			case MENU_ITEM_BOOL:
				*(uint8_t*)menuItems[curItem].data = !*(uint8_t*)menuItems[curItem].data;
				break;
			case MENU_ITEM_UINT8:
				*((uint8_t*)menuItems[curItem].data) -= 1;
				break;
			case MENU_ITEM_INT8:
				*((int8_t*)menuItems[curItem].data) -= 1;
				break;
			}
			break;
		}

		showMenu();
	}

exit:
	processButtons = 1;

	/* store settings to eeprom */
	EEPROM.update(ee_BAYANG_DISABLE_DYNTRIM, !Bayang_dyntrim);
	EEPROM.update(ee_PROTOCOL_ID, current_protocol);
	EEPROM.update(ee_REINIT_PROTO, reinit_proto);
	EEPROM.update(ee_SAVE_TRIMS, save_trims);

	oled.clearDisplay();
	display_update();

	if (reinit_proto) {
		init_protocol();
	}
}
#endif

// update ppm values out of ISR    
void update_ppm()
{
  static uint32_t thr,ail,ele,rud,aux1;
  static uint8_t i;

  thr = analogRead(THR_PIN);
  ail = analogRead(AIL_PIN);   
  ele = analogRead(ELE_PIN);   
  rud = analogRead(RUD_PIN);   
  aux1 = analogRead(BTN_PIN);
#ifdef DEBUG_ANALOG_PPM
  Serial.write('\r');
       Serial.write('\n');
       Serial.write('a');
       Serial.write('d');
       Serial.write('c');
       Serial.write(':');

     Serial.print(thr);
        Serial.write(' ');
Serial.print(ail);
        Serial.write(' ');
Serial.print(ele);
        Serial.write(' ');
Serial.print(rud);
        Serial.write(' ');
Serial.print(aux1);
        Serial.write(' ');

#endif

  ppm[THROTTLE] = (int)(PPM_MIN + (thr * PPM_RANGE)/PPM_RANGE_A);
  ppm[AILERON] = constrain((int)(PPM_MIN + (ail * PPM_RANGE)/PPM_RANGE_A) + ppm_bias[AILERON] + trim[AILERON], PPM_MIN, PPM_MAX);
  ppm[ELEVATOR] = constrain((int)(PPM_MIN + (ele * PPM_RANGE)/PPM_RANGE_A) + ppm_bias[ELEVATOR] + trim[ELEVATOR], PPM_MIN, PPM_MAX);
  ppm[RUDDER] = (int)(PPM_MIN + (rud * PPM_RANGE)/PPM_RANGE_A);

  /* 1/1 ... 1/3 */
  if (rudder_div != 2)
      ppm[RUDDER] = signed(ppm[RUDDER] - PPM_MID) * 2 / rudder_div + PPM_MID;

  //ppm[AUX1] = (int)(PPM_MIN + (aux1 * PPM_RANGE)/PPM_RANGE_A);

  btn = BUTTON_NONE;
	for (i = 0; i < BUTTON_COUNT; i++) {
	  if (aux1 >= (btns[i] - btns_range) && aux1 <= (btns[i] + btns_range)) {
		  btn = i;
		  break;
	  }
	}
#ifdef DEBUG_ANALOG_PPM
       Serial.write(' '); Serial.write(' '); Serial.write(' '); Serial.write(' ');
       Serial.write('b');
       Serial.write('t');
       Serial.write('n');Serial.write('(');
       Serial.print(aux1);
       Serial.write(')');
       Serial.write(':');
       
       Serial.print(btn);
#endif

	// defaults for buttons not acting as switches:
	ppm[AUX1] = 0; // see BUTTON_FLIP2
	ppm[AUX2] = 0; // see BUTTON_FLIP1

#ifdef DISPLAY_IFACE
	{
		static unsigned long time_on = 0;
		static uint8_t display_on = 1;

		if (time_on == 0)
			time_on = millis();
		if (display_on) {
			if (millis() - time_on > 10000) {
				oled.sendCommand(0xAE); // display off
				display_on = 0;
			}
		} else {
			if (btn != BUTTON_NONE &&
				btn != BUTTON_FLIP1 && btn != BUTTON_FLIP2) {
				oled.sendCommand(0xAF); // on
				display_on = 1;
				time_on = millis();
			}
		}
	}
#endif

	if (btn_last == BUTTON_NONE &&
		btn != BUTTON_NONE &&
#ifndef DISPLAY_IFACE
		btn != BUTTON_MODE &&
#endif
		btn != BUTTON_FLIP1 && btn != BUTTON_FLIP2) {
		tone(BUZ_PIN, 8000, 5);
	}
	// looks like the capacitor evens out the button input nicely, so no noise
	// cancelation here...
	if (processButtons) switch (btn) {
		case BUTTON_MODE:
			if (btn_last != BUTTON_NONE)
				break;
			aux_mode = (aux_mode + 1) % 3;

#ifndef DISPLAY_IFACE
			tone(BUZ_PIN, (aux_mode + 1) * 1000, 10);
#else
			display_update();
#endif

			if (aux_mode == 1)
				ppm[AUX5] = PPM_MIN_COMMAND + 1;
			else if (aux_mode == 2)
				ppm[AUX5] = PPM_MAX_COMMAND + 1;
			else
				ppm[AUX5] = 0;

			savePPM();
			break;
		case BUTTON_FLIP1:
			ppm[AUX2] = PPM_MAX_COMMAND + 1;
			break;
		case BUTTON_FLIP2:
			ppm[AUX1] = PPM_MAX_COMMAND + 1;
			break;
		case BUTTON_RUDDER_DIV:
			if (btn_last != BUTTON_NONE)
				break;

			rudder_div = rudder_div + 1;
			if (rudder_div == 7)
				rudder_div = 2;

			savePPM();
#ifdef DISPLAY_IFACE
			display_update();
#endif
			break;
		case BUTTON_TRIM_AILERON_MINUS:
		case BUTTON_TRIM_AILERON_PLUS:
		case BUTTON_TRIM_ELEVATOR_MINUS:
		case BUTTON_TRIM_ELEVATOR_PLUS:
			if (btn_last != BUTTON_NONE)
				break;

			if (btn == BUTTON_TRIM_AILERON_MINUS)
				trim[AILERON] -= 5;
			else if (btn == BUTTON_TRIM_AILERON_PLUS)
				trim[AILERON] += 5;
			else if (btn == BUTTON_TRIM_ELEVATOR_MINUS)
				trim[ELEVATOR] -= 5;
			else if (btn == BUTTON_TRIM_ELEVATOR_PLUS)
				trim[ELEVATOR] += 5;
			trim[AILERON] = constrain(trim[AILERON], -100, 100);
			trim[ELEVATOR] = constrain(trim[ELEVATOR], -100, 100);
#ifdef DISPLAY_IFACE
			display_update();
#endif
			if (save_trims) {
				EEPROM.update(ee_TRIM_ELEVATOR, trim[ELEVATOR] + 100);
				EEPROM.update(ee_TRIM_AILERON, trim[AILERON] + 100);
			}
			break;
		case BUTTON_CALIBRATE_BIAS:
			{
				static int calibrating = 0;
				if (calibrating)
					break;

				ppm_bias[AILERON] = 0;
				ppm_bias[ELEVATOR] = 0;
				trim[AILERON] = 0;
				trim[ELEVATOR] = 0;

				calibrating = 1;
				readPPMBias();
				savePPM();
				calibrating = 0;
#ifdef DISPLAY_IFACE
				display_update();
#endif
			}
			break;
#ifdef DISPLAY_IFACE
		case BUTTON_MENU:
			doMenu();
			break;
#endif
	}
	if (processButtons)
		btn_last = btn;

#ifdef DEBUG_ANALOG_PPM
#if 0
       Serial.write('\r');
       Serial.write('\n');
       Serial.write('p');
       Serial.write('p');
       Serial.write('m');
       Serial.write(':');
       for(uint8_t ch=0; ch<CHANNELS; ch++) {
        Serial.print(ppm[ch]);
        Serial.write(' ');
        }
#endif
#endif
}


void readPPMBias(void)
{
	uint32_t ail = 0, ele = 0;

	for (uint16_t i = 0; i < PPM_BIAS_READ_COUNT; i++) {
		update_ppm();
		ail += ppm[AILERON];
		ele += ppm[ELEVATOR];
		delay(PPM_BIAS_READ_DELAY);
	}

	ail /= PPM_BIAS_READ_COUNT;
	ele /= PPM_BIAS_READ_COUNT;

	// let's hope this doesn't overflow
	ppm_bias[AILERON] = PPM_MID - ail;
	ppm_bias[ELEVATOR] = PPM_MID - ele;
}

#if 0
// update ppm values out of ISR    
void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }
    #if 0
       Serial.write('\r');
       Serial.write('\n');
       Serial.write('p');
       Serial.write('p');
       Serial.write('m');
       Serial.write(':');
       for(uint8_t ch=0; ch<CHANNELS; ch++) {
        Serial.print(ppm[ch]);
        Serial.write(' ');
        }
        #endif     
}


#endif

#ifdef PPM_INPUT
void ISR_ppm()
{
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok=false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    }
    else{  //servo values between 510us and 2420us will end up here
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if(chan==3)
                ppm_ok = true; // 4 first channels Ok
        }
        chan++;
    }
}
#endif
