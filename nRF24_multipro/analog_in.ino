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
	BUTTON_HEADLESS = BUTTON_L_RIGHT,
	BUTTON_AUX1 = BUTTON_L_UP,
	BUTTON_AUX2 = BUTTON_L_DOWN,
	
	BUTTON_TRIM_AILERON_MINUS = BUTTON_R_LEFT,
	BUTTON_TRIM_AILERON_PLUS = BUTTON_R_RIGHT,
	BUTTON_TRIM_ELEVATOR_MINUS = BUTTON_R_DOWN,
	BUTTON_TRIM_ELEVATOR_PLUS = BUTTON_R_UP,
};

static int8_t ppm_bias[4];

// update ppm values out of ISR    
void update_ppm()
{
  static uint32_t thr,ail,ele,rud,aux1;
  static uint8_t btn, btn_last = BUTTON_NONE;
  static uint8_t i;
  static uint8_t aux_mode = 0;

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

	if (btn_last == BUTTON_NONE &&
		btn != BUTTON_NONE && btn != BUTTON_MODE &&
		btn != BUTTON_FLIP1 && btn != BUTTON_FLIP2) {
		tone(BUZ_PIN, 8000, 5);
	}
	// looks like the capacitor evens out the button input nicely, so no noise
	// cancelation here...
	switch (btn) {
		case BUTTON_MODE:
			if (btn_last != BUTTON_NONE)
				break;
			aux_mode = (aux_mode + 1) % 3;
			tone(BUZ_PIN, (aux_mode + 1) * 1000, 10);

			if (aux_mode == 1)
				ppm[AUX5] = PPM_MIN_COMMAND + 1;
			else if (aux_mode == 2)
				ppm[AUX5] = PPM_MAX_COMMAND + 1;
			else
				ppm[AUX5] = 0;

			break;
		case BUTTON_FLIP1:
			ppm[AUX2] = PPM_MAX_COMMAND + 1;
			break;
		case BUTTON_FLIP2:
			ppm[AUX1] = PPM_MAX_COMMAND + 1;
			break;
		case BUTTON_TRIM_AILERON_MINUS:
		case BUTTON_TRIM_AILERON_PLUS:
		case BUTTON_TRIM_ELEVATOR_MINUS:
		case BUTTON_TRIM_ELEVATOR_PLUS:
			if (btn_last != BUTTON_NONE)
				break;

			if (btn == BUTTON_TRIM_AILERON_MINUS)
				trim[AILERON]--;
			else if (btn == BUTTON_TRIM_AILERON_PLUS)
				trim[AILERON]++;
			else if (btn == BUTTON_TRIM_ELEVATOR_MINUS)
				trim[ELEVATOR]--;
			else if (btn == BUTTON_TRIM_ELEVATOR_PLUS)
				trim[ELEVATOR]++;
#ifdef DISPLAY_IFACE
			display_update();
#endif
			break;
	}
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
