

#define PPM_SCALE ((PPM_MAX-PPM_MIN)/(PPM_MAX_A - PPM_MIN_A))
#define PPM_RANGE (PPM_MAX-PPM_MIN)
#define PPM_RANGE_A (PPM_MAX_A-PPM_MIN_A)

#define BTN0_MINVAL 680
#define BTN0_MAXVAL 688
#define BTN1_MINVAL 451
#define BTN1_MAXVAL 459
#define BTN2_MINVAL 379
#define BTN2_MAXVAL 387
#define BTN3_MINVAL 288
#define BTN3_MAXVAL 296
#define BTN4_MINVAL 1
#define BTN4_MAXVAL 1




int THR_PIN=A7;
int RUD_PIN=A6;
int ELE_PIN=A3;
int AIL_PIN=A4;
int BTN_PIN=A5;



// update ppm values out of ISR    
void update_ppm()
{
  static uint32_t thr,ail,ele,rud,aux1;

  thr = analogRead(THR_PIN);
  ail = analogRead(AIL_PIN);   
  ele = analogRead(ELE_PIN);   
  rud = analogRead(RUD_PIN);   
  aux1 = analogRead(BTN_PIN);
#if 1
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
  ppm[AILERON] = (int)(PPM_MIN + (ail * PPM_RANGE)/PPM_RANGE_A);
  ppm[ELEVATOR] = (int)(PPM_MIN + (ele * PPM_RANGE)/PPM_RANGE_A);
  ppm[RUDDER] = (int)(PPM_MIN + (rud * PPM_RANGE)/PPM_RANGE_A);
  //ppm[AUX1] = (int)(PPM_MIN + (aux1 * PPM_RANGE)/PPM_RANGE_A);

  btn = 0;
  if(aux1 > BTN0_MINVAL && aux1 < BTN0_MAXVAL)
     btn |= 1<<0;
  if(aux1 > BTN1_MINVAL && aux1 < BTN1_MAXVAL)
     btn |= 1<<1;
  if(aux1 > BTN2_MINVAL && aux1 < BTN2_MAXVAL)
     btn |= 1<<2;
  if(aux1 > BTN3_MINVAL && aux1 < BTN3_MAXVAL)
     btn |= 1<<3;
  if(aux1 > BTN4_MINVAL && aux1 < BTN4_MAXVAL)
  
     btn |= 1<<4;

       Serial.write('\r');
       Serial.write('\n');
       Serial.write('b');
       Serial.write('t');
       Serial.write('n');Serial.write('(');
       Serial.print(aux1);
       Serial.write(')');
       Serial.write(':');
       
       for (uint8_t n=0; n<8;n++)
       {
        if (btn & 1<<n)
            Serial.write('1');
        else
            Serial.write('0');
        Serial.write(' ');
        }

     if(btn & (1<<2)) {
          if(mode_btn_last == 0) {
            mode++;
            if (mode>2) {
              mode = 0;
              }
            for (uint8_t m=0; m <= mode; m++){
              tone(BUZ_PIN, 4000, 10);
              delay(40);
            }
            mode_btn_last=1;
          }
        } else {
            mode_btn_last=0;
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
