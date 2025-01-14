#line 1 "C:/Users/User/Desktop/New folder (2)/TEST12JAN.c"








void pwm_init() {
 TRISC2_bit = 0;
 CCP1M3_bit = 1;
 CCP1M2_bit = 1;
 CCP1M1_bit = 0;
 CCP1M0_bit = 0;
 CCP1X_bit = 0;
 CCP1Y_bit = 0;
 T2CKPS0_bit = 1;
 T2CKPS1_bit = 1;
 TMR2ON_bit = 1;
 PR2 = 249;
}
void Delay_us(unsigned int microseconds) {
 unsigned int i;

 while (microseconds--) {
 for (i = 0; i < 12; i++) {
 asm nop;
 }
 }
}
void ATD_init(void){
 ADCON0 = 0x41;
 ADCON1 = 0xCE;
 }
unsigned int ATD_read(void){
 ADCON0 = ADCON0 | 0x04;
 while(ADCON0 & 0x04);
 return((ADRESH<<8) | ADRESL);
}
void msDelay(unsigned int mscnt) {
 unsigned int ms;
 unsigned int cnt;
 for (ms = 0; ms < mscnt; ms++) {
 for (cnt = 0; cnt < 155; cnt++);
 }
}
void Delay_ms(unsigned int milliseconds) {
 unsigned int i;

 while (milliseconds--) {
 for (i = 0; i < 238; i++) {
 Delay_us(1000);
 }
 }
}

void set_servo_position(int degrees) {
 int pulse_width;

 if (degrees < -90) degrees = -90;
 if (degrees > 90) degrees = 90;


 pulse_width = (degrees + 90) * 8 + 500;

 CCPR1L = pulse_width >> 2;
 CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4);
}
void Initialize() {

 TRISB = 0x00;
 PORTB = 0x00;
 TRISC = 0x00;
 PORTC = 0x00;
 TRISD = 0xFF;
 PORTD = 0xFF;
 ADCON0 = 0x01;
 ADCON1 = 0x0E;
}

void turn_right() {
 PORTB = PORTB & 0x00;
 PORTB = PORTB | 0x06;
}

void turn_left() {
 PORTB = PORTB & 0x00;
 PORTB = PORTB | 0x09;
}

void turn180() {
 PORTB = PORTB & 0x00;
 PORTB = PORTB | 0x06;
}

void stop() {
 PORTB = 0x00;
}

void forward() {
 PORTB = PORTB & 0x00;
 PORTB = PORTB | 0x03;
}

void backward() {
 PORTB = PORTB & 0x00;
 PORTB = PORTB | 0x0C;
}

void scan() {
 while(1){
 unsigned int analog_value = ATD_read();
 if ((PORTD & 0x01) == 0) {
 turn_right();
 msDelay(300);
 stop();
 continue;
 }
 if ((PORTD & 0x08) == 0) {
 turn_left();
 msDelay(300);
 stop();
 continue;
 }
 if ((PORTD & 0x04) == 0) {
 turn180();
 msDelay(500);
 stop();
 continue;
 }
 if (analog_value < 300) {

 set_servo_position(70);
 Delay_ms(750);


 set_servo_position(30);
 Delay_ms(750);
 continue;
 }
 }}
void main() {
 pwm_init();
 Initialize();
 ATD_init(void);
 while(1){
 scan();}
}
