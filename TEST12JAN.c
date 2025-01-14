//RIGHT SIDE FORWARD : B0
//LEFT SIDE FORWARD : B1
//RIGHT SIDE BACKWARD : B2
//LEFT SIDE BACKWARD : B3
//RIGHT FLAME SENSOR : D0
//REAR FLAME SENSOR : D3
//LEFT FLAME SENSOR : D2
//ANALOG FLAME SENSOR : A0
void pwm_init() {
    TRISC2_bit = 0; // Set RC2 pin as output
    CCP1M3_bit = 1; // Configure CCP1 module for PWM
    CCP1M2_bit = 1;
    CCP1M1_bit = 0;
    CCP1M0_bit = 0;
    CCP1X_bit = 0;
    CCP1Y_bit = 0;
    T2CKPS0_bit = 1; // Set Timer2 prescaler to 16
    T2CKPS1_bit = 1;
    TMR2ON_bit = 1; // Enable Timer2
    PR2 = 249; // Set period register for 50Hz frequency (20ms period)
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
 ADCON0 = 0x41;// ATD ON, Don't GO, Channel 1, Fosc/16
 ADCON1 = 0xCE;// All channels Analog, 500 KHz, right justified
 }
unsigned int ATD_read(void){
  ADCON0 = ADCON0 | 0x04;// GO
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
    // Ensure the angle is within the range that the servo can handle
    if (degrees < -90) degrees = -90;   // Minimum angle for the servo
    if (degrees > 90) degrees = 90;     // Maximum angle for the servo

    // Calculate the pulse width (500 to 2400 microseconds)
    pulse_width = (degrees + 90) * 8 + 500;  // Pulse width calculation for the servo

    CCPR1L = pulse_width >> 2;  // Set the high byte (most significant bits) for the pulse width
    CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4);  // Set the low byte (least significant bits) for the pulse width
}
void Initialize() {

    TRISB = 0x00; // Configure PORTB as output
    PORTB = 0x00; // Initialize PORTB to 0
    TRISC = 0x00; // Configure PORTC as input
    PORTC = 0x00; // Enable pull-up resistors for PORTC
    TRISD = 0xFF;
    PORTD = 0xFF;
    ADCON0 = 0x01;
    ADCON1 = 0x0E;
}

void turn_right() {
    PORTB = PORTB & 0x00;
    PORTB = PORTB | 0x06; // Set pins for turning right
}

void turn_left() {
    PORTB = PORTB & 0x00;
    PORTB = PORTB | 0x09; // Set pins for turning left
}

void turn180() {
    PORTB = PORTB & 0x00;
    PORTB = PORTB | 0x06; // Set pins for 180-degree turn
}

void stop() {
    PORTB = 0x00; // Stop movement
}

void forward() {
    PORTB = PORTB & 0x00;
    PORTB = PORTB | 0x03; // Set pins for moving forward
}

void backward() {
    PORTB = PORTB & 0x00;
    PORTB = PORTB | 0x0C; // Set pins for moving backward
}

void scan() {
     while(1){
      unsigned int analog_value = ATD_read();
        if ((PORTD & 0x01) == 0) {
            turn_right(); // Turn right
            msDelay(300); // Delay for turning
            stop();
            continue;       // Stop movement
        }
            if  ((PORTD & 0x08) == 0) {
            turn_left();
            msDelay(300);
            stop();
            continue;
        }
            if ((PORTD & 0x04) == 0) {
            turn180();    // Turn 180 degrees
            msDelay(500); // Delay for turning
            stop();       // Stop movement
            continue;     // Continue scanning
        }
         if (analog_value < 300) { // Assuming a threshold value of 512 for flame detection
         // Move servo to the left position (e.g., -25 degrees)
        set_servo_position(70);  // Set the servo position to -25 degrees (left)
        Delay_ms(750);  // Wait for 1000ms (1 second) to allow the servo to reach the position

        // Move servo to the right position (e.g., 25 degrees)
        set_servo_position(30);   // Set the servo position to 25 degrees (right)
        Delay_ms(750);  // Wait for 1000ms (1 second) to allow the servo to reach the position
        continue;
        }
        }}
void main() {
    pwm_init(); // Initialize PWM module
    Initialize(); // Initialize ports
    ATD_init(void);
    while(1){
    scan();}
}
//-------------------------------------