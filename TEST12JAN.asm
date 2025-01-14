
_pwm_init:

;TEST12JAN.c,9 :: 		void pwm_init() {
;TEST12JAN.c,10 :: 		TRISC2_bit = 0; // Set RC2 pin as output
	BCF        TRISC2_bit+0, BitPos(TRISC2_bit+0)
;TEST12JAN.c,11 :: 		CCP1M3_bit = 1; // Configure CCP1 module for PWM
	BSF        CCP1M3_bit+0, BitPos(CCP1M3_bit+0)
;TEST12JAN.c,12 :: 		CCP1M2_bit = 1;
	BSF        CCP1M2_bit+0, BitPos(CCP1M2_bit+0)
;TEST12JAN.c,13 :: 		CCP1M1_bit = 0;
	BCF        CCP1M1_bit+0, BitPos(CCP1M1_bit+0)
;TEST12JAN.c,14 :: 		CCP1M0_bit = 0;
	BCF        CCP1M0_bit+0, BitPos(CCP1M0_bit+0)
;TEST12JAN.c,15 :: 		CCP1X_bit = 0;
	BCF        CCP1X_bit+0, BitPos(CCP1X_bit+0)
;TEST12JAN.c,16 :: 		CCP1Y_bit = 0;
	BCF        CCP1Y_bit+0, BitPos(CCP1Y_bit+0)
;TEST12JAN.c,17 :: 		T2CKPS0_bit = 1; // Set Timer2 prescaler to 16
	BSF        T2CKPS0_bit+0, BitPos(T2CKPS0_bit+0)
;TEST12JAN.c,18 :: 		T2CKPS1_bit = 1;
	BSF        T2CKPS1_bit+0, BitPos(T2CKPS1_bit+0)
;TEST12JAN.c,19 :: 		TMR2ON_bit = 1; // Enable Timer2
	BSF        TMR2ON_bit+0, BitPos(TMR2ON_bit+0)
;TEST12JAN.c,20 :: 		PR2 = 249; // Set period register for 50Hz frequency (20ms period)
	MOVLW      249
	MOVWF      PR2+0
;TEST12JAN.c,21 :: 		}
L_end_pwm_init:
	RETURN
; end of _pwm_init

_Delay_us:

;TEST12JAN.c,22 :: 		void Delay_us(unsigned int microseconds) {
;TEST12JAN.c,25 :: 		while (microseconds--) {
L_Delay_us0:
	MOVF       FARG_Delay_us_microseconds+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay_us_microseconds+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay_us_microseconds+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay_us_microseconds+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay_us1
;TEST12JAN.c,26 :: 		for (i = 0; i < 12; i++) {
	CLRF       R2+0
	CLRF       R2+1
L_Delay_us2:
	MOVLW      0
	SUBWF      R2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Delay_us33
	MOVLW      12
	SUBWF      R2+0, 0
L__Delay_us33:
	BTFSC      STATUS+0, 0
	GOTO       L_Delay_us3
;TEST12JAN.c,27 :: 		asm nop;
	NOP
;TEST12JAN.c,26 :: 		for (i = 0; i < 12; i++) {
	INCF       R2+0, 1
	BTFSC      STATUS+0, 2
	INCF       R2+1, 1
;TEST12JAN.c,28 :: 		}
	GOTO       L_Delay_us2
L_Delay_us3:
;TEST12JAN.c,29 :: 		}
	GOTO       L_Delay_us0
L_Delay_us1:
;TEST12JAN.c,30 :: 		}
L_end_Delay_us:
	RETURN
; end of _Delay_us

_ATD_init:

;TEST12JAN.c,31 :: 		void ATD_init(void){
;TEST12JAN.c,32 :: 		ADCON0 = 0x41;// ATD ON, Don't GO, Channel 1, Fosc/16
	MOVLW      65
	MOVWF      ADCON0+0
;TEST12JAN.c,33 :: 		ADCON1 = 0xCE;// All channels Analog, 500 KHz, right justified
	MOVLW      206
	MOVWF      ADCON1+0
;TEST12JAN.c,34 :: 		}
L_end_ATD_init:
	RETURN
; end of _ATD_init

_ATD_read:

;TEST12JAN.c,35 :: 		unsigned int ATD_read(void){
;TEST12JAN.c,36 :: 		ADCON0 = ADCON0 | 0x04;// GO
	BSF        ADCON0+0, 2
;TEST12JAN.c,37 :: 		while(ADCON0 & 0x04);
L_ATD_read5:
	BTFSS      ADCON0+0, 2
	GOTO       L_ATD_read6
	GOTO       L_ATD_read5
L_ATD_read6:
;TEST12JAN.c,38 :: 		return((ADRESH<<8) | ADRESL);
	MOVF       ADRESH+0, 0
	MOVWF      R0+1
	CLRF       R0+0
	MOVF       ADRESL+0, 0
	IORWF      R0+0, 1
	MOVLW      0
	IORWF      R0+1, 1
;TEST12JAN.c,39 :: 		}
L_end_ATD_read:
	RETURN
; end of _ATD_read

_msDelay:

;TEST12JAN.c,40 :: 		void msDelay(unsigned int mscnt) {
;TEST12JAN.c,43 :: 		for (ms = 0; ms < mscnt; ms++) {
	CLRF       R1+0
	CLRF       R1+1
L_msDelay7:
	MOVF       FARG_msDelay_mscnt+1, 0
	SUBWF      R1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__msDelay37
	MOVF       FARG_msDelay_mscnt+0, 0
	SUBWF      R1+0, 0
L__msDelay37:
	BTFSC      STATUS+0, 0
	GOTO       L_msDelay8
;TEST12JAN.c,44 :: 		for (cnt = 0; cnt < 155; cnt++);
	CLRF       R3+0
	CLRF       R3+1
L_msDelay10:
	MOVLW      0
	SUBWF      R3+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__msDelay38
	MOVLW      155
	SUBWF      R3+0, 0
L__msDelay38:
	BTFSC      STATUS+0, 0
	GOTO       L_msDelay11
	INCF       R3+0, 1
	BTFSC      STATUS+0, 2
	INCF       R3+1, 1
	GOTO       L_msDelay10
L_msDelay11:
;TEST12JAN.c,43 :: 		for (ms = 0; ms < mscnt; ms++) {
	INCF       R1+0, 1
	BTFSC      STATUS+0, 2
	INCF       R1+1, 1
;TEST12JAN.c,45 :: 		}
	GOTO       L_msDelay7
L_msDelay8:
;TEST12JAN.c,46 :: 		}
L_end_msDelay:
	RETURN
; end of _msDelay

_Delay_ms:

;TEST12JAN.c,47 :: 		void Delay_ms(unsigned int milliseconds) {
;TEST12JAN.c,50 :: 		while (milliseconds--) {
L_Delay_ms13:
	MOVF       FARG_Delay_ms_milliseconds+0, 0
	MOVWF      R0+0
	MOVF       FARG_Delay_ms_milliseconds+1, 0
	MOVWF      R0+1
	MOVLW      1
	SUBWF      FARG_Delay_ms_milliseconds+0, 1
	BTFSS      STATUS+0, 0
	DECF       FARG_Delay_ms_milliseconds+1, 1
	MOVF       R0+0, 0
	IORWF      R0+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_Delay_ms14
;TEST12JAN.c,51 :: 		for (i = 0; i < 238; i++) {
	CLRF       R2+0
	CLRF       R2+1
L_Delay_ms15:
	MOVLW      0
	SUBWF      R2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Delay_ms40
	MOVLW      238
	SUBWF      R2+0, 0
L__Delay_ms40:
	BTFSC      STATUS+0, 0
	GOTO       L_Delay_ms16
;TEST12JAN.c,52 :: 		Delay_us(1000);
	MOVLW      3
	MOVWF      R12+0
	MOVLW      151
	MOVWF      R13+0
L_Delay_ms18:
	DECFSZ     R13+0, 1
	GOTO       L_Delay_ms18
	DECFSZ     R12+0, 1
	GOTO       L_Delay_ms18
	NOP
	NOP
;TEST12JAN.c,51 :: 		for (i = 0; i < 238; i++) {
	INCF       R2+0, 1
	BTFSC      STATUS+0, 2
	INCF       R2+1, 1
;TEST12JAN.c,53 :: 		}
	GOTO       L_Delay_ms15
L_Delay_ms16:
;TEST12JAN.c,54 :: 		}
	GOTO       L_Delay_ms13
L_Delay_ms14:
;TEST12JAN.c,55 :: 		}
L_end_Delay_ms:
	RETURN
; end of _Delay_ms

_set_servo_position:

;TEST12JAN.c,57 :: 		void set_servo_position(int degrees) {
;TEST12JAN.c,60 :: 		if (degrees < -90) degrees = -90;   // Minimum angle for the servo
	MOVLW      128
	XORWF      FARG_set_servo_position_degrees+1, 0
	MOVWF      R0+0
	MOVLW      127
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__set_servo_position42
	MOVLW      166
	SUBWF      FARG_set_servo_position_degrees+0, 0
L__set_servo_position42:
	BTFSC      STATUS+0, 0
	GOTO       L_set_servo_position19
	MOVLW      166
	MOVWF      FARG_set_servo_position_degrees+0
	MOVLW      255
	MOVWF      FARG_set_servo_position_degrees+1
L_set_servo_position19:
;TEST12JAN.c,61 :: 		if (degrees > 90) degrees = 90;     // Maximum angle for the servo
	MOVLW      128
	MOVWF      R0+0
	MOVLW      128
	XORWF      FARG_set_servo_position_degrees+1, 0
	SUBWF      R0+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__set_servo_position43
	MOVF       FARG_set_servo_position_degrees+0, 0
	SUBLW      90
L__set_servo_position43:
	BTFSC      STATUS+0, 0
	GOTO       L_set_servo_position20
	MOVLW      90
	MOVWF      FARG_set_servo_position_degrees+0
	MOVLW      0
	MOVWF      FARG_set_servo_position_degrees+1
L_set_servo_position20:
;TEST12JAN.c,64 :: 		pulse_width = (degrees + 90) * 8 + 500;  // Pulse width calculation for the servo
	MOVLW      90
	ADDWF      FARG_set_servo_position_degrees+0, 0
	MOVWF      R3+0
	MOVF       FARG_set_servo_position_degrees+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	MOVWF      R3+1
	MOVLW      3
	MOVWF      R2+0
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	MOVF       R2+0, 0
L__set_servo_position44:
	BTFSC      STATUS+0, 2
	GOTO       L__set_servo_position45
	RLF        R0+0, 1
	RLF        R0+1, 1
	BCF        R0+0, 0
	ADDLW      255
	GOTO       L__set_servo_position44
L__set_servo_position45:
	MOVLW      244
	ADDWF      R0+0, 0
	MOVWF      R3+0
	MOVF       R0+1, 0
	BTFSC      STATUS+0, 0
	ADDLW      1
	ADDLW      1
	MOVWF      R3+1
;TEST12JAN.c,66 :: 		CCPR1L = pulse_width >> 2;  // Set the high byte (most significant bits) for the pulse width
	MOVF       R3+0, 0
	MOVWF      R0+0
	MOVF       R3+1, 0
	MOVWF      R0+1
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	RRF        R0+1, 1
	RRF        R0+0, 1
	BCF        R0+1, 7
	BTFSC      R0+1, 6
	BSF        R0+1, 7
	MOVF       R0+0, 0
	MOVWF      CCPR1L+0
;TEST12JAN.c,67 :: 		CCP1CON = (CCP1CON & 0xCF) | ((pulse_width & 0x03) << 4);  // Set the low byte (least significant bits) for the pulse width
	MOVLW      207
	ANDWF      CCP1CON+0, 0
	MOVWF      R5+0
	MOVLW      3
	ANDWF      R3+0, 0
	MOVWF      R2+0
	MOVF       R2+0, 0
	MOVWF      R0+0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	RLF        R0+0, 1
	BCF        R0+0, 0
	MOVF       R0+0, 0
	IORWF      R5+0, 0
	MOVWF      CCP1CON+0
;TEST12JAN.c,68 :: 		}
L_end_set_servo_position:
	RETURN
; end of _set_servo_position

_Initialize:

;TEST12JAN.c,69 :: 		void Initialize() {
;TEST12JAN.c,71 :: 		TRISB = 0x00; // Configure PORTB as output
	CLRF       TRISB+0
;TEST12JAN.c,72 :: 		PORTB = 0x00; // Initialize PORTB to 0
	CLRF       PORTB+0
;TEST12JAN.c,73 :: 		TRISC = 0x00; // Configure PORTC as input
	CLRF       TRISC+0
;TEST12JAN.c,74 :: 		PORTC = 0x00; // Enable pull-up resistors for PORTC
	CLRF       PORTC+0
;TEST12JAN.c,75 :: 		TRISD = 0xFF;
	MOVLW      255
	MOVWF      TRISD+0
;TEST12JAN.c,76 :: 		PORTD = 0xFF;
	MOVLW      255
	MOVWF      PORTD+0
;TEST12JAN.c,77 :: 		ADCON0 = 0x01;
	MOVLW      1
	MOVWF      ADCON0+0
;TEST12JAN.c,78 :: 		ADCON1 = 0x0E;
	MOVLW      14
	MOVWF      ADCON1+0
;TEST12JAN.c,79 :: 		}
L_end_Initialize:
	RETURN
; end of _Initialize

_turn_right:

;TEST12JAN.c,81 :: 		void turn_right() {
;TEST12JAN.c,82 :: 		PORTB = PORTB & 0x00;
	MOVLW      0
	ANDWF      PORTB+0, 1
;TEST12JAN.c,83 :: 		PORTB = PORTB | 0x06; // Set pins for turning right
	MOVLW      6
	IORWF      PORTB+0, 1
;TEST12JAN.c,84 :: 		}
L_end_turn_right:
	RETURN
; end of _turn_right

_turn_left:

;TEST12JAN.c,86 :: 		void turn_left() {
;TEST12JAN.c,87 :: 		PORTB = PORTB & 0x00;
	MOVLW      0
	ANDWF      PORTB+0, 1
;TEST12JAN.c,88 :: 		PORTB = PORTB | 0x09; // Set pins for turning left
	MOVLW      9
	IORWF      PORTB+0, 1
;TEST12JAN.c,89 :: 		}
L_end_turn_left:
	RETURN
; end of _turn_left

_turn180:

;TEST12JAN.c,91 :: 		void turn180() {
;TEST12JAN.c,92 :: 		PORTB = PORTB & 0x00;
	MOVLW      0
	ANDWF      PORTB+0, 1
;TEST12JAN.c,93 :: 		PORTB = PORTB | 0x06; // Set pins for 180-degree turn
	MOVLW      6
	IORWF      PORTB+0, 1
;TEST12JAN.c,94 :: 		}
L_end_turn180:
	RETURN
; end of _turn180

_stop:

;TEST12JAN.c,96 :: 		void stop() {
;TEST12JAN.c,97 :: 		PORTB = 0x00; // Stop movement
	CLRF       PORTB+0
;TEST12JAN.c,98 :: 		}
L_end_stop:
	RETURN
; end of _stop

_forward:

;TEST12JAN.c,100 :: 		void forward() {
;TEST12JAN.c,101 :: 		PORTB = PORTB & 0x00;
	MOVLW      0
	ANDWF      PORTB+0, 1
;TEST12JAN.c,102 :: 		PORTB = PORTB | 0x03; // Set pins for moving forward
	MOVLW      3
	IORWF      PORTB+0, 1
;TEST12JAN.c,103 :: 		}
L_end_forward:
	RETURN
; end of _forward

_backward:

;TEST12JAN.c,105 :: 		void backward() {
;TEST12JAN.c,106 :: 		PORTB = PORTB & 0x00;
	MOVLW      0
	ANDWF      PORTB+0, 1
;TEST12JAN.c,107 :: 		PORTB = PORTB | 0x0C; // Set pins for moving backward
	MOVLW      12
	IORWF      PORTB+0, 1
;TEST12JAN.c,108 :: 		}
L_end_backward:
	RETURN
; end of _backward

_scan:

;TEST12JAN.c,110 :: 		void scan() {
;TEST12JAN.c,111 :: 		while(1){
L_scan21:
;TEST12JAN.c,112 :: 		unsigned int analog_value = ATD_read();
	CALL       _ATD_read+0
	MOVF       R0+0, 0
	MOVWF      scan_analog_value_L1+0
	MOVF       R0+1, 0
	MOVWF      scan_analog_value_L1+1
;TEST12JAN.c,113 :: 		if ((PORTD & 0x01) == 0) {
	MOVLW      1
	ANDWF      PORTD+0, 0
	MOVWF      R1+0
	MOVF       R1+0, 0
	XORLW      0
	BTFSS      STATUS+0, 2
	GOTO       L_scan23
;TEST12JAN.c,114 :: 		turn_right(); // Turn right
	CALL       _turn_right+0
;TEST12JAN.c,115 :: 		msDelay(300); // Delay for turning
	MOVLW      44
	MOVWF      FARG_msDelay_mscnt+0
	MOVLW      1
	MOVWF      FARG_msDelay_mscnt+1
	CALL       _msDelay+0
;TEST12JAN.c,116 :: 		stop();
	CALL       _stop+0
;TEST12JAN.c,117 :: 		continue;       // Stop movement
	GOTO       L_scan21
;TEST12JAN.c,118 :: 		}
L_scan23:
;TEST12JAN.c,119 :: 		if  ((PORTD & 0x08) == 0) {
	MOVLW      8
	ANDWF      PORTD+0, 0
	MOVWF      R1+0
	MOVF       R1+0, 0
	XORLW      0
	BTFSS      STATUS+0, 2
	GOTO       L_scan24
;TEST12JAN.c,120 :: 		turn_left();
	CALL       _turn_left+0
;TEST12JAN.c,121 :: 		msDelay(300);
	MOVLW      44
	MOVWF      FARG_msDelay_mscnt+0
	MOVLW      1
	MOVWF      FARG_msDelay_mscnt+1
	CALL       _msDelay+0
;TEST12JAN.c,122 :: 		stop();
	CALL       _stop+0
;TEST12JAN.c,123 :: 		continue;
	GOTO       L_scan21
;TEST12JAN.c,124 :: 		}
L_scan24:
;TEST12JAN.c,125 :: 		if ((PORTD & 0x04) == 0) {
	MOVLW      4
	ANDWF      PORTD+0, 0
	MOVWF      R1+0
	MOVF       R1+0, 0
	XORLW      0
	BTFSS      STATUS+0, 2
	GOTO       L_scan25
;TEST12JAN.c,126 :: 		turn180();    // Turn 180 degrees
	CALL       _turn180+0
;TEST12JAN.c,127 :: 		msDelay(500); // Delay for turning
	MOVLW      244
	MOVWF      FARG_msDelay_mscnt+0
	MOVLW      1
	MOVWF      FARG_msDelay_mscnt+1
	CALL       _msDelay+0
;TEST12JAN.c,128 :: 		stop();       // Stop movement
	CALL       _stop+0
;TEST12JAN.c,129 :: 		continue;     // Continue scanning
	GOTO       L_scan21
;TEST12JAN.c,130 :: 		}
L_scan25:
;TEST12JAN.c,131 :: 		if (analog_value < 300) { // Assuming a threshold value of 512 for flame detection
	MOVLW      1
	SUBWF      scan_analog_value_L1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__scan54
	MOVLW      44
	SUBWF      scan_analog_value_L1+0, 0
L__scan54:
	BTFSC      STATUS+0, 0
	GOTO       L_scan26
;TEST12JAN.c,133 :: 		set_servo_position(70);  // Set the servo position to -25 degrees (left)
	MOVLW      70
	MOVWF      FARG_set_servo_position_degrees+0
	MOVLW      0
	MOVWF      FARG_set_servo_position_degrees+1
	CALL       _set_servo_position+0
;TEST12JAN.c,134 :: 		Delay_ms(750);  // Wait for 1000ms (1 second) to allow the servo to reach the position
	MOVLW      8
	MOVWF      R11+0
	MOVLW      157
	MOVWF      R12+0
	MOVLW      5
	MOVWF      R13+0
L_scan27:
	DECFSZ     R13+0, 1
	GOTO       L_scan27
	DECFSZ     R12+0, 1
	GOTO       L_scan27
	DECFSZ     R11+0, 1
	GOTO       L_scan27
	NOP
	NOP
;TEST12JAN.c,137 :: 		set_servo_position(30);   // Set the servo position to 25 degrees (right)
	MOVLW      30
	MOVWF      FARG_set_servo_position_degrees+0
	MOVLW      0
	MOVWF      FARG_set_servo_position_degrees+1
	CALL       _set_servo_position+0
;TEST12JAN.c,138 :: 		Delay_ms(750);  // Wait for 1000ms (1 second) to allow the servo to reach the position
	MOVLW      8
	MOVWF      R11+0
	MOVLW      157
	MOVWF      R12+0
	MOVLW      5
	MOVWF      R13+0
L_scan28:
	DECFSZ     R13+0, 1
	GOTO       L_scan28
	DECFSZ     R12+0, 1
	GOTO       L_scan28
	DECFSZ     R11+0, 1
	GOTO       L_scan28
	NOP
	NOP
;TEST12JAN.c,139 :: 		continue;
	GOTO       L_scan21
;TEST12JAN.c,140 :: 		}
L_scan26:
;TEST12JAN.c,141 :: 		}}
	GOTO       L_scan21
L_end_scan:
	RETURN
; end of _scan

_main:

;TEST12JAN.c,142 :: 		void main() {
;TEST12JAN.c,143 :: 		pwm_init(); // Initialize PWM module
	CALL       _pwm_init+0
;TEST12JAN.c,144 :: 		Initialize(); // Initialize ports
	CALL       _Initialize+0
;TEST12JAN.c,145 :: 		ATD_init(void);
	CALL       _ATD_init+0
;TEST12JAN.c,146 :: 		while(1){
L_main29:
;TEST12JAN.c,147 :: 		scan();}
	CALL       _scan+0
	GOTO       L_main29
;TEST12JAN.c,148 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
