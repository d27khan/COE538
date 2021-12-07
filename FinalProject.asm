;***********************
;* This stationery serves as the framework for a                 *
;* user application (single file, absolute assembly application) *
;* For a more comprehensive program that                         *
;* demonstrates the more advanced functionality of this          *
;* processor, please see the demonstration applications          *
;* located in the examples subdirectory of the                   *
;* Freescale CodeWarrior for the HC12 Program directory          *
;***********************

; export symbols
            XDEF Entry, _Startup            ; export 'Entry' symbol
            ABSENTRY Entry        ; for absolute assembly: mark this as application entry point



; Include derivative-specific definitions 
		INCLUDE 'derivative.inc'
		
;definitions
OneSec        EQU 23 ; 1 second delay (at 23Hz)
TwoSec        EQU 46 ; 2 second delay (at 23Hz)
LCD_DAT       EQU PORTB ; LCD data port, bits - PB7,...,PB0
LCD_CNTR      EQU PTJ ; LCD control port, bits - PJ7(E),PJ6(RS)
LCD_E         EQU $80 ; LCD E-signal pin
LCD_RS        EQU $40 ; LCD RS-signal pin
FWD_INT       EQU 69 ; 3 second delay (at 23Hz)
REV_INT       EQU 69 ; 3 second delay (at 23Hz)
FWD_TRN_INT   EQU 46 ; 2 second delay (at 23Hz)
REV_TRN_INT   EQU 46 ; 2 second delay (at 23Hz)
;START         EQU 0
;FWD           EQU 1
;REV           EQU 2
;ALL_STP       EQU 3
;FWD_TRN       EQU 4
REV_TRN       EQU 5
;New equates for sensors
EF_SENSOR_UP     EQU   $60                       ; If SENSOR_LINE < EF_SENSOR_UP robot must shift right
EF_SENSOR_LOW     EQU   $B4                       ; If SENSOR_LINE > EF_SENSOR_LOW robot must shift left
SENSOR_A      EQU $70
SENSOR_B      EQU $70
SENSOR_C      EQU $C0
SENSOR_D      EQU $70
;New equates for guider
CLEAR_HOME    EQU $01 			 ; Clear the display and home the cursor 
INTERFACE     EQU $38			 ; 8-bit interface, two-line display 
CURSOR_OFF    EQU $0C 			 ; Display on, cursor off 
SHIFT_OFF     EQU $06			 ; Address increments, no character shift 
LCD_SEC_LINE  EQU 64			 ; Starting addr. of 2nd line of LCD (note decimal value!)
;Other codes
NULL          EQU 00 				 ; The string 'null terminator'
CR            EQU $0D 				 ; 'Carriage Return' character
SPACE         EQU ' ' 				 ; The 'space' character


;variable/data section
            ORG $3850 ; Where our TOF counter register lives
TOF_COUNTER RMB 1 ; The timer, incremented at 23Hz
AT_DEMO     RMB 1 ; The alarm time for this demo
TOF_COUNTER   dc.b 0                      ; The timer, incremented at 23Hz
CRNT_STATE    dc.b 3                      ; Current state register
T_FWD         ds.b 1                      ; FWD time
T_REV         ds.b 1                      ; REV time
T_FWD_TRN     ds.b 1                      ; FWD_TURN time
T_REV_TRN     ds.b 1                      ; REV_TURN time
TEN_THOUS     ds.b 1                      ; 10,000 digit
THOUSANDS     ds.b 1                      ; 1,000 digit
HUNDREDS      ds.b 1                      ; 100 digit
TENS          ds.b 1                      ; 10 digit
UNITS         ds.b 1                      ; 1 digit
NO_BLANK      ds.b 1                      ; Used in ’leading zero’ blanking by BCD2ASC
SENSOR_LINE FCB $01 ; Storage for guider sensor readings
SENSOR_BOW  FCB $23 ; Initialized to test values
SENSOR_PORT FCB $45
SENSOR_MID  FCB $67
SENSOR_STBD FCB $89
SENSOR_NUM  RMB 1 ; The currently selected sensor
TOP_LINE    RMB 20 ; Top line of display
            FCB NULL ; terminated by null
BOT_LINE    RMB 20 ; Bottom line of display
            FCB NULL ; terminated by null
CLEAR_LINE  FCC ' '
            FCB NULL ; terminated by null
TEMP        RMB 1 ; Temporary location



;code section
            ORG $4000 ; Where the code starts
Entry:
_Startup:
            LDS #$4000 ; initialize the stack pointer
            JSR initLCD ; initialize the LCD
            JSR clrLCD ; clear LCD & home cursor
            JSR ENABLE_TOF ; Jump to TOF initialization
            CLI ; Enable global interrupt
            
            LDAA #’A’ ; Display A (for 1 sec)
            JSR putcLCD ; --"--
            
            LDAA TOF_COUNTER ; Initialize the alarm time
            ADDA #OneSec ; by adding on the 1 sec delay
            STAA AT_DEMO ; and save it in the alarm
CHK_DELAY_1 LDAA TOF_COUNTER ; If the current time
            CMPA AT_DEMO ; equals the alarm time
            BEQ A1 ; then display B
            BRA CHK_DELAY_1 ; and check the alarm again
            
A1          LDAA #’B’ ; Display B (for 2 sec)
            JSR putcLCD ; --"--
            
            LDAA AT_DEMO ; Initialize the alarm time
            ADDA #TwoSec ; by adding on the 2 sec delay
            STAA AT_DEMO ; and save it in the alarm
CHK_DELAY_2 LDAA TOF_COUNTER ; If the current time
            CMPA AT_DEMO ; equals the alarm time
            BEQ A2 ; then display C
            BRA CHK_DELAY_2 ; and check the alarm again
            
A2          LDAA #’C’ ; Display C (forever)
            JSR putcLCD ; --"--
            SWI
            
            BSET DDRA,%11111100
            BSET DDRT,#$FF
            JSR STARFWD
            JSR PORTFWD
            JSR STARON
            JSR PORTON
            JSR STARREV
            JSR PORTREV
            JSR STAROFF
            JSR PORTOFF

MAIN        JSR UPDT_DISPL              ; -----------------------------------
            LDAA CRNT_STATE             ;                                   M
            JSR READ_SENSORS            ;                                   A
            JSR DISPATCHER              ;                                   I
            BRA MAIN                    ; ----------------------------------N

; data section     
;***********************
msg1          dc.b "Battery volt ",0
msg2          dc.b "State ",0
tab           dc.b "START ",0
              dc.b "FWD ",0
              dc.b "REV ",0
              dc.b "ALL_STP",0
              dc.b "FWD_TRN",0
              dc.b "REV_TRN",0


; Subroutine section

DISPATCHER    CMPA #START                 ; If it's the START state -------------------
              BNE NOT_START              	;                                           |
              JSR START_ST                ; then call START_ST routine                |
              BRA DISP_EXIT               ; and exit                                  |
                                          ;                                           |
NOT_START     CMPA #FORWARD           	  ; Else if it's the FORWARD state            |
              BNE NOT_FORWARD             ;                                           |
              JSR FWD_STATE               ; then call the FORWARD routine             |
              JMP DISP_EXIT               ; and exit                                  |
                                          ;                                           |
NOT_FORWARD   CMPA #REV                   ; Else if it's the REVERSE state            |
              BNE NOT_REVERSE             ;                                           |
              JSR REV_ST                  ; then call the REVERSE routine             |
              JMP DISP_EXIT               ; and exit                                  |
                                          ;                                           |
NOT_REVERSE   CMPA #ALL_STP         	    ; Else if it's the ALL_STOP state           |
              BNE NOT_ALL_STOP            ;                                           |
              JSR ALL_STP_ST              ; then call the ALL_STOP routine            |
              JMP DISP_EXIT               ; and exit                                  |
                                          ;                                           |
NOT_ALL_STOP  CMPA #FWD_TRN       	      ; Else if it's the FWD_TRN state            D
              BNE NOT_FWD_TRN             ;                                           I
              JSR FWD_TRN_ST              ; then call the FWD_TRN routine             S
              JMP DISP_EXIT               ; and exit                                  P
                                          ;                                           A                                          
                                          ;                                           T
NOT_FWD_TRN   CMPA #REV_TRN               ; Else if it's the REV_TRN state            C
              BNE NOT_REV_TRN             ;                                           H
              JSR REV_TRN_ST              ; then call REV_TRN_ST routine              E
              BRA DISP_EXIT               ; and exit                                  R
                                          ;                                           |
NOT_REV_TRN   SWI 	                      ; Else the CRNT_ST is not defined, so stop  |
DISP_EXIT     RTS                         ; Exit from the state dispatcher ------------
;***********************
START_ST      BRCLR PORTAD0,$04,NO_FWD    ; If /FWD_BUMP
              JSR INIT_FWD                ; Initialize the forward state
              MOVB #FORWARD,CURRENT_STATE ; Go into the FORWARD state
              BRA START_EXIT
NO_FWD        NOP                         ; Else
START_EXIT    RTS                         ; return to the MAIN routine
;***********************
FWD_ST        CMP SENSOR_LINE,EF_SENSOR_UP ; If SENSOR_LINE<EF_SENSOR_UP
              jl Less                      ; then
              jmp Both
        Less: JSR PORTFWD                  ; rotate right
        Both: JSR PORTREV                  ; Else if SENSOR_LINE>EF_SENSOR_UP rotate left
              BRSET PORTAD0,$04,NO_FWD_BUMP; If FWD_BUMP then
              JSR INIT_REV                ; Initialize the REVERSE routine
              MOVB #REVERSE,CURRENT_STATE ; set the state to REVERSE
              JMP FWD_EXIT                ; and return
              
NO_FWD_BUMP   BRSET PORTAD0,$08,NO_REAR_BUMP ; If REAR_BUMP, then we should stop
              JSR INIT_ALL_STP            ; so initialize the ALL_STOP state
              MOVB #ALL_STOP,CURRENT STATE; and change state to ALL)STOP
              JMP FWD_EXIT                ; and return
              
NO_REAR_BUMP  LDAA TOFCOUNTER             ; If Tc>Tfwd then
              CMPA T_FORWARD              ; the robot should make a turn
              BNE NO_FWD_TURN             ; so
              JSR INIT_FWD_TURN           ; initialize the FORWARD_TURN state
              MOVB #FORWARD_TURN,CURRENT_STATE; and go to that state
              JMP FWD_EXIT
              
NO_FWD_TRN    NOP                         ; Else
FWD_EXIT      RTS                         ; return to the MAIN routine
;***********************
REV_ST        CMP SENSOR_LINE,EF_SENSOR_UP ; If SENSOR_LINE<EF_SENSOR_UP
              jl Less                      ; then
              jmp Both
        Less: JSR PORTFWD                  ; rotate right
        Both: JSR PORTREV                  ; Else if SENSOR_LINE>EF_SENSOR_UP rotate left
              LDAA TOF_COUNTER            ; If Tc>Trev then
              CMPA T_REV                  ; the robot should make a FWD turn
              BNE NO_REV_TRN              ; so
              JSR INIT_REV_TRN            ; initialize the REV_TRN state
              MOVB #REV_TRN,CRNT_STATE    ; set state to REV_TRN
              BRA REV_EXIT                ; and return
NO_REV_TRN    NOP                         ; Else
REV_EXIT      RTS                         ; return to the MAIN routine
;***********************
ALL_STP_ST    BRSET PORTAD0,$04,NO_START  ; If FWD_BUMP
              BCLR PTT,%00110000          ; initialize the START state (both motors off)
              MOVB #START,CRNT_STATE      ; set the state to START
              BRA ALL_STP_EXIT            ; and return
NO_START      NOP                         ; Else
ALL_STP_EXIT  RTS                         ; return to the MAIN routine
;***********************
FWD_TRN_ST    LDAA TOF_COUNTER            ; If Tc>Tfwdturn then
              CMPA T_FWD_TRN              ; the robot should go FWD
              BNE NO_FWD_FT               ; so
              JSR INIT_FWD                ; initialize the FWD state
              MOVB #FWD,CRNT_STATE        ; set state to FWD
              BRA FWD_TRN_EXIT            ; and return
NO_FWD_FT     NOP                         ; Else
FWD_TRN_EXIT  RTS                         ; return to the MAIN routine
;***********************
REV_TRN_ST    LDAA TOF_COUNTER            ; If Tc>Trevturn then
              CMPA T_REV_TRN              ; the robot should go FWD
              BNE NO_FWD_RT               ; so
              JSR INIT_FWD                ; initialize the FWD state
              MOVB #FWD,CRNT_STATE        ; set state to FWD
              BRA REV_TRN_EXIT            ; and return
NO_FWD_RT     NOP                         ; Else
REV_TRN_EXIT  RTS                         ; return to the MAIN routine
;***********************
INIT_FWD      BCLR PORTA,%00000011        ; Set FWD direction for both motors
              BSET PTT,%00110000          ; Turn on the drive motors
              LDAA TOF_COUNTER            ; Mark the fwd time Tfwd
              ADDA #FWD_INT
              STAA T_FWD
              RTS
;***********************
INIT_REV      BSET PORTA,%00000011        ; Set REV direction for both motors
              BSET PTT,%00110000          ; Turn on the drive motors
              LDAA TOF_COUNTER            ; Mark the fwd time Tfwd
              ADDA #REV_INT
              STAA T_REV
              RTS
;***********************
INIT_ALL_STP  BCLR PTT,%00110000          ; Turn off the drive motors
              RTS
;***********************
INIT_FWD_TRN  BSET PORTA,%00000010        ; Set REV dir. for STARBOARD (right) motor
              LDAA TOF_COUNTER            ; Mark the fwd_turn time Tfwdturn
              ADDA #FWD_TRN_INT     
              STAA T_FWD_TRN
              RTS
;***********************
INIT_REV_TRN  BCLR PORTA,%00000010        ; Set FWD dir. for STARBOARD (right) motor
              LDAA TOF_COUNTER            ; Mark the fwd time Tfwd
              ADDA #REV_TRN_INT
              STAA T_REV_TRN
              RTS

;***********************
; guider subroutines
;***********************
; The following subroutine read the guider sensors and puts the results in RAM registers
READ_SENSORS  CLR SENSOR_NUM ; Select sensor number 0
              LDX #SENSOR_LINE ; Point at the start of the sensor array

RS_MAIN_LOOP  LDAA SENSOR_NUM ; Select the correct sensor input
              JSR SELECT_SENSOR ; on the hardware
              LDY #400 ; 20 ms delay to allow the
              JSR del_50us ; sensor to stabilize

              LDAA #%10000001 ; Start A/D conversion on AN1
              STAA ATDCTL5
              BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done

              LDAA ATDDR0L ; A/D conversion is complete in ATDDR0L
              STAA 0,X ; so copy it to the sensor register
              CPX #SENSOR_STBD ; If this is the last reading
              BEQ RS_EXIT ; Then exit

              INC SENSOR_NUM ; Else, increment the sensor number
              INX ; and the pointer into the sensor array
              BRA RS_MAIN_LOOP ; and do it again

RS_EXIT 	    RTS

;This subroutine selects  the sensor number passed in ACCA. The motor direction
; bits 0, 1, the guider sensor select bit 5 and the unused bits 6,7 in the
; same machine register PORTA are not affected.
; Bits PA2,PA3,PA4 are connected to a 74HC4051 analog mux on the guider board,
; which selects the guider sensor to be connected to AN1.
SELECT_SENSOR   PSHA ; Save the sensor number for the moment
                LDAA PORTA ; Clear the sensor selection bits to zeros
                ANDA #%11100011 ;
                STAA TEMP ; and save it into TEMP

                PULA ; Get the sensor number
                ASLA ; Shift the selection number left, twice
                ASLA ;
                ANDA #%00011100 ; Clear irrelevant bit positions

                ORAA TEMP ; OR it into the sensor bit positions
                STAA PORTA ; Update the hardware
                RTS

; This routine writes the sensor values to the LCD by passing sensor values in RAM locations SENSOR_LINE
; through SENSOR_STBD.
DP_FRONT_SENSOR EQU TOP_LINE+3
DP_PORT_SENSOR EQU BOT_LINE+0
DP_MID_SENSOR EQU BOT_LINE+3
DP_STBD_SENSOR EQU BOT_LINE+6
DP_LINE_SENSOR EQU BOT_LINE+9
; This section was made to display the sensor, but we updated it to use their reading to steer the eebot
DISPLAY_SENSORS 	LDAA SENSOR_BOW ; Get the FRONT sensor value
                  JSR BIN2ASC ; Convert to ascii string in D
                  LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
                  STD 0,X ; and write the 2 ascii digits there

                  LDAA SENSOR_PORT ; Repeat for the PORT value
                  JSR BIN2ASC
                  LDX #DP_PORT_SENSOR
                  STD 0,X

                  LDAA SENSOR_MID ; Repeat for the MID value
                  JSR BIN2ASC
                  LDX #DP_MID_SENSOR
                  STD 0,X

                  LDAA SENSOR_STBD ; Repeat for the STARBOARD value
                  JSR BIN2ASC
                  LDX #DP_STBD_SENSOR
                  STD 0,X

                  LDAA SENSOR_LINE ; Repeat for the LINE value
                  JSR BIN2ASC
                  LDX #DP_LINE_SENSOR
                  STD 0,X

                  LDAA #CLEAR_HOME ; Clear the display and home the cursor
                  JSR cmd2LCD ; "
                  LDY #40 ; Wait 2 ms until "clear display" command is complete
                  JSR del_50us

                  LDX #TOP_LINE ; Now copy the buffer top line to the LCD
                  JSR putsLCD

                  LDAA #LCD_SEC_LINE ; Position the LCD cursor on the second line
                  JSR LCD_POS_CRSR

                  LDX #BOT_LINE ; Copy the buffer bottom line to the LCD
                  JSR putsLCD
                  RTS


;From Assignment 2
;***********************
;* Initialization of the LCD: 4-bit data width, 2-line display,  *
;* turn on display, cursor and blinking off. Shift cursor right. *
;***********************
initLCD   BSET DDRS,%11110000 ; Configure pins PS7,PS6,PS5,PS4 for output
          BSET DDRE,%10010000 ; Configure pins PE7,PE4 for output
          LDY #2000 ; Wait for LCD to be ready
          JSR del_50us ; Jump to subroutine del_50us (delay)
          LDAA #$28 ; Set 4-bit data, 2-line display
          JSR cmd2LCD ; Jump to subroutine cmd2LCD (sends a command in accumulator A to the LCD)
          LDAA #$0C ; Display on, cursor off, blinking off
          JSR cmd2LCD ; Jump to subroutine cmd2LCD (sends a command in accumulator A to the LCD)
          LDAA #$06 ; Move cursor right after entering a character
          JSR cmd2LCD ; Jump to subroutine cmd2LCD (sends a command in accumulator A to the LCD)
          RTS
          
;***********
;* Clear display and home cursor *
;***********
clrLCD    LDAA #$01 ; Clear cursor and return to home position
          JSR cmd2LCD ; Jump to subroutine cmd2LCD (sends a command in accumulator A to the LCD)
          LDY #40 ; Wait until clear cursor command is complete
          JSR del_50us ; Jump to subroutine del_50us (delay)
          RTS

;*****************
;* ([Y] x 50us)-delay subroutine. E-clk=41,67ns. *
;*****************
del_50us: PSHX ; 2 E-clk
eloop:    LDX #30 ; 2 E-clk      -
iloop:    PSHA ; 2 E-clk         |
          PULA ; 3 E-clk         |
          PSHA ; 2 E-clk         |
          PULA ; 3 E-clk         |
          PSHA ; 2 E-clk         |
          PULA ; 3 E-clk         |
          PSHA ; 2 E-clk         | 50us
          PULA ; 3 E-clk         |
          PSHA ; 2 E-clk         |
          PULA ; 3 E-clk         |
          PSHA ; 2 E-clk         |
          PULA ; 3 E-clk         |
          PSHA ; 2 E-clk         | 
          PULA ; 3 E-clk         |
          NOP ; 1 E-clk          |
          NOP ; 1 E-clk          |
          DBNE X,iloop ; 3 E-clk -
          DBNE Y,eloop ; 3 E-clk
          PULX ; 3 E-clk
          RTS ; 5 E-clk

;*********************
;* This function sends a command in accumulator A to the LCD *
;*********************
cmd2LCD:  BCLR LCD_CNTR,LCD_RS ; select the LCD Instruction Register (IR)
          JSR dataMov ; send data to IR
          RTS

;**********************
;* This function outputs a NULL-terminated string pointed to by X *
;**********************
putsLCD   LDAA 1,X+ ; get one character from the string
          BEQ donePS ; reach NULL character?
          JSR putcLCD
          BRA putsLCD
donePS    RTS

;**********************
;* This function outputs the character in accumulator in A to LCD *
;**********************
putcLCD   BSET LCD_CNTR,LCD_RS ; select the LCD Data register (DR)
          JSR dataMov ; send data to DR
          RTS
          
;*********************
;* This function sends data to the LCD IR or DR depening on RS *
;*********************
dataMov   BSET LCD_CNTR,LCD_E ; pull the LCD E-sigal high
          STAA LCD_DAT ; send the upper 4 bits of data to LCD
          BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
          LSLA ; match the lower 4 bits with the LCD data pins
          LSLA ; match the lower 4 bits with the LCD data pins
          LSLA ; match the lower 4 bits with the LCD data pins
          LSLA ; match the lower 4 bits with the LCD data pins
          BSET LCD_CNTR,LCD_E ; pull the LCD E signal high
          STAA LCD_DAT ; send the lower 4 bits of data to LCD
          BCLR LCD_CNTR,LCD_E ; pull the LCD E-signal low to complete the write oper.
          LDY #1 ; adding this delay will complete the internal
          JSR del_50us ; operation for most instructions
          RTS

;From Lab 4 APPENDIX A          
;*******
;* Motor Control *
;*******

STARON  LDAA PTT
        ORAA #%00100000
        STAA PTT
        RTS
        
STAROFF LDAA PTT
        ANDA #%11011111
        STAA PTT
        RTS
        
PORTON  LDAA PTT
        ORAA #%00010000
        STAA PTT
        RTS
        
PORTOFF LDAA PTT
        ORAA #%11101111
        STAA PTT
        RTS
        
STARFWD LDAA PORTA
        ANDA #%11111101
        STAA PORTA
        RTS
        
STARREV LDAA PORTA
        ORAA #%00000010
        STAA PORTA
        RTS
        
PORTFWD LDAA PORTA
        ANDA #%11111110
        STAA PORTA
        RTS
        
PORTREV LDAA PORTA
        ORAA #%00000001
        STAA PTH
        RTS

;From Lab 4 APPENDIX B
;******
;* 5 Second Delay *
;******
DT_DEMO     EQU 115 ; 5 second delay

            ORG $3850
TOF_COUNTER RMB 1
AT_DEMO     RMB 1

            ORG $4000
Entry       LDS #$4000

            JSR ENABLE_TOF ; Jump to TOF init
            CLI
            
            LDAA TOF_COUNTER
            ADDA #DT_DEMO
            STAA AT_DEMO
            
CHK_DELAY   LDAA TOF_COUNTER
            CMPA AT_DEMO
            BEQ STOP_HERE
            
            NOP ; Do something during the display
            BRA CHK_DELAY ; and check the alarm again
            
STOP_HERE   SWI
;********************
ENABLE_TOF  LDAA #%10000000
            STAA TSCR1 ; Enable TCNT
            STAA TFLG2 ; Clear TOF
            LDAA #%10000100 ; Enable TOI and select prescale factor equal to 16
            STAA TSCR2
            RTS
;********************
TOF_ISR     INC TOF_COUNTER
            LDAA #%10000000 ; Clear by writing a logic 1 to the flag
            STAA TFLG2 ; TOF
            RTI
;********************
DISABLE_TOF LDAA #%00000100 ; Disable TOI and leave prescale factor at 16
            STAA TSCR2
            RTS

;*******
;* Interrupt Vectors *
;*******
ORG $FFFE
DC.W Entry ; Reset Vector
ORG $FFDE
DC.W TOF_ISR ; Timer Overflow Interrupt Vector
