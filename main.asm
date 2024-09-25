stm8/

	#include "mapping.inc"
  #include "stm8s003f3.inc"
	;all the libraries for peripherial are avaliable on https://github.com/aaw20904/STM8_library
	;--!!--SYSTEM CLOCK = 5MHz-----
	;please re-write it for case without crystal
	#define MIN_DURATION #$000A
	
	segment byte at 0000-200 'user_ram'
 
u16test          EQU $0000
u16delayDuration EQU $0002
_semaphore       EQU $0004

	segment 'rom'
main.l
	; initialize SP
	ldw X,#stack_end
	ldw SP,X
	
	#ifdef RAM0	
	; clear RAM0
ram0_start.b EQU $ram0_segment_start
ram0_end.b EQU $ram0_segment_end
	ldw X,#ram0_start
clear_ram0.l
	clr (X)
	incw X
	cpw X,#ram0_end	
	jrule clear_ram0
	#endif

	#ifdef RAM1
	; clear RAM1
ram1_start.w EQU $ram1_segment_start
ram1_end.w EQU $ram1_segment_end	
	ldw X,#ram1_start
clear_ram1.l
	clr (X)
	incw X
	cpw X,#ram1_end	
	jrule clear_ram1
	#endif

	; clear stack
stack_start.w EQU $stack_segment_start
stack_end.w EQU $stack_segment_end
	ldw X,#stack_start
clear_stack.l
	clr (X)
	incw X
	cpw X,#stack_end	
	jrule clear_stack


;                                      _      
;  _   _ ___  ___ _ __    ___ ___   __| | ___ 
; | | | / __|/ _ \ '__|  / __/ _ \ / _` |/ _ \
; | |_| \__ \  __/ |    | (_| (_) | (_| |  __/
;  \__,_|___/\___|_|     \___\___/ \__,_|\___|

 ;--init-clock-------------
 ;;EXTERNAL interrupt registers must be set before any interrupts!
   ; $0-fall and LOW, $1-rise, $2-fall, $3-rise & fall
  BSET EXTI_CR1 , #0 ;PAIS 0
	BSET EXTI_CR1 , #1 ;PAIS 1
 
  LD A, #00
	PUSH A
	CALL clkSetHsiDivider
	ADDW SP, #1
	LD A, #0
	PUSH A
	CALL clkSetCpuDivider
	ADDW SP, #1
	CALL clkSwitchToCrystal
	;----------------------
  
;peripherial ON
 LD A, #$8f
 PUSH A
 CALL clkBusPeripherial1
 ADDW SP,#1
 	;---init global variables: TIM1 period
  LDW X, #$0180
	LDW u16delayDuration, X
	LDW u16test, X
;PC3-TIM1 CH3
  BSET PC_DDR, #$3 ;output
  BSET PC_CR1, #$3 ; push-pull
  BSET PD_DDR, #4 ;PD4 - out
	BSET PD_CR1, #4 ;push-pull
;--PORTB EXTERN INTERRRUPT SET
	 BSET PA_CR1, #$3 ;pull-up resistor
	 
 ;==PROCEDURE==uart1ReceiverSetup
  LDW X, #$209;@baudRate16, divider
	PUSHW X
	LD A, #0  ;@dataLength8, $10->9bits (1stop 1 start) , $00->8bits
  PUSH A        
  LD A, #0;@stopBits8, active only when 8 bits: $00->1bit,      $20->2bits
  PUSH A  
	LD A, #0;@parity8, $00-disable, $04-enable: ($02-odd OR $00-even)
  PUSH A
	CALL uart1ReceiverSetup
  ADDW SP, #5  ;SP +5
	;===PROCEDURE 'tim1OnePulsePwmCh3Setup'
	;timer is stopped!To start, call - 'tim1OnePulseModeStart'
  LDW X, #$0062 ;@presc16,  $62=98, 5M/98=51KHz
  PUSHW x
	LDW X, #$0120;@base16, -> 255 is base of counter $ff->100Hz period
	PUSHW X
	LDW X, #$011f;@comp16, ->  pulse width, CCR1 content
	PUSHW X
	LD A, #$68;@mode8,  -> (CCMR) : $60 PWM_MODE1, $08 preload en.
	PUSH A
	LD A, #$82;@polarity8 -> (CCER)  $00 active HI,$02 active low
	PUSH A
	LD A, #$80;@preload8  ->  $80 preload enable, CR1 content
	PUSH A
	CALL tim1OnePulsePwmCh3Setup
	ADDW SP, #$09

	;;---set up PORTA pin for interrupt enable
	BSET PA_CR2, #$3 ;external interrupt PA3 turn on
	
infinite_loop.l
;  _                   
; | | ___   ___  _ __  
; | |/ _ \ / _ \| '_ \ 
; | | (_) | (_) | |_) |
; |_|\___/ \___/| .__/ 
;               |_|    
	NOP
	WFI
	jra infinite_loop
;	 _ _ _     
; | (_) |__  
; | | | '_ \ 
; | | | |_) |
; |_|_|_.__/ 
            
;-----L I B--
;-----FUNCTION
hexToNibble
;@character8
;SP+1
;-0  1    4
;[|A|RET|character]
	#define h01_char ($4,SP)
  PUSH A
  LD A, h01_char
	SUB A, #$30
	JRSLT _hxn_end_pr
		LD A, h01_char
		SUB A, #$3A
	JRSGT _letters_hex_
	;from 0 to 9
	  LD A, h01_char
		SUB A, #$30
		LD h01_char, A
		POP A
		RET
_letters_hex_
		LD A, h01_char
		SUB A, #$41
	JRSLT _hxn_end_pr
		LD A, h01_char
		SUB A, #$47
	JRSGT _hxn_end_pr 
		;from A to F
		LD A, h01_char
		SUB A, #$37
		LD h01_char, A
		POP A
		RET
_hxn_end_pr
   LD A, #2
	 NEG A
	 LD h01_char, A
	 POP A
	 RET
	;===PROCEDURE 'tim1OnePulsePwmCh3Setup'
	;timer is stopped!To start, call - 'tim1OnePulseModeStart'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
		;after return: SP+9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OnePulsePwmCh3Setup
			PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _012_v8a $00
	#define _012_prel $05
	#define _012_pol $06
	#define _012_mode $07
	#define _012_compH $08
	#define _012_compL $09
	#define _012_baseH $0A
	#define _012_baseL $0B
	#define _012_prescH $0C
	#define _012_prescL $0D
	;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_012_compH,SP)
	LD TIM1_CCR3H, A
	LD A, (_012_compL,SP)
	LD TIM1_CCR3L, A
		;--prescaler high byte firstly
	LD A, (_012_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_012_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_012_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_012_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_012_mode,SP)
	LD TIM1_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$F0
	LD (_012_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_012_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_012_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_012_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$03;//one pulse mode ,stopped
	NOP
	;--free memory
	ADDW SP, #$01
	NOP
	POP A
	RET
	
		;====P R O C E D U R E===turn on clk bus
	;@peripherial8
	;TIM1-$80,TIM3-$40,TIM2/5-$20,TIM4/6-$10,UART-see datasheet,
	;SPI-$2,I2C-1
	;STACK after return +1
clkBusPeripherial1
	PUSH A
	LD A, ($04,SP)
	LD CLK_PCKENR1, A
	POP A
	RET
	;==P R O C E D U R E=="set HSI divider"
	;--@ char divider
	;STACK after return +1
clkSetHsiDivider
	;--store registers A,X,Y,CC (1+2+2+1=6Bytes)
	PUSH A
	;-read default value 
	LD A, CLK_CKDIVR
	;---clear all the hsi divider bits
	AND A, #$E7
	;--1st paprameter has offset 9 bytes
	; because A,X,Y,CC,SP has ben stored later 
	OR A, ($04,SP)
	;---update CLK_CKDIVR
	LD CLK_CKDIVR, A
	;--restore registers
	POP A
	RET
	;===P R O C E D U R E=switch to Crystal
	;--NO PARAMS, using interrupts (irq 2)
	;STACK after return 0
clkSwitchToCrystal
	PUSH A
	;---Enable the switching mechanism
	BSET CLK_SWCR, #1 ;SWEN
	BSET CLK_SWCR, #2; SWIEN enable interrupt
	;---select source clock
	;0xE1: HSI selected as master clock source (reset value)
	;0xD2: LSI selected as master clock source (only if LSI_EN
	;option bit is set)
	;OUR CASE = 0xB4: HSE selected as master clock source
	LD A, #$B4
	LD CLK_SWR, A
	;waiting until clock system does 
	; switching process automatically
		NOP
		NOP
		NOP
		NOP
		;waiting until the HSI be ready
  WFI
	   ;!clear SWIF flag in the interrupt service routine
	POP A
	RET
	;======P R O C E D U R E==="set CPU divider"
	;@ char divider 
	;STACK after return +1
clkSetCpuDivider
		;--store registers A,X,Y,CC (1+2+2+1=6Bytes)
	PUSH A
	;-read default value 
	LD A, CLK_CKDIVR
	;---clear all the hsi divider bits
	AND A, #$f8
	;--1st paprameter has offset 9 bytes
	; because A,X,Y,CC,SP has ben stored later
	OR A, ($04,SP)
	;---update CLK_CKDIVR
	LD CLK_CKDIVR, A
		;--restore registers
	POP A
	RET	
 ;==PROCEDURE==uart1ReceiverSetup
;@baudRate16, divider
;@dataLength8, $10->9bits (1stop 1 start) , $00->8bits
          ;(set  manually below)
;@stopBits8, active only when 8 bits: $00->1bit, $20->2bits
;@parity8, $00-disable, $04-enable: ($02-odd OR $00-even)
;-----------
;SP +5
;  0  1 2 4    5         6      7      8 
;[v8a|A|RET|stopBits|dataLength|baudRate]

uart1ReceiverSetup
  ;--store A
	PUSH A
	;--allocate 1 byte
	SUBW SP, #$01
	#define _U1000_v8a $00 
	#define _U1000_parity $05
	#define _U1000_stopBits $06 
	#define _U1000_dataLength $07
	#define _U1000_baudRateH $08
	#define _U1000_baudRateL $09
	;--disable Rx, Tx
	BRES UART1_CR2, #$2; REN flag
	BRES UART1_CR2, #$3; TEN flag
  ;--when the length 9 bits-jump below
	LD A, #$10
	AND A, (_U1000_dataLength,SP)
	JRNE L_U1000_nsb ;when 9 bits-go to label
	;--when 8 bit set stop bits
	LD A, (_U1000_stopBits,SP)
	LD UART1_CR3, A
L_U1000_nsb
  ;--write data length and parity
	LD A,(_U1000_dataLength,SP)
	OR A, (_U1000_parity,SP)
	LD UART1_CR1, A
	;prepare UART1_BRR2
	;n4 n1
	LD A, (_U1000_baudRateL,SP)
	AND A, #$0F;
	LD ( _U1000_v8a,SP),A; store nibble 1
	LD A, (_U1000_baudRateH,SP)
	AND A, #$f0 ; nibble 4
	OR A, (_U1000_v8a,SP) ; n4+n1
	LD ( _U1000_v8a,SP),A; store BRR2 [n4,n1]
	;;--send -TO REGISTER BRR2
	LD UART1_BRR2 , A
	;----n3 n2
	LD A, (_U1000_baudRateH,SP)
	SWAP A
	AND A, #$F0
	LD ( _U1000_v8a,SP),A; store n3
	LD A, (_U1000_baudRateL,SP)
	SWAP A
	AND A, #$0f
	OR A, ( _U1000_v8a,SP)
	;--send to register BRR1
	LD UART1_BRR1, A
	; RIEN interrupt on receive
	BSET UART1_CR2, #$5 
	;--setting REN bit "Receiver enable"
	BSET UART1_CR2 , #$2
	;--restore stack
	ADDW SP, #$01
	POP A
	RET
;  ___ ____  ____  
; |_ _/ ___||  _ \ 
;  | |\___ \| |_) |
;  | | ___) |  _ < 
; |___|____/|_| \_\

;clock--Interrupt---Service----Routine
clockISR
  BRES CLK_SWCR, #3 ; clear SWIF (in the ISR)
  IRET
;---PORT A Interrupt service routine (Zero Cross Detection)
portA_ISR
 BTJT PD_IDR, #4, led_tst_off
  BSET PD_ODR, #4
	JP pa_isr_end
led_tst_off
  BRES PD_ODR, #4
pa_isr_end
  LDW X, u16delayDuration
	TNZW X
	JREQ _empty_t1
	LD A, XH
	LD TIM1_ARRH, A
	LD A, XL
	LD TIM1_ARRL, A
  DECW X
	LD A, XH
	LD TIM1_CCR3H, A
	LD A, XL
	LD TIM1_CCR3L, A
	BSET TIM1_CR1, #0
_empty_t1
  
 IRET
 
 ;-UART interrupt service routine
uartRxISR
 SUBW SP, #1 
 LD A, UART1_DR
 PUSH A
 CALL hexToNibble
 POP A
 JRMI _end_rx ;exit when symbol non-hex
 BTJT _semaphore, #3, _rx_4_nibble
  ;nib 4 
	CLRW X
	SWAP A
	LD XH, A
	LDW u16test, X
	BSET _semaphore, #3
	JP _end_rx
_rx_4_nibble
 BTJT _semaphore, #2, _rx_3_nibble
  ;nib 3
	CLRW X
	LDW X, u16test
  LD ($0,SP), A ;store as local var
	LD A, XH ;high byte
	OR A, ($0,SP) 
	LD XH, A
	LDW u16test, X
 BSET _semaphore, #2
 JP _end_rx
_rx_3_nibble
 BTJT _semaphore, #1, _rx_2_nibble
	;nib 2
	CLRW X
	SWAP A
	LDW X, u16test
	LD XL, A
	LDW u16test, X
 BSET _semaphore, #1
 JP _end_rx
_rx_2_nibble
  BTJT _semaphore, #0, _rx_1_nibble
	;nib 1
	CLRW X
	LDW X, u16test
  LD ($0,SP), A ;store as local var
	LD A, XL ;low byte
	OR A, ($0,SP) 
	LD XL, A
	LDW u16test, X
	LD A, #$F0
	AND  A, _semaphore ;clear semaphore
	LD _semaphore, A
	LDW Y, u16test ;copy word
	LDW u16delayDuration, Y ; 
_rx_1_nibble

_end_rx
  ADDW SP, #1 
 IRET
	
	interrupt NonHandledInterrupt
NonHandledInterrupt.l
	iret

	segment 'vectit'
	dc.l {$82000000+main}									; reset
	dc.l {$82000000+NonHandledInterrupt}	; trap
	dc.l {$82000000+NonHandledInterrupt}	; irq0
	dc.l {$82000000+NonHandledInterrupt}	; irq1
	dc.l {$82000000+clockISR}	; irq2
	dc.l {$82000000+portA_ISR}	; irq3
	dc.l {$82000000+NonHandledInterrupt}	; irq4
	dc.l {$82000000+NonHandledInterrupt}	; irq5
	dc.l {$82000000+NonHandledInterrupt}	; irq6
	dc.l {$82000000+NonHandledInterrupt}	; irq7
	dc.l {$82000000+NonHandledInterrupt}	; irq8
	dc.l {$82000000+NonHandledInterrupt}	; irq9
	dc.l {$82000000+NonHandledInterrupt}	; irq10
	dc.l {$82000000+NonHandledInterrupt}	; irq11
	dc.l {$82000000+NonHandledInterrupt}	; irq12
	dc.l {$82000000+NonHandledInterrupt}	; irq13
	dc.l {$82000000+NonHandledInterrupt}	; irq14
	dc.l {$82000000+NonHandledInterrupt}	; irq15
	dc.l {$82000000+NonHandledInterrupt}	; irq16
	dc.l {$82000000+NonHandledInterrupt}	; irq17
	dc.l {$82000000+uartRxISR}	; irq18
	dc.l {$82000000+NonHandledInterrupt}	; irq19
	dc.l {$82000000+NonHandledInterrupt}	; irq20
	dc.l {$82000000+NonHandledInterrupt}	; irq21
	dc.l {$82000000+NonHandledInterrupt}	; irq22
	dc.l {$82000000+NonHandledInterrupt}	; irq23
	dc.l {$82000000+NonHandledInterrupt}	; irq24
	dc.l {$82000000+NonHandledInterrupt}	; irq25
	dc.l {$82000000+NonHandledInterrupt}	; irq26
	dc.l {$82000000+NonHandledInterrupt}	; irq27
	dc.l {$82000000+NonHandledInterrupt}	; irq28
	dc.l {$82000000+NonHandledInterrupt}	; irq29

	end
