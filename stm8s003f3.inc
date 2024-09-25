;----STM8S003F3 peripherial define file
;--Registers of the MCU:
	
	;---CLK-----
	#define CLK_ICKR 	$50C0;
	#define CLK_ECKR 	$50C1;
	#define CLK_CMSR 	$50C3;
	#define CLK_SWR  	$50C4;
	#define CLK_SWCR  	$50C5;
	#define CLK_CKDIVR  	$50C6;
	#define CLK_PCKENR1  	$50C7;
	#define CLK_CSSR     	$50C8;
	#define CLK_CCOR     	$50C9;
	#define CLK_PCKENR2  	$50CA;
	
	#define HSERDY 	#$02
	#define HSEEN 	#$01
	
	#define SWIF    #$08 
	#define SWIEN 	#$04
  #define SWEN	  #$02
	#define SWBSY   #$01
	
	#define HSIDIV_1 #$00
	#define HSIDIV_2 #$08
	#define HSIDIV_4 #$10
	#define HSIDIV_8 #$18
	
	#define CPUDIV_1 #$00
	#define CPUDIV_2 #$01
	#define CPUDIV_4 #$02
	#define CPUDIV_8 #$03
	#define CPUDIV_16 #$04
	#define CPUDIV_32 #$05
	#define CPUDIV_64 #$06
	#define CPUDIV_128 #$07
	;-------ITC------
	#define EXTI_CR1  $50A0;
	#define EXTI_CR2  $50A1;
	;----FLASH-------
	#define	FLASH_CR1 $505A;
	#define	FLASH_CR2 $505B;
	#define	LASH_NCR2 $505C;
	#define	FLASH_FPR $505D;
	#define	FLASH _NFPR $505E; 	
	#define	FLASH _IAPSR $505F;
	;--------PORT A-----
	#define PA_ODR 	$5000;
	#define	PA_IDR 	$5001;
	#define	PA_DDR 	$5002;
	#define	PA_CR1	$5003;
	#define	PA_CR2  $5004;
	;--------PORT B------
	#define	PB_ODR 	$5005;
	#define	PB_IDR	$5006;
	#define	PB_DDR	$5007;
	#define	PB_CR1	$5008;
	#define	PB_CR2	$5009;
	;---------PORT  C-----
	#define	PC_ODR	$500A;
	#define	PC_IDR 	$500B;
	#define	PC_DDR 	$500C;
	#define	PC_CR1	$500D;
	#define	PC_CR2	$500E;
	
	;----PORT D-------
	#define	PD_ODR	$500F;
	#define	PD_IDR 	$5010;
	#define	PD_DDR 	$5011;
	#define	PD_CR1	$5012;
	#define	PD_CR2	$5013;
	;--------PORT E-------
	#define	PE_ODR	$5014;
	#define	PE_IDR 	$5015;
	#define	PE_DDR 	$5016;
	#define	PE_CR1	$5017;
	#define	PE_CR2	$5018;
	;---------PORT F------
	#define	PF_ODR	$5019;
	#define	PF_IDR 	$501A;
	#define	PF_DDR 	$501B;
	#define	PF_CR1	$501C;
	#define	PF_CR2	$501D;
	;------ITC---------
	#define	EXTI_CR1 $50A0;
	#define EXTI_CR2 $50A1;
	;--------TIM 1----------
	#define	TIM1_CR1  $5250;
	#define	TIM1_CR2  $5251;
	#define	TIM1_SMCR $5252;
	#define	TIM1_ETR  $5253;
	#define	TIM1_IER  $5254;
	#define	TIM1_SR1  $5255;
	#define	TIM1_SR2  $5256;
	#define	TIM1_EGR  $5257;
	#define	TIM1_CCMR1 $5258;
	#define	TIM1_CCMR2 $5259;
	#define	TIM1_CCMR3 $525A;
	#define	TIM1_CCMR4  $525B;
	#define	TIM1_CCER1 $525C;
	#define	TIM1_CCER2 $525D;
	#define	TIM1_CNTRH $525E;
	#define	TIM1_CNTRL  $255f;
	#define	TIM1_PSCRH  $5260;
	#define	TIM1_PSCRL $5261;
	#define	TIM1_ARRH  $5262;
	#define	TIM1_ARRL  $5263;
	#define	TIM1_RCR   $5264;
	#define	TIM1_CCR1H $5265;
	#define	TIM1_CCR1L $5266;
	#define	TIM1_CCR2H $5267;
	#define	TIM1_CCR2L $5268;
	#define	TIM1_CCR3H $5269;
	#define	TIM1_CCR3L $526A;
	#define	TIM1_CCR4H  $526B;
	#define	TIM1_CCR4L $526C;
	#define	TIM1_BKR   $526D;
	#define	TIM1_DTR   $526E;
	#define	TIM1_OISR  $526F;
	
	#define TIM_CR1_CENTER_ALINGN_1 #$20
	#define TIM_CR1_CENTER_ALINGN_2 #$40
	#define TIM_CR1_CENTER_ALINGN_3 #$60
  #define TIM_CR1_COUNTS_UP #$00
	#define TIM_CR1_COUNTS_DOWN #$10
	#define TIM_CR1_PRELOAD #$80
	#define TIM_CCMR_ACTIVE_ON_MATCH #$10
	#define TIM_CCMR_INACTIVE_ON_MATCH #$20
	#define TIM_CCMR_TOGGLE   #$30
	#define TIM_CCMR_FORCE_INACTIVE #$40 
	#define TIM_CCMR_FORCE_ACTIVE   #$50
	;-------------SPI----------
	#define SPI_CR1  $5200;
	#define	SPI_CR2 $5201;
	#define	SPI_ICR  $5202;
	#define	SPI_SR	$5203;
	#define	SPI_DR	$5204;
	#define	SPI_CRCPR	$5205;
	#define	SPI_RXCRCR	$5206;
	#define	SPI_TXCRCR	$5207;
	;---------------I2C----------
	#define	I2C_CR1		$5210;
	#define	I2C_CR2 	$5211;
	#define	I2C_FREQR	$5212;
	#define	I2C_OARL 	$5213;
	#define	I2C_OARH	$5214;
	#define	I2C_DR 		$5216;
	#define	I2C_SR1		$5217;
	#define	I2C_SR2 	$5218;
	#define	I2C_SR3		$5219;
	#define	I2C_ITR		$521A;
	#define	I2C_CCRL 	$521B;
	#define	I2C_CCRH	$521C;
	#define	I2C_TRISER	$521D;
	#define	I2C_PECR	$521E;
	;------------UART--------------
	#define	UART1_SR	$5230;
	#define	UART1_DR 	$5231;
	#define	UART1_BRR1	$5232;
	#define	UART1_BRR2	$5233;
	#define	UART1_CR1	$5234;
	#define	UART1_CR2	$5235;
	#define	UART1_CR3	$5236;
	#define	UART1_CR4	$5237;
	#define	UART1_CR5	$5238;
	#define	UART1_GTR	$5239;
	#define	UART1_PSCR	$523A;
	;----------TIM4----------------
	#define	TIM4_CR1	$5340;
	#define	TIM4_IER	$5343;
	#define	TIM4_SR		$5344;
	#define	TIM4_EGR	$5345;
	#define	TIM4_CNTR	$5346;
	#define	TIM4_PSCR	$5347;
	#define	TIM4_ARR 	$5348;
	;---------ADC1-----------------
	
	#define ADC_CSR 	$5400;
	#define	ADC_CR1		$5401;
	#define	ADC_CR2		$5402;
	#define	ADC_CR3		$5403;
	#define	ADC_DRH		$5404;
	#define	ADC_DRL		$5405;
	#define	ADC_TDRH	$5406;
	#define	ADC_TDRL	$5407;
	#define	ADC_HTRH	$5408;
	#define	ADC_HTRL	$5409;
	#define	ADC_LTRH	$540A;
	#define	ADC_LTRL	$540B;
	#define	ADC_AWSRH 	$540C;
	#define	ADC_AWSRL	$540D;
	#define	ADC_AWCRH	$540E;
	#define	ADC_AWCRL 	$540F;
	;;buffers
	#define	ADC_DB0RH 	$53E0;
	#define	ADC_DB0RL 	$53E1;
	#define	ADC_DB1RH 	$53E2;
	#define	ADC_DB1RL 	$53E3;
	#define	ADC_DB2RH 	$53E4;
	#define	ADC_DB2RL 	$53E5;
	#define	ADC_DB3RH 	$53E6;
	#define	ADC_DB3RL 	$53E7;
	#define	ADC_DB4RH 	$53E8;
	#define	ADC_DB4RL 	$53E9;
	#define	ADC_DB5RH 	$53EA;
	#define	ADC_DB5RL 	$53EB;
	#define	ADC_DB6RH 	$53EC;
	#define	ADC_DB6RL 	$53ED;
	#define	ADC_DB7RH 	$53EC;
	#define	ADC_DB7RL 	$53ED;
	#define	ADC_DB8RH 	$53EC;
	#define	ADC_DB8RL 	$53ED;
	
	
	;----------TIM2----------
	#define	TIM2_CR1 	$5300;
	#define	TIM2_IER	$5303;
	#define	TIM2_SR1 	$5304;
	#define	TIM2_SR2	$5305;
	#define	TIM2_EGR	$5306;
	#define	TIM2_CCMR1	$5307;
	#define	TIM2_CCMR2	$5308;
	#define	TIM2_CCMR3	$5309;
	#define	TIM2_CCER1	$530A;
	#define	TIM2_CCER2	$530B;
	#define	TIM2_CNTRH 	$530C;
	#define	TIM2_CNTRL  $530D;
	#define	TIM2_PSCR	$530E;
	#define	TIM2_ARRH	$530F;
	#define	TIM2_ARRL	$5310;
	#define	TIM2_CCR1H	$5311;
	#define	TIM2_CCR1L	$5312;
	#define	TIM2_CCR2H      $5313;
	#define	TIM2_CCR2L	$5314;
	#define	TIM2_CCR3H	$5315;
	#define	TIM2_CCR3L	$5316;
	
	
	