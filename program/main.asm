.include "D:\radio\AVRProjects\macro.inc"
.include "m8adef.inc"

;════════════════════════════════════════════════════════════════════════════════════════
;				ДЕФАЙНЫ 
;════════════════════════════════════════════════════════════════════════════════════════

.EQU	MAIN_FREQ = 8000000		; Частота МК, 8 МГц

.EQU	LINE_FREQ = 50			; Частота сети, 50 Гц

.EQU	LO_FUSE = 0x84			; Фьюзы
.EQU	HI_FUSE = 0xD9

.EQU	minPWM = 1
.EQU	maxPWM = 1023
.EQU	startPWM = 15

; Рассчет значений АЦП стаблизаиора Крамера
.EQU	xADC = 256					; Разрешение АЦП
						; Значения напряжений умножены на 10, чтобы не потерять десятичную часть
.EQU	Vref = 5.1 * 10				; Опорное напряжение
.EQU	Vin = 2.5 * 10				; Значение напряжения, которое изменяется и измеряется через делитель на два
.EQU	MULT = xADC/Vref
.SET	vADC = Vin * MULT		; Вычисляем мин. и макс. значения АЦП. 

; Предделитель Т/С0
.EQU	T0_PR_SCLR = 2 ; 0...7 - останов., 1, 8, 64, 256, 1024, Т0↓, Т0↑
			.EQU S_CS02 = (T0_PR_SCLR & 4)>>CS02	.EQU S_CS01 = (T0_PR_SCLR & 2)>>CS01	.EQU S_CS00 = (T0_PR_SCLR & 1)>>CS00

; Пресеты индикатора
	; Индикатор с общим Анодом
	; Сегменты - активный низкий
	; Разряды - активный высокий
.EQU	DP_ON	= ~0b01111111
.EQU	DP_OFF	= ~0b11111111

.EQU	NUM_OF_DIG	= 4
.EQU	FIRST_DIG	= ~0b0000_0001
.EQU	LAST_DIG	= ~(1<<(NUM_OF_DIG - 1))

; Программный счетчик вызова процедуры обновления индикатора
		; (MAIN_FREQ / Предделитель Т/С0 / (Предел счета)) / (LINE_FREQ * NUM_OF_DIG) 
.EQU	SFT_CNTR1_VALUE = (MAIN_FREQ / 8 / (256 - 128)) / (LINE_FREQ * NUM_OF_DIG)

; Пользовательские флаги
.EQU	REFR_IND_FLG	= 0b0000_0001
.EQU	INT_VOLT_SOURCE	= 0b0000_0010

;.DEF __ = R0
;.DEF __ = R1
;.DEF __ = R2
;.DEF __ = R3
;.DEF __ = R4
;.DEF __ = R5
;.DEF __ = R6
;.DEF __ = R7
;.DEF __ = R8
;.DEF __ = R9
;.DEF __ = R10
;.DEF __ = R11
;.DEF __ = R12
;.DEF __ = R13
;.DEF __ = R14
;.DEF __ = R15
.DEF TEMP = R16					; через этот регистр идут промежуточные опирации
.DEF TEMP2 = R17
.DEF TEMP3 = R18
.DEF TEMP4 = R19
.DEF TEMP5 = R20
.DEF TEMP6 = R21
.DEF TEMP7 = R22
;.DEF __ = R23
;.DEF __ = R24
.DEF USR_FLG = R25
;.DEF __ = R26 ; XL
;.DEF __ = R27 ; XH
;.DEF __ = R28 ; YL
;.DEF __ = R29 ; YH
;.DEF __ = R30 ; ZL
;.DEF __ = R31 ; ZH

;════════════════════════════════════════════════════════════════════════════════════════
;				ВЕКТОРА ПРЕРЫВАНИЙ 
;════════════════════════════════════════════════════════════════════════════════════════
.CSEG

.ORG	$0000		RJMP   Reset	;RESET
.ORG	INT0addr	RETI	; External Interrupt Request 0
.ORG	INT1addr	RETI	; External Interrupt Request 1
.ORG	OC2addr		RETI	; Timer/Counter2 Compare Match
.ORG	OVF2addr	RETI	; Timer/Counter2 Overflow
.ORG	ICP1addr	RETI	; Timer/Counter1 Capture Event
.ORG	OC1Aaddr	RETI	; Timer/Counter1 Compare Match A
.ORG	OC1Baddr	RETI	; Timer/Counter1 Compare Match B
.ORG	OVF1addr	RETI	; Timer/Counter1 Overflow
.ORG	OVF0addr	RJMP	T0_OVER	; Timer/Counter0 Overflow
.ORG	SPIaddr		RETI	; Serial Transfer Complete
.ORG	URXCaddr	RETI	; USART, Rx Complete
.ORG	UDREaddr	RETI	; USART Data Register Empty
.ORG	UTXCaddr	RETI	; USART, Tx Complete
.ORG	ADCCaddr	RJMP	ADCConvComp	; ADC Conversion Complete
.ORG	ERDYaddr	RETI	; EEPROM Ready
.ORG	ACIaddr		RETI	; Analog Comparator
.ORG	TWIaddr		RETI	; 2-wire Serial Interface
.ORG	SPMRaddr	RETI	; Store Program Memory Ready

.ORG	INT_VECTORS_SIZE	; Конец таблицы прерываний

;════════════════════════════════════════════════════════════════════════════════════════
;				ОБРАБОТЧИКИ ПРЕРЫВАНИЙ 
;════════════════════════════════════════════════════════════════════════════════════════

;----------- Прерывание АЦП -------------------------------------------------------------
ADCConvComp:
			SBI		PORTD, PD2		; Индикатор входа в прерывание АЦП

			PUSH	TEMP			; Сохранение помойки
			IN		TEMP, SREG		; Сохранение регистра статуса
			PUSH	TEMP
			PUSH	XH
			PUSH	XL
			PUSH	TEMP2
			PUSH	TEMP3
			PUSH	TEMP4
				
			SBRS	USR_FLG, 1					; Если флаг вн. ИОН установлен, производилось измерение внутреннего ИОН и перехода НЕ будет
			RJMP	KRAMER						; Иначе идем обрабатывать данные о напряжении стаба Крамера
						
	; Обработка значения напряжения внутреннего ИОН		
			CBR		USR_FLG, INT_VOLT_SOURCE	; Сброс флага преобразования для вн. ИОН

			IN		TEMP, ADCL			; Берем два байта АЦП и делаем из них смещение для загрузки из памяти программ
			IN		TEMP2, ADCL			; Младший байт берем дважды
			IN		TEMP3, ADCH			; В старшем байте смещение получится само-собой

										; Настройка АЦП для измерения напряжения вн. ИОН
			LDI		TEMP4, 0<<REFS1 | 0<<REFS0 | 1<<ADLAR | 1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 0<<MUX0	; Внутрений ИОН, опора - ножка AREF, выравнивание вправо
			OUT		ADMUX, TEMP4

			ANDI	TEMP, 0b0000_1111	; Выделяем младший полубайт
			SWAP	TEMP2				; Выделяем старший полубайт
			ANDI	TEMP2, 0b0000_1111

			LDI		XH, HIGH(IND_DATA)		; Адрес памяти индикатора
			LDI		XL, LOW(IND_DATA)

			LDI		ZH, HIGH(seg_table*2)	; Адрес начала таблицы индикатора
			; FIRST DIGIT
			LDI		ZL, LOW(seg_table*2)
			ADD		ZL, TEMP3
			LPM		TEMP3, Z		
			ST		X+, TEMP3
			; SECOND DIGIT
			LDI		ZL, LOW(seg_table*2)
			ADD		ZL, TEMP2
			LPM		TEMP2, Z		
			ST		X+, TEMP2
			; THIRD DIGIT
			LDI		ZL, LOW(seg_table*2)
			ADD		ZL, TEMP
			LPM		TEMP, Z		
			ST		X+, TEMP
			; FOURTH DIGIT
			LDI		TEMP, $FF
			ST		X, TEMP
						
			RJMP	EXIT

	; Корректриовка значения ШИМ стабилизатора Крамера
KRAMER:																	
			SBR		USR_FLG, INT_VOLT_SOURCE	; Установка флага, что след. преобразование для вн. ИОН

			CBI		PORTD, PD3		; Гашение индикатора предела

			IN		TEMP, ADCH		; Берем значение АЦП
							; Настройка АЦП для измерения напряжения стаба Крамера
			LDI		TEMP2, 0<<REFS1 | 0<<REFS0 | 0<<ADLAR | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0	; Внешний вход ADC0, опора - ножка AREF, выравнивание влево
			OUT		ADMUX, TEMP2

			IN		TEMP2, OCR1AL	; Берем значчение ШИМ
			IN		TEMP3, OCR1AH
			
			CPI		TEMP, vADC		; Сравниваем с константой напряжения
			BREQ	KRAMER_EXIT			; Если равно, сразу уходим
			BRCS	INCR			; Если значение из АЦП меньше константы, установится флаг переноса и выполнится переход
									; Иначе будет процедура уменьшения 
decr:		LDI		TEMP6, LOW(minPWM)
			LDI		TEMP7, HIGH(minPWM)
			CP		TEMP2, TEMP6	; Проверяем, не достиг ли минимума ШИМ
			CPC		TEMP3, TEMP7
			BREQ	VOLTLIMIT		; Если достиг, то уходим, уменьшать некуда, и зажигаем светодиод предела
			SUBI	TEMP2, 1		; Вычитание единицы
			SBCI	TEMP3, 0		; Вычитание нуля и флага переноса
			RJMP	KRAMER_EXIT

incr:		LDI		TEMP6, LOW(maxPWM)
			LDI		TEMP7, HIGH(maxPWM)
			CP		TEMP2, TEMP6	; Проверяем, не достиг ли предела ШИМ
			CPC		TEMP3, TEMP7
			BREQ	KRAMER_EXIT			; Если достиг, то выходим
			LDI		TEMP4, 1		; Загружаем единицу
			CLR		TEMP5			; Загружаем ноль
			ADD		TEMP2, TEMP4	; Складываем младший байт ШИМ с единицей
			ADC		TEMP3, TEMP5	; Складываем старший байт ШИМ с нулем и переносом
			RJMP	KRAMER_EXIT

VOLTLIMIT:	SBI		PORTD, PD3		

KRAMER_EXIT:	
			OUT		OCR1AH, TEMP3	; Запись нового значения ШИМ
			OUT		OCR1AL, TEMP2

EXIT:		POP		TEMP4
			POP		TEMP3
			POP		TEMP2
			POP		XL
			POP		XH
			POP		TEMP			; Восстанавливаем регистр статуса
			OUT		SREG, TEMP
			POP		TEMP			; Восстанавливаем помойку
			
			RETI					; Выход с разрешением прерываний
;----------------------------------------------------------------------------------------

;----------- Переполнение Т/С0 ----------------------------------------------------------
T0_OVER:	PUSH	TEMP			; Сохранение помойки
			IN		TEMP, SREG		; Сохранение регистра статуса
			PUSH	TEMP

			LDS		TEMP, SFT_CNTR1	; Берем программынй счетчик
			DEC		TEMP			; Уменьшаем
			
			BRBS	SREG_S, ZERO	; Если в регистре был ноль, то произойдет переход через ноль и установится флаг отрицательного значения
			BREQ	ZERO			; Если достигли нуля, то идем делать дела
			RJMP	NOT_ZERO		; Иначе просто сохраняем и уходим

ZERO:		SBR		USR_FLG, REFR_IND_FLG	; Установка флага обновления индикатора
			LDI		TEMP, SFT_CNTR1_VALUE	; Установка начального значения программного счетчика обновления индикатора

NOT_ZERO:	STS		SFT_CNTR1, TEMP
			
			POP		TEMP			; Восстанавливаем регистр статуса
			OUT		SREG, TEMP
			POP		TEMP			; Восстанавливаем помойку

			RETI
;----------------------------------------------------------------------------------------

;════════════════════════════════════════════════════════════════════════════════════════
;				НАЧАЛЬНАЯ ИНИЦИАЛИЗАЦИЯ
;════════════════════════════════════════════════════════════════════════════════════════

Reset:							; Отсюда стартуем после сброса
			SBI		DDRB, PB1					; Порт РВ1 на выход - тут транзистор понижайки
			SBI		PORTB, PB1					; Выход в HI - открываем транзистор для поддержания напряжения
												; пока не запустится ШИМ

			; Установка начльного значения ШИМ
			OUTI	OCR1AH, HIGH(startPWM)
			OUTI	OCR1AL, LOW(startPWM)
			; Сброс предделителя т/с0 и т/с1
			LDI		TEMP, 1<<PSR10
			OUT		SFIOR, TEMP			
			
		/*			
		Настройка Т/С1:
				Пределитель: CS1(2,1,0) = 000...111 - останов., 1, 8, 64, 256, 1024, Т0↓, Т0↑
				WGM1(3,2,1,0) = 0101 - 8b PWM; 0110 - 9b PWM; 0111 - 10b PWM
				COM1A/B(1,0) = 10 - PWM; 11 - negPWM
		*/	
			OUTI	TCCR1A, 1<<COM1A1 | 0<<COM1A0 | 0<<COM1B1 | 0<<COM1B0 | 1<<WGM11 | 1<<WGM10
			OUTI	TCCR1B, 0<<ICNC1 | 0<<ICES1 | 0<<WGM13 | 1<<WGM12 | 0<<CS12 | 0<<CS11 | 1<<CS10

		/*	
		Конфигурация АЦП:
				Вход АЦП: MUX(3,2,1,0) = 0000...0111 - ADC0...ADC7; 1110 - Внутрений ИОН 1,22В; 1111 - GND
				Опора АЦП: REFS(1,0) = 00 - AREF; 01 - AVcc; 11 - Внутрений ИОН 2,56В
				Режим преобразования: ADFR = 0 - один.; 1 - непр.
				Выравнивание: ADLAR = 0 - по правой границе; 1 - по левой границе 
				Предделитель: ADPS(2,1,0) = 000...111 - 2, 2, 4, 8, 16, 32, 64, 128
		*/			
			OUTI	ADMUX, 0<<REFS1 | 0<<REFS0 | 1<<ADLAR | 0<<MUX3 | 0<<MUX2 | 0<<MUX1 | 0<<MUX0
			OUTI	ADCSR, 1<<ADEN | 1<<ADSC | 1<<ADFR | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0

		; Инициализация стека
			OUTI	SPL, LOW(RAMEND)					
			OUTI	SPH, HIGH(RAMEND)
		
		; Отключить компаратор, запрет прерывания компаратора							
			OUTI	ACSR, 1<<ACD | 0<<ACIE				
		
		; Разрешить прерывания	
			SEI											

		/*
		Конфигурация UART
			OUTI	UBRRL, LOW(51)						; Инициализация UART
			OUTI	UBRRH, HIGH(51)						; Скорость 19200
			OUTI	UCSRB, 1<<RXEN|1<<TXEN|1<<RXCIE		; Включить приемник, включить передатчик, разрешить прерывание по приему
		*/

		/*
		Конфигурация SPI
				Предделитель: SPI2X, SPR1, SPR0 = 000...111 - 4, 16, 64, 128, 2, 8, 32, 64
				WCOL - Флаг конфликта записи, устанавливается при записи во время передачи байта
				SPIE: 1 = Прерывания разрешены
				SPIF - флаг прерывания
				SPE: 1 = Вкл. модуль SPI
				DORD: 0 = Little Endian; 1 = Big Endian
				MSTR: 0 = Slave; 1 = Master
				Активный уровень SCK: CPOL = 0 - HIGH; 1 - LOW
				Активный фронт SCK: CPHA = 0 - передний; 1 - задний
		*/
			IN		TEMP, DDRB
			ORI		TEMP, 1<<PB2 | 1<<PB3 | 1<<PB5
			OUT		DDRB, TEMP			; Ножки SPI на выход
			SBI		SPSR, SPI2X
			OUTI	SPCR, 0<<SPIE | 1<<SPE | 0<<DORD | 1<<MSTR | 0<<CPOL | 0<<CPHA | 0<<SPR1 | 1<<SPR0
			
	
		/*			
		Конфигурация Т/С0
				Пределитель: CS0(2,1,0) = 000...111 - останов., 1, 8, 64, 256, 1024, Т0↓, Т0↑
		*/			
			IN		TEMP, TIMSK
			ORI		TEMP, 1<<TOIE0						; Разрешить прерывание по переполнению
			OUT		TIMSK, TEMP
			OUTI	TCCR0, S_CS02<<CS02 | S_CS01<<CS01 | S_CS00<<CS00	; Т/С на 1 МГц
			
;════════════════════════════════════════════════════════════════════════════════════════
;				ОСНОВНАЯ ПРОГРАММА
;════════════════════════════════════════════════════════════════════════════════════════

			OUTI	DDRD, PD2 | PD3		; Тут два светодиода
			SBI		DDRB, PB0			; Тут нога ST индикатора

			LDI		XH, HIGH(IND_DATA)		; Адрес памяти индикатора
			LDI		XL, LOW(IND_DATA)
	; Заполенение памяти индикатора четырьмя символамми
			LDI		ZH, HIGH(seg_table*2)		; Адрес начала таблицы индикатора
	; Первый
			LDI		ZL, LOW(seg_table*2)+$0d	; Плюс смещение для нужного символа
			LPM		R0, Z
			ST		X+,	R0						; Запись символа в память индикатора
	; Второй		
			LDI		ZL, LOW(seg_table*2)+$0c	; Плюс смещение для нужного символа
			LPM		R0, Z
			ST		X+,	R0						; Запись символа в память индикатора
	; Третий
			LDI		ZL, LOW(seg_table*2)+$0d	; Плюс смещение для нужного символа
			LPM		R0, Z
			LDI		TEMP, DP_ON				; Включение десятичной точки в третьем разряде
			AND		R0, TEMP				; Сложение символа с точкой
			ST		X+,	R0						; Запись символа в память индикатора
	; Четвертый
			LDI		ZL, LOW(seg_table*2)+16		; Плюс смещение для нужного символа
			LPM		R0, Z
			ST		X, R0						; Запись символа в память индикатора
			
LOOP:		CBI		PORTD, PD2				; Гашение светодиода прерывания АЦП

			SBRC	USR_FLG, 0
			RCALL	REFR_IND

			RJMP	LOOP
;════════════════════════════════════════════════════════════════════════════════════════
;				ПРОЦЕДУРЫ
;════════════════════════════════════════════════════════════════════════════════════════

REFR_IND:	PUSH	TEMP
			PUSH	ZH
			PUSH	ZL			
			

NOT_T:		CBR		USR_FLG, REFR_IND_FLG	; Сброс флага обновления индикатора
			POP		ZL
			POP		ZH
			POP		TEMP
			RET
			
				
SPI_SEND:	OUT		SPDR, TEMP				; Отправка байта в SPI
	WAIT:	SBIS	SPSR, SPIF
			RJMP	WAIT
			SBI		SPSR, SPIF
			RET

;════════════════════════════════════════════════════════════════════════════════════════
;				ТАБЛИЦЫ
;════════════════════════════════════════════════════════════════════════════════════════

.org (flashend)-10
			;		A
			;	   ---
			;   F | G | B
			;	   ---
			;	E |   | C
			;	   ---  . Dp
			;		D
			;
			; Active Low, for ind. with common anode
			;	 DpGFEDCBA   DpGFEDCBA
seg_table:	.DB ~0b11000000, ~0b11111001		; 0, 1
			.DB ~0b10100100, ~0b10110000 		; 2, 3
			.DB	~0b10011001, ~0b10010010		; 4, 5
			.DB ~0b10000010, ~0b11111000 		; 6, 7
			.DB ~0b10000000, ~0b10010000		; 8, 9
			.DB ~0b10001000, ~0b10000011		; A, b
			.DB ~0b10100111, ~0b10100001		; c, d
			.DB ~0b10000110, ~0b10001110		; E, F
			.DB ~0b10101111, 0		; r

;════════════════════════════════════════════════════════════════════════════════════════
;				ОПЕРАТИВНАЯ ПАМЯТЬ
;════════════════════════════════════════════════════════════════════════════════════════

.DSEG
.ORG SRAM_START
IND_DATA:	.BYTE NUM_OF_DIG		; Байты информации индикатора (сегменты)
IND_DIG:	.BYTE 1		; Разряд индикатора
IND_CYCLE:	.BYTE 1		; Количество итераций обновления индикатора и смещение по таблицам индикатора
SFT_CNTR1:	.BYTE 1		; Программаный счетчик вызова процедуры обновления индикатора

;════════════════════════════════════════════════════════════════════════════════════════
;				EEPROM
;════════════════════════════════════════════════════════════════════════════════════════

.ESEG
.org $0000
ADCMULT:	.DB MULT