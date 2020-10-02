/* ELECTROSOFT ENGINEERING
 * File:   picuno.h
 * Author: hajazi
 * Editor: King Diaw Eh Sut Current Version: 1.11.2
 * Created on 20 Jun 2015
 * 2016-02-24 Modify for PIC18 series
 * Support MCU:
 *   PIC18F4520,PIC18F4550
 *   PIC18F45K50
 *   PIC18F44K22,PIC18F45K22,PIC18F46K22
 *   PIC18F46K80,PIC18F66K80
 */
#include "picuno.h"

//ADD-on DEFINITION START HERE

//ADD-on DEFINITION END HERE

byte TmrRxd1=0,TmrRxd2=0,HMSec=10,OneSec=100,TmrSec=60;

// UART DEFINITION
byte uart1_tx_pos=0,uart1_tx_cnt=0,uart1_cr_cnt=0,uart1_lf_cnt=0;
byte uart1_rx_pos=0,uart1_rx_cnt=0,uart1_rx_char,uart1_rx_ready=0;
byte uart1_txbuf[UART1_TX_MAX+1];
byte uart1_rxbuf[UART1_RX_MAX+1];

#ifdef UART2
byte uart2_tx_pos=0,uart2_tx_cnt=0,uart2_cr_cnt=0,uart2_lf_cnt=0;
byte uart2_rx_pos=0,uart2_rx_cnt=0,uart2_rx_char,uart2_rx_ready=0;
byte uart2_txbuf[UART2_TX_MAX+1];
byte uart2_rxbuf[UART2_RX_MAX+1];
#endif

// SOFTWARE TIMER
#define TIMER_MAX	10
#define TIMER_MST	10	// ms/tick
unsigned char TimerFlag = 0;
unsigned int TimerArray[TIMER_MAX];

//millis()
unsigned long sysTick=0;

// USER MULTI TASK VARIABLE
byte Task1=0,Task2=0,Task3=0,Task4=0;
byte Task5=0,Task6=0,Task7=0,Task8=0;

void (*INT0_InterruptHandler)(void);
void (*INT1_InterruptHandler)(void);
void (*INT2_InterruptHandler)(void);

// INTERRUPT ROUTINE
void interrupt InterruptHandlerHigh(void)
{
	byte i;
	unsigned int dd;
    //Peripheral Interrupt
    //INT0
    if(INTCONbits.INT0IE == 1 && INTCONbits.INT0IF == 1)
    {
        INT0_ISR();
    }
    else if(INTCON3bits.INT1IE == 1 && INTCON3bits.INT1IF == 1)
    {
        INT1_ISR();
    }
    else if(INTCON3bits.INT2IE == 1 && INTCON3bits.INT2IF == 1)
    {
        INT2_ISR();
    }

	//TIMER0 10ms Interrupt
	if (INTCONbits.TMR0IF)			//check TMR0 overflow
	{
#ifdef FOSC64
        TMR0H = 0x63;
        TMR0L = 0xBF;
#else
        TMR0 = 100;					//4,8,16MHz:100, 20MHz:60
#endif
		INTCONbits.TMR0IF = 0;		//clear interrupt flag
		intbit = 1;
        sysTick++;
		//
		if(TmrRxd1)
		{
			TmrRxd1--;
			if(!TmrRxd1)
			{
				uart1_rx_ready=1;
			}
		}
		//
#ifdef UART2
		if(TmrRxd2)
		{
			TmrRxd2--;
			if(!TmrRxd2)
			{
				uart2_rx_ready=1;
			}
		}
#endif
		//
		if(OneSec) OneSec--;
		if(OneSec==50) hfsbit = ~hfsbit;
		if(OneSec==0)
		{
			OneSec=100;
			secbit = ~secbit;
			if(TmrSec) TmrSec--;
		}
		//
		if(HMSec) HMSec--;
		if(!HMSec){HMSec=10; hmsbit = ~hmsbit;}
		//
		if(TimerFlag)	// to save processing time
		{
			TimerFlag = 0;
			for(i=0;i<TIMER_MAX;i++)
			{
				dd=TimerArray[i];
				if(dd) { dd--; TimerFlag=1; }
				TimerArray[i]=dd;
			}
		}
	}
	//UART Receive
	if(RCIF){			// no need to clear RCIF flag
		uart1_rx_char = RCREG1;	// MCU will clear it once read the data
		//if('a'<=uart1_rx_char && uart1_rx_char<='z') uart1_rx_char-=0x20;	// UpperCase
		if(uart1_rx_cnt < UART1_RX_MAX)
		{
			uart1_rxbuf[uart1_rx_cnt++] = uart1_rx_char;
		}
		uart1_rxbuf[uart1_rx_cnt]=0x00;
		if(uart1_rx_char==0x0D) uart1_cr_cnt++;
		if(uart1_rx_char==0x0A) uart1_lf_cnt++;
		TmrRxd1=10;
	}
	//UART Transmit
	if(TXIE && TXIF)
	{
		TXREG = uart1_txbuf[uart1_tx_pos++];
		if(uart1_tx_pos == UART1_TX_MAX) uart1_tx_pos = 0;
		uart1_tx_cnt--;
		if(!uart1_tx_cnt)
		{
			uart1_tx_pos = 0;
			TXIE = 0;
		}
	}

#ifdef UART2
	//UART2 Receive
	if(RC2IF){			// no need to clear RCIF flag
        if(RCSTA2bits.OERR2){
            RCSTA2bits.CREN2 = 0;
            RCSTA2bits.CREN2 = 1;
        }
		uart2_rx_char = RCREG2;	// MCU will clear it once read the data
		//if('a'<=uart2_rx_char && uart2_rx_char<='z') uart2_rx_char-=0x20;	// UpperCase
		if(uart2_rx_cnt < UART2_RX_MAX)
		{
			uart2_rxbuf[uart2_rx_cnt++] = uart2_rx_char;
		}
		uart2_rxbuf[uart2_rx_cnt]=0x00;
		if(uart2_rx_char==0x0D) uart2_cr_cnt++;
		if(uart2_rx_char==0x0A) uart2_lf_cnt++;
		TmrRxd2=10;
	}
	//UART2 Transmit
	if(TX2IE && TX2IF)
	{
		TXREG2 = uart2_txbuf[uart2_tx_pos++];
		if(uart2_tx_pos == UART2_TX_MAX) uart2_tx_pos = 0;
		uart2_tx_cnt--;
		if(!uart2_tx_cnt)
		{
			uart2_tx_pos = 0;
			TX2IE = 0;
		}
	}
#endif
}
//
//----------------------------------------------------------------------------
void init(void)
{
	byte i;
#if defined(_18F45K22)
    // SCS FOSC; IRCF 16MHz_HFINTOSC; IDLEN disabled; 
    OSCCON = 0x70;
    // PRISD enabled; SOSCGO disabled; MFIOSEL disabled; 
    OSCCON2 = 0x04;
    // INTSRC disabled; PLLEN enabled; TUN 0; 
    OSCTUNE = 0x40;
    // Wait for PLL to stabilize
    while(PLLRDY == 0)
    {
    }
#else
	OSCCON = OSCCON_VAL;			// setting internal oscillator
	i=255; while(i) i--;			// stablelize mcu
#endif
// CONFIGURE ALL PIN AS DIGITAL (DISABLE ANALOG)
// 18F4520, 18F4550
#ifdef ADC_ADCON
	ADCON1 = 0b0001111;
#endif
// 18F44K22, 18F45K22, 18F45K50, 18F46K22
#ifdef ADC_ANSEL
	ANSELA = 0b00000000;
	ANSELB = 0b00000000;
	ANSELC = 0b00000000;
	ANSELD = 0b00000000;
	ANSELE = 0b00000000;
	VREFCON0 = 0b10110000;	// Fixed Voltage Reference is enabled, x4 (4.096V)
	ADCON1 = 0b00110000;	//0b00110000=4.096V 0b00100000=2.048V 0b00000000=Vdd
	ADCON2 = 0b10000011;
#endif

// 18F46K80, 18F66K80
#ifdef ADC_ANCON
	ANCON0 = 0b00000000;
	ANCON1 = 0b00000000;
	ADCON1 = 0b00110000;	//0b00110000=4.096V 0b00100000=2.048V 0b00000000=Vdd
	ADCON2 = 0b10000011;
#endif

	LATA = 0x00;
	LATB = 0x00;
	LATC = 0x00;
	LATD = 0x00;
	LATE = 0x00;

	// Initialize Software Timer
	for(i=0;i<TIMER_MAX;i++) TimerArray[i]=0;

	//Initialize Timer0 for 16.384ms overflow
	T0CON = 0b00000000;		//
	TMR0IF = 0;
	TMR0IE = 1;				// Enable timer0 interrupt
	TMR0IP = 1;				// Make timer0 interrupt high priority
    
    //Initialize Timer1 for
    // Clearing IF flag.
    PIR1bits.TMR1IF = 0;
    //T1GSS T1G_pin; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO done; T1GSPM disabled; 
    T1GCON = 0x00;
    // T1CKPS 1:8; T1OSCEN disabled; T1SYNC synchronize; TMR1CS FOSC/4; TMR1ON disabled; T1RD16 enabled; 
    T1CON = 0x32; 
    
#ifdef FOSC4
	T0CON = 0b11000101;		// 4MHz 8bit,T0PS=1:64
#endif
#ifdef FOSC8
	T0CON = 0b11000110;		// 8MHz 8bit,T0PS=1:128
#endif
#ifdef FOSC16
	T0CON = 0b11000111;		// 16MHz 8bit,T0PS=1:256
#endif
#ifdef FOSC20
	T0CON = 0b11000111;		// 20MHz 8bit,T0PS=1:256
#endif   
#ifdef FOSC64
    //T0PS 1:4;T08BIT 16-bit;T0SE Increment hi-lo;T0SC FOSC/4; TMR0ON enable
    T0CON = 0b10010001;
#endif
	//Interrupt Setting
	PEIE = 1;			// enable perhipheral interrupts

	RCONbits.IPEN = 1;		//Enable interrupt priority
	INTCONbits.GIEH = 1;	//enable interrupts

}

// ==================================
// SYNCHRONOUS SERVICE
// ==================================
void sync()
{
	//
	syncbit = 0;
	if(intbit)
	{
		intbit = 0;
		syncbit = 1;	// 10ms sync pulse
	}
}

// ==================================
// CLOSE LOOP DELAY
// ==================================
//__delay_ms(x)
void delay(unsigned int ms)
{
	unsigned int i;
	
	ClrWdt();
	// never let exceed 65sec or change WDT prescaler above 16384
	while(ms){
		ms--;
		i=MSLOOP; //1ms delay counter
		while(i) i--;
	}
}

//millis()
//===================================
//return value in milisecond (resolution 10ms)
//===================================
unsigned long millis(void)
{
    return sysTick*10;
}

//pulseIn(byte pin, byte state, unsigned long timeout)
//============================================================
//return unsigned long
//pin -> PIN_RA0...RE3
//state -> type of pulse to read: either HIGH or LOW
//timeOut(option) -> the number of microseconds to wait for the pulse to start; 
//          default is one second. Allowed data types: unsigned long.
//============================================================
unsigned long pulseIn(byte pin, byte state,...)
{
   unsigned long timeOut=0,runTime=0;
   unsigned int timeHigh=0,timeLow=0;
   unsigned char lowByte,highByte;
   
   //Make sure pin is digital INPUT
   pinMode(pin,INPUT);
   
   va_list ap;
   va_start(ap, state);
   timeOut = va_arg(ap,unsigned long);
   va_end(ap);
   
   timeLow = (timeOut) & (0x0000FFFF);
   timeHigh = (timeOut>>16)&(0x0000FFFF);
   if((timeLow < 500)&&(timeHigh == 0))timeOut=1000000;
   runTime = millis();
 
   //Clear TMR1
   TMR1H=0;
   TMR1L=0;
   timeHigh=0;   
   
   if(state == HIGH){
       //Wait Until RISING Pulse Coming
       while((digitalRead(pin)==LOW) && (((millis() - runTime)*1000) < timeOut));
       //Start TMR1 to count time
       T1CONbits.TMR1ON = 1;    //Start
       while(digitalRead(pin)==HIGH){
           if(PIR1bits.TMR1IF == 1){
               PIR1bits.TMR1IF = 0;
               timeHigh++;
           }
           if((((millis() - runTime)*1000) > timeOut)) return 0;
       }
       T1CONbits.TMR1ON = 0;    //Stop
       lowByte = TMR1L;
       highByte = TMR1H;
       timeLow = ((uint16_t)highByte<<8)|lowByte;
   }
   else if(state == LOW){
       //Wait Until FALLING Pulse Coming
       while((digitalRead(pin)==HIGH) && (((millis() - runTime)*1000) < timeOut));
       //Start TMR1 to count time
       T1CONbits.TMR1ON = 1;    //Start
       while(digitalRead(pin)==LOW){
           if(PIR1bits.TMR1IF == 1){
               PIR1bits.TMR1IF = 0;
               timeHigh++;
           }
           if((((millis() - runTime)*1000) > timeOut)) return 0;           
       }
       T1CONbits.TMR1ON = 0;    //Stop
       lowByte = TMR1L;
       highByte = TMR1H;
       timeLow = ((uint16_t)highByte<<8)|lowByte;       
   }
   return ((((unsigned long)timeHigh<<16)+ timeLow)/2);
}
// ==================================
// SOFTWARE TIMER
// Resolution : 10ms
// ==================================
//
void timerClr(byte index)
{
	if(index>=TIMER_MAX) return;
	TimerArray[index]=0;
}	

void timerSet(byte index, unsigned long value)
{
	if(index>=TIMER_MAX) return;
	value /= TIMER_MST;
	TimerArray[index]=(unsigned int)value;
	TimerFlag=1;
}	

byte timerUp(byte index)
{
	if(index>=TIMER_MAX) return 0;
	if(TimerArray[index]) return 0;
	return 1;
}

byte timerBusy(byte index)
{
	if(index>=TIMER_MAX) return 0;
	if(TimerArray[index]) return 1;
	return 0;
}
// ==================================

void task(void)
{
#ifdef _TASK1
	Tasking1();
#endif
#ifdef _TASK2
	Tasking2();
#endif
#ifdef _TASK3
	Tasking3();
#endif
#ifdef _TASK4
	Tasking4();
#endif
#ifdef _TASK5
	Tasking5();
#endif
#ifdef _TASK6
	Tasking7();
#endif
#ifdef _TASK8
	Tasking8();
#endif
}

// ==================================
// MAIN FUNCTION
// ==================================
void main()
{
	init();
	setup();		// user function
	while (1)
	{
		ClrWdt();
		sync();
		loop();		// user function
		task();
	}
}
// ==================================


// =========================
// ANALOG INPUT CONFIGURATION
// Support AN0 to AN7 only
// =========================
void analogInput(byte ANx)
{
	byte d;
	
#ifdef ADC_ADCON
	d = ~(ANx+1);
	d &= 0x0F;
	ADCON1 = d;
#endif

#ifdef ADC_ANSEL
	d = (1 << ANx);
	ANSELA |= d;
#endif

#ifdef ADC_ANCON
	d = (1 << ANx);
	ANCON0 |= d;
#endif
}

// =========================
// PIN MODE CONFIGURATION
// =========================
// OUTPUT 0x0
// INPUT 0x1
// INPUT_PULLUP 0x2
// INPUT_ANALOG 0x3
void pinMode(byte pin, byte mode)
{
	switch(pin)
	{
		case PIN_RA0:
			if(mode==OUTPUT)
			{
				DIR_RA0 = 0;
				OUT_RA0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA0 = 1;
				OUT_RA0 = 0;
			}
			else if(mode==INPUT_ANALOG)
			{
				DIR_RA0 = 1;
				analogInput(AN0);
			}	
			else
			{
				DIR_RA0 = 1;
				OUT_RA0 = 0;
			}
		break;
		case PIN_RA1:
			if(mode==OUTPUT)
			{
				DIR_RA1 = 0;
				OUT_RA1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA1 = 1;
				OUT_RA1 = 0;
			}
			else if(mode==INPUT_ANALOG)
			{
				DIR_RA1 = 1;
				analogInput(AN1);
			}	
			else
			{
				DIR_RA1 = 1;
				OUT_RA1 = 0;
			}
		break;
		case PIN_RA2:
			if(mode==OUTPUT)
			{
				DIR_RA2 = 0;
				OUT_RA2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA2 = 1;
				OUT_RA2 = 0;
			}
			else if(mode==INPUT_ANALOG)
			{
				DIR_RA2 = 1;
				analogInput(AN2);
			}	
			else
			{
				DIR_RA2 = 1;
				OUT_RA2 = 0;
			}
		break;
		case PIN_RA3:
			if(mode==OUTPUT)
			{
				DIR_RA3 = 0;
				OUT_RA3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA3 = 1;
				OUT_RA3 = 0;
			}
			else if(mode==INPUT_ANALOG)
			{
				DIR_RA3 = 1;
				analogInput(AN3);
			}	
			else
			{
				DIR_RA3 = 1;
				OUT_RA3 = 0;
			}
		break;
		
#ifndef _18F46K80
#ifndef _18F66K80
		case PIN_RA4:
			if(mode==OUTPUT)
			{
				DIR_RA4 = 0;
				OUT_RA4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA4 = 1;
				OUT_RA4 = 0;
			}
			else
			{
				DIR_RA4 = 1;
				OUT_RA4 = 0;
			}
		break;
#endif
#endif
		case PIN_RA5:
			if(mode==OUTPUT)
			{
				DIR_RA5 = 0;
				OUT_RA5 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA5 = 1;
				OUT_RA5 = 0;
			}
			else if(mode==INPUT_ANALOG)
			{
				DIR_RA5 = 1;
				analogInput(AN4);
			}	
			else
			{
				DIR_RA5 = 1;
				OUT_RA5 = 0;
			}
		break;
		case PIN_RA6:
			if(mode==OUTPUT)
			{
				DIR_RA6 = 0;
				OUT_RA6 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA6 = 1;
				OUT_RA6 = 0;
			}
			else
			{
				DIR_RA6 = 1;
				OUT_RA6 = 0;
			}
		break;
		
#ifndef _18F4550
		case PIN_RA7:
			if(mode==OUTPUT)
			{
				DIR_RA7 = 0;
				OUT_RA7 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RA7 = 1;
				OUT_RA7 = 0;
			}
			else
			{
				DIR_RA7 = 1;
				OUT_RA7 = 0;
			}
		break;
#endif
		case PIN_RB0:
			if(mode==OUTPUT)
			{
				DIR_RB0 = 0;
				OUT_RB0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB0 = 1;
				OUT_RB0 = 0;
			}
			else
			{
				DIR_RB0 = 1;
				OUT_RB0 = 0;
			}
		break;
		case PIN_RB1:
			if(mode==OUTPUT)
			{
				DIR_RB1 = 0;
				OUT_RB1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB1 = 1;
				OUT_RB1 = 0;
			}
			else
			{
				DIR_RB1 = 1;
				OUT_RB1 = 0;
			}
		break;
		case PIN_RB2:
			if(mode==OUTPUT)
			{
				DIR_RB2 = 0;
				OUT_RB2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB2 = 1;
				OUT_RB2 = 0;
			}
			else
			{
				DIR_RB2 = 1;
				OUT_RB2 = 0;
			}
		break;
		case PIN_RB3:
			if(mode==OUTPUT)
			{
				DIR_RB3 = 0;
				OUT_RB3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB3 = 1;
				OUT_RB3 = 0;
			}
			else
			{
				DIR_RB3 = 1;
				OUT_RB3 = 0;
			}
		break;
		case PIN_RB4:
			if(mode==OUTPUT)
			{
				DIR_RB4 = 0;
				OUT_RB4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB4 = 1;
				OUT_RB4 = 0;
			}
			else
			{
				DIR_RB4 = 1;
				OUT_RB4 = 0;
			}
		break;
		case PIN_RB5:
			if(mode==OUTPUT)
			{
				DIR_RB5 = 0;
				OUT_RB5 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB5 = 1;
				OUT_RB5 = 0;
			}
			else
			{
				DIR_RB5 = 1;
				OUT_RB5 = 0;
			}
		break;
		case PIN_RB6:
			if(mode==OUTPUT)
			{
				DIR_RB6 = 0;
				OUT_RB6 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB6 = 1;
				OUT_RB6 = 0;
			}
			else
			{
				DIR_RB6 = 1;
				OUT_RB6 = 0;
			}
		break;
		case PIN_RB7:
			if(mode==OUTPUT)
			{
				DIR_RB7 = 0;
				OUT_RB7 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RB7 = 1;
				OUT_RB7 = 0;
			}
			else
			{
				DIR_RB7 = 1;
				OUT_RB7 = 0;
			}
		break;
		case PIN_RC0:
			if(mode==OUTPUT)
			{
				DIR_RC0 = 0;
				OUT_RC0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC0 = 1;
				OUT_RC0 = 0;
			}
			else
			{
				DIR_RC0 = 1;
				OUT_RC0 = 0;
			}
		break;
		case PIN_RC1:
			if(mode==OUTPUT)
			{
				DIR_RC1 = 0;
				OUT_RC1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC1 = 1;
				OUT_RC1 = 0;
			}
			else
			{
				DIR_RC1 = 1;
				OUT_RC1 = 0;
			}
		break;
		case PIN_RC2:
			if(mode==OUTPUT)
			{
				DIR_RC2 = 0;
				OUT_RC2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC2 = 1;
				OUT_RC2 = 0;
			}
			else
			{
				DIR_RC2 = 1;
				OUT_RC2 = 0;
			}
		break;
		
#ifndef _18F4550
#ifndef _18F45K50
		case PIN_RC3:
			if(mode==OUTPUT)
			{
				DIR_RC3 = 0;
				OUT_RC3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC3 = 1;
				OUT_RC3 = 0;
			}
			else
			{
				DIR_RC3 = 1;
				OUT_RC3 = 0;
			}
		break;
		case PIN_RC4:
			if(mode==OUTPUT)
			{
				DIR_RC4 = 0;
				OUT_RC4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC4 = 1;
				OUT_RC4 = 0;
			}
			else
			{
				DIR_RC4 = 1;
				OUT_RC4 = 0;
			}
		break;
		case PIN_RC5:
			if(mode==OUTPUT)
			{
				DIR_RC5 = 0;
				OUT_RC5 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC5 = 1;
				OUT_RC5 = 0;
			}
			else
			{
				DIR_RC5 = 1;
				OUT_RC5 = 0;
			}
		break;
#endif
#endif
		case PIN_RC6:
			if(mode==OUTPUT)
			{
				DIR_RC6 = 0;
				OUT_RC6 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC6 = 1;
				OUT_RC6 = 0;
			}
			else
			{
				DIR_RC6 = 1;
				OUT_RC6 = 0;
			}
		break;
		case PIN_RC7:
			if(mode==OUTPUT)
			{
				DIR_RC7 = 0;
				OUT_RC7 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RC7 = 1;
				OUT_RC7 = 0;
			}
			else
			{
				DIR_RC7 = 1;
				OUT_RC7 = 0;
			}
		break;
		case PIN_RD0:
			if(mode==OUTPUT)
			{
				DIR_RD0 = 0;
				OUT_RD0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD0 = 1;
				OUT_RD0 = 0;
			}
			else
			{
				DIR_RD0 = 1;
				OUT_RD0 = 0;
			}
		break;
		case PIN_RD1:
			if(mode==OUTPUT)
			{
				DIR_RD1 = 0;
				OUT_RD1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD1 = 1;
				OUT_RD1 = 0;
			}
			else
			{
				DIR_RD1 = 1;
				OUT_RD1 = 0;
			}
		break;
		case PIN_RD2:
			if(mode==OUTPUT)
			{
				DIR_RD2 = 0;
				OUT_RD2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD2 = 1;
				OUT_RD2 = 0;
			}
			else
			{
				DIR_RD2 = 1;
				OUT_RD2 = 0;
			}
		break;
		case PIN_RD3:
			if(mode==OUTPUT)
			{
				DIR_RD3 = 0;
				OUT_RD3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD3 = 1;
				OUT_RD3 = 0;
			}
			else
			{
				DIR_RD3 = 1;
				OUT_RD3 = 0;
			}
		break;
		case PIN_RD4:
			if(mode==OUTPUT)
			{
				DIR_RD4 = 0;
				OUT_RD4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD4 = 1;
				OUT_RD4 = 0;
			}
			else
			{
				DIR_RD4 = 1;
				OUT_RD4 = 0;
			}
		break;
		case PIN_RD5:
			if(mode==OUTPUT)
			{
				DIR_RD5 = 0;
				OUT_RD5 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD5 = 1;
				OUT_RD5 = 0;
			}
			else
			{
				DIR_RD5 = 1;
				OUT_RD5 = 0;
			}
		break;
		case PIN_RD6:
			if(mode==OUTPUT)
			{
				DIR_RD6 = 0;
				OUT_RD6 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD6 = 1;
				OUT_RD6 = 0;
			}
			else
			{
				DIR_RD6 = 1;
				OUT_RD6 = 0;
			}
		break;
		case PIN_RD7:
			if(mode==OUTPUT)
			{
				DIR_RD7 = 0;
				OUT_RD7 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RD7 = 1;
				OUT_RD7 = 0;
			}
			else
			{
				DIR_RD7 = 1;
				OUT_RD7 = 0;
			}
		break;
		case PIN_RE0:
			if(mode==OUTPUT)
			{
				DIR_RE0 = 0;
				OUT_RE0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE0 = 1;
				OUT_RE0 = 0;
			}
			else
			{
				DIR_RE0 = 1;
				OUT_RE0 = 0;
			}
		break;
		case PIN_RE1:
			if(mode==OUTPUT)
			{
				DIR_RE1 = 0;
				OUT_RE1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE1 = 1;
				OUT_RE1 = 0;
			}
			else
			{
				DIR_RE1 = 1;
				OUT_RE1 = 0;
			}
		break;
		case PIN_RE2:
			if(mode==OUTPUT)
			{
				DIR_RE2 = 0;
				OUT_RE2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE2 = 1;
				OUT_RE2 = 0;
			}
			else
			{
				DIR_RE2 = 1;
				OUT_RE2 = 0;
			}
		break;
		
#ifdef _18F66K80
		case PIN_RE3:
/*			if(mode==OUTPUT)
			{
				DIR_RE3 = 0;
				OUT_RE3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE3 = 1;
				OUT_RE3 = 0;
			}
			else
			{
				DIR_RE3 = 1;
				OUT_RE3 = 0;
			}
*/
		break;
		case PIN_RE4:
			if(mode==OUTPUT)
			{
				DIR_RE4 = 0;
				OUT_RE4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE4 = 1;
				OUT_RE4 = 0;
			}
			else
			{
				DIR_RE4 = 1;
				OUT_RE4 = 0;
			}
		break;
		case PIN_RE5:
			if(mode==OUTPUT)
			{
				DIR_RE5 = 0;
				OUT_RE5 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE5 = 1;
				OUT_RE5 = 0;
			}
			else
			{
				DIR_RE5 = 1;
				OUT_RE5 = 0;
			}
		break;
		case PIN_RE6:
			if(mode==OUTPUT)
			{
				DIR_RE6 = 0;
				OUT_RE6 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE6 = 1;
				OUT_RE6 = 0;
			}
			else
			{
				DIR_RE6 = 1;
				OUT_RE6 = 0;
			}
		break;
		case PIN_RE7:
			if(mode==OUTPUT)
			{
				DIR_RE7 = 0;
				OUT_RE7 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RE7 = 1;
				OUT_RE7 = 0;
			}
			else
			{
				DIR_RE7 = 1;
				OUT_RE7 = 0;
			}
		break;
		case PIN_RF0:
			if(mode==OUTPUT)
			{
				DIR_RF0 = 0;
				OUT_RF0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF0 = 1;
				OUT_RF0 = 0;
			}
			else
			{
				DIR_RF0 = 1;
				OUT_RF0 = 0;
			}
		break;
		case PIN_RF1:
			if(mode==OUTPUT)
			{
				DIR_RF1 = 0;
				OUT_RF1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF1 = 1;
				OUT_RF1 = 0;
			}
			else
			{
				DIR_RF1 = 1;
				OUT_RF1 = 0;
			}
		break;
		case PIN_RF2:
			if(mode==OUTPUT)
			{
				DIR_RF2 = 0;
				OUT_RF2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF2 = 1;
				OUT_RF2 = 0;
			}
			else
			{
				DIR_RF2 = 1;
				OUT_RF2 = 0;
			}
		break;
		case PIN_RF3:
			if(mode==OUTPUT)
			{
				DIR_RF3 = 0;
				OUT_RF3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF3 = 1;
				OUT_RF3 = 0;
			}
			else
			{
				DIR_RF3 = 1;
				OUT_RF3 = 0;
			}
		break;
		case PIN_RF4:
			if(mode==OUTPUT)
			{
				DIR_RF4 = 0;
				OUT_RF4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF4 = 1;
				OUT_RF4 = 0;
			}
			else
			{
				DIR_RF4 = 1;
				OUT_RF4 = 0;
			}
		break;
		case PIN_RF5:
			if(mode==OUTPUT)
			{
				DIR_RF5 = 0;
				OUT_RF5 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF5 = 1;
				OUT_RF5 = 0;
			}
			else
			{
				DIR_RF5 = 1;
				OUT_RF5 = 0;
			}
		break;
		case PIN_RF6:
			if(mode==OUTPUT)
			{
				DIR_RF6 = 0;
				OUT_RF6 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF6 = 1;
				OUT_RF6 = 0;
			}
			else
			{
				DIR_RF6 = 1;
				OUT_RF6 = 0;
			}
		break;
		case PIN_RF7:
			if(mode==OUTPUT)
			{
				DIR_RF7 = 0;
				OUT_RF7 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RF7 = 1;
				OUT_RF7 = 0;
			}
			else
			{
				DIR_RF7 = 1;
				OUT_RF7 = 0;
			}
		break;
		case PIN_RG0:
			if(mode==OUTPUT)
			{
				DIR_RG0 = 0;
				OUT_RG0 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RG0 = 1;
				OUT_RG0 = 0;
			}
			else
			{
				DIR_RG0 = 1;
				OUT_RG0 = 0;
			}
		break;
		case PIN_RG1:
			if(mode==OUTPUT)
			{
				DIR_RG1 = 0;
				OUT_RG1 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RG1 = 1;
				OUT_RG1 = 0;
			}
			else
			{
				DIR_RG1 = 1;
				OUT_RG1 = 0;
			}
		break;
		case PIN_RG2:
			if(mode==OUTPUT)
			{
				DIR_RG2 = 0;
				OUT_RG2 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RG2 = 1;
				OUT_RG2 = 0;
			}
			else
			{
				DIR_RG2 = 1;
				OUT_RG2 = 0;
			}
		break;
		case PIN_RG3:
			if(mode==OUTPUT)
			{
				DIR_RG3 = 0;
				OUT_RG3 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RG3 = 1;
				OUT_RG3 = 0;
			}
			else
			{
				DIR_RG3 = 1;
				OUT_RG3 = 0;
			}
		break;
		case PIN_RG4:
			if(mode==OUTPUT)
			{
				DIR_RG4 = 0;
				OUT_RG4 = 0;
			}
			else if(mode==INPUT_PULLUP)
			{
				DIR_RG4 = 1;
				OUT_RG4 = 0;
			}
			else
			{
				DIR_RG4 = 1;
				OUT_RG4 = 0;
			}
		break;
#endif
	}
}


// =========================
// READ DIGITAL INPUT
// =========================
char digitalRead(byte pin)
{
	char di=0;
	
	switch(pin)
	{
		//
		case PIN_RA0: di = INP_RA0; break;
		case PIN_RA1: di = INP_RA1; break;
		case PIN_RA2: di = INP_RA2; break;
		case PIN_RA3: di = INP_RA3; break;
		case PIN_RA4: di = INP_RA4; break;
		case PIN_RA5: di = INP_RA5; break;
		case PIN_RA6: di = INP_RA6; break;
#ifndef _18F4550
		case PIN_RA7: di = INP_RA7; break;
#endif
		//
		case PIN_RB0: di = INP_RB0; break;
		case PIN_RB1: di = INP_RB1; break;
		case PIN_RB2: di = INP_RB2; break;
		case PIN_RB3: di = INP_RB3; break;
		case PIN_RB4: di = INP_RB4; break;
		case PIN_RB5: di = INP_RB5; break;
		case PIN_RB6: di = INP_RB6; break;
		case PIN_RB7: di = INP_RB7; break;
		//
		case PIN_RC0: di = INP_RC0; break;
		case PIN_RC1: di = INP_RC1; break;
		case PIN_RC2: di = INP_RC2; break;

#ifndef _18F4550
		case PIN_RC3: di = INP_RC3; break;
#endif
		case PIN_RC4: di = INP_RC4; break;
		case PIN_RC5: di = INP_RC5; break;
		case PIN_RC6: di = INP_RC6; break;
		case PIN_RC7: di = INP_RC7; break;
		//
		case PIN_RD0: di = INP_RD0; break;
		case PIN_RD1: di = INP_RD1; break;
		case PIN_RD2: di = INP_RD2; break;
		case PIN_RD3: di = INP_RD3; break;
		case PIN_RD4: di = INP_RD4; break;
		case PIN_RD5: di = INP_RD5; break;
		case PIN_RD6: di = INP_RD6; break;
		case PIN_RD7: di = INP_RD7; break;
		//
		case PIN_RE0: di = INP_RE0; break;
		case PIN_RE1: di = INP_RE1; break;
		case PIN_RE2: di = INP_RE2; break;
		//
	}
	return di;
}

// =========================
// WRITE DIGITAL OUTPUT
// =========================
void digitalWrite(byte pin, byte value)
{
	switch(pin)
	{
		//
		case PIN_RA0: OUT_RA0 = value; break;
		case PIN_RA1: OUT_RA1 = value; break;
		case PIN_RA2: OUT_RA2 = value; break;
		case PIN_RA3: OUT_RA3 = value; break;
#ifndef _18F46K80
#ifndef _18F66K80
		case PIN_RA4: OUT_RA4 = value; break;
#endif
#endif
		case PIN_RA5: OUT_RA5 = value; break;
		case PIN_RA6: OUT_RA6 = value; break;
#ifndef _18F4550
		case PIN_RA7: OUT_RA7 = value; break;
#endif
		//
		case PIN_RB0: OUT_RB0 = value; break;
		case PIN_RB1: OUT_RB1 = value; break;
		case PIN_RB2: OUT_RB2 = value; break;
		case PIN_RB3: OUT_RB3 = value; break;
		case PIN_RB4: OUT_RB4 = value; break;
		case PIN_RB5: OUT_RB5 = value; break;
		case PIN_RB6: OUT_RB6 = value; break;
		case PIN_RB7: OUT_RB7 = value; break;
		//
		case PIN_RC0: OUT_RC0 = value; break;
		case PIN_RC1: OUT_RC1 = value; break;
		case PIN_RC2: OUT_RC2 = value; break;
#ifndef _18F4550
#ifndef _18F45K50
		case PIN_RC3: OUT_RC3 = value; break;
		case PIN_RC4: OUT_RC4 = value; break;
		case PIN_RC5: OUT_RC5 = value; break;
#endif
#endif
		case PIN_RC6: OUT_RC6 = value; break;
		case PIN_RC7: OUT_RC7 = value; break;
		//
		case PIN_RD0: OUT_RD0 = value; break;
		case PIN_RD1: OUT_RD1 = value; break;
		case PIN_RD2: OUT_RD2 = value; break;
		case PIN_RD3: OUT_RD3 = value; break;
		case PIN_RD4: OUT_RD4 = value; break;
		case PIN_RD5: OUT_RD5 = value; break;
		case PIN_RD6: OUT_RD6 = value; break;
		case PIN_RD7: OUT_RD7 = value; break;
		//
		case PIN_RE0: OUT_RE0 = value; break;
		case PIN_RE1: OUT_RE1 = value; break;
		case PIN_RE2: OUT_RE2 = value; break;
		//
	}
}

// =========================
// ANALOG DIGITAL CONVERTER
// =========================
unsigned int analogRead(byte chn)
{
	unsigned int adc;
#ifdef ROBOTIC_LAB
    chn = 24;
    ANSELD |= (1 << RV1);
#endif
	if(chn>PIN_RA7 && chn != 24) return 0;
	ADCON0 = (chn << 2) | 0x01;
	Nop(); Nop(); Nop(); Nop();
	Nop(); Nop(); Nop(); Nop();
	Nop(); Nop(); Nop(); Nop();
	Nop(); Nop(); Nop(); Nop();
	ADCON0bits.GO = 1;
	while(ADCON0bits.GO);
	adc = (ADRESH & 0b1111);
	adc <<= 8;
	adc |= ADRESL;
	return adc;
}

// ==================================
// EEPROM
// ==================================
//
unsigned int eeprom_readw(unsigned char address)	//8bit address
{
	unsigned int dd;
	
	dd = eeprom_read(address+1);
	dd <<= 8;
	dd |= eeprom_read(address);
	return dd;
}

void eeprom_writew(unsigned char address, unsigned int data)
{
	eeprom_write(address++,(unsigned char)(data));
	eeprom_write(address,(unsigned char)(data>>8));
}

// =========================
// KEYPAD
// =========================
byte keypad_mode = 0;
byte keypad_col1,keypad_col2,keypad_col3,keypad_col4;
byte keypad_row1,keypad_row2,keypad_row3,keypad_row4;

void keypad_begin34(byte col1, byte col2, byte col3, byte row1, byte row2, byte row3, byte row4)
{
  keypad_mode = 0;
  keypad_col1 = col1;
  keypad_col2 = col2;
  keypad_col3 = col3;
  keypad_row1 = row1;
  keypad_row2 = row2;
  keypad_row3 = row3;
  keypad_row4 = row4;
  //
	pinMode(keypad_col1,INPUT_PULLUP);
	pinMode(keypad_col2,INPUT_PULLUP);
	pinMode(keypad_col3,INPUT_PULLUP);
	//
  pinMode(keypad_row1,OUTPUT);
	pinMode(keypad_row2,OUTPUT);
	pinMode(keypad_row3,OUTPUT);
	pinMode(keypad_row4,OUTPUT);
  //
  digitalWrite(keypad_row1,HIGH);
  digitalWrite(keypad_row2,HIGH);
  digitalWrite(keypad_row3,HIGH);
  digitalWrite(keypad_row4,HIGH);
}

void keypad_begin44(byte col1, byte col2, byte col3, byte col4, byte row1, byte row2, byte row3, byte row4)
{
  keypad_mode = 1;
  keypad_col1 = col1;
  keypad_col2 = col2;
  keypad_col3 = col3;
  keypad_col4 = col4;
  keypad_row1 = row1;
  keypad_row2 = row2;
  keypad_row3 = row3;
  keypad_row4 = row4;
  //
	pinMode(keypad_col1,INPUT_PULLUP);
	pinMode(keypad_col2,INPUT_PULLUP);
	pinMode(keypad_col3,INPUT_PULLUP);
	pinMode(keypad_col4,INPUT_PULLUP);
  //
	pinMode(keypad_row1,OUTPUT);
	pinMode(keypad_row2,OUTPUT);
	pinMode(keypad_row3,OUTPUT);
	pinMode(keypad_row4,OUTPUT);
  //
  digitalWrite(keypad_row1,HIGH);
  digitalWrite(keypad_row2,HIGH);
  digitalWrite(keypad_row3,HIGH);
  digitalWrite(keypad_row4,HIGH);
}

//
byte keypadRead()
{
	byte kp;
    static uint8_t keyOld,keyNew,stateOld,stateNew;
	
    kp=0;
  // scan row1
	digitalWrite(keypad_row1,LOW);
	if(!digitalRead(keypad_col1)) kp='1';
	if(!digitalRead(keypad_col2)) kp='2';
	if(!digitalRead(keypad_col3)) kp='3';
	if(!digitalRead(keypad_col4) && keypad_mode==1) kp='A';
	digitalWrite(keypad_row1,HIGH);
  // scan row2
	digitalWrite(keypad_row2,LOW);
	if(!digitalRead(keypad_col1)) kp='4';
	if(!digitalRead(keypad_col2)) kp='5';
	if(!digitalRead(keypad_col3)) kp='6';
	if(!digitalRead(keypad_col4) && keypad_mode==1) kp='B';
	digitalWrite(keypad_row2,HIGH);
  // scan row3
	digitalWrite(keypad_row3,LOW);
	if(!digitalRead(keypad_col1)) kp='7';
	if(!digitalRead(keypad_col2)) kp='8';
	if(!digitalRead(keypad_col3)) kp='9';
	if(!digitalRead(keypad_col4) && keypad_mode==1) kp='C';
	digitalWrite(keypad_row3,HIGH);
  // scan row4
	digitalWrite(keypad_row4,LOW);
	if(!digitalRead(keypad_col1)) kp='*';
	if(!digitalRead(keypad_col2)) kp='0';
	if(!digitalRead(keypad_col3)) kp='#';
	if(!digitalRead(keypad_col4) && keypad_mode==1) kp='D'; // kp=0x0D;
	digitalWrite(keypad_row4,HIGH);
    
    if(!kp){kstate.s.idle= 1;kstate.s.press = 0; kstate.s.hold = 0;
    kstate.s.stateChange = 0;
    keyOld = kp;}
    else {kstate.s.idle = 0; kstate.s.press = 1;}
    if(kstate.s.press && !kstate.s.hold){
        keyNew = kp;
        if(keyOld != keyNew){
            keyOld = keyNew;
            kstate.s.hold = 1;
            return kp;
        }
    }
    return 0;
//    stateNew = kstate.s.hold;
//    if(stateOld != stateNew)
//    {
//        stateOld = stateNew;
//        kstate.s.stateChange = 1;
//    }
}

// =========================
// LCD MODULE FUNCTIONS
// =========================
//LCD Constant
#define LCD_CLS		0x01	// Clear screen
#define LCD_CURHOME	0x02	// Display Cursor Home
#define LCD_CHRMODE	0x04	// 0x04 0x02:Increament 0x01:Shift On
#define LCD_DISPLAY 0x08	// 0x08 0x04:DispOn 0x02:Underline 0x01:CurBlink
#define LCD_DISPOCB	0x0F	// Display DispOn,CurOn, BlinkOn
#define LCD_DISPON	0x0C	// Display DispOn,CurOff,BlinkOff
#define LCD_DISPOFF	0x08	// Display DispOff,CurOff,BlinkOff
#define LCD_LINE1	0x80	// LCD Line 1
#define LCD_LINE2	0xC0	// LCD Line 2

unsigned char lcd_mode = 0;
#define lcd_push() lcd_temp = LCD_DATA
#define lcd_pop() LCD_DATA = lcd_temp

byte lcd_temp;
byte lcd_disp = 0b00001100;	// 0b00001DCB D=ON
byte lcd_rs, lcd_en;
byte lcd_d0, lcd_d1, lcd_d2, lcd_d3;
byte lcd_d4, lcd_d5, lcd_d6, lcd_d7;
byte lcd_cols, lcd_rows;

//
//mili-second delay (max 65535ms)
void lcdDelay(unsigned int ms)
{
	unsigned int i;

	while(ms){
		ms--;
		i=MSLOOP;
		while(i) i--;
	}
}

//Generate Single Pulse at Pin E 
void e_pulse(byte k)
{
byte dd = k;
dd |= 0b00000100;
i2c_out(LCD_ADDRESS,dd);
Nop(); Nop(); Nop(); Nop();
dd &= 0b11111011;
i2c_out(LCD_ADDRESS,dd);
Nop(); Nop(); Nop(); Nop();
}

//Write a nibble to LCD
void lcd_nibble(byte nd)
{
	if(nd & 0x01) digitalWrite(lcd_d4,1); else digitalWrite(lcd_d4,0);
	if(nd & 0x02) digitalWrite(lcd_d5,1); else digitalWrite(lcd_d5,0);
	if(nd & 0x04) digitalWrite(lcd_d6,1); else digitalWrite(lcd_d6,0);
	if(nd & 0x08) digitalWrite(lcd_d7,1); else digitalWrite(lcd_d7,0);
	digitalWrite(lcd_en,1);
	Nop(); Nop(); Nop(); Nop();
	digitalWrite(lcd_en,0);
}

//Write a data to LCD
void lcd_putdata(byte d)
{
byte dd;
#ifdef LCD_DATA
	lcd_push();
#endif
	if(lcd_mode==0)
	{
		lcd_nibble(d>>4);		// write 4-bit atas
		lcd_nibble(d & 0x0F);	// write 4-bit bawah
	}
	else if(lcd_mode == 2)
	{	
	dd = ((d&0xF0)& 0xfe)|0b00001000;
	i2c_out(LCD_ADDRESS,dd);
	e_pulse(dd);
	dd = (((d<<4)&0xf0)& 0xfe)|0b00001000;
	i2c_out(LCD_ADDRESS,dd);
	e_pulse(dd);
	}
	else
	{
#ifdef LCD_DATA
		LCD_DATA = d;
#else
		if(d & 0x01) digitalWrite(lcd_d0,1); else digitalWrite(lcd_d0,0);
		if(d & 0x02) digitalWrite(lcd_d1,1); else digitalWrite(lcd_d1,0);
		if(d & 0x04) digitalWrite(lcd_d2,1); else digitalWrite(lcd_d2,0);
		if(d & 0x08) digitalWrite(lcd_d3,1); else digitalWrite(lcd_d3,0);
		if(d & 0x10) digitalWrite(lcd_d4,1); else digitalWrite(lcd_d4,0);
		if(d & 0x20) digitalWrite(lcd_d5,1); else digitalWrite(lcd_d5,0);
		if(d & 0x40) digitalWrite(lcd_d6,1); else digitalWrite(lcd_d6,0);
		if(d & 0x80) digitalWrite(lcd_d7,1); else digitalWrite(lcd_d7,0);
#endif
		digitalWrite(lcd_en,1);
		Nop(); Nop(); Nop(); Nop();
		digitalWrite(lcd_en,0);
	}
#ifdef LCD_DATA
	lcd_pop();
#endif
}

void lcd_clear(void){lcd_putdata(LCD_CLS); lcdDelay(2);}
void lcd_home(void){lcd_putdata(LCD_CURHOME); lcdDelay(1);}
void lcd_setCursor(byte col, byte row){lcd_putdata(row*0x40+col+0x80); lcdDelay(1);}
void lcd_cursor(void){lcd_disp |= 0b00000010; lcd_putdata(lcd_disp); lcdDelay(1);}
void lcd_noCursor(void){lcd_disp &= 0b11111101; lcd_putdata(lcd_disp); lcdDelay(1);}
void lcd_blink(void){lcd_disp |= 0b00000001; lcd_putdata(lcd_disp); lcdDelay(1);}
void lcd_noBlink(void){lcd_disp &= 0b11111110; lcd_putdata(lcd_disp); lcdDelay(1);}
void lcd_display(void){lcd_disp |= 0b00000100; lcd_putdata(lcd_disp); lcdDelay(1);}
void lcd_noDisplay(void){lcd_disp &= 0b11111011; lcd_putdata(lcd_disp); lcdDelay(1);}
void lcd_line1(byte CurX){lcd_putdata(LCD_LINE1+CurX); lcdDelay(1);}
void lcd_line2(byte CurX){lcd_putdata(LCD_LINE2+CurX); lcdDelay(1);}

//Print one character on LCD
void lcd_putc(byte d)
{
	byte i,dd;
	if(lcd_mode == 2)
	{
	dd = (d&0xF0)|0x09;
	i2c_out(LCD_ADDRESS,dd);
	e_pulse(dd);
	dd = ((d<<4)&0xf0) | 0x09;
	i2c_out(LCD_ADDRESS,dd);
	e_pulse(dd);
	dd &= 0b11111110;
	i2c_out(LCD_ADDRESS,dd);
	}
	else{
	//Direct to LCD
	digitalWrite(lcd_rs,1);
	lcd_putdata(d);
	digitalWrite(lcd_rs,0);
	//Data Delay
	//i=35; while(i) i--;	//53us for 4MHz
	i=140; while(i) i--;	//53us for 16MHz
	}
}

//Print 8bit binary data
void lcd_putb(byte d)
{
	byte i;

	for(i=0;i<8;i++){
		if(d & 0x80)
			lcd_putc('1');
		else
			lcd_putc('0');
		d <<= 1;	//shift to upper bit
	}
}

//Print data in hexa format
void lcd_puth(byte d)
{
	byte h;

	h = (d>>4)+'0';
	if(h>'9') h+=7;
	lcd_putc(h);
	h = (d & 0x0F)+'0';
	if(h>'9') h+=7;
	lcd_putc(h);
}

//Print integer data
void lcd_puti(unsigned int Val)
{
	byte Ch[5],i=0,d;

	do{
		//Val=65535
		//d=5 -> d=3 -> d=5 -> d=5 -> d=6
		d=(Val % 10); //d=(0..9)
		d+=0x30;		//convert nilai d(0..9) ke ASCII code '0'..'9'
		Ch[i++]=d;
		Val=Val / 10;
	}while(Val);
	while(i){
		i--;
		lcd_putc(Ch[i]);
	}
}

//Print string data
//void Print(const char *str)			//PIC16
//void Print(const rom char *str)		//MC18
void lcd_print(const char *str)
{
	while(*str){
		lcd_putc(*str);
		str++;
	}
}
void lcd_write(char *str)
{
	while(*str){
		lcd_putc(*str);
		str++;
	}
}

void lcd_init4(byte rs, byte en, byte d4, byte d5, byte d6, byte d7)
{
	lcd_mode = 0;
	lcd_rs = rs; lcd_en = en;
	lcd_d4 = d4; lcd_d5 = d5; lcd_d6 = d6; lcd_d7 = d7;
	pinMode(lcd_rs,OUTPUT);
	pinMode(lcd_en,OUTPUT);
	pinMode(lcd_d4,OUTPUT);
	pinMode(lcd_d5,OUTPUT);
	pinMode(lcd_d6,OUTPUT);
	pinMode(lcd_d7,OUTPUT);
}
void lcd_init8(byte rs, byte en, byte d0, byte d1, byte d2, byte d3, byte d4, byte d5, byte d6, byte d7)
{
	lcd_mode = 1;
	lcd_rs = rs; lcd_en = en;
	lcd_d0 = d0; lcd_d1 = d1; lcd_d2 = d2; lcd_d3 = d3;
	lcd_d4 = d4; lcd_d5 = d5; lcd_d6 = d6; lcd_d7 = d7;
	pinMode(lcd_rs,OUTPUT);
	pinMode(lcd_en,OUTPUT);
	pinMode(lcd_d0,OUTPUT);
	pinMode(lcd_d1,OUTPUT);
	pinMode(lcd_d2,OUTPUT);
	pinMode(lcd_d3,OUTPUT);
	pinMode(lcd_d4,OUTPUT);
	pinMode(lcd_d5,OUTPUT);
	pinMode(lcd_d6,OUTPUT);
	pinMode(lcd_d7,OUTPUT);
}

void lcd_i2c(void)
{
	lcd_mode = 2;
	i2c_init(100000);
}

void lcd_begin(byte cols, byte rows)
{
byte i,d;
	lcd_cols = cols; lcd_rows = rows;
#ifdef LCD_DATA
	lcd_push();
#endif
	//LCD Mode Initialize
	if(lcd_mode==0)
	{
		lcd_nibble(0x03);		//8bit cmd
		lcdDelay(7);		//4.1ms
		lcd_nibble(0x03);		//8bit cmd
		lcdDelay(1);		//100us
		lcd_nibble(0x03);		//8bit cmd
		lcdDelay(1);		//100us
		lcd_nibble(0x02);		//4bit cmd
		lcdDelay(1);		//100us
		lcd_putdata(0x28);				//4bit, font 5x7, 2-line
		lcdDelay(1);		//53us
	}
	else if(lcd_mode == 2)
	{
	
		d = 0b00111000;				//8bit mode, rs=0
		i2c_out(LCD_ADDRESS,d);
		e_pulse(d);
		delay(5);					//Delay 5ms
		e_pulse(d);
		delay(1);
		e_pulse(d);
		delay(1);
	
		d = 0b00101000;				//4 bit Mode, rs = 0;
		i2c_out(LCD_ADDRESS,d);
		e_pulse(d);
		delay(1);
	
		lcd_putdata(0x28);			//4bit, font 5x7, 2-line		
		lcdDelay(1);
	}
	else{
		lcd_putdata(0x30);		//8bit cmd
		lcdDelay(7);		//4.1ms
		lcd_putdata(0x30);		//8bit cmd
		lcdDelay(1);		//100us
		lcd_putdata(0x30);		//8bit cmd
		lcdDelay(1);		//100us
		lcd_putdata(0x38);		//8bit, font 5x7, 2-line
		lcdDelay(1);		//53us
	}

	//LCD Parameter Configuration
	lcd_putdata(0x08);				//display off
	lcdDelay(1);		//53us
	lcd_putdata(0x01);				//display clear
	lcdDelay(7);		//3ms
	lcd_putdata(0x06);				//cursor left->right, blink off
	lcdDelay(1);		//53us
	lcd_putdata(lcd_disp);			//display on
	lcdDelay(1);		//53us

	//Display Test
	//lcd_putc('A');
	//lcd_print(" ** HELLO **");
	lcd_temp = 0;
#ifdef LCD_DATA
	lcd_pop();
#endif
}


// ==============================
// SERIAL COMMUNICATION FUNCTIONS
// ==============================

void uart_brg_set(byte com, unsigned long speed)
{
	byte brgl,brgh;

	speed = speed / 10;
	speed = _XTAL_FREQ / speed;
	speed = speed / 4;
	speed = speed + 5;
	speed = speed / 10;
	speed--;
	brgh = (unsigned char)(speed>>8);
	brgl = (unsigned char)(speed);
	if(com==1)
	{
		SPBRGH = brgh;
		SPBRG1 = brgl;
	}
	else
	{
#ifdef UART2
		SPBRGH2 = brgh;
		SPBRG2 = brgl;
#endif
	}
}

// ======
// UART1
// ======
void Serial_begin(unsigned long speed)
{
	uart_brg_set(1,speed);
	
#if defined(_18F4520) || defined(_18F4550)
	BAUDCTLbits.BRG16 = 1;
#else
	BAUDCON1bits.BRG16 = 1;
#endif
	TXSTA=0x24;		//TXEN=1,BRGH=1
	RCSTA=0x90;		//SPEN=1,CREN=1
	RCIE = 1;		//Enable receiver interrupts
	TXIE = 0;		//Disable transmitter interrupts
	TXIP = 1;		//Make transmit interrupt high priority
	RCIP = 1;		//Make receive interrupt high priority
}

void Serial_end(void)
{
	TXSTA=0x00;		//TXEN=0,BRGH=0
	RCSTA=0x00;		//SPEN=0,CREN=0
	RCIE = 0;		//disable receiver interrupts
}

byte Serial_available(void)
{
	return uart1_rx_cnt;
}

void Serial_putc(byte d)
{
	unsigned char p;
	
	//while(!TXIF); TXREG = d;
	//if(uart1_tx_cnt < UART1_TX_MAX)	uart1_txbuf[uart1_tx_cnt++] = d;
	
	if(uart1_tx_cnt)
	{
		while(uart1_tx_cnt == UART1_TX_MAX);
		INTCONbits.GIEH = 0;
		p = (uart1_tx_cnt + uart1_tx_pos) % UART1_TX_MAX;
		uart1_txbuf[p] = d;
		uart1_tx_cnt++;
		INTCONbits.GIEH = 1;
	}
	else if(TXIF)
	{
		TXREG = d;
	}
	else
	{
		uart1_tx_cnt = 1;
		uart1_tx_pos = 0;
		uart1_txbuf[0] = d;
		TXIE = 1;
	}
}

void Serial_crlf(void)
{
	Serial_putc(0x0D);
	Serial_putc(0x0A);
}

void Serial_puth(byte d)
{
	byte h;

	h = (d>>4)+'0';
	if(h>'9') h+=7;
	Serial_putc(h);
	h = (d & 0x0F)+'0';
	if(h>'9') h+=7;
	Serial_putc(h);
}

void Serial_puti(unsigned int dd)
{
	byte Ch[5],i=0;

	do{
		Ch[i++]=(dd%10)+0x30;
		dd /= 10;
	}while(dd);
	while(i) Serial_putc(Ch[--i]);
}

void Serial_print(const char *str)
{
	while(*str){
		Serial_putc(*str);
		str++;
	}
}

void Serial_println(const char *str)
{
	while(*str){
		Serial_putc(*str);
		str++;
	}
	Serial_putc(0x0D);
	Serial_putc(0x0A);
}

void Serial_write(char *str)
{
	while(*str){
		Serial_putc(*str);
		str++;
	}
}

void Serial_writeln(char *str)
{
	while(*str){
		Serial_putc(*str);
		str++;
	}
	Serial_putc(0x0D);
	Serial_putc(0x0A);
}

byte Serial_read(void)
{
	byte d=0;
	
	if(uart1_rx_pos < uart1_rx_cnt)
	{
		d = uart1_rxbuf[uart1_rx_pos++];
		if(uart1_rx_pos == uart1_rx_cnt)
		{
			uart1_rx_pos = 0;
			uart1_rx_cnt = 0;
			uart1_rx_ready = 0;
		}
	}
	return d;
}

byte Serial_ready(void)
{
	return uart1_rx_ready;
}
	
void Serial_clear(void)
{
	uart1_cr_cnt = 0;
	uart1_lf_cnt = 0;
	uart1_rx_pos = 0;
	uart1_rx_cnt = 0;
	uart1_rx_ready = 0;
	uart1_rxbuf[0] = 0;
}

void Serial_readBytes(byte *buffer, byte length)
{
	byte i=0,j=length;
	
	if(j > uart1_rx_cnt) j = uart1_rx_cnt;
	while(i<j)
	{
		*buffer = uart1_rxbuf[i++];
		buffer++;
	}
}

void Serial_readString(byte *buffer, byte length)
{
	byte i=0,j=length;
	
	if(j > uart1_rx_cnt) j = uart1_rx_cnt;
	while(i<j)
	{
		*buffer = uart1_rxbuf[i++];
		buffer++;
	}
}

// ======
// UART2
// ======
#ifdef UART2
void Serial2_begin(unsigned long speed)
{
	uart_brg_set(2,speed);
	BAUDCON2bits.BRG16 = 1;
	TXSTA2=0x24;		//TXEN=1,BRGH=1
	RCSTA2=0x90;		//SPEN=1,CREN=1
	RC2IE = 1;		//Enable receiver interrupts
	PIE3bits.TX2IE = 0;		//Disable transmitter interrupts
	TX2IP = 1;		//Make transmit interrupt high priority
	RC2IP = 1;		//Make receive interrupt high priority
}

void Serial2_end(void)
{
	TXSTA2=0x00;		//TXEN=0,BRGH=0
	RCSTA2=0x00;		//SPEN=0,CREN=0
	RC2IE = 0;		//disable receiver interrupts
}

byte Serial2_available(void)
{
	return uart2_rx_cnt;
}

void Serial2_putc(byte d)
{
	unsigned char p;
	
	//while(!TX2IF); TXREG2 = d;
	//if(uart2_tx_cnt < UART2_TX_MAX)	uart2_txbuf[uart2_tx_cnt++] = d;
	
	if(uart2_tx_cnt)
	{
		while(uart2_tx_cnt == UART2_TX_MAX);
		INTCONbits.GIEH = 0;
		p = (uart2_tx_cnt + uart2_tx_pos) % UART2_TX_MAX;
		uart2_txbuf[p] = d;
		uart2_tx_cnt++;
		INTCONbits.GIEH = 1;
	}
	else if(TX2IF)
	{
		TXREG2 = d;
	}
	else
	{
		uart2_tx_cnt = 1;
		uart2_tx_pos = 0;
		uart2_txbuf[0] = d;
		TX2IE = 1;
	}
}

void Serial2_crlf(void)
{
	Serial2_putc(0x0D);
	Serial2_putc(0x0A);
}

void Serial2_puth(byte d)
{
	byte h;

	h = (d>>4)+'0';
	if(h>'9') h+=7;
	Serial2_putc(h);
	h = (d & 0x0F)+'0';
	if(h>'9') h+=7;
	Serial2_putc(h);
}

void Serial2_puti(unsigned int dd)
{
	byte Ch[5],i=0;

	do{
		Ch[i++]=(dd%10)+0x30;
		dd /= 10;
	}while(dd);
	while(i) Serial2_putc(Ch[--i]);
}

void Serial2_print(const char *str)
{
	while(*str){
		Serial2_putc(*str);
		str++;
	}
}

void Serial2_println(const char *str)
{
	while(*str){
		Serial2_putc(*str);
		str++;
	}
	Serial2_putc(0x0D);
	Serial2_putc(0x0A);
}

void Serial2_write(char *str)
{
	while(*str){
		Serial2_putc(*str);
		str++;
	}
}

void Serial2_writeln(char *str)
{
	while(*str){
		Serial2_putc(*str);
		str++;
	}
	Serial2_putc(0x0D);
	Serial2_putc(0x0A);
}

byte Serial2_read(void)
{
	byte d=0;
	
	if(uart2_rx_pos < uart2_rx_cnt)
	{
		d = uart2_rxbuf[uart2_rx_pos++];
		if(uart2_rx_pos == uart2_rx_cnt)
		{
			uart2_rx_pos = 0;
			uart2_rx_cnt = 0;
			uart2_rx_ready = 0;
		}
	}
	return d;
}

byte Serial2_ready(void)
{
	return uart2_rx_ready;
}
	
void Serial2_clear(void)
{
	uart2_cr_cnt = 0;
	uart2_lf_cnt = 0;
	uart2_rx_pos = 0;
	uart2_rx_cnt = 0;
	uart2_rx_ready = 0;
	uart2_rxbuf[0] = 0;
}

void Serial2_readBytes(byte *buffer, byte length)
{
	byte i=0,j=length;
	
	if(j > uart2_rx_cnt) j = uart2_rx_cnt;
	while(i<j)
	{
		*buffer = uart2_rxbuf[i++];
		buffer++;
	}
    Serial2_clear();
}

void Serial2_readString(byte *buffer, byte length)
{
	byte i=0,j=length;
	
	if(j > uart2_rx_cnt) j = uart2_rx_cnt;
	while(i<j)
	{
		*buffer = uart2_rxbuf[i++];
		buffer++;
	}
    Serial2_clear();
}
#endif

// ==============================
// SPI COMMUNICATION FUNCTIONS
// ==============================
byte spi_mode = 0;

void SPI_begin(byte mode)
{
	spi_mode = mode;

	DIR_SCK = 0;
	DIR_SDO = 0;
	DIR_SDI = 1;
	
	if(spi_mode == 0)			// Software SPI
	{
		OUT_SCK = 0;
		return;
	}
	
	SSPCON1bits.SSPEN = 0;		// disable SPI
	SSPSTATbits.SMP = 1;		// master SPI
	if(mode==1)					// clock mode, refer DS39977C page 298
	{
		SSPSTATbits.CKE = 0;
		SSPCON1bits.CKP = 0;
	}
	else if(mode==2)
	{
		SSPSTATbits.CKE = 0;
		SSPCON1bits.CKP = 1;
	}
	else if(mode==3)
	{
		SSPSTATbits.CKE = 1;
		SSPCON1bits.CKP = 0;
	}
	else if(mode==4)
	{
		SSPSTATbits.CKE = 1;
		SSPCON1bits.CKP = 1;
	}
	SSPCON1bits.SSPM = 0b0001;	// clock = FOSC/16 = 1MHz
	SSPCON1bits.SSPEN = 1;		// enable SPI
}

byte SPI_transfer(byte txd)
{
	byte rxd=0,i=8;
	
	//=== hardware SPI ===
	if(spi_mode)
	{
		SSPCON1bits.WCOL = 0;
		if(SSPSTATbits.BF) rxd = SSPBUF;
		SSPBUF = txd;
		while(!SSPSTATbits.BF);
		rxd = SSPBUF;
		return rxd;
	}
	
	//=== Software SPI ===
	while(i){
		if(txd & 0x80) OUT_SDO=1; else	OUT_SDO=0;
		rxd <<= 1;
		OUT_SCK=1;
		if(INP_SDI) rxd |= 0x01;
		txd <<= 1;
		OUT_SCK=0;
		i--;
	}
	OUT_SDO=0;

	return rxd;
}

// ==================================
// I2C COMMUNICATION FUNCTIONS
// ==================================
/*
void I2C_begin(void)
{
	DIR_SCL = 0;	// SCL as output
	DIR_SDA = 1;	// SDA as input
}

void I2C_start(void)
{
	OUT_SDA=1;			//SDA H
	DIR_SDA=1;
	Nop(); Nop(); Nop();
	OUT_SCL=1;			//SCL H
	Nop(); Nop(); Nop();
	OUT_SDA=0;			//SDA L
	Nop(); Nop();
	DIR_SDA=0;
	Nop(); Nop(); Nop();
	OUT_SCL=0;			//SCL L
	Nop(); Nop();
}

void I2C_stop(void)
{
	OUT_SDA=0;			//SDA L
	DIR_SDA=0;
	Nop(); Nop(); Nop();
	OUT_SCL=1;			//SCL H
	Nop(); Nop(); Nop();
	OUT_SDA=1;			//SDA H
	DIR_SDA=1;
	Nop(); Nop();
}

unsigned char I2C_out(unsigned char data)
{
	unsigned char i,d;

	i=0;
	d=data;
	while(i<8){
		if(d & 0x80){	//Check MSB(bit7)
			OUT_SDA=1;	//SDA H
			DIR_SDA=1;
		}else{
			OUT_SDA=0;	//SDA L
			DIR_SDA=0;
		}
		Nop();
		Nop();
		OUT_SCL=1;		//SCL H
		d=(d<<1);
		i++;
		OUT_SCL=0;		//SCL L
	}
	//Acknowledge bit
	//OUT_SDA=0;	//SDA H
	//DIR_SDA=0;
	OUT_SDA=1;	//SDA H
	DIR_SDA=1;
	Nop();
	OUT_SCL=1;			//SCL H
	Nop(); Nop();
	//d=0; //SDA;
    d=INP_SDA;
	OUT_SCL=0;			//SCL L

	return(d);
}

unsigned char I2C_inp(unsigned char AckFlag)
{
	unsigned char i,d;

	d=0;
	i=0;
	OUT_SDA=1;
	DIR_SDA=1;			//as input
	while(i<8){
		d <<= 1;
		OUT_SCL=1;		//SCL H
		Nop();
		if(INP_SDA)	//read reply data (bit)
			d |= 0x01;
		OUT_SCL=0;		//SCL L
		i++;
	}
	DIR_SDA=0;			//as output
	if(AckFlag)
		OUT_SDA=0;		//Acknowledge
	else
		OUT_SDA=1;		//Not Acknowledge
	DIR_SDA=0;			//as output
	OUT_SCL=1;			//SCL H
	Nop(); Nop();
	OUT_SCL=0;			//SCL L
	Nop();
	OUT_SDA=0;			//SDA L
	return d;
}
*/

byte i2c_error_flag = 0;

void i2c_init(unsigned long c)
{
    //Slew rate control disabled for standard speed mode (100 kHz and 1 MHz)
    SSPSTATbits.SMP = 1;
    
    //Select I2C Master Mode, Clock = Fosc/(4*(SSPADD + 1))
    SSPCON1bits.SSPM3 = 1;
    SSPCON1bits.SSPM2 = 0;
    SSPCON1bits.SSPM1 = 0;
    SSPCON1bits.SSPM0 = 0;
    
    SSPADD = (_XTAL_FREQ /(4*c))-1;
    pinMode(SCL,INPUT);
    pinMode(SDA,INPUT);

    //Clear the Write Collision Detect bit
    SSPCON1bits.WCOL = 0;
    
    //Clear the Receive Overflow Indicator bit
    SSPCON1bits.SSPOV = 0;
        
    //Enable the MSSP module
    SSPCON1bits.SSPEN = 1;

    //Send Stop condition, stop any previous communication
    SSPCON2bits.PEN = 1;
    while(SSPCON2bits.PEN == 1);
}

byte b_i2c_error_flag(void)
{
	return i2c_error_flag;
}

void i2c_idle(void)
{
	//Need to wait until all ACKEN,RSEN,PEN,SEN bit and RW bit is clear
	while(SSPCON2 & 0b00011111 | (SSPSTATbits.R_W));
}


void i2c_out(byte slave_address,byte data)
{
	//Clear the error flag before we start a new I2C operation
	i2c_error_flag = 0;

	//Send START bit
	SSPCON2bits.SEN = 1;
	while(SSPCON2bits.SEN == 1);

	//Send Slave address and indicate to write
	SSPBUF = (slave_address << 1) & 0xFE;

	//wait for message sending process complete
	i2c_idle();

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return;
	}

	//Send data
	SSPBUF = data;

	//wait for message sending process complete
	i2c_idle();	

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return;
	}

	//Send STOP bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);

	//Clear the error flag before exit
	i2c_error_flag = 0;
}


void i2c_write(byte slave_address,byte register_add,byte data)
{
	//Clear the error flag before we start a new I2C operation
	i2c_error_flag = 0;

	//Send START bit
	SSPCON2bits.SEN = 1;
	while(SSPCON2bits.SEN == 1);

	//Send Slave address and indicate to write
	SSPBUF = (slave_address << 1) & 0xFE;

	//wait for message sending process complete
	i2c_idle();

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return;
	}

	//Send the register address that we want to write
	SSPBUF = register_add;

	//wait for message sending process complete
	i2c_idle();	

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return;
	}

	//Send data
	SSPBUF = data;

	//wait for message sending process complete
	i2c_idle();	

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return;
	}

	//Send STOP bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);

	//Clear the error flag before exit
	i2c_error_flag = 0;
}

byte i2c_read(byte slave_address,byte register_add)
{
	byte rx_data;
	unsigned long count = 10000;
	
	//Clear the error flag before we start a new I2C operation
	i2c_error_flag = 0;

	//Send START bit
	SSPCON2bits.SEN = 1;
	while(SSPCON2bits.SEN == 1);
	
	//Send Slave address and indicate to write
	SSPBUF = (slave_address << 1) & 0xFE;
	
	//wait for message sending process complete
	i2c_idle();

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return 0;
	}

	//Send the register address that we want to write
	SSPBUF = register_add;

	//wait for message sending process complete
	i2c_idle();	

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return 0;
	}

	//Send RESTART bit
	SSPCON2bits.RSEN = 1;
	while(SSPCON2bits.RSEN == 1);

	//Send slave address and indicate to read
	SSPBUF = (slave_address << 1) | 0x01;

	//wait for message sending process complete
	i2c_idle();	

	if(SSPCON2bits.ACKSTAT == 1)
	{
	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);
	
	//Set the error flag and exit
	i2c_error_flag = 1;
	return 0;
	}

	//Enable receive
	SSPCON2bits.RCEN = 1;
	
	//wait until the data received
	while(SSPSTATbits.BF == 0)
	{
		//if timeout...
		if(--count == 0 )
		{
			//Send Stop bit
			SSPCON2bits.PEN = 1;
			while(SSPCON2bits.PEN == 1);
			
			//Set the error flag and exit
			i2c_error_flag = 1;
			return 0;
		}
	}	
	
	//Read the received data
	rx_data = SSPBUF;

	//Send Not Acknowledge
	SSPCON2bits.ACKDT = 1;
	SSPCON2bits.ACKEN = 1;
	while(SSPCON2bits.ACKEN == 1);

	//Send Stop bit
	SSPCON2bits.PEN = 1;
	while(SSPCON2bits.PEN == 1);

	//Clear the error flag and return the received data
	i2c_error_flag = 0;
	return rx_data;	
	
}
// ==================================
// PWM OUTPUT
// ==================================

//======================================
//Software PWM
//======================================
void pwm_on(void)   //1KHz
{
    //Use TIMER2 as PWM Generator
    
    // PR2 255; 
    PR2 = 0xFF;

    // TMR2 0; 
    TMR2 = 0x00;

    // Clearing IF flag.
    PIR1bits.TMR2IF = 0;

    // T2CKPS 1:16; T2OUTPS 1:4; TMR2ON on; 
    T2CON = 0x1E;

    //PWM1 - CCP1 at Pin RC2 ==================================================
	
	// CCP1M P1A,P1C: active high; P1B,P1D: active high; DC1B 1; P1M single; 
	CCP1CON = 0x1C;    
	
	// CCP1ASE operating; PSS1BD low; PSS1AC low; CCP1AS disabled; 
	ECCP1AS = 0x00;    
	
	// P1RSEN automatic_restart; P1DC 0; 
	PWM1CON = 0x80;    
	
	// STR1D P1D_to_port; STR1C P1C_to_port; STR1B P1B_to_port; STR1A P1A_to_CCP1M; STR1SYNC start_at_begin; 
	PSTR1CON = 0x01;    
	
	// CCPR1H 0; 
	CCPR1H = 0x00;    
	
	// CCPR1L 0; 
	CCPR1L = 0x00;    

	// Selecting Timer2
	CCPTMRS0bits.C1TSEL = 0x0;    
    
    //PWM2 - CCP2 at Pin RB3 ==================================================
    // CCP2M P2A,P2C: active high; P2B,P2D: active high; DC2B 3; P2M single; 
    CCP2CON = 0x3C;
    
    // CCP2ASE operating; PSS2BD low; PSS2AC low; CCP2AS disabled; 
    ECCP2AS = 0x00;
    
    // P2RSEN automatic_restart; P2DC 0; 
    PWM2CON = 0x80;
    
    // STR2D P2D_to_port; STR2C P2C_to_port; STR2B P2B_to_port; STR2A P2A_to_CCP2M; STR2SYNC start_at_begin; 
    PSTR2CON = 0x01;
    
    // CCPR2L 124; 
    CCPR2L = 0x00;
    
    // CCPR2H 0; 
    CCPR2H = 0x00;
      
    // Selecting Timer2
    CCPTMRS0bits.C2TSEL = 0x0;
    
    //PWM3 - CCP3 at Pin RB5 ===================================================
    // Set the EPWM3 to the options selected in the User Interface
	
	// CCP3M P3A,P3C: active high; P3B,P3D: active high; DC3B 3; P3M single; 
	CCP3CON = 0x0C;    
	
	// CCP3ASE operating; PSS3BD low; PSS3AC low; CCP3AS disabled; 
	ECCP3AS = 0x00;    
	
	// P3RSEN automatic_restart; P3DC 0; 
	PWM3CON = 0x80;    
	
	// STR3D P3D_to_port; STR3C P3C_to_port; STR3B P3B_to_port; STR3A P3A_to_CCP3M; STR3SYNC start_at_begin; 
	PSTR3CON = 0x01;    
	
	// CCPR3H 0; 
	CCPR3H = 0x00;    
	
	// CCPR3L 124; 
	CCPR3L = 0x00;    

	// Selecting Timer2
	CCPTMRS0bits.C3TSEL = 0x0;
    
    //PWM4 - CCP4 at Pin RD1 ===================================================
	
	// CCP4M PWM; DC4B 1; 
	CCP4CON = 0x1C;    
	
	// CCPR4L 0; 
	CCPR4L = 0x00;    
	
	// CCPR4H 0; 
	CCPR4H = 0x00;    

	// Selecting Timer 2
	CCPTMRS1bits.C4TSEL = 0x0;    
    
    pinMode(PIN_RC2,OUTPUT);
    pinMode(PIN_RB3,OUTPUT);
    pinMode(PIN_RB5,OUTPUT);
    pinMode(PIN_RD1,OUTPUT);    
}

void pwm_off(void)
{
T2CONbits.TMR2ON = 0;
}

void pwm_out(unsigned char ch,unsigned int dutyValue)
{

	if(ch == 0x01)
		{
		   // Writing to 8 MSBs of pwm duty cycle in CCPRL register
            CCPR1L = (dutyValue & 0x00FF);
		}
	else if(ch == 0x02)
		{
            // Writing to 8 MSBs of pwm duty cycle in CCPRL register
             CCPR2L = (dutyValue & 0x00FF);
		}
	else if(ch == 0x03)
		{
            // Writing to 8 MSBs of pwm duty cycle in CCPRL register
             CCPR3L = (dutyValue & 0x00FF);
		}   
	else if(ch == 0x04)
		{
            // Writing to 8 MSBs of pwm duty cycle in CCPRL register
             CCPR4L = (dutyValue & 0x00FF);
		}    
}

//ORIGINAL START HERE
//void pwm_on(byte PWMch)
//{
//	if(PWMch & 0x01){
//		//Setup PWM1
//		PR2=100;	//Full Scale
//		CCPR1L=50;	//50%
//		CCP1CON=0b00001100;	//PWM Mode
//		T2CONbits.TMR2ON=1;	//Start PWM
//	}
//#ifdef PWM2
//	if(PWMch & 0x02){
//		//Setup PWM2
//		PR4=100;	//Full Scale
//		CCPR2L=50;	//50%
//		CCP2CON=0b00001100;	//PWM Mode
//		T4CONbits.TMR4ON=1;	//Start PWM
//	}
//#endif
//}
//
//void pwm_Off(byte PWMch)
//{
//	if(PWMch & 0x01){
//		//Clear PWM1
//		T2CONbits.TMR2ON=0;	//Stop PWM
//		CCP1CON=0b00000000;	//I/O Mode
//	}
//#ifdef PWM2
//	if(PWMch & 0x02){
//		//Clear PWM2
//		T4CONbits.TMR4ON=0;	//Stop PWM
//		CCP2CON=0b00000000;	//I/O Mode
//	}
//#endif
//}
//
//void pwm_out(byte PWMch, byte Ratio)
//{
//	if(PWMch & 0x01){
//		//Set PWM1
//		CCPR1L=Ratio;	//%
//	}
//	if(PWMch & 0x02){
//		//Set PWM2
//		CCPR2L=Ratio;	//%
//	}
//}
//ORIGINAL END HERE


// ==================================
// STRING FUNCTIONS
// ==================================

//Find String2 within String1
//Found     : return +1 from position
//Not Found : return 0
unsigned char InStr(unsigned char SPos, unsigned char *Str1, const char *Str2)
{
	unsigned char l1=0,l2=0,i,j,c1,c2;

	i=SPos;
	while(Str1[l1]) l1++;		//get string length1
	while(Str2[l2]) l2++;		//get string length2
	if(l2>l1) return 0;
	l1=l1-l2;
	if(i>l1) return 0;
	while(i<=l1){
		j=0;
		c1=Str1[i];
		c2=Str2[0];
		if(c1==c2){
			while(j<l2){
				if(c1!=c2) goto NEXTC;
				j++;
				c1=Str1[i+j];
				c2=Str2[j];
			}
			return i+1;
		}
NEXTC:
		i++;
	}
	return 0;
}

//Find a character within String1
//Found     : return +1 from position
//Not Found : return 0
unsigned char FindChar(unsigned char SPos, unsigned char *Str1, unsigned char FC)
{
	unsigned char i,CH;

	i=SPos;
	CH=Str1[i];
	while(CH){
		if(CH==FC) return i+1;
		i++;
		CH=Str1[i];
	}
	return 0;
}

//Convert valid string number
unsigned int StrToUInt(unsigned char *Str1)
{
	unsigned char c,i=0;
	unsigned int dd=0;
	
	while(1){
		c=Str1[i++];
		if('0'<=c && c<='9'){		//numeric char
			dd *= 10;
			dd += (c-'0');
		}else{						//exception
			break;
		}
		if(i>5) break;
	}
	return(dd);
}

//ADD-on FUNCTION START HERE
void INT0_ISR(void)
{
    EXT_INT0_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT0_CallBack();    
}

void INT0_CallBack(void)
{
    // Add your custom callback code here
    if(INT0_InterruptHandler)
    {
        INT0_InterruptHandler();
    }
}

void INT0_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT0_InterruptHandler = InterruptHandler;
}

void INT1_ISR(void)
{
    EXT_INT1_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT1_CallBack();    
}

void INT1_CallBack(void)
{
    // Add your custom callback code here
    if(INT1_InterruptHandler)
    {
        INT1_InterruptHandler();
    }
}

void INT1_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT1_InterruptHandler = InterruptHandler;
}

void INT2_ISR(void)
{
    EXT_INT2_InterruptFlagClear();

    // Callback function gets called everytime this ISR executes
    INT2_CallBack();    
}


void INT2_CallBack(void)
{
    // Add your custom callback code here
    if(INT2_InterruptHandler)
    {
        INT2_InterruptHandler();
    }
}

void INT2_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT2_InterruptHandler = InterruptHandler;
}

void attachInterrupt(byte interruptSource, void (* InterruptHandler)(void),byte mode){
    if(EXT_INT0 == interruptSource){
      EXT_INT0_InterruptFlagClear();    
      if(RISING == mode)EXT_INT0_risingEdgeSet();
      else if(FALLING == mode)EXT_INT0_fallingEdgeSet();
      INT0_InterruptHandler = InterruptHandler; 
      EXT_INT0_InterruptEnable();  
    }
    else if(EXT_INT1 == interruptSource){
      EXT_INT1_InterruptFlagClear();    
      if(RISING == mode)EXT_INT1_risingEdgeSet();
      else if(FALLING == mode)EXT_INT1_fallingEdgeSet();
      INT1_InterruptHandler = InterruptHandler; 
      EXT_INT1_InterruptEnable();  
    }
    else if(EXT_INT2 == interruptSource){
      EXT_INT2_InterruptFlagClear();    
      if(RISING == mode)EXT_INT2_risingEdgeSet();
      else if(FALLING == mode)EXT_INT2_fallingEdgeSet();
      INT2_InterruptHandler = InterruptHandler; 
      EXT_INT2_InterruptEnable();  
    }    
}

void detachInterrupt(byte interruptSource){
    if(EXT_INT0 == interruptSource)
        EXT_INT0_InterruptDisable();
    else if(EXT_INT1 == interruptSource)
        EXT_INT1_InterruptDisable();
    else if(EXT_INT2 == interruptSource)
        EXT_INT2_InterruptDisable();
}
//ADD-on FUNCTION END HERE