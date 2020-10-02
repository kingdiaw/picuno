/* ELECTROSOFT ENGINEERING
 * File:   picuno.h
 * Author: hajazi
 * Created on 20 Jun 2015
 * 2016-02-24 Modify for PIC18 series
 * Support MCU:
 *   PIC18F4520,PIC18F4550
 *   PIC18F45K50
 *   PIC18F44K22,PIC18F45K22,PIC18F46K22
 *   PIC18F46K80,PIC18F66K80
 */

#include <htc.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#ifndef PICUNO_H
#define PICUNO_H

/////////////////////////////////////
/// USER CONFIGURATION AREA
/////////////////////////////////////
// UART BUFFER LENGTH
#define UART1_TX_MAX	32
#define UART1_RX_MAX	95
#define UART2_TX_MAX	32
#define UART2_RX_MAX	95
// I2C - SCL PIN
#define DIR_SCL		DIR_RB7
#define OUT_SCL		OUT_RB7
// I2C - SDA PIN
#define DIR_SDA		DIR_RB6
#define OUT_SDA		OUT_RB6
#define INP_SDA		INP_RB6
// SPI - PIN Direction
#define DIR_SCK		DIR_RB4
#define DIR_SDO		DIR_RB2
#define DIR_SDI		DIR_RB1
// SPI - PIN INP/OUT
#define OUT_SCK		OUT_RB4
#define OUT_SDO		OUT_RB2
#define INP_SDI		INP_RB1

// LCD_i2c
#define	LCD_ADDRESS	0x27	//PCF8574
//#define	LCD_ADDRESS	0x3F        //PCF8574A

// USER MULTI TASKING
//#define _TASK1
//#define _TASK2
//#define _TASK3
//#define _TASK4
//#define _TASK5
//#define _TASK6
//#define _TASK7
//#define _TASK8

#define _MCU_NOT_SUPPORT

/////////////////////////////////////
/// MCU CONFIGURATION BIT
/////////////////////////////////////

#if defined(_18F4520)
#pragma config OSC = INTIO67    // Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
#define ADC_ADCON
#undef _MCU_NOT_SUPPORT
#endif

#if defined(_18F4550)
#pragma config FOSC= INTOSCIO_EC// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
#undef _MCU_NOT_SUPPORT
#define ADC_ADCON
#define	ROBOTIC_LAB
//#define MSSP1
#endif

#if defined(_18F45K50)
#pragma config FOSC = INTOSCIO  // Oscillator Selection (Internal oscillator)
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
#undef _MCU_NOT_SUPPORT
#define ADC_ANSEL
#endif

#if defined(_18F44K22) || defined(_18F45K22) || defined(_18F46K22)
//#pragma config FOSC = INTIO67   // Oscillator Selection (Internal oscillator)
//#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled)
//#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
//#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
//#pragma config CCP3MX = PORTE0    // P3A/CCP3 Mux bit->P3A/CCP3 input/output is mulitplexed with RE0
#undef _MCU_NOT_SUPPORT
#define ADC_ANSEL
#define UART2
#define MSSP1
#define	ROBOTIC_LAB
#endif

#if defined(_18F46K80) || defined(_18F66K80)
#pragma config FOSC = INTIO2    // Oscillator Selection (Internal oscillator)
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)
#undef _MCU_NOT_SUPPORT
#define ADC_ANCON
#define UART2
#endif

#ifdef _MCU_NOT_SUPPORT
#error PICUNO18 ERROR: Select MCU not support for current version
#endif


/////////////////////////////////////
/// INTERNAL OSCILLATOR DEFINITION
/////////////////////////////////////
#if defined(_18F4520) || defined(_18F4550)
  #define _XTAL_FREQ	8000000L
  #define FREQ			8 //MHz
  #define FOSC8
#elif defined(_18F45K22)
    #define _XTAL_FREQ  64000000UL
    #define FREQ        64  //MHz
    #define FOSC64
#else
  #define _XTAL_FREQ	16000000L
  #define FREQ			16 //MHz
  #define FOSC16
#endif
//Internal Oscillator Setting - rujuk datasheet
#define OSCCON_VAL 	0b01110010	// 18F4550:8MHz, others 16MHz
	  //OSCCON_VAL	0b01100010	// 18F4550:4MHz
	  //OSCCON_VAL	0b01010010	// OTHERS :4MHz

/////////////////////////////////////////////////
//DEFINE MACRO AREA
////////////////////////////////////////////////
#define EXT_INT0_InterruptFlagClear()       (INTCONbits.INT0IF = 0)
#define EXT_INT0_InterruptDisable()     (INTCONbits.INT0IE = 0)
#define EXT_INT0_InterruptEnable()       (INTCONbits.INT0IE = 1)
#define EXT_INT0_risingEdgeSet()          (INTCON2bits.INTEDG0 = 1)
#define EXT_INT0_fallingEdgeSet()          (INTCON2bits.INTEDG0 = 0)
#define EXT_INT1_InterruptFlagClear()       (INTCON3bits.INT1IF = 0)
#define EXT_INT1_InterruptDisable()     (INTCON3bits.INT1IE = 0)
#define EXT_INT1_InterruptEnable()       (INTCON3bits.INT1IE = 1)
#define EXT_INT1_risingEdgeSet()          (INTCON2bits.INTEDG1 = 1)
#define EXT_INT1_fallingEdgeSet()          (INTCON2bits.INTEDG1 = 0)
#define EXT_INT2_InterruptFlagClear()       (INTCON3bits.INT2IF = 0)
#define EXT_INT2_InterruptDisable()     (INTCON3bits.INT2IE = 0)
#define EXT_INT2_InterruptEnable()       (INTCON3bits.INT2IE = 1)
#define EXT_INT2_risingEdgeSet()          (INTCON2bits.INTEDG2 = 1)
#define EXT_INT2_fallingEdgeSet()          (INTCON2bits.INTEDG2 = 0)


#define MSLOOP		((FREQ*16)+(FREQ/4))


///////////////////////////////////////////////
//DEFINE CONSTANT AREA
//////////////////////////////////////////////

#define HIGH		0x01
#define LOW			0x00

#define TRUE		0x01
#define FALSE		0x00
#define true		0x01
#define false		0x00

#define DIR_OUT		0x00
#define DIR_INP		0x01

#define OUTPUT		0x00
#define INPUT		0x01
#define INPUT_PULLUP 0x02
#define INPUT_ANALOG 0x03

#define PI			3.1415926535897932384626433832795
#define HALF_PI		1.5707963267948966192313216916398
#define TWO_PI		6.283185307179586476925286766559
#define DEG_TO_RAD	0.017453292519943295769236907684886
#define RAD_TO_DEG	57.295779513082320876798154814105
#define EULER		2.718281828459045235360287471352

#define SERIAL		0x0
#define DISPLAY		0x1

#define LSBFIRST	0
#define MSBFIRST	1

#define CHANGE		1
#define FALLING		2
#define RISING		3
#define EXT_INT0    0
#define EXT_INT1    1
#define EXT_INT2    2


//=============================================================================
// ANALOG INPUT DEFINITION
//=============================================================================
//
#define AN0			0
#define AN1			1
#define AN2			2
#define AN3			3
#define AN4			4
#define AN5			5
#define AN6			6
#define AN7			7
#define RV1         4

//=============================================================================
// IO DEFINITION
//=============================================================================
//ADD-on START HERE

#ifdef MSSP1                //For PIC18F45K22
#define SSPSTAT SSP1STAT
#define SSPCON1 SSP1CON1
#define SSPBUF  SSP1BUF
#define SCL     PIN_RC3
#define SDA     PIN_RC4
#else                       //For PIC18F4550
#define SCL     PIN_RC3
#define SDA     PIN_RC4
#endif

//ADD-on END HERE

//ORIGINAL START HERE
#define PIN_RA0		0
#define DIR_RA0		TRISAbits.TRISA0
#define INP_RA0		PORTAbits.RA0
#define OUT_RA0		LATAbits.LATA0
//
#define PIN_RA1		1
#define DIR_RA1		TRISAbits.TRISA1
#define INP_RA1		PORTAbits.RA1
#define OUT_RA1		LATAbits.LATA1
//
#define PIN_RA2		2
#define DIR_RA2		TRISAbits.TRISA2
#define INP_RA2		PORTAbits.RA2
#define OUT_RA2		LATAbits.LATA2
//
#define PIN_RA3		3
#define DIR_RA3		TRISAbits.TRISA3
#define INP_RA3		PORTAbits.RA3
#define OUT_RA3		LATAbits.LATA3
//
#define PIN_RA4		4
#define DIR_RA4		TRISAbits.TRISA4
#define INP_RA4		PORTAbits.RA4
#define OUT_RA4		LATAbits.LATA4
//
#define PIN_RA5		5
#define DIR_RA5		TRISAbits.TRISA5
#define INP_RA5		PORTAbits.RA5
#define OUT_RA5		LATAbits.LATA5
//
#define PIN_RA6		6
#define DIR_RA6		TRISAbits.TRISA6
#define INP_RA6		PORTAbits.RA6
#define OUT_RA6		LATAbits.LATA6
//
#define PIN_RA7		7
#define DIR_RA7		TRISAbits.TRISA7
#define INP_RA7		PORTAbits.RA7
#define OUT_RA7		LATAbits.LATA7
//
#define PIN_RB0		8
#define DIR_RB0		TRISBbits.TRISB0
#define INP_RB0		PORTBbits.RB0
#define OUT_RB0		LATBbits.LATB0
//
#define PIN_RB1		9
#define DIR_RB1		TRISBbits.TRISB1
#define INP_RB1		PORTBbits.RB1
#define OUT_RB1		LATBbits.LATB1
//
#define PIN_RB2		10
#define DIR_RB2		TRISBbits.TRISB2
#define INP_RB2		PORTBbits.RB2
#define OUT_RB2		LATBbits.LATB2
//
#define PIN_RB3		11
#define DIR_RB3		TRISBbits.TRISB3
#define INP_RB3		PORTBbits.RB3
#define OUT_RB3		LATBbits.LATB3
//
#define PIN_RB4		12
#define DIR_RB4		TRISBbits.TRISB4
#define INP_RB4		PORTBbits.RB4
#define OUT_RB4		LATBbits.LATB4
//
#define PIN_RB5		13
#define DIR_RB5		TRISBbits.TRISB5
#define INP_RB5		PORTBbits.RB5
#define OUT_RB5		LATBbits.LATB5
//
#define PIN_RB6		14
#define DIR_RB6		TRISBbits.TRISB6
#define INP_RB6		PORTBbits.RB6
#define OUT_RB6		LATBbits.LATB6
//
#define PIN_RB7		15
#define DIR_RB7		TRISBbits.TRISB7
#define INP_RB7		PORTBbits.RB7
#define OUT_RB7		LATBbits.LATB7
//
#define PIN_RC0		16
#define DIR_RC0		TRISCbits.TRISC0
#define INP_RC0		PORTCbits.RC0
#define OUT_RC0		LATCbits.LATC0
//
#define PIN_RC1		17
#define DIR_RC1		TRISCbits.TRISC1
#define INP_RC1		PORTCbits.RC1
#define OUT_RC1		LATCbits.LATC1
//
#define PIN_RC2		18
#define DIR_RC2		TRISCbits.TRISC2
#define INP_RC2		PORTCbits.RC2
#define OUT_RC2		LATCbits.LATC2
//
#define PIN_RC3		19
#define DIR_RC3		TRISCbits.TRISC3
#define INP_RC3		PORTCbits.RC3
#define OUT_RC3		LATCbits.LATC3
//
#define PIN_RC4		20
#define DIR_RC4		TRISCbits.TRISC4
#define INP_RC4		PORTCbits.RC4
#define OUT_RC4		LATCbits.LATC4
//
#define PIN_RC5		21
#define DIR_RC5		TRISCbits.TRISC5
#define INP_RC5		PORTCbits.RC5
#define OUT_RC5		LATCbits.LATC5
//
#define PIN_RC6		22
#define DIR_RC6		TRISCbits.TRISC6
#define INP_RC6		PORTCbits.RC6
#define OUT_RC6		LATCbits.LATC6
//
#define PIN_RC7		23
#define DIR_RC7		TRISCbits.TRISC7
#define INP_RC7		PORTCbits.RC7
#define OUT_RC7		LATCbits.LATC7
//
#define PIN_RD0		24
#define DIR_RD0		TRISDbits.TRISD0
#define INP_RD0		PORTDbits.RD0
#define OUT_RD0		LATDbits.LATD0
//
#define PIN_RD1		25
#define DIR_RD1		TRISDbits.TRISD1
#define INP_RD1		PORTDbits.RD1
#define OUT_RD1		LATDbits.LATD1
//
#define PIN_RD2		26
#define DIR_RD2		TRISDbits.TRISD2
#define INP_RD2		PORTDbits.RD2
#define OUT_RD2		LATDbits.LATD2
//
#define PIN_RD3		27
#define DIR_RD3		TRISDbits.TRISD3
#define INP_RD3		PORTDbits.RD3
#define OUT_RD3		LATDbits.LATD3
//
#define PIN_RD4		28
#define DIR_RD4		TRISDbits.TRISD4
#define INP_RD4		PORTDbits.RD4
#define OUT_RD4		LATDbits.LATD4
//
#define PIN_RD5		29
#define DIR_RD5		TRISDbits.TRISD5
#define INP_RD5		PORTDbits.RD5
#define OUT_RD5		LATDbits.LATD5
//
#define PIN_RD6		30
#define DIR_RD6		TRISDbits.TRISD6
#define INP_RD6		PORTDbits.RD6
#define OUT_RD6		LATDbits.LATD6
//
#define PIN_RD7		31
#define DIR_RD7		TRISDbits.TRISD7
#define INP_RD7		PORTDbits.RD7
#define OUT_RD7		LATDbits.LATD7
//
#define PIN_RE0		32
#define DIR_RE0		TRISEbits.TRISE0
#define INP_RE0		PORTEbits.RE0
#define OUT_RE0		LATEbits.LATE0
//
#define PIN_RE1		33
#define DIR_RE1		TRISEbits.TRISE1
#define INP_RE1		PORTEbits.RE1
#define OUT_RE1		LATEbits.LATE1
//
#define PIN_RE2		34
#define DIR_RE2		TRISEbits.TRISE2
#define INP_RE2		PORTEbits.RE2
#define OUT_RE2		LATEbits.LATE2
//
#define PIN_RE3		35
#define DIR_RE3		TRISEbits.TRISE3
#define INP_RE3		PORTEbits.RE3
#define OUT_RE3		LATEbits.LATE3
//
#define PIN_RE4		36
#define DIR_RE4		TRISEbits.TRISE4
#define INP_RE4		PORTEbits.RE4
#define OUT_RE4		LATEbits.LATE4
//
#define PIN_RE5		37
#define DIR_RE5		TRISEbits.TRISE5
#define INP_RE5		PORTEbits.RE5
#define OUT_RE5		LATEbits.LATE5
//
#define PIN_RE6		38
#define DIR_RE6		TRISEbits.TRISE6
#define INP_RE6		PORTEbits.RE6
#define OUT_RE6		LATEbits.LATE6
//
#define PIN_RE7		39
#define DIR_RE7		TRISEbits.TRISE7
#define INP_RE7		PORTEbits.RE7
#define OUT_RE7		LATEbits.LATE7
//
#define PIN_RF0		40
#define DIR_RF0		TRISFbits.TRISF0
#define INP_RF0		PORTFbits.RF0
#define OUT_RF0		LATFbits.LATF0
//
#define PIN_RF1		41
#define DIR_RF1		TRISFbits.TRISF1
#define INP_RF1		PORTFbits.RF1
#define OUT_RF1		LATFbits.LATF1
//
#define PIN_RF2		42
#define DIR_RF2		TRISFbits.TRISF2
#define INP_RF2		PORTFbits.RF2
#define OUT_RF2		LATFbits.LATF2
//
#define PIN_RF3		43
#define DIR_RF3		TRISFbits.TRISF3
#define INP_RF3		PORTFbits.RF3
#define OUT_RF3		LATFbits.LATF3
//
#define PIN_RF4		44
#define DIR_RF4		TRISFbits.TRISF4
#define INP_RF4		PORTFbits.RF4
#define OUT_RF4		LATFbits.LATF4
//
#define PIN_RF5		45
#define DIR_RF5		TRISFbits.TRISF5
#define INP_RF5		PORTFbits.RF5
#define OUT_RF5		LATFbits.LATF5
//
#define PIN_RF6		46
#define DIR_RF6		TRISFbits.TRISF6
#define INP_RF6		PORTFbits.RF6
#define OUT_RF6		LATFbits.LATF6
//
#define PIN_RF7		47
#define DIR_RF7		TRISFbits.TRISF7
#define INP_RF7		PORTFbits.RF7
#define OUT_RF7		LATFbits.LATF7
//
#define PIN_RG0		48
#define DIR_RG0		TRISGbits.TRISG0
#define INP_RG0		PORTGbits.RG0
#define OUT_RG0		LATGbits.LATG0
//
#define PIN_RG1		49
#define DIR_RG1		TRISGbits.TRISG1
#define INP_RG1		PORTGbits.RG1
#define OUT_RG1		LATGbits.LATG1
//
#define PIN_RG2		50
#define DIR_RG2		TRISGbits.TRISG2
#define INP_RG2		PORTGbits.RG2
#define OUT_RG2		LATGbits.LATG2
//
#define PIN_RG3		51
#define DIR_RG3		TRISGbits.TRISG3
#define INP_RG3		PORTGbits.RG3
#define OUT_RG3		LATGbits.LATG3
//
#define PIN_RG4		52
#define DIR_RG4		TRISGbits.TRISG4
#define INP_RG4		PORTGbits.RG4
#define OUT_RG4		LATGbits.LATG4
//ORIGINAL END HERE

//=============================================================================
#define SET_BIT(x) x = 1; asm("nop")
#define CLR_BIT(x) x = 0; asm("nop")

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define lowByte(w) ((byte) ((w) & 0xff))
#define highByte(w) ((byte) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define bit(b) (1UL << (b))

//=============================================================================
typedef signed int 		bool;
typedef signed int 		BOOL;
typedef unsigned int 	word;
typedef unsigned int 	WORD;
typedef unsigned char 	byte;
typedef unsigned char 	BYTE;
typedef signed char 	int8_t;
typedef unsigned char 	uint8_t;
//typedef signed short	int16_t;
//typedef unsigned short 	uint16_t;
typedef signed int 		int16_t;
typedef unsigned int 	uint16_t;
typedef signed long		int32_t;
typedef unsigned long	uint32_t;

union
{
	struct
	{
		unsigned bit0:1;
		unsigned bit1:1;
		unsigned bit2:1;
		unsigned bit3:1;
		unsigned bit4:1;
		unsigned bit5:1;
		unsigned bit6:1;
		unsigned bit7:1;
	}Bit;
	byte Byte;
} Flags;

union
{
    struct
    {       uint8_t hold:1;
            uint8_t press:1;
            uint8_t idle:1;
            uint8_t stateChange:1;
            uint8_t reserved:4;
    }s;
}kstate;

#define intbit Flags.Bit.bit0
#define hmsbit Flags.Bit.bit1
#define hfsbit Flags.Bit.bit2
#define secbit Flags.Bit.bit3
#define minbit Flags.Bit.bit4
#define daybit Flags.Bit.bit5
#define _rsvbit Flags.Bit.bit6
#define syncbit Flags.Bit.bit7

//=============================================================================

//ADD-ON START HERE

//ADD-ON END HERE

void init(void);
void setup(void);
void loop(void);

void pinMode(byte, byte);
void analogInput(byte ANx);
//
unsigned int analogRead(byte);
void analogReference(byte mode);
void analogWrite(byte, int);
//
char digitalRead(byte);
void digitalWrite(byte, byte);

unsigned int eeprom_readw(unsigned char);
void eeprom_writew(unsigned char, unsigned int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned int);
void delayMicroseconds(unsigned int us);
//unsigned long pulseIn(byte pin, byte state, unsigned long timeout);
unsigned long pulseIn(byte pin, byte state,...);
void timerClr(byte);
void timerSet(byte, unsigned long);
byte timerUp(byte);
byte timerBusy(byte);

// USER MULTI TASK
// TASK VARIABLE
extern byte Task1,Task2,Task3,Task4;
extern byte Task5,Task6,Task7,Task8;


// EXTERNAL FUNCTION
extern void Tasking1();
extern void Tasking2();
extern void Tasking3();
extern void Tasking4();
extern void Tasking5();
extern void Tasking6();
extern void Tasking7();
extern void Tasking8();

void keypadInit();
void keypad_begin34(byte col1, byte col2, byte col3, byte row1, byte row2, byte row3, byte row4);
void keypad_begin44(byte col1, byte col2, byte col3, byte col4, byte row1, byte row2, byte row3, byte row4);
byte keypadRead();

// LCD
void lcd_clear(void);
void lcd_home(void);
void lcd_setCursor(byte, byte);
void lcd_cursor(void);
void lcd_noCursor(void);
void lcd_blink(void);
void lcd_noBlink(void);
void lcd_display(void);
void lcd_noDisplay(void);
void lcd_line1(byte);
void lcd_line2(byte);
//
void lcd_putc(byte);				// print a character
void lcd_putb(byte);				// print 8-bit binary
void lcd_puth(byte);				// print hexa-decimal
void lcd_puti(uint16_t);			// print sign integer
void lcd_print(const char *data);	// print constant string
void lcd_write(char *data);			// print buffer string
//
void lcd_init4(byte rs, byte en, byte d4, byte d5, byte d6, byte d7);
void lcd_init8(byte rs, byte en, byte d0, byte d1, byte d2, byte d3, byte d4, byte d5, byte d6, byte d7);
void lcd_i2c(void);
void e_pulse(byte k);
void lcd_begin(byte cols, byte rows);

// UART1
extern byte uart1_rx_ready;
extern byte uart1_lf_cnt;
extern byte uart1_cr_cnt;
extern byte uart1_rxbuf[];
void Serial_begin(unsigned long);
void Serial_end(void);
void Serial_putc(byte d);
void Serial_crlf(void);
void Serial_puth(byte);
void Serial_puti(unsigned int);
void Serial_print(const char *);
void Serial_println(const char *);
void Serial_write(char *);
void Serial_writeln(char *);
byte Serial_read(void);
byte Serial_ready(void);
void Serial_clear(void);
void Serial_readBytes(byte *, byte);
void Serial_readString(byte *, byte);
byte Serial_available(void);
// UART2
extern byte uart2_rx_ready;
extern byte uart2_lf_cnt;
extern byte uart2_cr_cnt;
extern byte uart2_rxbuf[];
void Serial2_begin(unsigned long);
byte Serial2_available(void);
void Serial2_end(void);
void Serial2_putc(byte d);
void Serial2_crlf(void);
void Serial2_puth(byte);
void Serial2_puti(unsigned int);
void Serial2_print(const char *);
void Serial2_println(const char *);
void Serial2_write(char *);
void Serial2_writeln(char *);
byte Serial2_read(void);
byte Serial2_ready(void);
void Serial2_clear(void);
void Serial2_readBytes(byte *, byte);
void Serial2_readString(byte *, byte);

// SPI
void SPI_begin(byte);
byte SPI_transfer(byte);

// I2C

void I2C_begin(void);
void I2C_start(void);
void I2C_stop(void);
unsigned char I2C_out(unsigned char);
unsigned char I2C_inp(unsigned char);

void i2c_init(unsigned long c);
byte b_i2c_error_flag(void);
void i2c_out(byte slave_address,byte data);
byte i2c_read(byte slave_address,byte register_add);
void i2c_write(byte slave_address,byte register_add,byte data);
void i2c_idle(void);

// PWM
void pwm_on(void);
void pwm_off(void);
void pwm_out(unsigned char ch,unsigned int dutyValue);

//ORIGINAL START HERE
//void pwm_on(byte PWMch);
//void pwm_Off(byte PWMch);
//void pwm_out(byte PWMch, byte Ratio);
//ORIGINAL END HERE

// STRING
unsigned char InStr(unsigned char, unsigned char *, const char *);
unsigned char FindChar(unsigned char, unsigned char *, unsigned char);
unsigned int StrToUInt(unsigned char *);

//Interrupt Service Routine
void attachInterrupt(byte interruptSource, void (* InterruptHandler)(void),byte mode); 
void detachInterrupt(byte interruptSource);
void INT0_ISR(void);
void INT0_CallBack(void);
void INT0_SetInterruptHandler(void (* InterruptHandler)(void));
extern void (*INT0_InterruptHandler)(void);
void INT1_ISR(void);
void INT1_CallBack(void);
void INT1_SetInterruptHandler(void (* InterruptHandler)(void));
extern void (*INT1_InterruptHandler)(void);
void INT2_ISR(void);
void INT2_CallBack(void);
void INT2_SetInterruptHandler(void (* InterruptHandler)(void));
extern void (*INT2_InterruptHandler)(void);


#endif // PICUNO_H
