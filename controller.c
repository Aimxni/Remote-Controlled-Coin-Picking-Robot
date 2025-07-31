#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/

 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Turn off secondary oscillator on A4 and B4

// Defines
#define SYSCLK 40000000L
#define FREQ 100000L // We need the ISR for timer 1 every 10 us
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define BUTTON_PORT PORTBbits.RB6  // Button connected to pin 15 (RB6)
#define BUTTON_MODE PORTBbits.RB10  // Button connected to pin 04 (RB0)
  
volatile int ISR_pwm1=0, ISR_pwm2=150, ISR_cnt=0;
//ISR_pwm1= 21

// The Interrupt Service Routine for timer 1 is used to generate one or more standard
// hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
// between 0.6ms and 2.4ms.

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0

	ISR_cnt++;
	if(ISR_cnt==ISR_pwm1)
	{
		LATAbits.LATA1 = 0;
	}
	if(ISR_cnt==ISR_pwm2)
	{
//		LATBbits.LATB4 = 0;
	}
	if(ISR_cnt>=42)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		LATAbits.LATA1 = 1;
//		LATBbits.LATB4 = 1;
	}
}

void SetupTimer1 (void)
{
	// Explanation here: https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/8)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

void uart_puts(char * s)
{
	while(*s)
	{
		putchar(*s);
		s++;
	}
}

char HexDigit[]="0123456789ABCDEF";

void PrintNumber(long int val, int Base, int digits)
{ 
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	uart_puts(&buff[j+1]);
}

// Good information about ADC in PIC32 found here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/adc
void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET=0x8000;      // Enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;        // Begin sampling
    while(AD1CON1bits.SAMP);     // wait until acquisition is done
    while(!AD1CON1bits.DONE);    // wait until conversion done
 
    return ADC1BUF0;             // result stored in ADC1BUF0
}


void ConfigurePins(void)
{
    // Configure pins as analog inputs
    ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
    TRISBbits.TRISB2 = 1;   // set RB2 as an input
    ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
    TRISBbits.TRISB3 = 1;   // set RB3 as an input
    
    ////////////////
    //PWM SPEAKER //
    ////////////////
    TRISAbits.TRISA1 = 0;  // Set RA1 as an output
    ANSELAbits.ANSA1 = 0;  // Ensure digital function (disable analog)
	
	//////////////////
	//TOGGLE BUTTONS /
	//////////////////
	//ANSELBbits.ANSB6 = 1;	
	TRISBbits.TRISB6 = 1; // Set RB6 as input (button)
	
	//ANSELAbits.ANSB10 = 0;
	TRISBbits.TRISB10 = 1; // Set RA00 as input
	//LATAbits.LATA0 = 1   // Set high
	CNPUBbits.CNPUB10 = 1;   // Enable pull-up resistor for RB6
		
	// Configure digital input pin to measure signal period
	ANSELB &= ~(1<<5); // Set RB5 as a digital I/O (pin 14 of DIP28)
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
    
    // We can do the three lines above using this instead:
    // ANSELBbits.ANSELB5=0;  Not needed because RB5 can not be analog input?
    // TRISBbits.TRISB5=1;
    // CNPUBbits.CNPUB5=1;
    
    // Configure output pins
	//TRISAbits.TRISA0 = 0; // pin  2 of DIP28
	TRISAbits.TRISA1 = 0; // pin  3 of DIP28
	TRISBbits.TRISB0 = 0; // pin  4 of DIP28
	TRISBbits.TRISB1 = 0; // pin  5 of DIP28
	TRISAbits.TRISA2 = 0; // pin  9 of DIP28
	//TRISAbits.TRISA3 = 0; // pin 10 of DIP28
	//TRISBbits.TRISB4 = 0; // pin 11 of DIP28
	INTCONbits.MVEC = 1;
}

void PrintFixedPoint (unsigned long number, int decimals)
{
	int divider=1, j;
	
	j=decimals;
	while(j--) divider*=10;
	
	PrintNumber(number/divider, 10, 1);
	uart_puts(".");
	PrintNumber(number%divider, 10, decimals);
}

//////////////////////////////////////////////////////////////////
//FUNCTIONS NEEDED FOR THE COMMUNICATION USING JD40 RADIO SIGNAL//
//////////////////////////////////////////////////////////////////

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = c;          // echo
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

/////////////////////////////////////////////////////////
// UART1 functions used to communicate with the JDY40  //
/////////////////////////////////////////////////////////

// TXD1 is in pin 26
// RXD1 is in pin 24

int UART1Configure(int desired_baud)
{
	int actual_baud;

    // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
    // 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2

	// Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
    // priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
    // TRISx register must also be configured for input (set to ?1?)."
    
    ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB13 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
    U1RXRbits.U1RXR = 3; // SET U1RX to RB13

    // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
    // RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7

    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
	
    U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U1STA = 0x1400;     // enable TX and RX
    U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    actual_baud = SYSCLK / (16 * (U1BRG+1));

    U1MODESET = 0x8000;     // enable UART1

    return actual_baud;
}

void putc1 (char c)
{
	while( U1STAbits.UTXBF);   // wait while TX buffer full
	U1TXREG = c;               // send single character to transmit buffer
}
 
int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U1STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

// Copied from here: https://forum.microchip.com/s/topic/a5C3l000000MRVAEA4/t335776
void delayus(uint16_t uiuSec)
{
    uint32_t ulEnd, ulStart;
    ulStart = _CP0_GET_COUNT();
    ulEnd = ulStart + (SYSCLK / 2000000) * uiuSec;
    if(ulEnd > ulStart)
        while(_CP0_GET_COUNT() < ulEnd);
    else
        while((_CP0_GET_COUNT() > ulStart) || (_CP0_GET_COUNT() < ulEnd));
}

unsigned int SerialReceive1_timeout(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
    int timeout_cnt;
 
    while(num_char < max_size)
    {
    	timeout_cnt=0;
    	while(1)
    	{
	        if(U1STAbits.URXDA) // check if data is available in RX buffer
	        {
	        	*buffer = U1RXREG; // copy RX buffer into *buffer pointer
	        	break;
	        }
	        if(++timeout_cnt==100) // 100 * 100us = 10 ms
	        {
	        	*buffer = '\n';
	        	break;
	        }
	        delayus(100);
	    }
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}


void delayms(int len)
{
	while(len--) wait_1ms();
}

void ClearFIFO (void)
{
	unsigned char c;
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	while(U1STAbits.URXDA) c=U1RXREG;
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1(s);
	U1STA = 0x1400;     // enable TX and RX, clear FIFO
	SerialReceive1(buff, sizeof(buff)-1);
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	delayms(10);
	printf("Response: %s\n", buff);
}

void ReceptionOff (void)
{
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1("AT+DVID0000\r\n"); // Some unused id, so that we get nothing.
	delayms(10);
	ClearFIFO();
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
}


 void communicateWithSlave_test(void) 
 {
    char buff[10];  // Buffer for messages
    static int cont1 = 0, cont2 = 0;
    int timeout_cnt = 0;

    // Construct a test message
    sprintf(buff, "%03d,%03d\n", cont1, cont2);

    putc1('!'); // Send attention character
    delayms(5); // Give slave time to get ready

    SerialTransmit1(buff); // Send test message

    // Increment counters
    if (++cont1 > 200) cont1 = 0;
    if (++cont2 > 200) cont2 = 0;

    delayms(5); // Allow processing time for the slave

    // Clear receive FIFO
    if (U1STAbits.URXDA) {
        SerialReceive1_timeout(buff, sizeof(buff) - 1);
    }

    putc1('@'); // Request message from the slave

    timeout_cnt = 0;
    while (1) {
        if (U1STAbits.URXDA) break; // Data received
        if (++timeout_cnt > 250) break; // Timeout after 25ms
        delayus(100); // Small delay
    }

    if (U1STAbits.URXDA) {
        SerialReceive1_timeout(buff, sizeof(buff) - 1);
        if (strlen(buff) == 5) {
            printf("Slave says: %s\r\n", buff);
        } else {
            printf("*** BAD MESSAGE ***: %s\r\n", buff);
        }
    } else {
        printf("NO RESPONSE\r\n");
    }

    delayms(50); // Control communication frequency
}

void sendDataToSlave_listen(const char *data) 
{
    char buff[10];  // Buffer for received data
    int timeout_cnt = 0;

    putc1('!'); // Send attention character
    delayms(5); // Give slave time to get ready

    SerialTransmit1(data); // Send the provided data

    delayms(5); // Allow processing time for the slave

    // ?? Clear receive FIFO to remove old data
    if (U1STAbits.URXDA) {
        SerialReceive1_timeout(buff, sizeof(buff) - 1);
    }

    putc1('@'); // Request message from the slave

    // ?? Wait for response with timeout
    timeout_cnt = 0;
    while (1) {
        if (U1STAbits.URXDA) break;  // If data arrives, exit loop
        if (++timeout_cnt > 250) break; // Timeout after 25ms
        delayus(100); // Small delay for stability
    }

    // ?? Handle slave response
    if (U1STAbits.URXDA) {
        SerialReceive1_timeout(buff, sizeof(buff) - 1);
        if (strlen(buff) == 5) {
            printf("Slave says: %s\r\n", buff); // Valid response
        } else {
            printf("*** BAD MESSAGE ***: %s\r\n", buff); // Malformed response
        }
    } else {
        printf("NO RESPONSE\r\n"); // Slave did not respond
    }

    delayms(50); // Control communication frequency
}

void sendDataToSlave(const char *data) 
 
 {
  putc1('!'); // Send attention character  
  delayms(5); // Give slave time to get ready 
  SerialTransmit1(data); // Send the provided data
  delayms(5); // Allow processing time for the slave
 }

// In order to keep this as nimble as possible, avoid
// using floating point or printf() on any of its forms!
void main(void)
{

	/////////////////
	//MODE VARIABLE//
	/////////////////
	
	char mode = 0;
	char coins = 0;
    /////////////////////////////////////////////////////////
    //MASTER-SLAVE (BEST NAME EVER) COMMUNICATION VARIABLES//
    /////////////////////////////////////////////////////////
    
    int timeout_cnt=0;
    int cont1=0, cont2=100;
   
    
    //////////////////////
    //JOYSTICK VARIABLES//
    //////////////////////
	volatile unsigned long t=0;
    int adcval_x;
    int adcval_y;
    long int v;
    char slave_movement; 
    
     
    char buffer[16];
    char buff[80];
    
    int f;
	//unsigned long int count, f;
	//unsigned char LED_toggle=0;

   
    DDPCON = 0;
	CFGCON = 0;
    
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    UART1Configure(9600);    //Configure UART1 to communicate with JDY40 with a baud rate of 9600
    
    delayms(500); // Give putty time to start before we send stuff.
    printf("JDY40 test program. PIC32 behaving as Master.\r\n");

    //////////////////////////////
    //PIN CONFIG FOR JDY40 (RB14)/
    //////////////////////////////
     
	// RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB &= ~(1<<14);  // configure pin RB14 as output
	LATB |= (1<<14);    // 'SET' pin of JDY40 to 1 is normal operation mode

	ReceptionOff();

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVIDF4D3\r\n");
	SendATCommand("AT+RFC111\r\n");
    
    //////////////////////////////////
    //ADC CONFIGURATION FOR JOYSTICK//
    //////////////////////////////////
    
    ConfigurePins();
    SetupTimer1();
  
    ADCConf(); // Configure ADC
    LCD_4BIT();

    waitms(500); // Give PuTTY time to start
	//uart_puts("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	//uart_puts("\r\nPIC32 multi I/O example.\r\n");
	//uart_puts("Measures the voltage at channels 4 and 5 (pins 6 and 7 of DIP28 package)\r\n");
	//uart_puts("Measures period on RB5 (pin 14 of DIP28 package)\r\n");
	//uart_puts("Toggles RA0, RA1, RB0, RB1, RA2 (pins 2, 3, 4, 5, 9, of DIP28 package)\r\n");
	//uart_puts("Generates Servo PWM signals at RA3, RB4 (pins 10, 11 of DIP28 package)\r\n\r\n");
	
	//LCDprint("Ref: 61.2740KHz",1,1);
	
	while(1)
	{
	    
	    LCDprint("Ref: 61.2740KHz",1,1);
	    /*
		// Change the servo PWM signals
		if (ISR_pwm1<200)
		{
			ISR_pwm1++;
		}
		else
		{
			ISR_pwm1=100;	
		}

		if (ISR_pwm2>100)
		{
			ISR_pwm2--;
		}
		else
		{
			ISR_pwm2=200;	
		}

		waitms(200);
		*/
		
	    //sprintf(buff, "%03d,%03d\n", cont1, cont2); // Construct a test message
	    
	    
		//putc1('!'); // Send a message to the slave. First send the 'attention' character which is '!'
		// Wait a bit so the slave has a chance to get ready
		//delayms(5); // This may need adjustment depending on how busy is the slave
		//SerialTransmit1(buff); // Send the test message
		
		//if(++cont1>200) cont1=0; // Increment test counters for next message
		//if(++cont2>200) cont2=0;
		
		//delayms(5); // This may need adjustment depending on how busy is the slave

		// Clear the receive 8-level FIFO of the PIC32MX, so we get a fresh reply from the slave
		//if(U1STAbits.URXDA) SerialReceive1_timeout(buff, sizeof(buff)-1);

	    //putc1('@'); // Request a message from the slave
		
		//putc1('F');
		
		//timeout_cnt=0;
		
		/*
		
		while(1)
		{
			if(U1STAbits.URXDA) break; // Something has arrived
			if(++timeout_cnt>250) break; // Wait up to 25ms for the repply
			delayus(100); // 100us*250=25ms
		}
		
		if(U1STAbits.URXDA) // Something has arrived from the slave
		{
			SerialReceive1_timeout(buff, sizeof(buff)-1); // Get the message from the slave
			if(strlen(buff)==5) // Check for valid message size
			{
				printf("Slave says: %s\r\n", buff);
			}
			else
			{
				printf("*** BAD MESSAGE ***: %s\r\n", buff);
			}
		}
		else // Timed out waiting for reply
		{
			printf("NO RESPONSE\r\n", buff);
		}
		
		delayms(50);  // Set the information interchange pace: communicate about every 50ms
		
		*/
		/////////////////////////////////////////
	    //READS THE X-MOVEMENTS OF THE JOYSTICK//
	    /////////////////////////////////////////
	    
    	adcval_x = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number	
		//uart_puts("ADC[4]=0x");
		//PrintNumber(adcval_x, 16, 3);
		//uart_puts(", V=");
		v=(adcval_x*3290L)/1023L; // 3.290 is VDD
		//PrintFixedPoint(v, 3);
		//uart_puts("V ");
		
		/////////////////////////////////////////
		//READS THE Y-MOVEMENTS OF THE JOYSTICK//
        /////////////////////////////////////////
			
		adcval_y=ADCRead(5);
		//uart_puts("ADC[5]=0x");
		//PrintNumber(adcval_y, 16, 3);
		//uart_puts(", V=");
		v=(adcval_y*3290L)/1023L; // 3.290 is VDD
		
		
		// x coordinate conditions
		
		if(adcval_x < 25 && adcval_y < 650 && adcval_y > 25)
		{
		//LCDprint("Right upd", 1, 1);
		//sprintf(buffer,"%d",adcval_x);
		//LCDprint(buffer,2,1);
		               //123456789012345678
		sendDataToSlave("100010001000100011");
		
		}
		else if (adcval_x > 650 && adcval_x < 1100 && adcval_y < 650 && adcval_y > 25) {
		//LCDprint("Left", 1, 1);
		sendDataToSlave("213021001100213001");
		
		}
		
		// y  coordinate conditions
		else if(adcval_y < 25 && adcval_x < 650 && adcval_x > 25)
		{
		LCDprint("Up", 1, 1);
		//sprintf(buffer,"%d",adcval_y);
		//LCDprint(buffer,2,1);
		               //123456789012345678
		sendDataToSlave("989898989898989898");
		
		}
		else if (adcval_y > 650 && adcval_y < 1100 && adcval_x < 650 && adcval_x > 25) {
		//LCDprint("Down", 1, 1);
		               //123456789012345678
		sendDataToSlave("767676767676767676");
		
		}
		else if (adcval_y < 650 && adcval_y > 25 && adcval_x < 650 && adcval_x > 25){
		//LCDprint("No movement",1,1);
		//sprintf(buffer,"%d",adcval_y);
		//LCDprint(buffer,2,1);
		               //123456789012345678
		sendDataToSlave("No movement No mov");               
		}
		
		///////////////////
		//TOGGLE BUTTONS //
		///////////////////
		
		if (BUTTON_PORT == 0) 
		{	
			waitms(25);   // Button debouncing time
			
			if (BUTTON_PORT == 0) 
			{
			
				coins++;
				
				LCDprint("COIN",2,1);
				    	           //123456789012345678
				sendDataToSlave("555555555555555555");
			
			}
		}
			
		// end of the output messages:
		// start of the input messages from the slave:
		
		if (BUTTON_MODE == 0)
		{
			waitms(25);
			
			if (BUTTON_MODE == 0) 
			{
			    
                mode = !mode;
                ISR_pwm1= 21;
                waitms(1000);
                ISR_pwm1= 0;
				LCDprint("MODE",2,1);
				
			    	           //123456789012345678
				sendDataToSlave("444444444444444444");
			   delayms(5);
			   
		       putc1('@'); // Send attention character  
		       delayms(5); // Give slave time to get ready 
		       SerialReceive1_timeout(buff, sizeof(buff) - 1);
		       delayms(5); // Allow processing time for the slave
		       f = atoi(buff);
		       LCDprint(buff,2,1);
		       //delayms(50);
		       
		       
		     

			}
			
			
		}
		
		
      	//sendDataToSlave("444444444444444444");
	    putc1('@'); // Send attention character  
	    delayms(5); // Give slave time to get ready 
	    SerialReceive1_timeout(buff, sizeof(buff) - 1);
		delayms(5); // Allow processing time for the slave
		f = atoi(buff);
		LCDprint(buff,2,1);
		       //delayms(50);
		       
       if(f > 61100)
		       {
		       ISR_pwm1 = 42;
		       }
		       
		       else if (f <59000)
		       
		       {
		       ISR_pwm1 = 15;
		       
		       } 
       
       

		/*
		while(1)
		{
			if(U1STAbits.URXDA) break; // Something has arrived
			if(++timeout_cnt>250) break; // Wait up to 25ms for the repply
			delayus(100); // 100us*250=25ms
		}
		
		if(U1STAbits.URXDA) // Something has arrived from the slave
		{
			SerialReceive1_timeout(buff, sizeof(buff)-1); // Get the message from the slave
			if(strlen(buff)==5) // adjust to match strlen given as a signal or change to matching words
			{
				printf("Slave says: %s\r\n", buff);
				// Speaker code below:
				// Insert:
				//    - PWM for speaker
				//    - Powering the speaker on
			}
			else
			{
				printf("*** BAD MESSAGE ***: %s\r\n", buff);
			}
		}
		else // Timed out waiting for reply
		{
			printf("NO RESPONSE\r\n", buff);
		}
	    */
	}
		
}
