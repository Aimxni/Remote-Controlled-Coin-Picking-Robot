//  EFM8LB1 with multiplexed 7-segment displays
//  Copyright (c) 2014-2018 Jesus Calvino-Fraga
//  ~C51~

#include <EFM8LB1.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200
#define SARCLK 18000000L

#define SEG_A P1_1
#define SEG_B P1_3
#define SEG_C P1_5
#define SEG_D P1_6
#define SEG_E P1_7
#define SEG_G P1_4
#define SEG_F P1_2
#define DP    P2_0

#define CA1   P1_0
#define CA2   P0_6
#define CA3   P0_7

//code unsigned char seven_seg[] = { 0b00100000, 0xF9, 0b01000100, 0b01010000, 0x99, 0x92, 0x82, 0xF8,
//                                   0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E };


code unsigned char seven_seg[] = { ~0b00100000, ~0xF9, ~0b01000100, ~0b01010000, ~0x99, ~0x92, ~0x82, ~0xF8,
								   ~0x80, ~0x90, ~0x88, ~0x83, ~0xC6, ~0xA1, ~0x86, ~0x8E };                                 
                              
                                   
volatile unsigned char ISR_state=0;
volatile unsigned char disp3, disp2, disp1;
unsigned char dp_pos=0;

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN |= 0x80;
	RSTSRC = 0x02;

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	// Configure the pins used for square output
	P0MDOUT|=0b_1101_0001; // Enable UART0 TX as push-pull output (P0.4) as well as P0.6 and P0.7
	P1MDOUT|=0b_1111_1111; 
	P2MDOUT|=0b_0000_0001;
	
	
	//P2MDOUT &= ~(1 << 2); // Set P2.2 as open-drain (input)
	//P2MDOUT &= ~(1 << 3); // Set P2.3 as input (open-drain)
	
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00; // Enable T0 on P0.0
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	// Initialize timer 4 for periodic interrupts
	SFRPAGE=0x10;
	TMR4CN0=0x00;
	TMR4RL=0x10000L-(SYSCLK)/(1000L*12L); // 1 miliseconds interrupt
	TMR4=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_0100; // Enable Timer4 interrupts
	TR4=1;         // Start Timer4 (TMR4CN0 is bit addressable)
	
	EA=1;
	
	SFRPAGE=0x00;
	
	return 0;
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}


unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

void Load_Segments(unsigned char todisp)
{
	ACC=todisp;
	SEG_A=ACC_0;
	SEG_B=ACC_1;
	SEG_C=ACC_2;
	SEG_D=ACC_3;
	SEG_E=ACC_4;
	SEG_F=ACC_5;
	SEG_G=ACC_6;
	DP=ACC_7;
}

void Timer4_ISR (void) interrupt INTERRUPT_TIMER4
{
	SFRPAGE=0x10;
	TF4H = 0; // Clear Timer4 interrupt flag
	
	//CA3=1;
	CA2=1;
	CA1=1;
	Load_Segments(0xff);

	switch(ISR_state)
	{
		case 0:
		    Load_Segments(disp3);
			//CA3=0;
			ISR_state=1;
		break;
		case 1:
		    Load_Segments(disp2);
			CA2=0;
			ISR_state=2;
		break;
		case 2:
		    Load_Segments(disp1);
			CA1=0;
			ISR_state=0;
		break;
		default:
			ISR_state=0;
		break;
	}
}

void Send_7Seg (unsigned int x)
{
	disp3=seven_seg[x/100];
	disp2=seven_seg[(x/10)%10];
	disp1=seven_seg[x%10];
	
	switch(dp_pos)
	{
		case 0:
			disp1&=0b_0111_1111; // Turn on the decimal point of the first digit
			dp_pos=1;
			break;
		case 1:
			disp2&=0b_0111_1111; // Turn on the decimal point of the second digit
			dp_pos=2;
			break;
		case 2:
			disp3&=0b_0111_1111; // Turn on the decimal point of the third digit
			dp_pos=0;
			break;
		default:
		    dp_pos=0;
		    break;
	}
}






void main (void)
{
	long int j1, v1;
	unsigned int j=0;
	
	
	//InitPinADC(0,7);
	//InitADC();
	
	
	waitms(500); // Give PuTTY a chance to start
	
	printf("\n\nEFM8 multiplexed 7-segment displays test.\n");
	
	//P0_7 = 0;

	
	while(1)
	{
        //j1 = ADC_at_Pin(QFP32_MUX_P0_7);
        //v1 = (j1 * 33000L) / 0x3FFF; // in mV
        
        
        if (P2_4 == 0) {
		j = 0;
		}
	
		if (j > 20) {
		j = 0;
		}
        

        Send_7Seg(j);

        if (P0_7 == 0)
        {
        j++;
        
        while (P0_7 == 0);
       
        }
    }
}
