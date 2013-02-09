#include <string.h>
#include "LPC11Uxx.h"
#include "DataTypes.h"

#define NO_OF_SERVOS		24
//#define SYS_TICK_RATE 		SystemCoreClock / 100000
//#define TICKS_PER_PERIOD	SYS_TICK_RATE / (SystemCoreClock / 50)
//#define TICKS_PER_PERIOD	2000


ui8 ServoVal[ NO_OF_SERVOS ];
unsigned int ServoCntDown[ NO_OF_SERVOS ];
unsigned int ServoPort[ NO_OF_SERVOS ];
unsigned int ServoPin[ NO_OF_SERVOS ];
unsigned int ServoPinMask[ NO_OF_SERVOS ];

/*
void SysTick_Handler ( void )
{

}
*/

void TIMER32_1_IRQHandler ( void )
{
	LPC_CT32B1->IR = 0x3F;
}

void TIMER32_0_IRQHandler ( void )
{
	LPC_CT32B0->IR = 0x3F;
}

uint8_t count = 0;

/*
void TIMER32_1_IRQHandler ( void )
{
	uint8_t i = 0;

	if ( LPC_CT32B1->IR & (0x01<<0) )
	{
		// clear interrupt flag
		LPC_CT32B1->IR = 0x1<<0;
		// Enable TMR32_0, set it to go off in 1ms
		tmr32b0Start(SystemCoreClock/1000);

		for (i=0; i<NO_OF_SERVOS; i++)
		{
			if(count > ServoVal[i])
			{
				LPC_GPIO->SET[ServoPort[i]] = ServoPinMask[i];
			}
		}
	}

	LPC_GPIO->NOT[1] = BIT22;
}


void TIMER32_0_IRQHandler ( void )
{
uint8_t i = 0;

	// clear all interrupt flags
	LPC_CT32B0->IR = 0xFF;

	// Next interrupt
	LPC_CT32B0->MR0 = 188;

	if(count > ServoVal[0]) LPC_GPIO->CLR[ServoPort[0]] = ServoPinMask[0];
	if(count > ServoVal[1]) LPC_GPIO->CLR[ServoPort[1]] = ServoPinMask[1];
	if(count > ServoVal[2]) LPC_GPIO->CLR[ServoPort[2]] = ServoPinMask[2];
	if(count > ServoVal[3]) LPC_GPIO->CLR[ServoPort[3]] = ServoPinMask[3];
	if(count > ServoVal[4]) LPC_GPIO->CLR[ServoPort[4]] = ServoPinMask[4];

	if(count > ServoVal[5]) LPC_GPIO->CLR[ServoPort[5]] = ServoPinMask[5];
	if(count > ServoVal[6]) LPC_GPIO->CLR[ServoPort[6]] = ServoPinMask[6];
	if(count > ServoVal[7]) LPC_GPIO->CLR[ServoPort[7]] = ServoPinMask[7];
	if(count > ServoVal[8]) LPC_GPIO->CLR[ServoPort[8]] = ServoPinMask[8];
	if(count > ServoVal[9]) LPC_GPIO->CLR[ServoPort[9]] = ServoPinMask[9];

	if(count > ServoVal[10]) LPC_GPIO->CLR[ServoPort[10]] = ServoPinMask[10];
	if(count > ServoVal[11]) LPC_GPIO->CLR[ServoPort[11]] = ServoPinMask[11];
	if(count > ServoVal[12]) LPC_GPIO->CLR[ServoPort[12]] = ServoPinMask[12];
	if(count > ServoVal[13]) LPC_GPIO->CLR[ServoPort[13]] = ServoPinMask[13];
	if(count > ServoVal[14]) LPC_GPIO->CLR[ServoPort[14]] = ServoPinMask[14];

	if(count > ServoVal[15]) LPC_GPIO->CLR[ServoPort[15]] = ServoPinMask[15];
	if(count > ServoVal[16]) LPC_GPIO->CLR[ServoPort[16]] = ServoPinMask[16];
	if(count > ServoVal[17]) LPC_GPIO->CLR[ServoPort[17]] = ServoPinMask[17];
	if(count > ServoVal[18]) LPC_GPIO->CLR[ServoPort[18]] = ServoPinMask[18];
	if(count > ServoVal[19]) LPC_GPIO->CLR[ServoPort[19]] = ServoPinMask[19];

	if(count > ServoVal[20]) LPC_GPIO->CLR[ServoPort[20]] = ServoPinMask[20];
	if(count > ServoVal[21]) LPC_GPIO->CLR[ServoPort[21]] = ServoPinMask[21];
	if(count > ServoVal[22]) LPC_GPIO->CLR[ServoPort[22]] = ServoPinMask[22];
	if(count > ServoVal[23]) LPC_GPIO->CLR[ServoPort[23]] = ServoPinMask[23];

	count++;

	if(count == 0)
	{
		//turn off this interrupt, wait for the timer2 interrupt to set you going again
		tmr32b0Stop();
		// clear interrupt flag
		LPC_CT32B0->IR = 0x1<<0;
	}
	else
	{

	}


	LPC_GPIO->NOT[1] = BIT21;
}
*/



void ServoInit ( void )
{
unsigned int i = 0;

	// PIO0_5
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 5;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	// PIO0_4
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 4;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	// PIO1_20
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 20;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	// PIO1_27
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 27;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	// PIO1_26
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 26;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_20
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 20;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_19
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 19;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_25
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 25;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_16
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 16;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_19
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 19;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_18
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 18;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_17
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 17;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_15
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 15;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_16
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 16;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_22
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 22;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_14
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 14;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_13
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 13;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_14
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 14;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_13
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 13;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_12
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 12;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_11
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 11;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_29
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 29;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO0_22
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 0;
	ServoPin[ i ] = 22;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	//PIO1_21
	ServoVal[ i ] = 128;
	ServoPort[ i ] = 1;
	ServoPin[ i ] = 21;
	ServoPinMask[ i ] = 1 << ServoPin[ i ];
	i++;

	// Turn off I2C
	LPC_IOCON->PIO0_5 = 0 | (1<<8);
	LPC_IOCON->PIO0_4 = 0 | (1<<8);

	// Enable open drain
	LPC_IOCON->PIO1_20 |= (1<<10);
	LPC_IOCON->PIO1_27 |= (1<<10);
	LPC_IOCON->PIO1_26 |= (1<<10);
	LPC_IOCON->PIO0_20 |= (1<<10);
	LPC_IOCON->PIO1_19 |= (1<<10);
	LPC_IOCON->PIO1_25 |= (1<<10);
	LPC_IOCON->PIO1_16 |= (1<<10);
	LPC_IOCON->PIO0_19 |= (1<<10);
	LPC_IOCON->PIO0_18 |= (1<<10);
	LPC_IOCON->PIO0_17 |= (1<<10);
	LPC_IOCON->PIO1_15 |= (1<<10);
	LPC_IOCON->PIO0_16 |= (1<<10);
	LPC_IOCON->PIO1_22 |= (1<<10);
	LPC_IOCON->PIO1_14 |= (1<<10);
	LPC_IOCON->PIO1_13 |= (1<<10);
	LPC_IOCON->TRST_PIO0_14 |= (1<<10);
	LPC_IOCON->TDO_PIO0_13 |= (1<<10);
	LPC_IOCON->TMS_PIO0_12 |= (1<<10);
	LPC_IOCON->TDI_PIO0_11 |= (1<<10);
	LPC_IOCON->PIO1_29 |= (1<<10);
	LPC_IOCON->PIO0_22 |= (1<<7);
	LPC_IOCON->PIO1_21 |= (1<<10);

	GPIOSetDir(1, 21, 1);

	for (i=0; i<NO_OF_SERVOS; i++)
	{
		GPIOSetDir( ServoPort[i], ServoPin[i], 1 );
		GPIOSetBitValue( ServoPort[i], ServoPin[i], 0 );
	}
}

void tmr32b0Stop ( void )
{
	LPC_CT32B0->TCR = 0;
}

void tmr32b1Stop ( void )
{
	LPC_CT32B1->TCR = 0;
}

void tmr32b0Start ( uint32_t matchVal )
{
	LPC_CT32B0->MR0 = matchVal;
	LPC_CT32B0->TCR = 1;
}

void tmr32b1Start ( uint32_t matchVal )
{
	LPC_CT32B1->MR0 = matchVal;
	LPC_CT32B1->TCR = 1;
}

void initTimers ( void )
{
	// Enable the clock
	// CT32B0
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);
	// CT32B1
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10);

	// reset timer
	LPC_CT32B0->TCR = 0x02;
	LPC_CT32B1->TCR = 0x02;

	//LPC_CT32B0->MR0 = TimerInterval;
	//LPC_CT32B1->MR0 = TimerInterval;

	// No GPIO toggling on match
	LPC_CT32B0->EMR = 0;
	LPC_CT32B1->EMR = 0;

	// Interrupt and Reset on MR0
	LPC_CT32B0->MCR = 3;
	LPC_CT32B1->MCR = 3;

	//NVIC_EnableIRQ(TIMER_32_0_IRQn);
	//NVIC_EnableIRQ(TIMER_32_1_IRQn);

	// Go
	//LPC_CT32B0->TCR = 1;
	//LPC_CT32B1->TCR = 1;
}

#define SERVO_STEPS	256

bool servicesServos = FALSE;
ui32 servoStep[2][SERVO_STEPS];
ui32 ServoClearAll[2] = {0,0};


void UpdateServoPos ( void )
{
ui16 i = 0;

	for (i=0; i<SERVO_STEPS; i++)
	{
		servoStep[0][i] = 0;
		servoStep[1][i] = 0;

		if(i > ServoVal[0]) servoStep[ServoPort[0]][i] |= ServoPinMask[0];
		if(i > ServoVal[1]) servoStep[ServoPort[1]][i] |= ServoPinMask[1];
		if(i > ServoVal[2]) servoStep[ServoPort[2]][i] |= ServoPinMask[2];
		if(i > ServoVal[3]) servoStep[ServoPort[3]][i] |= ServoPinMask[3];
		if(i > ServoVal[4]) servoStep[ServoPort[4]][i] |= ServoPinMask[4];

		if(i > ServoVal[5]) servoStep[ServoPort[5]][i] |= ServoPinMask[5];
		if(i > ServoVal[6]) servoStep[ServoPort[6]][i] |= ServoPinMask[6];
		if(i > ServoVal[7]) servoStep[ServoPort[7]][i] |= ServoPinMask[7];
		if(i > ServoVal[8]) servoStep[ServoPort[8]][i] |= ServoPinMask[8];
		if(i > ServoVal[9]) servoStep[ServoPort[9]][i] |= ServoPinMask[9];

		if(i > ServoVal[10]) servoStep[ServoPort[10]][i] |= ServoPinMask[10];
		if(i > ServoVal[11]) servoStep[ServoPort[11]][i] |= ServoPinMask[11];
		if(i > ServoVal[12]) servoStep[ServoPort[12]][i] |= ServoPinMask[12];
		if(i > ServoVal[13]) servoStep[ServoPort[13]][i] |= ServoPinMask[13];
		if(i > ServoVal[14]) servoStep[ServoPort[14]][i] |= ServoPinMask[14];

		if(i > ServoVal[15]) servoStep[ServoPort[15]][i] |= ServoPinMask[15];
		if(i > ServoVal[16]) servoStep[ServoPort[16]][i] |= ServoPinMask[16];
		if(i > ServoVal[17]) servoStep[ServoPort[17]][i] |= ServoPinMask[17];
		if(i > ServoVal[18]) servoStep[ServoPort[18]][i] |= ServoPinMask[18];
		if(i > ServoVal[19]) servoStep[ServoPort[19]][i] |= ServoPinMask[19];

		if(i > ServoVal[20]) servoStep[ServoPort[20]][i] |= ServoPinMask[20];
		if(i > ServoVal[21]) servoStep[ServoPort[21]][i] |= ServoPinMask[21];
		if(i > ServoVal[22]) servoStep[ServoPort[22]][i] |= ServoPinMask[22];
		if(i > ServoVal[23]) servoStep[ServoPort[23]][i] |= ServoPinMask[23];
	}

	ServoClearAll[0] = 0;
	ServoClearAll[1] = 0;
	ServoClearAll[ServoPort[0]] |= ServoPinMask[0];
	ServoClearAll[ServoPort[1]] |= ServoPinMask[1];
	ServoClearAll[ServoPort[2]] |= ServoPinMask[2];
	ServoClearAll[ServoPort[3]] |= ServoPinMask[3];
	ServoClearAll[ServoPort[4]] |= ServoPinMask[4];

	ServoClearAll[ServoPort[5]] |= ServoPinMask[5];
	ServoClearAll[ServoPort[6]] |= ServoPinMask[6];
	ServoClearAll[ServoPort[7]] |= ServoPinMask[7];
	ServoClearAll[ServoPort[8]] |= ServoPinMask[8];
	ServoClearAll[ServoPort[9]] |= ServoPinMask[9];

	ServoClearAll[ServoPort[10]] |= ServoPinMask[10];
	ServoClearAll[ServoPort[11]] |= ServoPinMask[11];
	ServoClearAll[ServoPort[12]] |= ServoPinMask[12];
	ServoClearAll[ServoPort[13]] |= ServoPinMask[13];
	ServoClearAll[ServoPort[14]] |= ServoPinMask[14];

	ServoClearAll[ServoPort[15]] |= ServoPinMask[15];
	ServoClearAll[ServoPort[16]] |= ServoPinMask[16];
	ServoClearAll[ServoPort[17]] |= ServoPinMask[17];
	ServoClearAll[ServoPort[18]] |= ServoPinMask[18];
	ServoClearAll[ServoPort[19]] |= ServoPinMask[19];

	ServoClearAll[ServoPort[20]] |= ServoPinMask[20];
	ServoClearAll[ServoPort[21]] |= ServoPinMask[21];
	ServoClearAll[ServoPort[22]] |= ServoPinMask[22];
	ServoClearAll[ServoPort[23]] |= ServoPinMask[23];

}

extern ui16 SpiRecvBuffer[];

int main ( void )
{
static volatile uint32_t idle = 0;
static ui8 moveServoCnt = 0;
volatile uint16_t i = 0;
volatile ui8 *NewServoValues;

	SystemCoreClockUpdate();
	LowPowerInit();
	//USB_CdcInit();
	ServoInit();

	// Initialise the SSP port
	SpiSlave_Init();

	// PIO0_23 LED
	GPIOSetDir(0, 23, 1);
	GPIOSetBitValue(0, 23, 0);

	//SysTick_Config(SystemCoreClock / 1000000);

	initTimers();

	LPC_SYSCON->PDRUNCFG = 0;

	tmr32b1Start(SystemCoreClock / 50);

	//__WFI();

	UpdateServoPos();

	while (! LPC_CT32B1->IR);

	while (1)
	{
		// Clear the IF for the 50hz timer
		LPC_CT32B1->IR = 0x3F;

		// Set all servos
		for (i=0; i<NO_OF_SERVOS; i++)
		{
			LPC_GPIO->SET[ServoPort[i]] = ServoPinMask[i];
		}

		// Wait for 1ms
		LPC_CT32B0->MR0 = 48000;
		LPC_CT32B0->IR = 0x3F;
		// Hold in rest and then run
		LPC_CT32B0->TCR = 2;
		LPC_CT32B0->TCR = 1;
		//__WFI();
		// Test Toggle
		//LPC_GPIO->NOT[1] = BIT21;
		while (! LPC_CT32B0->IR);

		// Configure match, Reset and start the timer
		LPC_CT32B0->IR = 0x3F;
		// Hold in rest and then run
		LPC_CT32B0->TCR = 2;
		LPC_CT32B0->MR0 = 188;
		LPC_CT32B0->TCR = 1;

		// Test Toggle
		//LPC_GPIO->NOT[1] = BIT21;

		servicesServos = TRUE;

		while (servicesServos)
		{
			// Set the port up for this step
			LPC_GPIO->CLR[0] = servoStep[0][count];
			LPC_GPIO->CLR[1] = servoStep[1][count];

			count++;
			while (! LPC_CT32B0->IR);
			LPC_CT32B0->IR = 0x3F;

			if (0 == count) servicesServos = FALSE;
		}
		LPC_GPIO->CLR[0] = ServoClearAll[0];
		LPC_GPIO->CLR[1] = ServoClearAll[1];

		//LPC_GPIO->NOT[1] = BIT21;

		// Stop TMR0
		LPC_CT32B0->TCR = 0;

		// You now have 18ms to do you own thing, you must be asleep in time for the next round though
		// Dont stay up late!

		// Enable the interrupt
		SpiSlave_InterruptControl( true );

		if ( SpiSlave_GetServoUpdate( &ServoVal ))
		{
			// We got an update
			UpdateServoPos();
		}

		//__WFI();
		while (! LPC_CT32B1->IR);

		SpiSlave_InterruptControl( false );

	}

}
