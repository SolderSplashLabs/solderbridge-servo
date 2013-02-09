
#include "LPC11Uxx.h"
#include "DataTypes.h"
#include "SpiSlave.h"

#define SPI_MAX_MESSAGE_LEN 	20


#define SPI_START_BYTE 			'#'

#define SOLDERBRIDGE_TYPE		1
#define SOLDERBRIDGE_VER		1


bool SpiStartByteRxd = false;


ui8  SpiMsgStartByte = 0;
ui8  SpiMsgLen = 0;
ui16 SpiRecvBuffer[SPI_MAX_MESSAGE_LEN];
ui16 SpiStartMask = 0xFF00;
ui16 SpiCmdMask = 0x00FF;
ui16 SpiStartByte = 0x3200;

ui8 interruptOverRunStat0 = 0;
ui8 interruptRxTimeoutStat0 = 0;

ui8 ServoBuffer[24];
bool ServoBufferUpdated = true;

volatile ui8 bytesToFollow = 0;
// ------------------------------------------------------------------------------------------------------------------
//
// SSP0_IRQHandler
//
// SPI 0 Interrupt, for overrun
//
// ------------------------------------------------------------------------------------------------------------------
void SSP0_IRQHandler(void)
{
ui32 regValue;
volatile ui8 offset = 0;

	regValue = LPC_SSP0->MIS;

	// Receive overrun interrupt
	if ( regValue & SSPMIS_RORMIS )
	{
		// This is bad, it means we are not servicing the SPI often enough are we are missing bytes!
		interruptOverRunStat0++;

		// clear interrupt
		LPC_SSP0->ICR = SSPICR_RORIC;
	}

	// Receive timeout interrupt
	if ( regValue & SSPMIS_RTMIS )
	{
		//

		interruptRxTimeoutStat0++;
		// clear interrupt
		LPC_SSP0->ICR = SSPICR_RTIC;
	}


	// Empty the RX fifo
	// TODO : Limit how much we do per call
	while ( LPC_SSP0->SR & SSPSR_RNE )
	{
		if ( SpiStartByteRxd )
		{
			// we can not over run the SPI data buffer
			if (SpiMsgLen >= SPI_MAX_MESSAGE_LEN) break;

			SpiRecvBuffer[SpiMsgLen] = LPC_SSP0->DR;
			SpiMsgLen++;
		}
		else
		{
			SpiMsgLen = 0;
			SpiRecvBuffer[SpiMsgLen] = LPC_SSP0->DR;

			if ( SPI_START_BYTE == (ui8)(SpiRecvBuffer[0] >> 8) )
			{
				SpiStartByteRxd = true;
				SpiMsgLen = 1;
			}
			else
			{
				SpiMsgLen = 0;
			}
		}
	}

	// Theres nothing to send
	// TODO : Check that the data has left the periperal
	if ( LPC_SSP0->SR & SSPSR_TFE )
	{
		// TX Fifo is empty, disable output
		LPC_SSP0->CR1 |= SSPCR1_SOD;
	}

	if ( SpiStartByteRxd )
	{
		switch ((ui8)SpiRecvBuffer[0])
		{
			case SB_WHOS_THERE :
				// Enable Output
				LPC_SSP0->CR1 &= ~SSPCR1_SOD;
				LPC_SSP0->DR = (SOLDERBRIDGE_TYPE << 8) | SOLDERBRIDGE_VER;

				SpiRecvBuffer[0] = 0;
				SpiStartByteRxd = false;
				SpiMsgLen = 0;
			break;

			case SB_SERVO_MOVE :
				if ( SpiMsgLen > 2)
				{
					bytesToFollow = 0x00FF & SpiRecvBuffer[1];
					bytesToFollow = (bytesToFollow / 2) + (bytesToFollow % 2);

					offset = SpiRecvBuffer[1] >> 8;

					// TODO : Limit bytes to max of 12

					if ( SpiMsgLen > bytesToFollow + 1)
					{
						// We have it all
						if ((offset + bytesToFollow) > 24)
						{
							// this would overflow our buffer!
						}
						else
						{
							memcpy(&ServoBuffer[offset], &SpiRecvBuffer[2], (0x00FF & SpiRecvBuffer[1]));
							ServoBufferUpdated = true;
						}

						SpiRecvBuffer[0] = 0;
						SpiStartByteRxd = false;
						SpiMsgLen = 0;
					}
				}
			break;
		}

		if (SpiMsgLen >= SPI_MAX_MESSAGE_LEN)
		{
			// We have a full buffer that hasnt been processed, lets junk it
			SpiRecvBuffer[0] = 0;
			SpiStartByteRxd = false;
			SpiMsgLen = 0;
		}
	}

}

// ------------------------------------------------------------------------------------------------------------------
//
// SpiSlave_Task
//
// ------------------------------------------------------------------------------------------------------------------
void SpiSlave_InterruptControl ( bool enable )
{
	if (enable)
	{
		// Enable the SSP Interrupt
		NVIC_EnableIRQ(SSP0_IRQn);
		LPC_SSP0->IMSC = SSPIMSC_RXIM | SSPIMSC_RORIM | SSPIMSC_RTIM;

		// Trigger the interrupt, so we can perform any needed fifo emptying etc..
		NVIC_SetPendingIRQ(SSP0_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(SSP0_IRQn);
	}
}

// ------------------------------------------------------------------------------------------------------------------
//
// SpiSlave_Task
//
// ------------------------------------------------------------------------------------------------------------------
void SpiSlave_Task ( void )
{
	// run the interrupt code
	SSP0_IRQHandler();

	// if we have a buffer and the CS line has now transitioned high
}

// ------------------------------------------------------------------------------------------------------------------
//
// SpiSlave_GetServoUpdate
//
// Updates the supplied buffer if we have been sent a position update
//
// ------------------------------------------------------------------------------------------------------------------
bool SpiSlave_GetServoUpdate ( ui16 *buffer )
{
bool result = FALSE;

	if ((ServoBufferUpdated) && ( buffer ))
	{
		// Todo : were copying all of the buffer over, eventually it will all be initalised but at first it wont be
		memcpy(buffer, &ServoBuffer[0], 24);

		ServoBufferUpdated = false;
		result = TRUE;
	}

	return ( result );
}


// ------------------------------------------------------------------------------------------------------------------
//
// SpiSlave_Init
//
// ------------------------------------------------------------------------------------------------------------------
void SpiSlave_Init ( void )
{
	// ??
	LPC_SYSCON->PRESETCTRL |= (0x1<<0);

	// Enable the SPI Clock?
	LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<11);

	// Divide the SPI clock by 12 to give us 4 clocks per 1mhz
	//LPC_SYSCON->SSP0CLKDIV = 12;
	// NOTE : were a slave not sure we get a to choose!
	LPC_SYSCON->SSP0CLKDIV = 1;

	// SPI CLk
	LPC_IOCON->PIO0_6 = 2;

	// SSP CS Enable
	LPC_IOCON->PIO0_2 = 1;

	// MISO Enable
	LPC_IOCON->PIO0_8 = 1;

	// MOSI Enable
	LPC_IOCON->PIO0_9 = 1;

	// Set DSS data to 16-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 3 ( 4clocks - 1 )
	LPC_SSP0->CR0 = 0x000F | SSPCR0_SPO | SSPCR0_SPH | 3<<8;

	// SSE - Enabled, MS - Slave, SOD - Output disabled
	LPC_SSP0->CR1 = 0x000E;

	/* Set SSPINMS registers to enable interrupts */
	/* enable all error related interrupts */
	//LPC_SSP0->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;

	// Enable the SSP Interrupt
	//NVIC_EnableIRQ(SSP0_IRQn);
	//LPC_SSP0->IMSC = SSPIMSC_RXIM | SSPIMSC_RORIM | SSPIMSC_RTIM;

}
