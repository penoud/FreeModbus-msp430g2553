/*
 * FreeModbus Libary: MSP430 Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.3 2006/11/19 03:57:49 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Defines ------------------------------------------*/
#define U0_CHAR                 ( 0x10 )        /* Data 0:7-bits / 1:8-bits */

#define DEBUG_PERFORMANCE       ( 1 )

#if DEBUG_PERFORMANCE == 1
#define DEBUG_PIN_RX            ( 1 )
#define DEBUG_PIN_TX            ( 2 )
#define DEBUG_PORT_DIR          ( P1DIR )
#define DEBUG_PORT_OUT          ( P1OUT )
#define DEBUG_INIT( )           \
  do \
  { \
    DEBUG_PORT_DIR |= ( 1 << DEBUG_PIN_RX ) | ( 1 << DEBUG_PIN_TX ); \
    DEBUG_PORT_OUT &= ~( ( 1 << DEBUG_PIN_RX ) | ( 1 << DEBUG_PIN_TX ) ); \
  } while( 0 ); 
#define DEBUG_TOGGLE_RX( ) DEBUG_PORT_OUT ^= ( 1 << DEBUG_PIN_RX )
#define DEBUG_TOGGLE_TX( ) DEBUG_PORT_OUT ^= ( 1 << DEBUG_PIN_TX )

#else

#define DEBUG_INIT( )
#define DEBUG_TOGGLE_RX( )
#define DEBUG_TOGGLE_TX( )
#endif

/* ----------------------- Static variables ---------------------------------*/
UCHAR           ucGIEWasEnabled = FALSE;
UCHAR           ucCriticalNesting = 0x00;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    ENTER_CRITICAL_SECTION(  );
    if( xRxEnable )
    {
        IE2 |= UCA0RXIE;
    }
    else
    {
        IE2 &= ~UCA0RXIE;
    }
    if( xTxEnable )
    {
	//Set RTS
        IE2 |= UCA0TXIE;
        IFG2 |= UCA0TXIFG;
    }
    else
    {
        IE2 &= ~UCA0TXIE;
	//CLEAR CTS
	while (UCA0STAT & UCBUSY){
		;
	}
    }
    EXIT_CRITICAL_SECTION(  );
}

BOOL
xMBPortSerialInit( UCHAR ucPort, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
    BOOL            bInitialized = TRUE;
    USHORT          UxCTL = 0;
    USHORT          UxBR = ( USHORT ) ( SMCLK / ulBaudRate );

    switch ( eParity )
    {
    case MB_PAR_NONE:
        break;
    case MB_PAR_ODD:
        UCA0CTL0 |= UCPAR;
        break;
    case MB_PAR_EVEN:
        UCA0CTL0 &= ~UCPAR;
        break;
    }
    switch ( ucDataBits )
    {
    case 8:
        UCA0CTL0 &= ~UC7BIT;
        break;
    case 7:
        UCA0CTL0 |= UC7BIT;
        break;
    default:
        bInitialized = FALSE;
    }
    if( bInitialized )
    {
        ENTER_CRITICAL_SECTION(  );
        
	/* Configure hardware UART */
	P1SEL = BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
	P1SEL2 = BIT1 + BIT2 ; // P1.1 = RXD, P1.2=TXD
	UCA0CTL1 |= UCSSEL_2; // Use SMCLK
	UCA0BR0 = 104; // Set baud rate to 38400 with 4MHz clock (Data Sheet 15.3.13)
	UCA0BR1 = 0; // Set baud rate to 38400 with 4MHz clock
	UCA0MCTL = UCBRS0; // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST; // Initialize USCI state machine
	IE2 |= UCA0RXIE; // Enable USCI_A0 RX interrupt

        EXIT_CRITICAL_SECTION(  );

        DEBUG_INIT( );
    }
    return bInitialized;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{ 

	//Set RTS
	UCA0TXBUF = ucByte;
	IE2 |= UCA0TXIE; 					//Enable USCI_A0 TX interrupt

	//reset RTS
	while (UCA0STAT & UCBUSY){
		;
	}
	return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    *pucByte = UCA0RXBUF;
    return TRUE;
}

#if defined (__GNUC__)
interrupt (USCIAB0RX_VECTOR) prvvMBSerialRXIRQHandler( void )
#else
void
prvvMBSerialRXIRQHandler( void ) __interrupt[USCIAB0RX_VECTOR]
#endif
{
    DEBUG_TOGGLE_RX( );
    pxMBFrameCBByteReceived(  );
}

#if defined (__GNUC__)
interrupt (USCIAB0TX_VECTOR) prvvMBSerialTXIRQHandler( void )
#else
void
prvvMBSerialTXIRQHandler( void ) __interrupt[USCIAB0TX_VECTOR]
#endif
{
    DEBUG_TOGGLE_TX( );
    pxMBFrameCBTransmitterEmpty(  );
}

void
EnterCriticalSection( void )
{
    USHORT usOldSR;
    if( ucCriticalNesting == 0 )
    {
#if defined (__GNUC__)
        usOldSR = READ_SR;
        _DINT( );
#else
        usOldSR = _DINT( );
#endif
        ucGIEWasEnabled = usOldSR & GIE ? TRUE : FALSE;
    }
    ucCriticalNesting++;
}

void
ExitCriticalSection( void )
{
    ucCriticalNesting--;
    if( ucCriticalNesting == 0 )
    {
        if( ucGIEWasEnabled )
        {
            _EINT(  );
        }
    }
}
