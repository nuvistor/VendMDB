/*-------------------------------------------------------
bill acceptor IO procedures
b_acceptor.c
-------------------------------------------------------*/

#include "main.h"
#include "b_acceptor.h"
#include "ba_ict104.h"
#include "queue.h"
#include "uart_bits.h"

//
#define BA_TX_BUF_SZ    8

queue_t ba_tx_stream;
uint8_t ba_tx_buf[BA_TX_BUF_SZ];
//
#define BA_RX_BUF_SZ    100

queue_t ba_rx_stream;
uint8_t ba_rx_buf[BA_RX_BUF_SZ];
//

/*-------------------------------------------------------

-------------------------------------------------------*/
void ba_buf_init( void )
{
        buf_init( &ba_rx_stream, ba_rx_buf, BA_RX_BUF_SZ );
        buf_init( &ba_tx_stream, ba_tx_buf, BA_TX_BUF_SZ ); 
}  

/*-------------------------------------------------------

-------------------------------------------------------*/
void ba_ict104_rx_irq_callback( void )
{
    char status,data;
    
    status=UCSR2A;
    data=UDR2;
    
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
       buf_put_data( &ba_rx_stream, data );
    }
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void ba_ict104_tx_irq_callback( void )
{
        if( buf_get_cnt( &ba_tx_stream ) > 0 )
        {
                UDR2 = buf_get_data_unsafe( &ba_tx_stream ); 
        };
}

            
/*-------------------------------------------------------
// Write a character to the USART2 Transmitter buffer
-------------------------------------------------------*/
#pragma used+
void ba_putchar2(char c)
{
    while( buf_get_cnt( &ba_tx_stream ) == BA_TX_BUF_SZ );
    CLI();
    if( (buf_get_cnt( &ba_tx_stream ) >0) || ((UCSR2A & DATA_REGISTER_EMPTY)==0)  )
    {
        buf_put_data( &ba_tx_stream, c );
    }else
    {
        UDR2=c;
    };
    SEI();  
}
#pragma used-



/*-------------------------------------------------------

-------------------------------------------------------*/
void ict104_uart_init( void )
{
        // PORT
        DDRH &= ~((1<<0)|(1<<1)); // PH0 RXD = in
        DDRH |= (1<<1); // PH1 TXD = out
        
        // USART2 initialization
        // Communication Parameters: 8 Data, 1 Stop, Even Parity
        // USART2 Receiver: On
        // USART2 Transmitter: On
        // USART2 Mode: Asynchronous
        // USART2 Baud Rate: 9600
        UCSR2A=0x00;
        UCSR2B=0xD8;
        UCSR2C=0x26;
        UBRR2H=0x00;
        UBRR2L=0x67;
} 
