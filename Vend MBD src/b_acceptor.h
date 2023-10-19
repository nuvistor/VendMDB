/*-------------------------------------------------------
bill acceptor IO
b_acceptor.h
-------------------------------------------------------*/
#ifndef B_ACCEPTOR_H
#define B_ACCEPTOR_H

#include "queue.h"

extern queue_t ba_rx_stream;

void ict104_uart_init( void );
void ba_putchar2(char c);
void ba_buf_init( void );

void ba_ict104_rx_irq_callback( void );
void ba_ict104_tx_irq_callback( void );

#endif B_ACCEPTOR_H