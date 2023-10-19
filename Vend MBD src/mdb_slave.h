/*-------------------------------------------------------
mdb_slave.h
-------------------------------------------------------*/
#ifndef _MDB_SLAVE_H
#define _MDB_SLAVE_H

enum{
SBV_CREDIT_IDLE=0,
SBV_CREDIT_PROC_BUSY,
SBV_CREDIT_PROC_END,
SBV_CREDIT_BREAK_ERROR,
SBV_CREDIT_VALIDATOR_NOT_READY,
SBV_CREDIT_REJECTED
};

uint8_t slave_bv_bill_insert( uint32_t value );
void slave_bv_init( void );
void slave_bv_proc( void );
void mdb_slave_rx_irq_handler( void );
void mdb_slave_tx_irq_handler( void );
void slave_bv_insert_credit( uint32_t credit );

extern uint8_t sbv_credit_proc_sta; // credit processing status

#endif  _MDB_SLAVE_H