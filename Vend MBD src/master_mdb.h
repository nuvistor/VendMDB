/*-------------------------------------------------------
master_mdb.h
-------------------------------------------------------*/

#ifndef _MASTER_MDB_H
#define _MASTER_MDB_H

void mdb_rx_irq_callback( void );
void mdb_tx_irq_callback( void );

void mdb_validator_init( void );
void mdb_validator_process( void );

#endif _MASTER_MDB_H