/*-------------------------------------------------------
Bill acceptor ICT104 protocol mechanism

ba_ict104.h
-------------------------------------------------------*/

#ifndef _BA_ICT104_H
#define _BA_ICT104_H

#include <stdint.h>

typedef struct{
        uint8_t state;
        uint8_t failure:1;
        uint8_t comm_failure:1; // communication failure timeout
        uint8_t enabled:1;
        uint8_t pwr_on:1;
        uint8_t fishing:1;
        uint8_t validated:1; // money validated
        uint8_t bill_val_err:1; // eeprom bill table err
        uint8_t inhibit_rq:1; // inhibit request
        uint8_t enable_rq:1; // enable request
}ba_status_t;

extern volatile ba_status_t ba_status;
extern eeprom uint8_t ba_inhibit;

typedef struct {
        uint8_t tot_bills;
        uint32_t values[10];
        uint8_t cs; // checksum
        uint8_t csn;
}bill_values_t;

extern eeprom bill_values_t bill_values[];
extern eeprom uint8_t ba_inhibit;

        void ict104_process( void );
        void ba_buf_init( void );
        void ict104_reset( void );
        
        void correct_bill_values( void );
        void bill_values_write_cs( void );
#endif _BA_ICT104_H