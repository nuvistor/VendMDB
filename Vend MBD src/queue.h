/*-------------------------------------------------------
queue.h
-------------------------------------------------------*/
#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>

// Queue buffer
typedef struct{
        uint16_t buf_sz;
        uint8_t wr_idx;
        uint8_t rd_idx;
        uint8_t cnt;
        uint8_t buf_ovf:1; // buffer overflow flag
        uint8_t *data;       
}queue_t;

// Queue buf for objects
/*typedef struct{
        uint16_t items_total;
        uint16_t data_buf_sz; // size in bytes
        uint8_t wr_item_idx;
        uint8_t rd_item_idx;
        uint8_t wr_byte_idx;
        uint8_t rd_byte_idx;
        uint8_t cnt;
        uint8_t buf_ovf:1; // buffer overflow flag
        uint8_t *data;        
}queue_obj_t;*/


void buf_init( queue_t *b, uint8_t* buf, uint16_t buf_sz );
void buf_put_data( queue_t* b, uint8_t data );
uint8_t buf_get_cnt( queue_t *b );
uint8_t buf_get_data( queue_t *b );
uint8_t buf_get_data_unsafe( queue_t *b );

#endif QUEUE_H
