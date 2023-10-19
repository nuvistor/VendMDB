/*-------------------------------------------------------
queue.c
-------------------------------------------------------*/
#include "queue.h"

/*-------------------------------------------------------

-------------------------------------------------------*/
void buf_init( queue_t *b, uint8_t* buf, uint16_t buf_sz )
{
        b->buf_sz  = buf_sz; 
        b->buf_ovf = 0;
        b->cnt     = 0;
        b->rd_idx  = 0;
        b->wr_idx  = 0;
        b->data    = buf;        
}

/*-------------------------------------------------------
// put data to FIFO
// Arguments: Ptr to Buf, Data
-------------------------------------------------------*/
void buf_put_data( queue_t* b, uint8_t data )
{
    b->data[ b->wr_idx++ ] = data;
    
   if ( (b->wr_idx) == (b->buf_sz))  (b->wr_idx) = 0;
   if (++(b->cnt) == (b->buf_sz))
   {
            b->cnt = 0;
            b->buf_ovf = 1;
   }
}
             
/*-------------------------------------------------------
//put item(object, buffer) to FIFO
//Arguments: ptr to buf, ptr to object, obj size in bytes
-------------------------------------------------------*/
/*void buf_put_item( queue_obj_t *pq, void *obj, uint8_t obj_sz )
{
        
}*/
           
/*-------------------------------------------------------
get number of data items counter in FIFO
-------------------------------------------------------*/
uint8_t buf_get_cnt( queue_t *b )
{
        return (b->cnt);
}

            
/*-------------------------------------------------------
get data from FIFO
Arguments: Ptr to Buf
Return: data
-------------------------------------------------------*/
uint8_t buf_get_data( queue_t *b )
{
    uint8_t  data;  
    
    data = b->data[b->rd_idx++];
    
    if ((b->rd_idx) == (b->buf_sz)) (b->rd_idx)=0;
    
    #asm("cli")
    --(b->cnt);
    #asm("sei")
    
    return data;
}

/*-------------------------------------------------------
get data from FIFO
Arguments: Ptr to Buf
Return: data
-------------------------------------------------------*/
uint8_t buf_get_data_unsafe( queue_t *b )
{
    uint8_t  data;  
    
    data = b->data[b->rd_idx++];
    
    if ((b->rd_idx) == (b->buf_sz)) (b->rd_idx)=0;
    
    //#asm("cli")
    --(b->cnt);
    //#asm("sei")
    
    return data;
}