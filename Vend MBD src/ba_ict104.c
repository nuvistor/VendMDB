
/*-------------------------------------------------------
ba_ict104.c
уровень протокола купюроприЄмника ICT104
-------------------------------------------------------*/

#include "main.h"
#include "b_acceptor.h"
#include "ba_ict104.h"
#include "sw_timers.h"
#include "logger.h"
#include "queue.h"

#define BA_STATE_UKNOWN 0xFF;

volatile ba_status_t ba_status;//={ BA_STATE_UKNOWN, 0, 0, 0};

// Bytes from acceptor
enum{
//Power UP
BA_RX_PWRON1=0x80,
BA_RX_PWRON2=0x8F,
// Escrow
BA_RX_VALIDATED=0x81,
BA_RX_BILL_VAL1=0x40,
BA_RX_BILL_VAL2=0x41,
BA_RX_BILL_VAL3=0x42,
BA_RX_BILL_VAL4=0x43,
BA_RX_BILL_VAL5=0x44,
BA_RX_BILL_VAL6=0x45,
BA_RX_BILL_VAL7=0x46,
BA_RX_BILL_VAL8=0x47,
BA_RX_BILL_VAL9=0x48,
BA_RX_BILL_VAL10=0x49,
BA_RX_STACKING=0x10,
BA_RX_REJECT=0x11,
// Bill acceptor status
BA_RX_MOTOR_FAILURE=0x20,
BA_RX_CS_ERROR=0x21,
BA_RX_BILL_JAM=0x22,
BA_RX_BILL_REMOVE=0x23,
BA_RX_STACKER_OPEN=0x24,
BA_RX_SENSOR_PROBLEM=0x25,
BA_RX_BILL_FISH=0x27,
BA_RX_STACKER_PROBLEM=0x28,
BA_RX_BILL_REJECT=0x29,
BA_RX_INVALID_CMD=0x2A,
BA_RX_ERROR_STATUS_EXC=0x2F,
BA_RX_ENABLE=0x3E,
BA_RX_INHIBIT=0x5E
//
};

enum{
BA_CMD_ACCEPT=0x02, // bill accept or pwr on enable
BA_CMD_REJECT=0x0F, // reject money
BA_CMD_HOLD=0x18, // hold money in escrow
BA_CMD_STATUS_REQUEST=0x0C,
BA_CMD_ENABLE=0x3E,
BA_CMD_INHIBIT=0x5E,
BA_CMD_RESET=0x30 // software reset
};

uint8_t ict_reset_step = 0;



eeprom bill_values_t bill_values[2]=
{{
6,
{500, 1000, 5000, 10000, 50000, 100000, 0, 0, 0, 0}},
{
6,
{500, 1000, 5000, 10000, 50000, 100000, 0, 0, 0, 0}}
};

eeprom uint8_t ba_inhibit = 1;

/*-------------------------------------------------------
проверить значени€ банкнот, если надо - скорректировать
-------------------------------------------------------*/
void correct_bill_values( void )
{
        uint8_t good_idx[2]={0,0};
        uint8_t i, j;
        uint8_t cs;
        eeprom uint8_t *p_bv[2];
        // найти целые и испорченные экземпл€ры записей
        for(i=0; i<2; i++)
        {
                cs = 0;
                p_bv[i] = (eeprom uint8_t*) &bill_values[i];
                //
                for(j=0; j<sizeof(bill_values_t); j++)
                {
                        cs += *p_bv[i];
                        p_bv[i]++;        
                };
                if( (cs==bill_values[i].cs) && (cs == ~bill_values[i].csn) )
                {
                        good_idx[i] = 1;
                }
                if( (bill_values[i].tot_bills==0) || (bill_values[i].tot_bills>10) )
                {
                        good_idx[i] = 0;
                }
                for( j=0; j<10; j++ )
                { 
                        if( bill_values[i].values > 1000000 )
                        { 
                                good_idx[i] = 0;        
                        };
                };
        };
        //
        ba_status.bill_val_err=0;
        ba_status.inhibit_rq = 0;
        //
        p_bv[0] = (eeprom uint8_t*) &bill_values[0];
        p_bv[1] = (eeprom uint8_t*) &bill_values[1];
        //
        if(good_idx[0] && good_idx[1]) // все хорошо - ничего не делать
        {
        }else
        if((good_idx[0]==0)&&(good_idx[1]==1)) // первый экземпл€р плохой, второй хороший
        {
                for(j=0; j<sizeof(bill_values_t); j++)
                {
                        *p_bv[0] = *p_bv[1]; 
                        p_bv[0]++;
                        p_bv[1]++;        
                };
        }else//
        if((good_idx[0]==1)&&(good_idx[1]==0)) // первый экземпл€р хороший, второй плохой
        {
                for(j=0; j<sizeof(bill_values_t); j++)
                {
                        *p_bv[1] = *p_bv[0]; 
                        p_bv[0]++;
                        p_bv[1]++;        
                };
        }else // инициализировать правильные значени€
        {
                ba_status.inhibit_rq = 1;
                ba_status.bill_val_err=1;
                ba_inhibit = 1;
                for(i=0; i<2; i++)
                {
                        bill_values[i].tot_bills = 6;
                        bill_values[i].values[0] = 500;
                        bill_values[i].values[1] = 1000;
                        bill_values[i].values[2] = 5000;
                        bill_values[i].values[3] = 10000;
                        bill_values[i].values[4] = 50000;
                        bill_values[i].values[5] = 100000; 
                };
                bill_values_write_cs();
        };
       
}

/*-------------------------------------------------------
посчитать и записать контр суммы
-------------------------------------------------------*/
void bill_values_write_cs( void )
{
        uint8_t i, j;
        uint8_t cs;
        eeprom uint8_t *p_bv;
        //
        for(i=0; i<2; i++)
        {
                cs = 0;
                p_bv = (eeprom uint8_t*) &bill_values[i];
                //
                for(j=0; j<sizeof(bill_values_t); j++)
                {
                        cs += *p_bv;
                        p_bv++;        
                };
                //
                bill_values[i].cs = cs;
                bill_values[i].csn = ~cs;
        };  
}
                 
/*-------------------------------------------------------

-------------------------------------------------------*/
void ict104_send_cmd( uint8_t cmd )
{        
        ba_putchar2( cmd );
}
              
/*-------------------------------------------------------

-------------------------------------------------------*/
void ict104_reset( void )
{
        CLI();
        ba_buf_init();
        SEI();
        
        ict104_send_cmd( BA_CMD_RESET );
        sw_set_timeout( TIM4_BA_POLL, 150 ); // 15 sec timeout
        sw_set_timeout( TIM5_BA_COMM_FAIL, 200 ); // 20 sec
        
        ba_status.fishing = 0;
        ba_status.enabled = 0;
        ba_status.failure = 0;
        
        ict_reset_step = 0;
        ba_status.validated = 0;
        
}


/*-------------------------------------------------------

-------------------------------------------------------*/
void ict104_process( void )
{
        uint8_t d;
        static uint32_t bill_val = 0; 
         
        //
        if( buf_get_cnt( &ba_rx_stream ) > 0 ) // есть необработанные данные в буфере 
        {        
            d = buf_get_data( &ba_rx_stream );
            
            if( d == BA_RX_PWRON1 )
            {
                ict_reset_step = 1;    
            }else if( d != BA_RX_PWRON2 )
            {
                ict_reset_step = 0;
            };
            
            switch( d )
            { 
                case BA_RX_PWRON2:
                        if( ict_reset_step == 1 )
                        {
                                ict_reset_step = 0;
                                ict104_send_cmd( BA_CMD_ACCEPT );
                                ba_status.pwr_on = 0;
                                ba_status.fishing = 0;
                                if( ba_status.bill_val_err || ba_inhibit )
                                        ict104_send_cmd( BA_CMD_INHIBIT );
                                else
                                        ict104_send_cmd( BA_CMD_ENABLE );
                                ict104_send_cmd( BA_CMD_STATUS_REQUEST );
                        };
                        break;
                // Escrow
                case BA_RX_VALIDATED:
                        ba_status.validated = 1;
                        break;
                case BA_RX_STACKING:
                        if( (bill_val==0) || (ba_status.validated == 0) )
                        { 
                                break;
                        };
                        // send money to system
                        // записать кредит в ќ«”
                        status_add_money( bill_val );
                        status_add_bill();
                        //
                        copy_state2record();
                        write_log_record(); // добавить запись в журнал
                        //
                        generate_credit_pulse( bill_val );
                        //
                        ba_status.validated = 0;
                        bill_val = 0;
                        break;
                case BA_RX_REJECT:
                        ba_status.validated = 0;
                        bill_val = 0;
                        break;
                // Status
                case BA_RX_MOTOR_FAILURE:
                case BA_RX_CS_ERROR:
                case BA_RX_BILL_JAM:
                case BA_RX_STACKER_OPEN:
                case BA_RX_SENSOR_PROBLEM:
                case BA_RX_STACKER_PROBLEM:
                        ba_status.failure = 1;
                        break;
                case BA_RX_BILL_FISH:
                        ba_status.fishing = 1;
                        break;
                case BA_RX_ENABLE:
                        ba_status.enabled = 1;
                        if( ba_status.inhibit_rq )
                        {        
                                ict104_send_cmd( BA_CMD_INHIBIT );
                                ba_status.inhibit_rq = 0;
                        };
                        break;
                case BA_RX_INHIBIT:
                        if( ba_status.enable_rq )
                        {
                                ict104_send_cmd( BA_CMD_ENABLE );
                                ba_status.enable_rq = 0;
                        };
                        ba_status.enabled = 0;
                        break; 
            };
            //
            
            //
            if( (d >= BA_RX_BILL_VAL1) && (d <= BA_RX_BILL_VAL10) ) // denomination
            { 
                bill_val = 0;
                if((bill_values[0].tot_bills>(d-BA_RX_BILL_VAL1)) && (ba_status.bill_val_err==0) ) // если номинал записан=0, то Reject
                {
                        if(bill_values[0].values[(d-BA_RX_BILL_VAL1)]>0)//(d-BA_RX_BILL_VAL1)) 
                                bill_val = bill_values[0].values[(d-BA_RX_BILL_VAL1)];
                };
                if( bill_val > 0 )
                        ict104_send_cmd( BA_CMD_ACCEPT ); //accept money in stacker
                else
                        ict104_send_cmd( BA_CMD_REJECT );
            };
            //
            sw_set_timeout( TIM4_BA_POLL, 20 ); // 2 sec
            sw_set_timeout( TIM5_BA_COMM_FAIL, 200 ); // 20 sec
            ba_status.comm_failure = 0;
        };
        //
        
        if( sw_get_timeout( TIM4_BA_POLL ) == 0 )
        { 
                sw_set_timeout( TIM4_BA_POLL, 20 ); // 2 sec
                ict104_send_cmd( BA_CMD_STATUS_REQUEST ); // request BA status 
        };
        //
        if( sw_get_timeout( TIM5_BA_COMM_FAIL ) == 0 )
        {       
                ba_status.comm_failure = 1;
        };
                
}



/*-------------------------------------------------------

-------------------------------------------------------*/
