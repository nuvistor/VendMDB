/*-------------------------------------------------------
master_mdb.c
-------------------------------------------------------*/
#include <mega2560.h>
#include <stdint.h>
#include "uart_bits.h"
#include "sw_timers.h"
#include "PT\pt.h"
#include "main.h"
#include <string.h>
#include "ba_ict104.h"
#include "logger.h"

/*-------------------------------------------------------
USARTx Registers
-------------------------------------------------------*/
#define MDB_MST_UDRx     UDR2
#define MDB_MST_UCSRxA   UCSR2A
#define MDB_MST_UCSRxB   UCSR2B
#define MDB_MST_UCSRxC   UCSR2C
#define MDB_MST_UBRRxH   UBRR2H
#define MDB_MST_UBRRxL   UBRR2L

#define MDB_MST_TX_DIR  DDRH
#define MDB_MST_TX_PORT PORTH
#define MDB_MST_TX_PIN  1

#define MDB_MST_RX_DIR  DDRH
#define MDB_MST_RX_PIN  0
#define MDB_MST_RX_PORT PORTH

/*-------------------------------------------------------

-------------------------------------------------------*/
#define MDB_MAX_BLOCK_SZ        36

/*-------------------------------------------------------
MDB response codes
-------------------------------------------------------*/
#define MDB_ACK 0x00 // acknowledgment/checksum correct
#define MDB_NAK 0xFF // Negative acknowledge
#define MDB_RET 0xAA // Retransmit the previously sent data.
                     // Only the VMC can transmit this byte


/*-------------------------------------------------------
VMC to bill validator commands
-------------------------------------------------------*/
#define VCMD_RESET       0x30 // Command for bill validator to self-reset
#define VCMD_SETUP       0x31  // Request for bill validator setup information
#define VCMD_SECURITY    0x32  // Sets Validator Security Mode
#define VCMD_POLL        0x33  // Request for Bill Validator activity Status
#define VCMD_BILL_TYPE   0x34  // Indicates Bill Type enable or disable
#define VCMD_ESCROW      0x35  // Sent by VMC to indicate action for escrow
#define VCMD_STACKER     0x36  // Indicates stacker full and the number of bills
#define VCMD_EXPANSION   0x37  // Command to allow addition of features and future enhancements

/*-------------------------------------------------------
Slave BV responses
-------------------------------------------------------*/

// respone to SETUP/STATUS 0x31
typedef struct{
        uint8_t FeatureLevel; // 1=Level1, 2=Level2
        uint16_t CountryCode;
        uint16_t ScalingFactor;
        uint8_t DecimalPlaces;
        uint16_t StackerCapacity;
        uint16_t SecurityLevels;
        uint8_t Escrow; // 0=have no escrow, 1=has escrow 
        uint8_t BillType[16];
}bv_rsp_setup_t;

// response to SECURITY 0x32
typedef struct{
        uint16_t SecurityLevels; 
}bv_rsp_security_t;

// response to POLL 0x33
typedef struct{
        uint8_t Activity[16];
}bv_rsp_poll_t;

// Response Bill Accepted Status 
// bill audit and routing accepted mask: 1xxx0000
#define VRSP_BILL_ACCEPTED_FLAG       (1<<7)
#define VRSP_BILL_ROUTING_MASK        0xF0
// 
#define VRSP_BILL_STACKED                0x80 // BILL STACKED
#define VRSP_BILL_ESCROW                 0x90 // ESCROW POSITION
#define VRSP_BILL_RETURNED               0xA0 // BILL RETURNED
#define VRSP_BILL_RECYCLER               0xB0 // BILL TO RECYCLER
#define VRSP_BILL_REJECTED               0xC0 // DISABLED BILL REJECTED
#define VRSP_BILL_RECYCLER_MANUAL        0xD0 // BILL TO RECYCLER – MANUAL FILL
#define VRSP_BILL_DISPENSE               0xE0 // MANUAL DISPENSE
#define VRSP_BILL_CASHBOX                0xF0 // TRANSFERRED FROM RECYCLER TO CASHBOX
// bill type 
#define VRSP_POLL_BILL_TYPE_MASK         ((1<<3)|(1<<2)|(1<<1)) // Bill Type (0 to 15)

// Bill Validator Status
#define VRSP_POLL_DEFECTIVE_MOTOR        0x01 // One of the motors has failed to perform its expected assignment.
#define VRSP_POLL_SENSOR_PROBLEM         0x02 // One of the sensors has failed to provide its response
#define VRSP_POLL_VALIDATOR_BUSY         0x03 // The validator is busy and can not answer a detailed command right now
#define VRSP_POLL_ROM_ERROR              0x04 // The validators internal ecksum does not match the calculated checksum
#define VRSP_POLL_JAMMED                 0x05 // A bill(s) has jammed in the acceptance path
#define VRSP_POLL_RESET                  0x06 // The validator has been reset since the last POLL
#define VRSP_POLL_REMOVED                0x07 // A bill in the escrow position has been removed by an unknown means
#define VRSP_POLL_CASHBOX_OPEN           0x08 // The validator has tected the cash box to be open or removed
#define VRSP_POLL_DISABLED               0x09 // The validator has been disabled, by the VMC or because of internal conditions
#define VRSP_POLL_INVALID_ESCROW         0x0A // An ESCROW command was requested for a bill not in the escrow position
#define VRSP_POLL_REJECTED               0x0B // A bill was detected, but rejected because it could not be identified
#define VRSP_POLL_CREDIT_REMOVED         0x0C // There has been an attempt to remove a credited (stacked) bill
//
#define VRSP_POLL_ID_DISABLED_ATTEMPTS_MASK      ((1<<7)|(1<<6)|(1<<5))
#define VRSP_POLL_ID_DISABLED_ATTEMPTS_MASK_VAL  ((0<<7)|(1<<6)|(0<<5))
#define VRSP_POLL_DISABLED_ATTEMPTS_NUMBER_MASK  0x1F  

// Bill Recycler
#define VRSP_POLL_ESCROW_RQ              0x21 // An escrow lever activation has been detected
#define VRSP_POLL_DISPENSER_PAYOUT_BUSY  0x22 // The dispenser is busy activating payout devices
#define VRSP_POLL_DISPENSER_BUSY         0x23 // The dispenser is busy and can not answer a detailed command right now
#define VRSP_POLL_DISPENSER_DEFECT_SENS  0x24 // The dispenser has detected one of the dispenser sensors behaving abnormally
#define VRSP_POLL_DISPENSER_MOTOR_PROBL  0x26 // Dispenser did not start / motor problem
#define VRSP_POLL_DISPENSER_JAM          0x27 // A dispenser payout attempt has resulted in jammed condition
#define VRSP_POLL_DISPENSER_ROM_ERROR    0x28 // The dispensers internal checksum does not match the calculated checksum
#define VRSP_POLL_DISPENSER_DISABLED     0x29 // dispenser disabled because of error or bill in escrow position
#define VRSP_POLL_BILL_WAITING           0x2A // waiting for customer removal
#define VRSP_POLL_FILLED_KEY_PRESSED     0x2F // The VMC should request a new DISPENSER STATUS

// File Transport Layer

// response to BILL TYPE 0x34
typedef struct{
        uint16_t BillEnable;
        uint16_t EscrowEnable;
} bv_rsp_bill_type_t;


// response to ESCROW 0x35
typedef struct{
        uint8_t EscrowStatus; // 0=return bill in the escrpw position
                              // xxxxxxx1=stack the bill
} bv_rsp_ecrow_t;

// response to STACKER 0x36
typedef struct{
        uint16_t BillsNumber; // bill number of full condition
} bv_rsp_stacker_t;

// response to EXP L1 ID
typedef struct{
        uint8_t MfCode[3];      // ascii
        uint8_t SerialNum[12];  // ascii
        uint8_t ModelNum[12];   // ascii
        uint8_t SoftVersion[2]; // BCD
} bv_rsp_id;


/*-------------------------------------------------------
MDB software timers
-------------------------------------------------------*/
#define MDB_TIM4 TIM4_BA_POLL
#define MDB_TIM5 TIM5_BA_COMM_FAIL

/*-------------------------------------------------------

-------------------------------------------------------*/
typedef struct{
        // RX engine
        uint8_t rx_buf[36];
        volatile
        uint8_t rx_byte_cnt; // счётчик принимаемого байта
        volatile
        uint8_t rx_data_ready; // блок успешно принят
        volatile
        uint8_t rx_err;        // ошибка приёма блока
        volatile 
        uint8_t rx_enable;     // enable rx buffer
        volatile
        uint8_t rx_ACK_code;   // первый байт ответа, обычно Ack
        uint8_t rx_timeoutNAK; // no answer NAK
        bv_rsp_setup_t Setup;
        bv_rsp_id      ID;
        // флаги статуса валидатора, меняются по приходу ответов
        uint16_t BillsInStacker;
        uint8_t StackerFull;
        uint8_t JustReset;
        uint8_t Disabled;
        uint8_t RejectedUknown;
        // TX engine
        uint8_t tx_buf[36];
        uint8_t tx_byte_cnt;  // счётчик отправляемого байта
        volatile
        uint8_t tx_sz;        // размер блока на передачу
        volatile
        uint8_t tx_busy;      // передатчик занят - ещё идёт передача
        //
        //uint8_t bus_state;
        //uint16_t bill_values[16];
} mdb_status_t;

/*-------------------------------------------------------

-------------------------------------------------------*/
mdb_status_t mdb_status;
uint8_t tx_data[36];
uint32_t bill_values_table[16];
//

/*-------------------------------------------------------

-------------------------------------------------------*/
uint8_t mdb_get_chk( uint8_t *buf, uint8_t sz );
void mdb_rx_byte( uint8_t data, uint8_t mode );
void mdb_tx_byte( void );
PT_THREAD(mdb_bvalidator_thread(struct pt *pt));

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_master_uart_init( void )
{
    CLI();
    CLR_BIT(MDB_MST_RX_DIR,MDB_MST_RX_PIN); // PH0 RXD = in
    SET_BIT(MDB_MST_RX_PORT,MDB_MST_RX_PIN); // pull-up
    //
    SET_BIT(MDB_MST_TX_DIR,MDB_MST_TX_PIN); // PH1 TXD = out 
    SET_BIT(MDB_MST_TX_PORT,MDB_MST_TX_PIN); // =1
    SEI();
    
    // USART2 initialization
    // Communication Parameters: 9 Data, 1 Stop, No Parity
    // USART2 Receiver: On
    // USART2 Transmitter: On
    // USART2 Mode: Asynchronous
    // USART2 Baud Rate: 9600
    MDB_MST_UCSRxA=0x00;
    MDB_MST_UCSRxB=0xDC;
    MDB_MST_UCSRxC=0x06;
    MDB_MST_UBRRxH=0x00;
    MDB_MST_UBRRxL=0x67;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_master_uart_disable( void )
{
        MDB_MST_UCSRxB=0x00;
}


/*-------------------------------------------------------

-------------------------------------------------------*/
void purge_rx_buf( void )
{
        mdb_status.rx_byte_cnt = 0;
        mdb_status.rx_data_ready = 0;
        mdb_status.rx_err = 0;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void purge_tx_buf( void )
{
        mdb_status.tx_byte_cnt = 0;
        mdb_status.tx_busy = 0; // lock transmitter
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_rx_start( void )
{
        purge_rx_buf();
        mdb_status.rx_enable = 1;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_tx_start( void )
{       
        //
        purge_tx_buf();
        //
        mdb_status.tx_busy = 1;
        //
        if( mdb_status.tx_sz > 1 )
        { // adress + chk
                CLI();
                SET_BIT(MDB_MST_UCSRxB, TXB8); // MODE BIT = 1
                SEI(); 
        }else // tx_sz==1 
        {// ack/nak/ret  
                CLI();
                CLR_BIT(MDB_MST_UCSRxB, TXB8); // MODE BIT = 0
                SEI(); 
        }; 
        MDB_MST_UDRx = mdb_status.tx_buf[ mdb_status.tx_byte_cnt++ ];  
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_rx_irq_callback( void )
{
    char status,data,mode;
    
    status= MDB_MST_UCSRxA;
    mode  = MDB_MST_UCSRxB & (1<<RXB8);
    data  = MDB_MST_UDRx;
    
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
       mdb_rx_byte( data, mode );
    }
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_tx_irq_callback( void )
{
        if( mdb_status.tx_byte_cnt < mdb_status.tx_sz )
        {
                CLR_BIT(MDB_MST_UCSRxB, TXB8); // MODE BIT = 0 => data/chk
                MDB_MST_UDRx = mdb_status.tx_buf[ mdb_status.tx_byte_cnt++ ];
        }else
        {
                mdb_status.tx_busy = 0; // unlock transmitter       
        };
}


/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_rx_byte( uint8_t data, uint8_t mode )
{
        uint8_t chk;
        
        if( mdb_status.rx_enable )
        { 
                if( mdb_status.rx_byte_cnt >= MDB_MAX_BLOCK_SZ )
                {
                        mdb_status.rx_err = 1;
                        mdb_status.rx_enable = 0;
                }else
                { 
                    mdb_status.rx_buf[ mdb_status.rx_byte_cnt++ ] = data;
                    if( mode ) // last byte marker?
                    {
                            mdb_status.rx_data_ready = 1;
                            mdb_status.rx_enable = 0;
                            // если принято больше 1 байт - проверка суммы
                            if( mdb_status.rx_byte_cnt > 1 )
                            { 
                                chk = mdb_get_chk( mdb_status.rx_buf, mdb_status.rx_byte_cnt-1 );
                                if( chk != mdb_status.rx_buf[mdb_status.rx_byte_cnt-1] ) // checksum error?
                                { 
                                        mdb_status.rx_err = 1; // Error
                                }else
                                {       // check ok
                                        mdb_status.rx_ACK_code = mdb_status.rx_buf[0]; // data or JUST_RESET
                                };       
                            }else // rx_byte_cnt == 1
                            {
                                mdb_status.rx_ACK_code = mdb_status.rx_buf[0]; // ACK*-NAK*
                            };
                    };
                };  
        };
}

/*-------------------------------------------------------
simple checksum
not performed on respones!!!: ACK, NAK, RET
-------------------------------------------------------*/
uint8_t mdb_get_chk( uint8_t *buf, uint8_t sz )
{
        uint8_t chk = 0; // check sum byte
        uint8_t i;
        
        for( i=0; i<sz; i++ )
        { 
                chk += buf[i];
        };
        
        return chk;
}

/*-------------------------------------------------------
bus reset for all peripherals
-------------------------------------------------------*/
struct pt mdb_cmd_pt;
struct pt mdb_cmd_pt2;

PT_THREAD(mdb_hard_reset_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        
        mdb_status.rx_enable = 0;
        mdb_master_uart_init();
        mdb_master_uart_disable();
        //
        sw_set_timeout( MDB_TIM4, 3 ); // delay ~300ms
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0) );
        CLI();
        CLR_BIT(MDB_MST_TX_PORT, MDB_MST_TX_PIN);
        SEI();
        sw_set_timeout( MDB_TIM4, 3 ); // delay ~300ms
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0) ); 
        CLI();
        SET_BIT(MDB_MST_TX_PORT, MDB_MST_TX_PIN);
        SEI();
        // delay to init peripheral devices
        sw_set_timeout( MDB_TIM4, 6 ); // communication startup delay ~500ms
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0) );
        //
        mdb_master_uart_init();
        
        //
        PT_END(pt);
}


/*-------------------------------------------------------
VMC -> validator(periph) cmd+data
-------------------------------------------------------*/
void mdb_send_cmd( uint8_t cmd, uint8_t *data_bytes, uint8_t data_sz, uint8_t timeout )
{
        uint8_t i,j,chk;
        //                         
        mdb_status.tx_sz = 1+data_sz+1; // cmd+[sz]+chk
        mdb_status.tx_buf[0] = cmd;
        chk = cmd;
        //
        j=1;
        for( i=0; i<data_sz; i++ )
        {
                mdb_status.tx_buf[j++] = data_bytes[i];
                chk += data_bytes[i];         
        };
        //
        mdb_status.tx_buf[j] = chk;
        //
        mdb_status.rx_ACK_code = MDB_NAK;
        //
        mdb_rx_start();
        mdb_tx_start();
        // cmd respone time
        sw_set_timeout( MDB_TIM4, timeout );                 
}

/*-------------------------------------------------------
ACK VMC response
-------------------------------------------------------*/
PT_THREAD(mdb_send_ACK_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        mdb_status.tx_buf[0] = MDB_ACK;
        mdb_status.tx_sz = 1;
        mdb_tx_start();
        PT_WAIT_WHILE( pt, mdb_status.tx_busy );
        //
        PT_END(pt);        
}

/*-------------------------------------------------------
NAK VMC response
-------------------------------------------------------*/
PT_THREAD(mdb_send_NAK_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        mdb_status.tx_buf[0] = MDB_NAK;
        mdb_status.tx_sz = 1;
        mdb_tx_start();
        PT_WAIT_WHILE( pt, mdb_status.tx_busy );
        //
        PT_END(pt);
}

/*-------------------------------------------------------
Retransmit/RET last block VMC response
-------------------------------------------------------*/
PT_THREAD(mdb_send_RET_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        mdb_status.tx_buf[0] = MDB_RET;
        mdb_status.tx_sz = 1;
        mdb_tx_start();
        PT_WAIT_WHILE( pt, mdb_status.tx_busy );
        //
        PT_END(pt);
}

/*-------------------------------------------------------
RESET comand
-------------------------------------------------------*/
PT_THREAD(mdb_validator_softReset_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        mdb_send_cmd( VCMD_RESET, NULL, 0, 1*10 ); // communication startup delay ~500ms   
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&
                (!mdb_status.rx_data_ready)&&
                (!mdb_status.rx_err) );
        //
        PT_END(pt);
}

/*-------------------------------------------------------
POLL validator state
-------------------------------------------------------*/
PT_THREAD(mdb_validator_POLL_thread(struct pt *pt))
{
        uint8_t i, bill_status;
        
        PT_BEGIN(pt);
        //
        mdb_send_cmd( VCMD_POLL, NULL, 0, 2 ); //    
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&
                        (!mdb_status.rx_data_ready)&&
                        (!mdb_status.rx_err) );
        //
        if( mdb_status.rx_data_ready && (!mdb_status.rx_err) )
        { 
                PT_SPAWN( pt, &mdb_cmd_pt2, mdb_send_ACK_thread(&mdb_cmd_pt2) ); // ACK
                //
                if( mdb_status.rx_ACK_code != MDB_ACK ) // расширенный ответ - имеются данные
                {
                    for( i=0; i<mdb_status.rx_byte_cnt-1; i++ ) // ответ может иметь до 16 сообщений
                    {
                            bill_status = mdb_status.rx_buf[i];
                            if     ( bill_status == VRSP_POLL_RESET )   {mdb_status.JustReset = 1;}
                            else if( bill_status == VRSP_POLL_DISABLED ){mdb_status.Disabled = 1;}
                            else if( bill_status == VRSP_POLL_REJECTED ){mdb_status.RejectedUknown = 1;}         
                    };
                }        
        };
        //
        PT_END(pt);
}

/*-------------------------------------------------------
Request validator Setup(STATUS) information
-------------------------------------------------------*/
PT_THREAD(mdb_validator_SETUP_thread(struct pt *pt))
{
        uint8_t i;
        uint32_t k;
        //
        
        PT_BEGIN(pt);
        //
        mdb_send_cmd( VCMD_SETUP, NULL, 0, 2 ); //    
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&
                        (!mdb_status.rx_data_ready)&&
                        (!mdb_status.rx_err) );
        //
        if( mdb_status.rx_data_ready && (!mdb_status.rx_err) )
        {
                //
                PT_SPAWN( pt, &mdb_cmd_pt2, mdb_send_ACK_thread(&mdb_cmd_pt2) ); // ACK
                //
                memcpy( (uint8_t*)&mdb_status.Setup, (uint8_t*)&mdb_status.rx_buf, sizeof(bv_rsp_setup_t) );
                mdb_status.Setup.ScalingFactor = (((uint16_t)mdb_status.rx_buf[3])<<8) + mdb_status.rx_buf[4]; // байты идут в обратном порядке - собираем вручную 
                // номиналы банкнот
                k = 1;
                for( i=0; i<mdb_status.Setup.DecimalPlaces; i++ )
                {    
                        k *= 10;
                };
                for( i=0; i<16; i++ )
                {
                        bill_values_table[i] = ( ((uint32_t)mdb_status.Setup.BillType[i])*mdb_status.Setup.ScalingFactor ) / k;         
                };
        };
        if( sw_get_timeout(MDB_TIM4)==0 )
        { 
                mdb_status.rx_timeoutNAK = 1;
        };
        
        // ----- Test RET ----------
        /*mdb_rx_start();
        PT_SPAWN( pt, &mdb_cmd_pt2, mdb_send_RET_thread(&mdb_cmd_pt2) );
        sw_set_timeout(MDB_TIM4, 2);
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&
                        (!mdb_status.rx_data_ready)&&
                        (!mdb_status.rx_err) );
        PT_SPAWN( pt, &mdb_cmd_pt2, mdb_send_ACK_thread(&mdb_cmd_pt2) ); // ACK
        */
        // ---------------------
        
        //
        PT_END(pt);
}

/*-------------------------------------------------------
SECURITY
-------------------------------------------------------*/
PT_THREAD(mdb_validator_SECURITY_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        
        //
        PT_END(pt);
}

/*-------------------------------------------------------
BILL TYPE 0x34
Настройка разрешенных банкнот
Y1-Y2 = разрешение банкнот
Y3-Y4 = разрешение условного депонирования
-------------------------------------------------------*/
uint16_t accepted_bill_types;
uint16_t bill_escrow_enable;

PT_THREAD(mdb_validator_BILL_TYPE_thread(struct pt *pt))
{
        uint8_t i, j, mask;
        
        //
        PT_BEGIN(pt);
        //
        tx_data[0] = accepted_bill_types >> 8;   // Y1 [b15...b8]
        tx_data[1] = accepted_bill_types & 0xFF; // Y2 [b7...b0]
        
        tx_data[2] = bill_escrow_enable >> 8;    // Y3 [b15...b8]
        tx_data[3] = bill_escrow_enable & 0xFF;  // Y4 [b7...b0]
        //
        mdb_send_cmd( VCMD_BILL_TYPE, tx_data, 4, 2 ); // set bill types and escrow   
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&(!mdb_status.rx_data_ready)&&(!mdb_status.rx_err) );
        //
        if( mdb_status.rx_ACK_code == MDB_ACK )
        { // bill types set ok
        }else
        { 
                NOP(); // Nak
        };
        //
        PT_END(pt);
}

/*-------------------------------------------------------
ESCROW 0x35 - проглотить банкноту
-------------------------------------------------------*/
PT_THREAD(mdb_validator_ESCROW_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        tx_data[0] = 1;   // send bill to stacker
        //
        mdb_send_cmd( VCMD_ESCROW, tx_data, 1, 3 ); // set bill types and escrow   
        // 
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&(!mdb_status.rx_data_ready)&&(!mdb_status.rx_err) );
        //
        if( mdb_status.rx_ACK_code == MDB_ACK )
        { // bill types set ok
        }else
        {
                NOP();
        };
        //
        PT_END(pt);
}

/*-------------------------------------------------------
STACKER 0x36
-------------------------------------------------------*/
PT_THREAD(mdb_validator_STACKER_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        mdb_send_cmd( VCMD_STACKER, NULL, 0, 3 );
        // 
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&(!mdb_status.rx_data_ready)&&(!mdb_status.rx_err) );
        //
        if( mdb_status.rx_data_ready && (!mdb_status.rx_err) )
        {
                //
                PT_SPAWN( pt, &mdb_cmd_pt2, mdb_send_ACK_thread(&mdb_cmd_pt2) ); 
                //
                mdb_status.StackerFull = mdb_status.rx_buf[0] & (1<<7); // stacker full flag
                mdb_status.BillsInStacker = ( (uint16_t) (mdb_status.rx_buf[0] & (~(1<<7))) ) << 8; 
                mdb_status.BillsInStacker |= mdb_status.rx_buf[1];
                //  
        } 
        //
        PT_END(pt);
}

/*-------------------------------------------------------
L1 identification without option bits
0x37 0x00
-------------------------------------------------------*/
PT_THREAD(mdb_validator_Level1_ID_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        tx_data[0] = 0x00;
        mdb_send_cmd( VCMD_EXPANSION, tx_data, 1, 3 );
        // 
        PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0)&&(!mdb_status.rx_data_ready)&&(!mdb_status.rx_err) );
        //
        if( mdb_status.rx_data_ready && (!mdb_status.rx_err) )
        {
                //
                PT_SPAWN( pt, &mdb_cmd_pt2, mdb_send_ACK_thread(&mdb_cmd_pt2) ); 
                //
                memcpy( &mdb_status.ID, mdb_status.rx_buf, 29 );  
                //
        };
        //
        PT_END(pt);
}

/*-------------------------------------------------------

-------------------------------------------------------*/
struct pt mdb_bvalidator_pt;

/*-------------------------------------------------------
Функции связи с основной программой
-------------------------------------------------------*/
void mdb_validator_init( void )
{
        PT_INIT( &mdb_bvalidator_pt );
}

void mdb_validator_process( void )
{
        mdb_bvalidator_thread( &mdb_bvalidator_pt );
}

/*-------------------------------------------------------
Поток основной обработки состояний валидатора
-------------------------------------------------------*/
uint8_t accepted_bill_number;
uint8_t check_stacker=0;
uint8_t bill_enable_rq = 0;

PT_THREAD(mdb_bvalidator_thread(struct pt *pt))
{
        uint8_t bill, routing;
        //
        
        PT_BEGIN(pt);
        //
        
        // full mdb hardware bus reset
        mdb_status.JustReset = 0;
        PT_INIT( &mdb_cmd_pt );
        PT_SPAWN( pt, &mdb_cmd_pt, mdb_hard_reset_thread(&mdb_cmd_pt) );
        //
        
        sw_set_timeout(MDB_TIM5, 10*10);
        // validator software reset
        do{      
                // POLL interval
                sw_set_timeout( MDB_TIM4, 2 ); // 200mS poll interval
                PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0) );
                //
                PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_softReset_thread(&mdb_cmd_pt) );
                if(sw_get_timeout(MDB_TIM5)==0)
                { 
                        PT_RESTART(pt); // no response timeout to reset mdb
                };
        }while( mdb_status.rx_ACK_code != MDB_ACK );
        
        sw_set_timeout(MDB_TIM5, 30*10);
        //
        while(1)
        {
                // POLL interval
                sw_set_timeout( MDB_TIM4, 2 ); // 200mS poll interval
                PT_WAIT_WHILE( pt, (sw_get_timeout(MDB_TIM4)>0) );
                //
                PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_POLL_thread(&mdb_cmd_pt) );
                if( mdb_status.rx_data_ready && (!mdb_status.rx_err) )
                {
                        //
                        sw_set_timeout(MDB_TIM5, 30*10);
                        //
                        if( mdb_status.JustReset ) // JUST RESET -- init sequence
                        {
                                //
                                PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_SETUP_thread(&mdb_cmd_pt) ); // SETUP
                                //
                                PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_Level1_ID_thread(&mdb_cmd_pt) ); // Level 1 ID
                                //
                                #ifdef MDB_MST_CHECK_STACKER // проверить полный ли стекер и запретить приём банкнот
                                PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_STACKER_thread(&mdb_cmd_pt) );
                                //
                                if( mdb_status.StackerFull )
                                {       // disable all bill types when stacker full
                                        accepted_bill_types = 0x0000;
                                        bill_escrow_enable  = 0x0000;
                                        mdb_status.Disabled = 1;        
                                }else
                                {       // all bill types enabled
                                        accepted_bill_types = 0xFFFF;
                                        bill_escrow_enable  = 0xFFFF;
                                        mdb_status.Disabled = 0;
                                };
                                #else  MDB_MST_CHECK_STACKER // do not check stacker - когда нет стекера или датчика стекера
                                accepted_bill_types = 0xFFFF;
                                bill_escrow_enable  = 0xFFFF;
                                #endif MDB_MST_CHECK_STACKER
                                PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_BILL_TYPE_thread(&mdb_cmd_pt) ); // set BILL TYPES
                                //
                                mdb_status.JustReset = 0; 
                        }else
                        {
                            if( mdb_status.rx_ACK_code != MDB_ACK ) // вставлена банкнота или пришёл статусный код
                            {
                                // bill accepted
                                bill = mdb_status.rx_buf[0];
                                if( bill & VRSP_BILL_ACCEPTED_FLAG ) // флаг банкнота принята  Z1=1yyyxxxx
                                {
                                        accepted_bill_number = bill & VRSP_POLL_BILL_TYPE_MASK; // Z1=0000xxxx
                                        routing = bill & VRSP_BILL_ROUTING_MASK; // 01110000
                                        if( routing == VRSP_BILL_STACKED )
                                        { 
                                            //
                                            #ifdef MDB_MST_CHECK_STACKER // проверять полный ли стекер и тогда запретить приём банкнот
                                            PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_STACKER_thread(&mdb_cmd_pt) );
                                            //
                                            if( mdb_status.StackerFull )
                                            {       // disable all bill types when stacker full
                                                    accepted_bill_types = 0x0000;
                                                    bill_escrow_enable  = 0x0000;
                                                    mdb_status.Disabled = 1;
                                                    PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_BILL_TYPE_thread(&mdb_cmd_pt) ); // Disable BILL TYPES        
                                            };
                                            //
                                            #endif MDB_MST_CHECK_STACKER
                                            //
                                                
                                            // *** Vend valid action ***
                                            // send money to system
                                            // записать кредит в ОЗУ
                                            status_add_money( bill_values_table[accepted_bill_number] );
                                            status_add_bill();
                                            //
                                            copy_state2record();
                                            write_log_record(); // добавить запись в журнал
                                            //
                                            generate_credit_pulse( bill_values_table[accepted_bill_number] );
                                            //
                                        };
                                        if( routing == VRSP_BILL_ESCROW ) 
                                        {
                                            PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_ESCROW_thread(&mdb_cmd_pt) );
                                        };
                                        if( routing == VRSP_BILL_RETURNED ) {};
                                        if( routing == VRSP_BILL_RECYCLER) {};
                                        if( routing == VRSP_BILL_REJECTED) {};
                                        if( routing == VRSP_BILL_RECYCLER_MANUAL) {};
                                        if( routing == VRSP_BILL_DISPENSE) {};
                                        if( routing == VRSP_BILL_CASHBOX) {};
                                        
                                }else if( bill & 0x0F )
                                { 
                                    if( bill == VRSP_POLL_DEFECTIVE_MOTOR ){ ba_status.failure = 1; };
                                    if( bill == VRSP_POLL_SENSOR_PROBLEM ) { ba_status.failure = 1; };
                                    if( bill == VRSP_POLL_VALIDATOR_BUSY) {};
                                    if( bill == VRSP_POLL_ROM_ERROR) { ba_status.failure = 1; };
                                    if( bill == VRSP_POLL_JAMMED) { ba_status.failure = 1; };
                                    if( bill == VRSP_POLL_RESET) {};
                                    if( bill == VRSP_POLL_REMOVED) {};
                                    if( bill == VRSP_POLL_CASHBOX_OPEN) {};
                                    if( bill == VRSP_POLL_DISABLED) 
                                    { 
                                        //ba_status.enabled = 0;                 
                                    };
                                    if( bill == VRSP_POLL_REJECTED) {};
                                    if( bill == VRSP_POLL_CREDIT_REMOVED ){};
                                };
                                // Авторазрешение приёма банкнот
                                #ifdef MDB_MST_BILL_AUTOENABLE
                                if( mdb_status.Disabled && (!mdb_status.StackerFull) )
                                {
                                    // all bill types enabled
                                    accepted_bill_types = 0xFFFF;
                                    bill_escrow_enable  = 0xFFFF;
                                    PT_SPAWN( pt, &mdb_cmd_pt, mdb_validator_BILL_TYPE_thread(&mdb_cmd_pt) ); // set BILL TYPES
                                };
                                #endif MDB_MST_BILL_AUTOENABLE
                                //
                            };
                        }; 
                };
                
                //
                if(sw_get_timeout(MDB_TIM5)==0)
                { 
                        PT_RESTART(pt); // no response timeout to reset mdb
                };
                //
                /*
                if( ba_status.inhibit_rq )
                {        
                        ba_send_cmd( BA_CMD_INHIBIT );
                        ba_status.inhibit_rq = 0;
                };
                */
        };
        
        //
        PT_END(pt);
}


