/*-------------------------------------------------------
mdb_slave.c
Bill Validator Emulation
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
#include "mdb_slave.h"

/*-------------------------------------------------------
USARTx Registers
-------------------------------------------------------*/
#define MDB_SLA_UDRx     UDR1
#define MDB_SLA_UCSRxA   UCSR1A
#define MDB_SLA_UCSRxB   UCSR1B
#define MDB_SLA_UCSRxC   UCSR1C
#define MDB_SLA_UBRRxH   UBRR1H
#define MDB_SLA_UBRRxL   UBRR1L

#define MDB_SLA_TX_DIR  DDRD
#define MDB_SLA_TX_PIN  3
#define MDB_SLA_TX_PORT PORTD

#define MDB_SLA_RX_DIR  DDRD
#define MDB_SLA_RX_PIN  2
#define MDB_SLA_RX_PORT PORTD

/*-------------------------------------------------------

-------------------------------------------------------*/
#define MDB_MAX_BLOCK_SZ        36

/*-------------------------------------------------------
MDB response codes
-------------------------------------------------------*/
#define MDB_ACK 0x00 // acknowledgment/checksum correct
#define MDB_NAK 0xFF // Retransmit the previously sent data.
                     // Only the VMC can transmit this byte
#define MDB_RET 0xAA // Negative acknowledge


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
//
#define VCMD_EXP_L1_ID   0x00  // LEVEL 1 IDENTIFICATION WITHOUT OPTION BITS

/*-------------------------------------------------------
Slave BV responses
-------------------------------------------------------*/

// respone to SETUP/STATUS 0x31
typedef struct{
        uint8_t FeatureLevel; // 1=Level1, 2=Level2
        uint8_t CountryCode[2];
        uint8_t ScalingFactor[2];
        uint8_t DecimalPlaces;
        uint8_t StackerCapacity[2];
        uint8_t SecurityLevels[2];
        uint8_t Escrow; // 0=have no escrow, 1=has escrow 
        uint8_t BillType[16];
}sbv_rsp_setup_t;

// response to SECURITY 0x32
typedef struct{
        uint8_t SecurityLevels[2]; 
}sbv_rsp_security_t;

// response to POLL 0x33
typedef struct{
        uint8_t Activity[16];
}bv_rsp_poll_t;

// Response Bill Accepted Status 
// bill audit and routing accepted mask: 1xxx0000
#define VRSP_POLL_ACCEPTED_FLAG       (1<<7)
#define VRSP_POLL_ROUTING_MASK        0xF0
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
#define VRSP_STAT_DEFECTIVE_MOTOR        0x01 // One of the motors has failed to perform its expected assignment.
#define VRSP_STAT_SENSOR_PROBLEM         0x02 // One of the sensors has failed to provide its response
#define VRSP_STAT_VALIDATOR_BUSY         0x03 // The validator is busy and can not answer a detailed command right now
#define VRSP_STAT_ROM_ERROR              0x04 // The validators internal ecksum does not match the calculated checksum
#define VRSP_STAT_JAMMED                 0x05 // A bill(s) has jammed in the acceptance path
#define VRSP_STAT_RESET                  0x06 // The validator has been reset since the last POLL
#define VRSP_STAT_REMOVED                0x07 // A bill in the escrow position has been removed by an unknown means
#define VRSP_STAT_CASHBOX_OPEN           0x08 // The validator has tected the cash box to be open or removed
#define VRSP_STAT_DISABLED               0x09 // The validator has been disabled, by the VMC or because of internal conditions
#define VRSP_STAT_INVALID_ESCROW         0x0A // An ESCROW command was requested for a bill not in the escrow position
#define VRSP_STAT_REJECTED               0x0B // A bill was detected, but rejected because it could not be identified
#define VRSP_STAT_CREDIT_REMOVED         0x0C // There has been an attempt to remove a credited (stacked) bill
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
// not used in Level 1

// response to BILL TYPE 0x34
typedef struct{
        uint8_t BillEnable[2];
        uint8_t EscrowEnable[2];
} sbv_bill_type_t;


// response to ESCROW 0x35
typedef struct{
        uint8_t EscrowStatus; // 0=return bill in the escrpw position
                              // xxxxxxx1=stack the bill
} sbv_rsp_ecrow_t;

// response to STACKER 0x36
typedef struct{
        uint8_t BillsNumber[2]; // bill number of full condition
} sbv_rsp_stacker_t;

// response to EXP L1 ID
typedef struct{
        uint8_t MfCode[3];      // ascii
        uint8_t SerialNum[12];  // ascii
        uint8_t ModelNum[12];   // ascii
        uint8_t SoftVersion[2]; // BCD
} sbv_rsp_id_t;

// Bill Validator state
typedef struct{
        // rx buf
        uint8_t rx_buf[MDB_MAX_BLOCK_SZ];
        uint8_t rx_byte_cnt;
        uint8_t rx_block_sz; // длина принимаего блока для этой команды
        uint8_t rx_block_enable;
        uint8_t rx_exp_cmd; // expansion command
        uint8_t rx_chk; // checksum
        // tx buf
        uint8_t tx_buf[MDB_MAX_BLOCK_SZ];
        uint8_t tx_byte_cnt;
        uint8_t tx_sz;
        uint8_t tx_chk; // check sum
        uint8_t tx_use_chk;
        uint8_t vmc_ack_rq; // 0=не требуется/ACK пришло, 1=требуется
        // status
        //
        uint16_t ScalingFactor;
        uint32_t DecimalPlacesK;
        uint16_t BillEnable;
        uint16_t EscrowEnable;
        //
        volatile      
        uint8_t state;
        uint8_t bill_insert_idx;
        volatile
        uint8_t bill_processing;
        volatile 
        uint8_t bill_rejected;
        uint8_t invalid_escrow;
        uint8_t error_code;
        
} mdb_bv_status_t;

enum{
BV_STA_JUST_RESET=0,
BV_STA_JUST_RESET_WAIT_ACK,
BV_STA_IDLE,
BV_STA_BILL_ESCROW,
BV_STA_BILL_STACKED,
BV_STA_BILL_REJECTED,
BV_STA_BILL_REMOVED,
// Error states
BV_STA_DEFECTIVE_MOTOR,
BV_STA_SENSOR_PROBLEM,
BV_STA_ROM_ERROR,
BV_STA_JAMMED
};

enum{
SLAVE_BV_CREDIT_PROCESS_START=0,
SLAVE_BV_CREDIT_PROCESS_END,
SLAVE_BV_CREDIT_PROCESS_ERROR,
};

// bill values lookup table
typedef struct{
        uint32_t bill[16];
        uint8_t  total;
} sbv_bill_values_t;

/*-------------------------------------------------------

-------------------------------------------------------*/
mdb_bv_status_t bv_status; // slave bill validator status
// Factory data constant response
//flash 
sbv_rsp_setup_t rsp_setup={
0x01,        // feature level
{0x18,0x60}, // currency code bcd
{0xc3,0x50}, // scaling factor
0x02,        // decimal places
{0x00,0xc8}, // stacker capacity
{0x00,0x1f}, // security levels
0xff,        // escrow
{0x01,0x02,0x0A,0x14,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}// bill type credit
};
flash 
sbv_rsp_id_t rsp_level1_id={
{0x49,0x43,0x54},// manufacture code
{0x2f,0x2f,0x2f,0x2f,0x2f,0x2f,0x2f,0x2f,0x2f,0x2f,0x2f,0x2f},//serial number
{0x4c,0x38,0x33,0x2b,0x55,0x5a,0x53,0x35,0x30,0x30,0x30,0x30},//model number/tuning revision
{0x02,0x03}//software version
};
// 
sbv_rsp_security_t rsp_security={0x00, 0x00}; // set security to bills (2 bytes, vmc cmd 0x32)
sbv_bill_type_t rsp_bill_type_en;//={}; // set accepted bill types(4 bytes, vmc cmd 0x34)

sbv_rsp_stacker_t rsp_stacker; // get number of bills in stacker

sbv_bill_values_t sbv_bill_values;

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_rx_byte( uint8_t data, uint8_t mode );
uint8_t mdb_get_chk( uint8_t *buf, uint8_t sz );
void slave_set_bill_value( uint8_t idx, uint32_t val );
void slave_set_bv_scale_factor( uint16_t scale_factor );
void slave_set_bv_decimal_k( uint8_t dp );
void slave_calc_bill_cnt( void );
void slave_set_bv_bill_cnt( uint8_t total );

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_uart_init( void )
{
    CLI();
    SET_BIT( MDB_SLA_TX_DIR, MDB_SLA_TX_PIN ); // TX pin = out
    SET_BIT( MDB_SLA_TX_PORT, MDB_SLA_TX_PIN ); // =1
    //
    CLR_BIT( MDB_SLA_RX_DIR, MDB_SLA_RX_PIN ); // RX pin = in
    SET_BIT( MDB_SLA_RX_PORT, MDB_SLA_RX_PIN ); // pull-up
    SEI();
    // USART2 initialization
    // Communication Parameters: 9 Data, 1 Stop, No Parity
    // USART2 Receiver: On
    // USART2 Transmitter: On
    // USART2 Mode: Asynchronous
    // USART2 Baud Rate: 9600
    MDB_SLA_UCSRxA=0x00;
    MDB_SLA_UCSRxB=0xDC;
    MDB_SLA_UCSRxC=0x06;
    MDB_SLA_UBRRxH=0x00;
    MDB_SLA_UBRRxL=0x67;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_purge_rx_buf( void )
{
        bv_status.rx_byte_cnt = 0;
        //mdb_status.rx_data_ready = 0;
        //bv_status.rx_err = 0;
        bv_status.rx_chk = 0;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_purge_tx_buf( void )
{
        bv_status.tx_byte_cnt = 0;
        //mdb_bv_status_t.tx_busy = 0; // lock transmitter
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_rx_start( void )
{
        mdb_slave_purge_rx_buf();
        bv_status.rx_block_enable = 1;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_tx_start( void )
{
        //
        mdb_slave_purge_tx_buf();
        //
        if( !bv_status.tx_use_chk )
        { // adress or ack/nak/ret
                //CLI();
                SET_BIT(MDB_SLA_UCSRxB, TXB8); // MODE BIT = 1
                //SEI(); 
        }else
        {// data/chk  
                //CLI();
                CLR_BIT(MDB_SLA_UCSRxB, TXB8); // MODE BIT = 0
                //SEI(); 
        };                                                         
        bv_status.tx_chk = bv_status.tx_buf[0];
        MDB_SLA_UDRx = bv_status.tx_buf[0];
        bv_status.tx_byte_cnt++;   
}

/*-------------------------------------------------------
RXC
-------------------------------------------------------*/
void mdb_slave_rx_irq_handler( void )
{
    char status,data,mode;
    
    status= MDB_SLA_UCSRxA;
    mode  = MDB_SLA_UCSRxB & (1<<RXB8);
    data  = MDB_SLA_UDRx;
    
    if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
    {
       mdb_slave_rx_byte( data, mode );
    }
}

/*-------------------------------------------------------
TXC
-------------------------------------------------------*/
void mdb_slave_tx_irq_handler( void )
{
        if( !bv_status.tx_use_chk ) // ack ?
        { 
                return;
        };
        //
        if( bv_status.tx_byte_cnt < bv_status.tx_sz )
        {
                bv_status.tx_chk += bv_status.tx_buf[ bv_status.tx_byte_cnt ]; 
                MDB_SLA_UDRx = bv_status.tx_buf[ bv_status.tx_byte_cnt++ ];
        }else if(bv_status.tx_byte_cnt == bv_status.tx_sz)   // DATA ?
        { 
                bv_status.tx_byte_cnt++;
                SET_BIT(MDB_SLA_UCSRxB, TXB8); // MODE BIT = 1 => chk 
                MDB_SLA_UDRx = bv_status.tx_chk;              
        };
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void mdb_slave_rx_byte( uint8_t data, uint8_t mode )
{ 
        //
        if( mode ) // cmd
        { 
                bv_status.rx_chk = data;
                bv_status.rx_byte_cnt = 0;
                bv_status.rx_buf[ bv_status.rx_byte_cnt++ ] = data;
                // set block size
                switch( data ) // первый байт - начало пакета
                { 
                        case VCMD_RESET:
                        case VCMD_SETUP:
                        case VCMD_POLL:
                        case VCMD_STACKER:
                                bv_status.rx_block_sz = 2;
                                bv_status.rx_block_enable = 1;
                                break;
                        case VCMD_SECURITY:
                                bv_status.rx_block_sz = 4;
                                bv_status.rx_block_enable = 1;
                                break;
                        case VCMD_BILL_TYPE:
                                bv_status.rx_block_sz = 6;
                                bv_status.rx_block_enable = 1;
                                break;
                        case VCMD_ESCROW:
                                bv_status.rx_block_sz = 3;
                                bv_status.rx_block_enable = 1;
                                break;
                        case VCMD_EXPANSION:
                                // EXP , Sub-cmd, Chk
                                bv_status.rx_block_sz = 3;
                                bv_status.rx_block_enable = 1;
                                bv_status.rx_exp_cmd = 1;
                                break; 
                };
        }else
        {       // байты пакета после команды
                if( bv_status.rx_block_enable )
                {   //
                    bv_status.rx_buf[ bv_status.rx_byte_cnt++ ] = data;
                    //
                    // expansion sub-command
                    if( bv_status.rx_exp_cmd && (bv_status.rx_byte_cnt==2) ) 
                    { 
                        // set rx data block size
                        switch( data )
                        {
                                case VCMD_EXP_L1_ID: // Level 1 ID without Option bits
                                        bv_status.rx_block_sz = 3;
                                        break;
                        };
                    };
                    //
                    if( bv_status.rx_byte_cnt == bv_status.rx_block_sz ) // end of block?
                    {
                            bv_status.rx_block_enable = 0;
                            //
                            if( bv_status.rx_chk == bv_status.rx_buf[ bv_status.rx_block_sz-1 ] ) // checksum
                            {  
                                switch(bv_status.rx_buf[0])
                                {
                                        case VCMD_RESET: // 0x30
                                                bv_status.state = BV_STA_JUST_RESET;
                                                bv_status.tx_buf[0] = MDB_ACK;
                                                bv_status.tx_sz = 1;
                                                bv_status.tx_use_chk = 0;
                                                mdb_slave_tx_start();
                                                //
                                                // on reset bills and escrow disabled
                                                rsp_bill_type_en.BillEnable[0] = 0;
                                                rsp_bill_type_en.BillEnable[1] = 0;
                                                rsp_bill_type_en.EscrowEnable[0] = 0;
                                                rsp_bill_type_en.EscrowEnable[1] = 0;
                                                bv_status.BillEnable = 0;
                                                bv_status.EscrowEnable = 0;
                                                bv_status.bill_processing = 0;
                                                //
                                                bv_status.invalid_escrow = 0;
                                                bv_status.vmc_ack_rq = 0;  
                                                break;
                                        case VCMD_POLL: // 0x33
                                                //     
                                                //
                                                if( bv_status.invalid_escrow )
                                                { 
                                                        bv_status.invalid_escrow = 0;
                                                        //
                                                        // запрос Escrow в неверном положении( не Escrow )
                                                        bv_status.tx_buf[0] = VRSP_STAT_INVALID_ESCROW;
                                                        bv_status.tx_sz = 1;
                                                        bv_status.tx_use_chk = 1;
                                                        mdb_slave_tx_start();
                                                        bv_status.vmc_ack_rq = 1;
                                                        break;       
                                                };
                                                //
                                                switch( bv_status.state )
                                                { 
                                                        case BV_STA_JUST_RESET:
                                                        case BV_STA_JUST_RESET_WAIT_ACK:
                                                                bv_status.tx_buf[0] = VRSP_STAT_RESET;
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                if( (rsp_bill_type_en.BillEnable[0]==0) && (rsp_bill_type_en.BillEnable[1]==0) )
                                                                {
                                                                        bv_status.tx_buf[1] = VRSP_STAT_DISABLED;
                                                                        bv_status.tx_sz = 2; 
                                                                };
                                                                mdb_slave_tx_start();
                                                                bv_status.state = BV_STA_JUST_RESET_WAIT_ACK;
                                                                bv_status.vmc_ack_rq = 1;   
                                                                break;
                                                        case BV_STA_IDLE:
                                                                bv_status.tx_buf[0] = MDB_ACK;
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 0;
                                                                if( bv_status.BillEnable == 0 )
                                                                {
                                                                        bv_status.tx_buf[0] = VRSP_STAT_DISABLED;
                                                                        //bv_status.tx_sz = 1;
                                                                        bv_status.tx_use_chk = 1;
                                                                };
                                                                mdb_slave_tx_start();
                                                                bv_status.vmc_ack_rq = 1;
                                                                break;
                                                        case BV_STA_BILL_ESCROW:
                                                                bv_status.tx_buf[0] = VRSP_BILL_ESCROW | bv_status.bill_insert_idx;
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                bv_status.vmc_ack_rq = 1;
                                                                break;
                                                        case BV_STA_BILL_STACKED:
                                                                bv_status.tx_buf[0] = VRSP_BILL_STACKED | bv_status.bill_insert_idx;
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                //
                                                                bv_status.vmc_ack_rq = 1;
                                                                //
                                                                break;
                                                        case BV_STA_BILL_REJECTED:
                                                                bv_status.tx_buf[0] = VRSP_BILL_REJECTED | bv_status.bill_insert_idx;                                                               
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                //
                                                                bv_status.vmc_ack_rq = 1;
                                                                //
                                                                break;
                                                        // ERROR CODES
                                                        case BV_STA_DEFECTIVE_MOTOR:
                                                                bv_status.tx_buf[0] = VRSP_STAT_DEFECTIVE_MOTOR;                                                               
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                bv_status.vmc_ack_rq = 1;
                                                                break;
                                                        case BV_STA_SENSOR_PROBLEM:
                                                                bv_status.tx_buf[0] = VRSP_STAT_SENSOR_PROBLEM;                                                               
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                bv_status.vmc_ack_rq = 1;
                                                                break;
                                                        case BV_STA_ROM_ERROR:
                                                                bv_status.tx_buf[0] = VRSP_STAT_ROM_ERROR;                                                               
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                bv_status.vmc_ack_rq = 1;
                                                                break;
                                                        case BV_STA_JAMMED:
                                                                bv_status.tx_buf[0] = VRSP_STAT_JAMMED;                                                               
                                                                bv_status.tx_sz = 1;
                                                                bv_status.tx_use_chk = 1;
                                                                mdb_slave_tx_start();
                                                                bv_status.vmc_ack_rq = 1;
                                                                break;         
                                                };
                                                break;
                                        case VCMD_SETUP: // 0x31
                                                memcpy( bv_status.tx_buf, &rsp_setup, sizeof(rsp_setup) );
                                                bv_status.tx_sz = sizeof(rsp_setup);
                                                bv_status.tx_use_chk = 1;
                                                mdb_slave_tx_start();
                                                //
                                                bv_status.vmc_ack_rq = 1;
                                                //  
                                                break;
                                        case VCMD_SECURITY: // 0x32
                                                // security levels not implemented
                                                bv_status.tx_buf[0] = MDB_ACK;
                                                bv_status.tx_sz = 1;
                                                bv_status.tx_use_chk = 0;
                                                mdb_slave_tx_start();
                                                bv_status.vmc_ack_rq = 0;
                                                break;
                                        case VCMD_BILL_TYPE: // 0x34
                                                // ACK
                                                bv_status.tx_buf[0] = MDB_ACK;
                                                bv_status.tx_sz = 1;
                                                bv_status.tx_use_chk = 0;
                                                mdb_slave_tx_start();
                                                //
                                                memcpy( &rsp_bill_type_en, &bv_status.rx_buf[1], sizeof(rsp_bill_type_en) );
                                                //
                                                bv_status.BillEnable = (rsp_bill_type_en.BillEnable[0]<<8U) + rsp_bill_type_en.BillEnable[1];
                                                bv_status.EscrowEnable = (rsp_bill_type_en.EscrowEnable[0]<<8U) + rsp_bill_type_en.EscrowEnable[1];
                                                //
                                                bv_status.vmc_ack_rq = 0;
                                                break;
                                        case VCMD_ESCROW: // 0x35
                                                bv_status.tx_buf[0] = MDB_ACK;
                                                bv_status.tx_sz = 1;
                                                bv_status.tx_use_chk = 0;
                                                mdb_slave_tx_start();
                                                //
                                                // Escrow action
                                                //
                                                if( bv_status.state == BV_STA_BILL_ESCROW )
                                                {
                                                        if( bv_status.rx_buf[1] & 0x01 )
                                                                bv_status.state = BV_STA_BILL_STACKED;
                                                        else    // VMC отклонил приём кредита
                                                                bv_status.state = BV_STA_BILL_REJECTED; 
                                                }else
                                                {   // неверный запрос escrow
                                                    bv_status.invalid_escrow = 1;    
                                                };
                                                //
                                                bv_status.vmc_ack_rq = 0;
                                                break;
                                        case VCMD_STACKER: // 0x36
                                                // пока не ясно как взаимодействовать со счётчиком стекера
                                                memcpy( bv_status.tx_buf, &rsp_stacker, sizeof(rsp_stacker) );
                                                bv_status.tx_sz = sizeof(rsp_stacker);
                                                bv_status.tx_use_chk = 1;
                                                bv_status.vmc_ack_rq = 1;
                                                //
                                                mdb_slave_tx_start();
                                                //
                                                break;
                                        case VCMD_EXPANSION: // 0x37
                                                switch( bv_status.rx_buf[1] )
                                                {
                                                        case VCMD_EXP_L1_ID: // Level 1 ID without Option bits
                                                            // load from flash
                                                            memcpyf( bv_status.tx_buf, &rsp_level1_id, sizeof(rsp_level1_id) );
                                                            bv_status.tx_sz = sizeof(rsp_level1_id);
                                                            bv_status.tx_use_chk = 1;
                                                            mdb_slave_tx_start(); 
                                                            //
                                                            bv_status.vmc_ack_rq = 1;
                                                            //
                                                            break;
                                                };  
                                                break;
                                };
                            }; 
                    }else
                    {
                        bv_status.rx_chk += data;
                    };
                }else
                {       // ACK-NAK from MASTER?
                        if( bv_status.vmc_ack_rq ) // ack request
                        {
                                switch(data)
                                {
                                        case MDB_ACK:
                                                bv_status.vmc_ack_rq = 0; // подтверждено - теперь если придёт RET, то не будет реакции
                                                //
                                                switch( bv_status.state )
                                                {
                                                        case BV_STA_JUST_RESET_WAIT_ACK:
                                                                bv_status.state = BV_STA_IDLE;
                                                                if( bv_status.error_code != 0xFF )
                                                                {       // перейти в состояние ошибки
                                                                        bv_status.state = bv_status.error_code;
                                                                };
                                                                break;
                                                        case BV_STA_BILL_ESCROW:
                                                                break;
                                                        case BV_STA_BILL_STACKED:
                                                                bv_status.state = BV_STA_IDLE; // vend valid
                                                                bv_status.bill_processing = 0;
                                                                break;
                                                        case BV_STA_BILL_REJECTED:
                                                                bv_status.state = BV_STA_IDLE; // vend not valid
                                                                bv_status.bill_processing = 0;
                                                                bv_status.bill_rejected = 1;
                                                                break;
                                                }; 
                                                break;
                                        case MDB_RET:
                                                // если не было подтверждения и пришёл запрос на повторную передачу
                                                mdb_slave_tx_start();
                                                break;
                                };
                        }; // ack-nak from master
                };
        };
}


/*-------------------------------------------------------

-------------------------------------------------------*/
typedef struct{
        uint8_t b_idx[20];
        uint8_t total;
        uint8_t cnt;
} bill_insert_t;

bill_insert_t bill_to_insert;

uint32_t credit_insert_val=0; // 53000=50000+2000+1000
uint8_t credit_insert_rq=0; // insert request

uint8_t sbv_credit_proc_sta=SBV_CREDIT_IDLE; // credit processing status


/*-------------------------------------------------------
Break credit into single bills
return:
0 = error
1 = success
-------------------------------------------------------*/
uint8_t brk_credit( uint32_t credit )
{
        uint8_t i, j;
        uint8_t max_bill_idx;
        uint32_t max_bill_val;       
        
        // 
        if( bv_status.BillEnable == 0 )
        { 
                return 0;
        }; 
        //
        bill_to_insert.total = 0;
        for( j=0; j<20; j++ )
        {
            max_bill_idx = 0;
            max_bill_val = 0;
            for( i=0; i<sbv_bill_values.total; i++ )
            { 
                    if( (credit >= max_bill_val) && (credit >= sbv_bill_values.bill[i]) && ((1U<<i)&bv_status.BillEnable) ) // найти максимальный номинал
                    { 
                            max_bill_val = sbv_bill_values.bill[i];
                            max_bill_idx = i;
                    };                
            };
            //
            if( max_bill_val > 0 ) // постановка банкноты в очередь
            {
                credit -= max_bill_val;
                bill_to_insert.b_idx[bill_to_insert.total++] = max_bill_idx;
            };
            //
            if( credit == 0 ) // разбивка завершена
            { 
                return 1;
            };
            //   
        };
        //
        if( credit > 0 ) // не удалось разбить целиком
        {
                return 0;  // error
        }else
        { 
                return 1;
        };
        
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void slave_bv_insert_credit( uint32_t credit )
{
        credit_insert_val = credit;
        credit_insert_rq = 1;
        sbv_credit_proc_sta = SBV_CREDIT_PROC_BUSY;        
}

/*-------------------------------------------------------
Slave Bill Validator thread
-------------------------------------------------------*/
struct pt mdb_slave_pt;

PT_THREAD(mdb_slave_bv_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        //
        
        while( 1 )
        { 
                PT_YIELD( pt );
                //
                if( credit_insert_rq )
                { 
                        credit_insert_rq = 0;
                        //
                        //
                        if( bv_status.state == BV_STA_IDLE ) // валидатор готов
                        {
                            if( brk_credit( credit_insert_val) ) // разделить кредит на банкноты
                            {
                                    // Bills Insert
                                    bill_to_insert.cnt = 0;
                                    while( bill_to_insert.cnt < bill_to_insert.total )
                                    { 
                                            if( bv_status.state != BV_STA_IDLE )
                                            { 
                                                sbv_credit_proc_sta = SBV_CREDIT_VALIDATOR_NOT_READY;
                                                break;    
                                            };
                                            bv_status.bill_rejected = 0;
                                            bv_status.bill_insert_idx = bill_to_insert.b_idx[bill_to_insert.cnt++];
                                            bv_status.bill_processing = 1;
                                            if( (1U<<bv_status.bill_insert_idx) & bv_status.EscrowEnable )
                                            { 
                                                    bv_status.state = BV_STA_BILL_ESCROW;
                                            }else   // если Escrow не разрешён, направить банкноту сразу в stacker
                                            {
                                                    bv_status.state = BV_STA_BILL_STACKED;
                                            };
                                            //
                                            // добавить таймаут обработки кредита
                                            //                                       
                                            PT_WAIT_WHILE( pt, bv_status.bill_processing==1 );
                                            //
                                            if( bv_status.bill_rejected )
                                            { // send message to App that VMC rejected bill
                                                    sbv_credit_proc_sta = SBV_CREDIT_REJECTED;
                                                    break;
                                            }
                                            //
                                    };               
                            }else
                            { 
                                    //
                                    // credit break(cut) error
                                    sbv_credit_proc_sta = SBV_CREDIT_BREAK_ERROR;
                                    //
                            };
                        }else
                        { 
                                // валидатор не готов
                                sbv_credit_proc_sta = SBV_CREDIT_VALIDATOR_NOT_READY;
                                //
                        };
                        //
                        if(sbv_credit_proc_sta == SBV_CREDIT_PROC_BUSY)
                        {       // обработка/передача кредита завершена
                                sbv_credit_proc_sta = SBV_CREDIT_PROC_END;       
                        };
                        //
                };
        };
        
        
        //
        PT_END(pt);
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void slave_bv_init( void )
{
        slave_set_bv_decimal_k( 2 );
        slave_set_bv_scale_factor( 50000 );
        //
        slave_set_bill_value( 0, 1000 );
        slave_set_bill_value( 1, 2000 );
        slave_set_bill_value( 2, 5000 );
        slave_set_bill_value( 3, 10000 );
        slave_set_bill_value( 4, 20000 );
        slave_set_bill_value( 5, 50000 );
        slave_set_bill_value( 6, 100000 );
        //
        slave_calc_bill_cnt(); // фиксировать кол-во принимаемых банкнот
        //
        
        PT_INIT( &mdb_slave_pt );
        mdb_slave_uart_init();
        
        // временно разрешить приём всех банкнот для отладки
        //rsp_bill_type_en.BillEnable[0] = 0xFF;
        //rsp_bill_type_en.BillEnable[1] = 0xFF;
        
        bv_status.error_code = 0xFF; // no error  
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void slave_bv_proc( void )
{
        mdb_slave_bv_thread( &mdb_slave_pt );
}

/*-------------------------------------------------------
set new bill value to validator SETUP section
Arguments:
val = new value
idx = index in Setup table
-------------------------------------------------------*/
void slave_set_bill_value( uint8_t idx, uint32_t val )
{
        uint8_t bill_type;
        uint8_t i;
        //
        //
        bill_type = (val * bv_status.DecimalPlacesK)/bv_status.ScalingFactor; 
        //
        rsp_setup.BillType[idx] = bill_type;
        //
        sbv_bill_values.bill[idx] = val;
        //          
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void slave_set_bv_scale_factor( uint16_t scale_factor )
{
        bv_status.ScalingFactor = scale_factor;
}

/*-------------------------------------------------------

-------------------------------------------------------*/
void slave_set_bv_decimal_k( uint8_t dp )
{
        uint32_t k;
        uint8_t i;
        //
        rsp_setup.DecimalPlaces = dp;
        //
        k = 1;
        for( i=0; i<dp; i++ )
        { 
                k*=10;
        };
        //
        bv_status.DecimalPlacesK = k;
}

/*-------------------------------------------------------
вычислить кол-во банкнот в таблице значений sbv_bill_values.total 
-------------------------------------------------------*/
void slave_calc_bill_cnt( void )
{
        uint8_t i,j;
        //
        sbv_bill_values.total = 0;
        j = 15;
        for( i=0; i<16; i++ ) // просматриваем таблицу сверху-вниз
        { 
                if( sbv_bill_values.total>0 ) // если счёт открыт - прибавляем
                {
                        sbv_bill_values.total++;
                }
                else if( rsp_setup.BillType[j]>0 ) // если счёт не открыт - начинаем
                {
                        sbv_bill_values.total++;
                };
                //
                j--;
                //        
        };
}

/*-------------------------------------------------------
установить общее кол-во принимаемых банкнот
-------------------------------------------------------*/
void slave_set_bv_bill_cnt( uint8_t total )
{
        sbv_bill_values.total = total;
}

/*-------------------------------------------------------
Disable bills
-------------------------------------------------------*/
void slave_bv_clear_bills( void )
{
        uint8_t i;
        //
        for( i=0; i<16; i++ )
        { 
                sbv_bill_values.bill[i] = 0;
                //
                rsp_setup.BillType[i] = 0; 
        };                               
        //
        sbv_bill_values.total = 0;
}

/*-------------------------------------------------------
имитация ошибки
если код=0xFF, то возврат в состояние сброса 
-------------------------------------------------------*/
void slave_bv_set_error( uint8_t code )
{
        bv_status.error_code = code;
        if( code != 0xFF )
        {       
                bv_status.state = code;
        }else
        { 
                bv_status.state = BV_STA_JUST_RESET;
        };
}