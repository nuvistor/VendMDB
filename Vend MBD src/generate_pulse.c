/*-------------------------------------------------------
generate_pulse.h
Генерация и вывод импульсов денег на крсело
-------------------------------------------------------*/
#include "main.h"
#include "PT\pt.h"
#include "sw_timers.h"

/*-------------------------------------------------------
Generate credit pulse
-------------------------------------------------------*/
#define SET_CREDIT_HI() PORTK |= (1<<0)
#define SET_CREDIT_LO() PORTK &=~(1<<0)

// соотношение импульсов и денег
#define PULSE_QUANT     1      // импульсов
#define PULSE_PRICE     1000    // на сумму
// перевод кредита в импульсы
#define CREDIT2PULSE(crd)       (((uint32_t)crd*PULSE_QUANT)/PULSE_PRICE)
//
struct pt credit_pulse_pt;
uint32_t credit_pulse = 0;

// generate single credit pulse
//  этот поток генерирует одиночные импульсы пока не закончится счётчик импульсов 
PT_THREAD(credit_pulse_thread(struct pt *pt))
{
        PT_BEGIN(pt);
        
        #ifdef GENERATE_PULSE_ENABLE
        
        PT_WAIT_WHILE( pt, credit_pulse == 0 );
        SET_CREDIT_HI();  // 1
        sw_set_timeout( TIM0_CREDIT_PULSE, 2 );
        PT_WAIT_WHILE( pt, sw_get_timeout(TIM0_CREDIT_PULSE)>0 );
        SET_CREDIT_LO(); // 0
        sw_set_timeout( TIM0_CREDIT_PULSE, 2 );
        PT_WAIT_WHILE( pt, sw_get_timeout(TIM0_CREDIT_PULSE)>0 );
        
        credit_pulse--;
        
        #endif GENERATE_PULSE_ENABLE
        
        PT_END(pt);
}

// перевести кредит в импульсы по прайсу, добавить к счтчику имп.
void generate_credit_pulse( uint32_t crd )
{
        credit_pulse += CREDIT2PULSE( crd );
}