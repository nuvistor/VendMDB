/*-------------------------------------------------------
generate_pulse.h
Генерация и вывод импульсов денег на крсело
-------------------------------------------------------*/
#ifndef _GENERATE_PULSE_H
#define _GENERATE_PULSE_H

extern struct pt credit_pulse_pt;

PT_THREAD(credit_pulse_thread(struct pt *pt));
void generate_credit_pulse( uint32_t crd );


#endif  _GENERATE_PULSE_H