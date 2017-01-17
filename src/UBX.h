#ifndef MGC_UBX_H
#define MGC_UBX_H

#include <avr/io.h>

extern uint8_t  UBX_model;
extern uint16_t UBX_rate;

void UBX_Init(void);
void UBX_Task(void);
void UBX_Update(void);

#endif
