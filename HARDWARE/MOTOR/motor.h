#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"

#define dirction PEout(5)
#define motor_enable PEout(6)
#define Pluse PBout(8)	// Âö³å

void Motor_Init(void);
void Encoder_Init(void);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM3_Int_Init(void);
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM5_Init(uint32_t prd,uint32_t psc);

#endif
