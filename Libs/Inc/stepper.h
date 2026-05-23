#ifndef STEPPER_H_
#define STEPPER_H_

#include "stm32f1xx_hal.h"
#include <stdlib.h>

// Enumeración para legibilidad del microstepping
typedef enum {
    STEP_FULL     = 1,
    STEP_HALF     = 2,
    STEP_QUARTER  = 4,
    STEP_EIGHTH   = 8,
    STEP_SIXTEENTH = 16,
    STEP_THIRTYTWO = 32
} Stepper_Mode;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} Stepper_Pin;

typedef struct {
    // Configuración de Hardware
    Stepper_Pin step;
    Stepper_Pin dir;
    Stepper_Pin m0, m1, m2;

    TIM_HandleTypeDef* htim;
    uint32_t channel;

    // Estado de Control
    volatile uint32_t steps_to_move;
    uint32_t period_ticks;
    Stepper_Mode current_mode;
} Stepper_Handler;

// Funciones Principales
void Stepper_Init(Stepper_Handler* hmotor);
void Stepper_SetMicrostepping(Stepper_Handler* hmotor, Stepper_Mode mode);
HAL_StatusTypeDef Stepper_Move(Stepper_Handler* hmotor, int32_t steps, uint32_t speed_hz);
void Stepper_Stop(Stepper_Handler* hmotor);

#endif
