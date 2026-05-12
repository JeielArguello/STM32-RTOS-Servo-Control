#include "stepper.h"

void Stepper_Init(Stepper_Handler* hmotor) {
    // Aseguramos que el motor esté detenido al iniciar
    Stepper_Stop(hmotor);
}

void Stepper_SetMicrostepping(Stepper_Handler* hmotor, Stepper_Mode mode) {
    hmotor->current_mode = mode;

    // Tabla lógica para DRV8825: M0, M1, M2
    // Full: L,L,L | Half: H,L,L | 1/4: L,H,L | 1/8: H,H,L | 1/16: L,L,H | 1/32: H,L,H (y otros)
    switch(mode) {
        case STEP_FULL:
            HAL_GPIO_WritePin(hmotor->m0.port, hmotor->m0.pin, 0);
            HAL_GPIO_WritePin(hmotor->m1.port, hmotor->m1.pin, 0);
            HAL_GPIO_WritePin(hmotor->m2.port, hmotor->m2.pin, 0);
            break;
        case STEP_SIXTEENTH:
            HAL_GPIO_WritePin(hmotor->m0.port, hmotor->m0.pin, 0);
            HAL_GPIO_WritePin(hmotor->m1.port, hmotor->m1.pin, 0);
            HAL_GPIO_WritePin(hmotor->m2.port, hmotor->m2.pin, 1);
            break;
        case STEP_THIRTYTWO:
            HAL_GPIO_WritePin(hmotor->m0.port, hmotor->m0.pin, 1);
            HAL_GPIO_WritePin(hmotor->m1.port, hmotor->m1.pin, 1);
            HAL_GPIO_WritePin(hmotor->m2.port, hmotor->m2.pin, 1);
            break;
        // ... Agregar los casos restantes según el datasheet del DRV8825
        default: break;
    }
}

void Stepper_Move(Stepper_Handler* hmotor, int32_t steps, uint32_t speed_hz) {
    if (steps == 0) return;

    // 1. Dirección (Pin DIR)
    HAL_GPIO_WritePin(hmotor->dir.port, hmotor->dir.pin, (steps > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // 2. Preparar conteo (cada paso requiere 2 toggles: subida y bajada)
    hmotor->steps_to_move = abs(steps) * 2;

    // 3. Calcular periodo (Asumiendo Timer clock a 1MHz para precisión de 1us)
    // Frecuencia de toggle = speed_hz * 2. Periodo = 1.000.000 / (speed_hz * 2)
    hmotor->period_ticks = 1000000 / (speed_hz * 2);

    // 4. Configurar el primer evento de Output Compare
    uint32_t now = __HAL_TIM_GET_COUNTER(hmotor->htim);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, now + hmotor->period_ticks);

    // 5. Iniciar Timer con Interrupción
    HAL_TIM_OC_Start_IT(hmotor->htim, hmotor->channel);
}

void Stepper_Stop(Stepper_Handler* hmotor) {
    HAL_TIM_OC_Stop_IT(hmotor->htim, hmotor->channel);
    hmotor->steps_to_move = 0;
}
