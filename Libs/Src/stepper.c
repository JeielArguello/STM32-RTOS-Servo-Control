#include "stepper.h"

// Conversión RPM a Hz (parametrizada por modo de microstepping)
// pasos_totales = 200 × modo
// Hz = (RPM × 200 × modo × 2 toggles/paso) / 60 seg
uint32_t Stepper_RPM_to_Hz(Stepper_Handler* hmotor, int32_t rpm) {
    if (rpm < 0) rpm = -rpm;
    uint32_t pasos_por_vuelta = 200 * (hmotor->current_mode);
    return (rpm * pasos_por_vuelta * 2) / 60;
}

// Conversión Hz a RPM (inversa, parametrizada por modo)
// RPM = (Hz × 60) / (200 × modo × 2)
int32_t Stepper_Hz_to_RPM(Stepper_Handler* hmotor, int32_t hz) {
    uint32_t pasos_por_vuelta = 200 * (hmotor->current_mode);
    return (hz * 60) / (pasos_por_vuelta * 2);
}


void Stepper_Init(Stepper_Handler* hmotor) {
    // Aseguramos que el motor esté detenido al iniciar
    hmotor->steps_to_move = 0;
    hmotor->period_ticks = 0;
    hmotor->is_running = 0;  // Inicializar como detenido
    // Setear CCR = 15 que garantice el ancho de pulso mínimo que exige el datasheet del DRV8825
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->channel, 5);
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
        case STEP_HALF:
        	HAL_GPIO_WritePin(hmotor->m0.port, hmotor->m0.pin, 1);
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

void Stepper_SetSteps(Stepper_Handler* hmotor, int32_t steps) {
    // Setear dirección basada en el signo de steps
    HAL_GPIO_WritePin(hmotor->dir.port, hmotor->dir.pin, (steps > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    // Cantidad de pasos a ejecutar (cada paso = 2 toggles en PWM mode)
    hmotor->steps_to_move = abs(steps);
}

void Stepper_SetSpeed(Stepper_Handler* hmotor, int32_t speed_rpm) {
    // speed_rpm: velocidad en RPM 
    // La conversión a Hz ocurre aquí internamente
    
    HAL_GPIO_WritePin(hmotor->dir.port, hmotor->dir.pin, (speed_rpm > 0) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    if(hmotor->current_rpm == speed_rpm) {
        return;
    }
    hmotor->current_rpm = speed_rpm;  // Guardar RPM actual para referencia (ej. cálculo de velocidad en SensorTask)

    // Convertir RPM a Hz
    uint32_t speed_hz = Stepper_RPM_to_Hz(hmotor, speed_rpm);
    speed_hz = (speed_hz == 0) ? 1 : speed_hz;  // Evitar división por cero
    
    
    uint32_t arr_value = (1000000  / speed_hz) - 1;
    // Limitar para timer de 16 bits
    if (arr_value > 65535) {
        arr_value = 65535;
    }
    if (arr_value < 10) {
        arr_value = 10;  // Mínimo para estabilidad
    }
    
    // Guardar para referencia
    hmotor->period_ticks = arr_value;
    

    // Setear AUTORELOAD (controla la frecuencia del PWM)
    __HAL_TIM_SET_AUTORELOAD(hmotor->htim, arr_value);
}

HAL_StatusTypeDef Stepper_Start(Stepper_Handler* hmotor) {
    if (hmotor->steps_to_move == 0) return HAL_ERROR;
    
    // Verificar si el timer ya está activo (evitar llamar Start dos veces)
    if (hmotor->is_running) return HAL_OK;  // Ya está corriendo, no hacer nada
    
    // Limpiar flags y iniciar en PWM mode
    // El ISR se dispara una vez por período (1 ISR = 1 paso)
    __HAL_TIM_CLEAR_FLAG(hmotor->htim, TIM_FLAG_CC2);
    HAL_StatusTypeDef status = HAL_TIM_OC_Start(hmotor->htim, hmotor->channel);
    
    // Marcar como activo si la llamada fue exitosa
    if (status == HAL_OK) {
        hmotor->is_running = 1;
    }
    
    return status;
}

void Stepper_Stop(Stepper_Handler* hmotor) {
    HAL_TIM_OC_Stop(hmotor->htim, hmotor->channel);
    hmotor->steps_to_move = 0;
    hmotor->is_running = 0;  // Marcar como detenido
    hmotor->current_rpm = 0;  
}
