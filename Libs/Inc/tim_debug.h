#ifndef TIM_DEBUG_H
#define TIM_DEBUG_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

typedef struct {
    uint32_t autoreload_value;       // Valor actual de AUTORELOAD (controla Hz)
    uint32_t current_frequency_hz;   // Frecuencia actual configurada
    uint32_t isr_count;              // Total de ISRs ejecutados
    uint32_t isr_frequency_per_sec;  // ISRs por segundo (frecuencia real)
    uint8_t is_running;              // ¿Timer está activo?
    uint8_t error_flags;             // Bits de error del timer
    uint32_t last_update_ms;         // Última vez que se actualizó (ms)
} TIM2_Debug_t;

extern TIM2_Debug_t tim2_debug;

/**
 * Inicializar estructura de debug
 */
void TIM2_Debug_Init(void);

/**
 * Actualizar stats en tiempo real (llamar desde ISR del timer)
 * Incrementa contador de ISR
 */
void TIM2_Debug_ISR_Callback(void);

/**
 * Actualizar información del timer (llamar periódicamente desde ControlTask)
 */
void TIM2_Debug_Update(TIM_HandleTypeDef *htim);

/**
 * Obtener reporte actual
 */
void TIM2_Debug_GetReport(TIM2_Debug_t *report);

#endif // TIM_DEBUG_H
