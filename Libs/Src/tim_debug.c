#include "tim_debug.h"
#include "FreeRTOS.h"
#include "task.h"

// Estructura global de debug
TIM2_Debug_t tim2_debug = {0};

// Variables privadas para tracking de ISRs
static uint32_t isr_count_total = 0;
static uint32_t isr_count_per_sec = 0;
static uint32_t last_update_tick = 0;

void TIM2_Debug_Init(void) {
    tim2_debug.autoreload_value = 0;
    tim2_debug.current_frequency_hz = 0;
    tim2_debug.isr_count = 0;
    tim2_debug.isr_frequency_per_sec = 0;
    tim2_debug.is_running = 0;
    tim2_debug.error_flags = 0;
    tim2_debug.last_update_ms = 0;
    
    isr_count_total = 0;
    isr_count_per_sec = 0;
    last_update_tick = xTaskGetTickCount();
}

// Se debe llamar desde la ISR del timer (TIM2_IRQHandler)
void TIM2_Debug_ISR_Callback(void) {
    isr_count_total++;
    isr_count_per_sec++;
}

// Se debe llamar periódicamente desde ControlTask
void TIM2_Debug_Update(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM2) return;
    
    // 1. Leer AUTORELOAD (controla la frecuencia)
    tim2_debug.autoreload_value = __HAL_TIM_GET_AUTORELOAD(htim);
    
    // 2. Calcular frecuencia teórica
    // Timer clock = 2MHz, cada período = AUTORELOAD counts
    // Hz = 2000000 / AUTORELOAD
    if (tim2_debug.autoreload_value > 0) {
        tim2_debug.current_frequency_hz = 2000000 / tim2_debug.autoreload_value;
    } else {
        tim2_debug.current_frequency_hz = 0;
        tim2_debug.error_flags |= 0x01;  // Error: AUTORELOAD = 0
    }
    
    // 3. Actualizar contador total de ISRs
    tim2_debug.isr_count = isr_count_total;
    
    // 4. Calcular ISRs por segundo cada ~100ms
    uint32_t current_tick = xTaskGetTickCount();
    uint32_t tick_diff = current_tick - last_update_tick;
    
    if (tick_diff >= 100) {  // Cada 100ms
        // ISRs por segundo = ISRs en 100ms × 10
        tim2_debug.isr_frequency_per_sec = isr_count_per_sec * 10;
        isr_count_per_sec = 0;
        last_update_tick = current_tick;
    }
    
    // 5. Verificar si el timer está corriendo
    tim2_debug.is_running = (htim->State == HAL_TIM_STATE_BUSY) ? 1 : 0;
    
    // 6. Guardar timestamp de última actualización
    tim2_debug.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void TIM2_Debug_GetReport(TIM2_Debug_t *report) {
    if (report == NULL) return;
    *report = tim2_debug;
}
