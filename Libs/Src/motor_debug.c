#include "motor_debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

/* ============= Variables Globales ============= */

static MotorDebug_t motor_data = {0};

/* ============= Funciones Implementadas ============= */

void MotorDebug_Init(void) {
    memset(&motor_data, 0, sizeof(MotorDebug_t));
    motor_data.pos_actual = 0;
    motor_data.pos_max_alcanzada = 0;
    motor_data.error_pos_max = 0;
    motor_data.error_pos_actual = 0;
    motor_data.start_time_ms = 0;
    motor_data.end_time_ms = 0;
    motor_data.duration_ms = 0;
    motor_data.convergence_time_ms = 0;
    motor_data.converged = 0;
    motor_data.active = 0;
    motor_data.samples_collected = 0;
}

void MotorDebug_Start(int32_t setpoint) {
    // Setpoint es genérico (puede representar grados o RPM según uso)
    motor_data.setpoint = setpoint;
    motor_data.pos_actual = 0;
    motor_data.pos_max_alcanzada = 0;
    motor_data.error_pos_max = (setpoint < 0) ? -setpoint : setpoint;  // Error inicial = |setpoint|
    motor_data.error_pos_actual = (setpoint < 0) ? -setpoint : setpoint;
    motor_data.start_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    motor_data.end_time_ms = 0;
    motor_data.duration_ms = 0;
    motor_data.convergence_time_ms = 0;
    motor_data.converged = 0;
    motor_data.active = 1;
    motor_data.samples_collected = 0;
}

void MotorDebug_Update(int32_t pos_actual, int32_t error_pos) {
    if (!motor_data.active) return;
    
    motor_data.pos_actual = pos_actual;
    motor_data.samples_collected++;
    
    // Actualizar posición máxima alcanzada
    if (pos_actual < 0) pos_actual = -pos_actual;  // valor absoluto
    if (pos_actual > motor_data.pos_max_alcanzada) {
        motor_data.pos_max_alcanzada = pos_actual;
    }
    
    // Actualizar error máximo
    if (error_pos < 0) error_pos = -error_pos;  // valor absoluto
    if (error_pos > motor_data.error_pos_max) {
        motor_data.error_pos_max = error_pos;
    }
    motor_data.error_pos_actual = error_pos;
    
    // Detectar convergencia (cuando |error| < 5 grados)
    int32_t tolerancia = 5;  // ±5 grados de tolerancia
    
    if (!motor_data.converged && error_pos < tolerancia) {
        motor_data.converged = 1;
        uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        motor_data.convergence_time_ms = current_ms - motor_data.start_time_ms;
    }
}

void MotorDebug_Stop(void) {
    if (!motor_data.active) return;
    
    uint32_t current_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    motor_data.end_time_ms = current_ms;
    motor_data.duration_ms = motor_data.end_time_ms - motor_data.start_time_ms;
    motor_data.active = 0;
}

MotorDebug_t* MotorDebug_GetData(void) {
    return &motor_data;
}
