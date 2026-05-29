#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H

#include <stdint.h>

/* ============= Estructura de Debug del Motor (Control de Posición) ============= */

typedef struct {
    // Setpoint
    int32_t setpoint_deg;          // Posición solicitada (en grados)
    
    // Mediciones de posición
    int32_t pos_actual;            // Posición actual del encoder (en grados)
    int32_t pos_max_alcanzada;     // Posición máxima durante la ejecución
    int32_t error_pos_max;         // Error máximo de posición |setpoint - actual|
    int32_t error_pos_actual;      // Error de posición actual
    
    // Tiempos
    uint32_t start_time_ms;        // Tiempo de inicio (en ms desde boot)
    uint32_t end_time_ms;          // Tiempo de fin (en ms desde boot)
    uint32_t duration_ms;          // Duración total (ms)
    uint32_t convergence_time_ms;  // Tiempo para converger al setpoint (ms)
    
    // Estados
    uint8_t converged;             // ¿Llegó a setpoint? (dentro de tolerancia)
    uint8_t active;                // ¿Está en progreso?
    uint32_t samples_collected;    // Cantidad de muestras del encoder procesadas
} MotorDebug_t;

/* ============= Funciones de Interface ============= */

/**
 * @brief Inicializa la estructura de debug del motor
 */
void MotorDebug_Init(void);

/**
 * @brief Inicia el tracking del control de posición
 * @param setpoint_deg: Posición deseada en grados
 */
void MotorDebug_Start(int32_t setpoint_deg);

/**
 * @brief Actualiza las mediciones con los datos actuales del encoder
 * Se debe llamar desde ControlTask en cada ciclo
 * @param pos_actual: Posición actual del encoder (en grados)
 * @param error_pos: Error actual de posición (setpoint_deg - pos_actual)
 */
void MotorDebug_Update(int32_t pos_actual, int32_t error_pos);

/**
 * @brief Finaliza el tracking
 */
void MotorDebug_Stop(void);

/**
 * @brief Obtiene los datos de debug del movimiento actual/anterior
 * @return Puntero a la estructura MotorDebug_t
 */
MotorDebug_t* MotorDebug_GetData(void);

#endif /* MOTOR_DEBUG_H */
