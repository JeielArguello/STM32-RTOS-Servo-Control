#ifndef HID_MANAGER_H
#define HID_MANAGER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ============= Estructuras HID ============= */

typedef struct __attribute__((packed)) {
    uint8_t reportID;     // ID del reporte HID
    int32_t position;     // Posición actual del motor
    int32_t velocity;    // Velocidad calculada
    uint8_t status_flags; // Errores, límites, etc.
} HID_Report_t;

typedef enum {
    HID_STATUS_OK = 0x00,
    HID_STATUS_ERROR_MOTOR = 0x01,
    HID_STATUS_ERROR_SENSOR = 0x02,
    HID_STATUS_MOTOR_MOVING = 0x04,
    HID_STATUS_MOTOR_STOPPED = 0x08,
} HID_Status_t;

/* ============= Funciones de Interface ============= */

/**
 * @brief Inicializa el gestor HID
 * @note Debe ser llamado una sola vez durante la inicialización
 */
void HID_Manager_Init(void);

/**
 * @brief Actualiza los datos HID con nueva información
 * @param position: Nueva posición del motor
 * @param velocity: Nueva velocidad
 * @param status: Flags de estado
 * @note Función thread-safe usando mutex
 */
void HID_Manager_Update(int32_t position, int32_t velocity, uint8_t status);

/**
 * @brief Envía el reporte HID actual al host
 * @return HAL_OK si se envió correctamente
 */
HAL_StatusTypeDef HID_Manager_SendReport(void);

/**
 * @brief Obtiene el reporte actual
 * @return Puntero a la estructura del reporte
 */
HID_Report_t* HID_Manager_GetReport(void);

/**
 * @brief Obtiene el mutex del reporte para acceso sincronizado
 * @return Handle al mutex
 */
SemaphoreHandle_t HID_Manager_GetMutex(void);

/**
 * @brief Establece el flag de estado del HID
 * @param status: Flag a establecer (ver HID_Status_t)
 */
void HID_Manager_SetStatus(HID_Status_t status);

/**
 * @brief Limpia el flag de estado del HID
 * @param status: Flag a limpiar
 */
void HID_Manager_ClearStatus(HID_Status_t status);

/**
 * @brief Obtiene el flag de estado actual
 * @return Byte con flags de estado
 */
uint8_t HID_Manager_GetStatus(void);

/**
 * @brief Resetea el gestor HID a su estado inicial
 */
void HID_Manager_Reset(void);

#endif /* HID_MANAGER_H */
