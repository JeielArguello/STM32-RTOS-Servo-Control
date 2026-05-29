#include "hid_manager.h"
#include "usbd_customhid.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============= Variables Globales ============= */

static HID_Report_t hid_report_buffer = {0};
static SemaphoreHandle_t hid_mutex;
static uint8_t hid_initialized = 0;

// Manejador USB externo (debe estar disponible)
extern USBD_HandleTypeDef hUsbDeviceFS;

/* ============= Funciones Implementadas ============= */

void HID_Manager_Init(void) {
    if (hid_initialized) {
        return;  // Ya inicializado
    }
    
    // Crear mutex para acceso sincronizado al reporte
    hid_mutex = xSemaphoreCreateMutex();
    if (hid_mutex == NULL) {
        Error_Handler();
    }
    
    // Inicializar estructura del reporte
    hid_report_buffer.reportID = 0x01;
    hid_report_buffer.position = 0;
    hid_report_buffer.velocity = 0;
    hid_report_buffer.status_flags = HID_STATUS_OK;
    
    hid_initialized = 1;
}

void HID_Manager_Update(int32_t position, int32_t velocity, uint8_t status) {
    if (hid_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdPASS) {
        hid_report_buffer.reportID = 0x01;
        hid_report_buffer.position = position;
        hid_report_buffer.velocity = velocity;
        hid_report_buffer.status_flags = status;
        xSemaphoreGive(hid_mutex);
    }
}

HAL_StatusTypeDef HID_Manager_SendReport(void) {
    if (hid_mutex == NULL) {
        return HAL_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_OK;  // Default OK (incluso si BUSY, reintentar es normal)
    
    if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdPASS) {
        // Enviar el reporte por USB HID
        // Nota: El tamaño del reporte debe coincidir con lo definido en el descriptor
        uint8_t result = USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&hid_report_buffer, sizeof(HID_Report_t));
        
        // USBD_BUSY: USB está ocupado pero no es error, reintentar en siguiente ciclo
        // USBD_OK: Envío exitoso
        if (result != USBD_OK && result != USBD_BUSY) {
            // Solo error si es algo diferente a OK o BUSY
            status = HAL_ERROR;
        }
        // Si es USBD_BUSY o USBD_OK → retornar HAL_OK (permitir reintentos)
        
        xSemaphoreGive(hid_mutex);
    }
    
    return status;
}

HID_Report_t* HID_Manager_GetReport(void) {
    return &hid_report_buffer;
}

SemaphoreHandle_t HID_Manager_GetMutex(void) {
    return hid_mutex;
}

void HID_Manager_SetStatus(HID_Status_t status) {
    if (hid_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdPASS) {
        hid_report_buffer.status_flags |= (uint8_t)status;
        xSemaphoreGive(hid_mutex);
    }
}

void HID_Manager_ClearStatus(HID_Status_t status) {
    if (hid_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdPASS) {
        hid_report_buffer.status_flags &= ~(uint8_t)status;
        xSemaphoreGive(hid_mutex);
    }
}

uint8_t HID_Manager_GetStatus(void) {
    uint8_t status = 0;
    
    if (hid_mutex != NULL && xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdPASS) {
        status = hid_report_buffer.status_flags;
        xSemaphoreGive(hid_mutex);
    }
    
    return status;
}

void HID_Manager_Reset(void) {
    if (hid_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(hid_mutex, portMAX_DELAY) == pdPASS) {
        hid_report_buffer.reportID = 0x01;
        hid_report_buffer.position = 0;
        hid_report_buffer.velocity = 0;
        hid_report_buffer.status_flags = HID_STATUS_OK;
        xSemaphoreGive(hid_mutex);
    }
}
