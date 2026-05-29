/*
 * taskStepper.h
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#ifndef TASKSTEPPER_H_
#define TASKSTEPPER_H_

#include "main.h"
#include "board_config.h"

#include "stepper.h"

#include <stdlib.h>

// ========== ENUM DE TAREAS ==========
typedef enum {
	TASK_UNKNOWN = 0,
	TASK_APP_INIT,
	TASK_DRIVER,
	TASK_SENSOR,
	TASK_CONTROL,
	TASK_INPUT_HID,
	TASK_OUTPUT_HID,
	TASK_MONITOR,
	TASK_ISR_TIMER,
	TASK_ISR_I2C,
	TASK_ISR_USB
} TaskID_t;

// Variable global para debugging
extern TaskID_t current_error_task;
extern TaskHandle_t current_error_task_handle;
extern uint32_t error_line;
extern const char* error_file;

void Error_Handler_Task(TaskID_t task_id, uint32_t line, const char* file);

// ========== MACRO PARA REPORTAR ERRORES CON INFO DE TAREA ==========
// Uso: ERROR_TASK(TASK_DRIVER);
#define ERROR_TASK(task_id) Error_Handler_Task(task_id, __LINE__, __FILE__)

void AppInit(void * pvParameters);

void StartSensorTask(void * argument);  // Procesa datos del AS5600
void StartControlTask(void * argument); // Ejecuta el algoritmo PID
void StartDriverTask(void * argument);  // Actúa sobre el DRV8825

/* Tareas de Comunicación y Supervisión */
void StartInputHIDTask(void * argument);  // Procesa reportes recibidos
void StartOutputHIDTask(void * argument); // Envía telemetría a la PC
void StartMonitorTask(void * argument);   // Seguridad y LEDs

void CallbackTimerSensor(TimerHandle_t xTimer);


#endif /* TASKSTEPPER_H_ */
