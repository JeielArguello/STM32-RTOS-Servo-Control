/*
 * taskStepper.h
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#ifndef TASKSTEPPER_H_
#define TASKSTEPPER_H_

#include "main.h"
#include "tim.h"
#include "Encoder_types.h"
#include <stdlib.h>


extern EncoderData_t sensor_data;
extern Stepper_Handler motor;

void AppInit(void * pvParameters);

void StartSensorTask(void * argument);  // Procesa datos del AS5600
void StartControlTask(void * argument); // Ejecuta el algoritmo PID
void StartDriverTask(void * argument);  // Actúa sobre el DRV8825

/* Tareas de Comunicación y Supervisión */
void StartInputHIDTask(void * argument);  // Procesa reportes recibidos
void StartOutputHIDTask(void * argument); // Envía telemetría a la PC
void StartMonitorTask(void * argument);   // Seguridad y LEDs



#endif /* TASKSTEPPER_H_ */
