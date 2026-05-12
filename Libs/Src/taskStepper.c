/*
 * taskStepper.c
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#include "taskStepper.h"


EncoderData_t sensor_data = {0};
Stepper_Handler motor = {0};
//Message_Handler mensajeHID;

/* Handles de Colas (Queues) */
QueueHandle_t Queue1_ComHandle = NULL;    // PC -> Control (Consignas)
QueueHandle_t Queue2_SensorHandle = NULL; // Sensor -> Control/Monitor/Output (Posición)
QueueHandle_t Queue3_PosHandle = NULL;    // Control -> Driver (Pasos/Velocidad)

/* Handles de Semáforos */
SemaphoreHandle_t Sem1_HID_RxHandle = NULL;  // ISR USB -> InputHIDTask
SemaphoreHandle_t Sem2_DMA_RxHandle = NULL;  // ISR DMA -> SensorTask


void AppInit(void * pvParameters){

	HAL_TIM_Base_Start_IT(&htim1);

	Sem1_HID_RxHandle = xSemaphoreCreateBinary();
	if(Sem1_HID_RxHandle  == NULL) {
		// Error al crear el semáforo
		HAL_Delay(1000);
	}

	Sem2_DMA_RxHandle = xSemaphoreCreateBinary();
	if(Sem2_DMA_RxHandle  == NULL) {
		// Error al crear el semáforo
		HAL_Delay(1000);
	}


	Queue1_ComHandle = xQueueCreate(2,sizeof(uint8_t));    // PC -> Control
	Queue2_SensorHandle = xQueueCreate(1,sizeof(EncoderData_t*)); // Sensor -> Control/Monitor/Output
	Queue3_PosHandle = xQueueCreate(2,sizeof(uint8_t));     // Control -> Driver

	if(Queue1_ComHandle  == NULL) {
			// Error al crear el semáforo
		HAL_Delay(1000);
	}

	if(Queue2_SensorHandle  == NULL) {
				// Error al crear el semáforo
		HAL_Delay(1000);
	}

	if(Queue3_PosHandle  == NULL) {
		HAL_Delay(1000);
	}


	xTaskCreate(StartDriverTask, "DriverTask", 100, NULL, 4, NULL);
	xTaskCreate(StartSensorTask, "SensorTask", 100, NULL, 4, NULL);
	xTaskCreate(StartControlTask, "ControlTask", 100, NULL, 3, NULL);
	xTaskCreate(StartInputHIDTask, "InputHIDTask", 100, NULL, 1, NULL);
	xTaskCreate(StartOutputHIDTask, "OutputHIDTask", 100, NULL, 1, NULL);
	xTaskCreate(StartMonitorTask, "MonitorTask", 100, NULL, 1, NULL);


	for(;;){
		vTaskDelay(portMAX_DELAY);
	}
}


void StartDriverTask(void *argument) {
	// Inicialización del motor (usando tu nueva librería)
	    motor.step = (Stepper_Pin){TIM2_STEP_GPIO_Port, TIM2_STEP_Pin};
	    motor.dir  = (Stepper_Pin){GPIO_DIR_GPIO_Port, GPIO_DIR_Pin};
	    motor.htim = &htim2;
	    motor.channel = TIM_CHANNEL_2;
	    motor.m0 = (Stepper_Pin){GPIO_M0_GPIO_Port, GPIO_M0_Pin};
	    motor.m1 = (Stepper_Pin){GPIO_M1_GPIO_Port, GPIO_M1_Pin};
	    motor.m2 = (Stepper_Pin){GPIO_M2_GPIO_Port, GPIO_M2_Pin};
	    Stepper_Init(&motor);
	    Stepper_SetMicrostepping(&motor, STEP_FULL); // 200 pasos por vuelta

	    BaseType_t xStatus;
	    Stepper_Handler * pvPos = NULL;
	    for(;;){
	    	//Me bloqueo esperando la posicion absoluta
	    	xStatus = xQueueReceive( Queue3_PosHandle, pvPos,portMAX_DELAY);

			if( xStatus == pdPASS )
			{

				Stepper_Move(&motor, 200, 500);
	    	}

	    }
}

void StartSensorTask(void *argument) {

    uint16_t last_raw = 0;

    // El periodo de muestreo (ej: 10ms = 100Hz)
    //TickType_t xLastWakeTime = xTaskGetTickCount();
    //const TickType_t xFrequency = pdMS_TO_TICKS(1);



    for(;;) {
    	if(xSemaphoreTake(Sem2_DMA_RxHandle, portMAX_DELAY) == pdPASS) {

    	// 1. Leer el encoder vía I2C

            sensor_data.raw_angle = ((uint16_t)sensor_data.buffer[0] << 8) | sensor_data.buffer[1];
            // 2. Lógica de conteo de vueltas
            int16_t diff = sensor_data.raw_angle - last_raw;
            if (diff > 2048) sensor_data.rotations--;
            else if (diff < -2048) sensor_data.rotations++;

            last_raw = sensor_data.raw_angle;

            // 3. Calcular ángulo acumulado
            sensor_data.total_degrees = (sensor_data.rotations * 360.0f) + (sensor_data.raw_angle * 360.0f / 4096.0f);

            // 4. Encolar los datos
            xQueueSendToBack(Queue2_SensorHandle, &sensor_data, 0);
        }

        // Esperar de forma eficiente hasta el próximo ciclo
       // vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void StartControlTask(void *argument) {

	EncoderData_t* pvSensor = NULL;
	BaseType_t xStatus;

    for(;;){

    	xStatus = xQueueReceive( Queue2_SensorHandle, pvSensor,portMAX_DELAY);
		if( xStatus == pdPASS )
		{

			xQueueSendToBack(Queue3_PosHandle, &sensor_data, 0);

    	}

    }
}

void StartInputHIDTask(void *argument) {


    for(;;){

    }
}

void StartOutputHIDTask(void *argument) {

	EncoderData_t* pvSensor = NULL;
	BaseType_t xStatus;

    for(;;){

    	xStatus = xQueueReceive( Queue2_SensorHandle, pvSensor, portMAX_DELAY);
    	if( xStatus == pdPASS )
    	{

    	}
    }
}

void StartMonitorTask(void *argument) {

	EncoderData_t* pvSensor = NULL;
	BaseType_t xStatus;
	TickType_t timeSleep = pdMS_TO_TICKS(2000);

	for(;;){
		xStatus = xQueueReceive( Queue2_SensorHandle, pvSensor, timeSleep);
		if( xStatus == pdPASS )
		{

    	}

		HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);

    }
}


/* ---------------------------------- CALLBACKS ----------------------------------*/

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (motor.steps_to_move > 0) {
            uint32_t val = __HAL_TIM_GET_COMPARE(htim, motor.channel);
            __HAL_TIM_SET_COMPARE(htim, motor.channel, val + motor.period_ticks);
            motor.steps_to_move--;
        } else {
            HAL_TIM_OC_Stop_IT(htim, motor.channel);


        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Despertamos a la Sensor_Task para que procese los datos
        xSemaphoreGiveFromISR(Sem2_DMA_RxHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}





