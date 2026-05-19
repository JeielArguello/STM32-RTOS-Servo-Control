/*
 * taskStepper.c
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#include "taskStepper.h"
#include "usb_device.h"
#include "usbd_custom_hid_if.h"


typedef struct __attribute__((packed)) {
    uint8_t reportID;     // ID del reporte HID
    int32_t position;     // Posición actual del motor
    uint32_t velocity;       // Velocidad calculada
    uint8_t status_flags; // Errores, límites, etc.
} HID_Report_t;

HID_Report_t report_send = {0};
EncoderData_t sensor_data = {0};
Stepper_Handler motor = {0};

/* Handles de Colas (Queues) */
QueueHandle_t Queue1_ComHandle = NULL;    // PC -> Control (Consignas)
QueueHandle_t Queue2_SensorHandle = NULL; // Sensor -> Control/Monitor/Output (Posición)
QueueHandle_t Queue3_PosHandle = NULL;    // Control -> Driver (Pasos/Velocidad)

/* Handles de Semáforos */
SemaphoreHandle_t Sem1_HID_RxHandle = NULL;  // ISR USB -> InputHIDTask
SemaphoreHandle_t Sem2_DMA_RxHandle = NULL;  // ISR DMA -> SensorTask
SemaphoreHandle_t Sem3_Mutex_Sensor = NULL; // Mutex del sensor
SemaphoreHandle_t Sem4_Mutex_Report = NULL; // Mutex del report

static void UpdateHIDData(int32_t pos, uint32_t vel){

	  // 1. Tomar mutex de reporte
	  if(xSemaphoreTake(Sem4_Mutex_Report, portMAX_DELAY) == pdPASS) {

			// 2. Cargamos los datos en la estructura global del reporte
			// Esta es la HID_Report_t empaquetada que definimos al inicio
			report_send.reportID = 0x01;  // ID de reporte de entrada
			report_send.position = pos;
			report_send.velocity = vel;
			// 3. Liberar Mutex rápido
			xSemaphoreGive(Sem4_Mutex_Report);

	  }
}


/* --------------------------- Tareas ------------------------------------------------*/


void AppInit(void * pvParameters){

	HAL_TIM_Base_Start_IT(&SAMPLE_TIMER_HANDLE);

	Sem1_HID_RxHandle = xSemaphoreCreateBinary();
	if(Sem1_HID_RxHandle  == NULL) {
		// Error al crear el semáforo
		Error_Handler();
	}

	Sem2_DMA_RxHandle = xSemaphoreCreateBinary();
	if(Sem2_DMA_RxHandle  == NULL) {
		// Error al crear el semáforo
		Error_Handler();
	}

	Sem3_Mutex_Sensor = xSemaphoreCreateMutex();
	if(Sem3_Mutex_Sensor == NULL) {
		Error_Handler();
	}

	Sem4_Mutex_Report = xSemaphoreCreateMutex();
	if(Sem4_Mutex_Report  == NULL) {
		// Error al crear el semáforo
		Error_Handler();
	}

	MX_USB_DEVICE_Init();


	Queue1_ComHandle = xQueueCreate(2,sizeof(int32_t));    // PC -> Control
	Queue2_SensorHandle = xQueueCreate(5,sizeof(EncoderData_t*)); // Sensor -> Control/Monitor/Output
	Queue3_PosHandle = xQueueCreate(2,sizeof(int32_t));     // Control -> Driver

	if(Queue1_ComHandle  == NULL) {
			// Error al crear el semáforo
		Error_Handler();
	}

	if(Queue2_SensorHandle  == NULL) {
				// Error al crear el semáforo
		Error_Handler();
	}

	if(Queue3_PosHandle  == NULL) {
		Error_Handler();
	}


	HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);

	xTaskCreate(StartDriverTask, "DriverTask", 100, NULL, 4, NULL);
	xTaskCreate(StartSensorTask, "SensorTask", 100, NULL, 4, NULL);
	xTaskCreate(StartControlTask, "ControlTask", 100, NULL, 3, NULL);
	xTaskCreate(StartInputHIDTask, "InputHIDTask", 100, NULL, 4, NULL);
	xTaskCreate(StartOutputHIDTask, "OutputHIDTask", 100, NULL, 4, NULL);
	xTaskCreate(StartMonitorTask, "MonitorTask", 100, NULL, 3, NULL);


	for(;;){
		vTaskDelay(portMAX_DELAY);
	}
}



void StartDriverTask(void *argument) {
	// Inicialización del motor (usando tu nueva librería)
	    motor.step = (Stepper_Pin){TIM2_STEP_GPIO_Port, TIM2_STEP_Pin};
	    motor.dir  = (Stepper_Pin){GPIO_DIR_GPIO_Port, GPIO_DIR_Pin};
	    motor.htim = &DRIVER_TIMER_HANDLE;
	    motor.channel = DRIVER_TIMER_CHANNEL;
	    motor.m0 = (Stepper_Pin){GPIO_M0_GPIO_Port, GPIO_M0_Pin};
	    motor.m1 = (Stepper_Pin){GPIO_M1_GPIO_Port, GPIO_M1_Pin};
	    motor.m2 = (Stepper_Pin){GPIO_M2_GPIO_Port, GPIO_M2_Pin};
	    Stepper_Init(&motor);
	    Stepper_SetMicrostepping(&motor, STEP_FULL); // 200 pasos por vuelta

	    int32_t target_pos; // Variable local para guardar el dato recibido
		BaseType_t xStatus;

		for(;;) {
			// Le pasamos la dirección de nuestra variable local
			xStatus = xQueueReceive(Queue3_PosHandle, &target_pos, portMAX_DELAY);

			if(xStatus == pdPASS) {
				// Ahora 'target_pos' tiene el valor real enviado por la otra tarea
				// Aquí podrías calcular la diferencia para mover el motor
				Stepper_Move(&motor, 200, 500);
			}
		}
}

void StartSensorTask(void *argument) {
    uint16_t last_raw = 0;
    float last_degrees = 0.0f;
    const float dt = 0.001f; // 1ms (frecuencia del TIM1)
    EncoderData_t *p_sensor = &sensor_data;

    for(;;) {
        // 1. Esperar al DMA (Sincronizado con TIM1)
        if(xSemaphoreTake(Sem2_DMA_RxHandle, portMAX_DELAY) == pdPASS) {

            // 2. Procesar datos crudos fuera del mutex para minimizar latencia
            uint16_t current_raw = ((uint16_t)sensor_data.buffer[0] << 8) | sensor_data.buffer[1];
            int16_t diff = current_raw - last_raw;
            if (diff !=0){
				// 3. Tomar Mutex para actualizar la estructura global
				if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {

					sensor_data.raw_angle = current_raw;

					// Lógica de conteo de vueltas
					if (diff > 2048)  sensor_data.rotations--;
					else if (diff < -2048) sensor_data.rotations++;

					last_raw = current_raw;

					// Calcular posición total
					sensor_data.total_degrees = (sensor_data.rotations * 360.0f) +
												(sensor_data.raw_angle * 360.0f / 4096.0f);

					// 4. Cálculo de Velocidad (Grados por segundo)
					// v = (pos_actual - pos_anterior) / dt
					sensor_data.velocity_dps = (sensor_data.total_degrees - last_degrees) / dt;
					last_degrees = sensor_data.total_degrees;

					// 5. Liberar Mutex rápido
					xSemaphoreGive(Sem3_Mutex_Sensor);


					// 6. Encolar copia de los datos para la ControlTask (PID)
					if(xQueueSendToBack(Queue2_SensorHandle, &p_sensor, 0) != pdPASS){
						uint8_t pepe = sensor_data.buffer[0];
						pepe++;
					}
				}
            }
        }
    }
}


void StartControlTask(void *argument) {

	EncoderData_t* pvSensor;
	int32_t target_pos = 10;
	BaseType_t xStatus;

    for(;;){

    	xStatus = xQueuePeek( Queue2_SensorHandle, &pvSensor,portMAX_DELAY);
		if( xStatus == pdPASS )
		{

			xQueueSendToBack(Queue3_PosHandle, &target_pos, 0);

    	}

    }
}

void StartInputHIDTask(void *argument) {


    HID_Report_t incoming_report;

    for(;;) {
        // 1. Esperar notificación del USB (Bloqueo eficiente)
    	 if(xSemaphoreTake(Sem1_HID_RxHandle, portMAX_DELAY) == pdPASS) {

			// 2. Obtener el puntero al buffer de recepción
			USBD_CUSTOM_HID_HandleTypeDef *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

			// 3. Copiar de forma segura los datos al reporte local
			// hhid->Report_buf contiene el reporte recibido (incluyendo ID)
			memcpy(&incoming_report, hhid->Report_buf, sizeof(HID_Report_t));

			// 4. Procesar según el ID del reporte
			if (incoming_report.reportID == 0x02) { // Reporte de consignas

				// Ejemplo: El PC manda una nueva posición deseada
				// Suponiendo que usas el campo 'position' como Setpoint
				int32_t new_setpoint = incoming_report.position;

				// 5. Enviar el nuevo Setpoint a la ControlTask
				xQueueSend(Queue1_ComHandle, &new_setpoint, 0);
			}

        }
    }
}

void StartOutputHIDTask(void *argument) {
    // 1. Variable local para recibir la COPIA de los datos del sensor
    EncoderData_t* sensor_local;

    // 2. Estructura del reporte (la que definimos de 10 bytes)
    report_send.reportID = 0x01; // ID de reporte de entrada al PC

    BaseType_t xStatus;

    for(;;) {
        // Bloqueo hasta que la SensorTask mande datos frescos
        // Pasamos la dirección de la variable local (&sensor_local)
        xStatus = xQueueReceive(Queue2_SensorHandle, &sensor_local, portMAX_DELAY);

        if(xStatus == pdPASS) {
        	if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {

				// 3. Mapeo de datos del sensor al reporte HID
				// Casteamos a los tipos que definimos en la struct empaquetada
        		 UpdateHIDData(sensor_data.total_degrees, sensor_data.velocity_dps);


				// 4. Envío por USB
				// El tamaño es sizeof(HID_Report_t), que debería ser 10
				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report_send, sizeof(HID_Report_t));

                xSemaphoreGive(Sem3_Mutex_Sensor);
        	}
        }
    }
}

void StartMonitorTask(void *argument) {
    EncoderData_t sensor_local;
    BaseType_t xStatus;

    // Definimos el período de monitoreo (ej. cada 500ms)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(500);

    for(;;) {
        // 1. timeout de 0 (no bloqueante) porque queremos controlar el tiempo con vTaskDelayUntil
        do {
            xStatus = xQueueReceive(Queue2_SensorHandle, &sensor_local, 0);
        } while(xStatus == pdPASS);

        // 2. Lógica de control/diagnóstico
        if (sensor_local.status != 0) {
            // activar un buzzer o reportar un error de hardware
        }

        // 3. Heartbeat: Toggleamos el LED para indicar que el sistema está vivo
        HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);

        // 4. Bloqueo preciso y determinístico de la tarea de monitoreo
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/* ---------------------------------- CALLBACKS ----------------------------------*/

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == motor.htim->Instance) { // Usamos la instancia de la struct motor
        if (motor.steps_to_move > 0) {
            // 1. Obtener el valor actual del registro de comparación (CCR)
            uint32_t val = __HAL_TIM_GET_COMPARE(htim, motor.channel);

            // 2. Programar el próximo pulso sumando el período
            // Usamos el casting a uint16_t porque el TIM2/3 de la Bluepill es de 16 bits
            __HAL_TIM_SET_COMPARE(htim, motor.channel, (uint16_t)(val + motor.period_ticks));

            motor.steps_to_move--;
        } else {
            // 3. Si terminamos, apagamos el timer de forma limpia
            HAL_TIM_OC_Stop_IT(htim, motor.channel);

        }
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == ENCODER_I2C_INSTANCE) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Despertamos a la Sensor_Task para que procese los datos
        xSemaphoreGiveFromISR(Sem2_DMA_RxHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}



