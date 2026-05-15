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

//extern USBD_HandleTypeDef hUsbDeviceFS;
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
SemaphoreHandle_t Sem3_Mutex_Sensor = NULL;


void AppInit(void * pvParameters){

	HAL_TIM_Base_Start_IT(&SAMPLE_TIMER_HANDLE);

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

	Sem3_Mutex_Sensor = xSemaphoreCreateMutex();

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
	    motor.htim = &DRIVER_TIMER_HANDLE;
	    motor.channel = DRIVER_TIMER_CHANNEL;
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
    float last_degrees = 0.0f;
    const float dt = 0.001f; // 1ms (frecuencia del TIM1)

    for(;;) {
        // 1. Esperar al DMA (Sincronizado con TIM1)
        if(xSemaphoreTake(Sem2_DMA_RxHandle, portMAX_DELAY) == pdPASS) {

            // 2. Procesar datos crudos fuera del mutex para minimizar latencia
            uint16_t current_raw = ((uint16_t)sensor_data.buffer[0] << 8) | sensor_data.buffer[1];
            int16_t diff = current_raw - last_raw;

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

                // 6. Actualizar Reporte USB (HID)
                // Adaptamos a tu estructura de 10 bytes: pos (int32), vel (uint32), flags (uint8)
                UpdateHIDData((int32_t)sensor_data.total_degrees,
                             (uint32_t)sensor_data.velocity_dps);

                // 7. Encolar copia de los datos para la ControlTask (PID)
                xQueueSendToBack(Queue2_SensorHandle, (void*)&sensor_data, 0);
            }
        }
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
    // Buffer donde el stack USB deposita los datos
    extern uint8_t USBD_CustomHID_fops_FS[];
    extern USBD_HandleTypeDef hUsbDeviceFS;

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
				// Podés usar una Queue o una variable global protegida
				xQueueSend(Queue_SetpointHandle, &new_setpoint, 0);
			}
        }
    }
}

void StartOutputHIDTask(void *argument) {

	EncoderData_t* pvSensor = NULL;
	BaseType_t xStatus;

    for(;;){

    	xStatus = xQueueReceive( Queue2_SensorHandle, pvSensor, portMAX_DELAY);
    	if( xStatus == pdPASS )
    	{
    		//USBD_CUSTOM_HID_SendReport()
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
    if (htim->Instance == DRIVER_TIMER_INSTANCE) {
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
    if (hi2c->Instance == ENCODER_I2C_INSTANCE) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Despertamos a la Sensor_Task para que procese los datos
        xSemaphoreGiveFromISR(Sem2_DMA_RxHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}



