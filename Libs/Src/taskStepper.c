/*
 * taskStepper.c
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#include "taskStepper.h"
#include "usb_device.h"
#include "usbd_custom_hid_if.h"
#include "timers.h"


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
QueueHandle_t Queue1_ComHandle;    // PC -> Control (Consignas)
QueueHandle_t Queue2_SensorHandle; // Sensor -> Control/Monitor/Output (Posición)
QueueHandle_t Queue3_PosHandle;    // Control -> Driver (Pasos/Velocidad)

/* Handles de Semáforos */
SemaphoreHandle_t Sem1_HID_RxHandle;  // ISR USB -> InputHIDTask
SemaphoreHandle_t Sem2_DMA_RxHandle;  // ISR DMA -> SensorTask
SemaphoreHandle_t Sem3_Mutex_Sensor; // Mutex del sensor
SemaphoreHandle_t Sem4_Mutex_Report; // Mutex del report

TimerHandle_t xTimerSensorHandle;


static void UpdateHIDData(int32_t pos, uint32_t vel){

	  // 1. Tomar mutex de reporte
	  if(xSemaphoreTake(Sem4_Mutex_Report, portMAX_DELAY) == pdPASS) {

			// 2. Cargamos los datos en la estructura global del reporte
			report_send.reportID = 0x01;  // ID de reporte de entrada
			report_send.position = pos;
			report_send.velocity = vel;
			// 3. Liberar Mutex rápido
			xSemaphoreGive(Sem4_Mutex_Report);

	  }
}


/* --------------------------- Tareas ------------------------------------------------*/




void AppInit(void * pvParameters){


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



	Queue1_ComHandle = xQueueCreate(2,sizeof(int32_t));    // PC -> Control
	Queue2_SensorHandle = xQueueCreate(5,sizeof(EncoderData_t)); // Sensor -> Control/Monitor/Output
	Queue3_PosHandle = xQueueCreate(2,sizeof(int32_t));     // Control -> Driver

	if(Queue1_ComHandle  == NULL) {
		// Error al crear la Queue
		Error_Handler();
	}

	if(Queue2_SensorHandle  == NULL) {
		// Error al crear la Queue
		Error_Handler();
	}

	if(Queue3_PosHandle  == NULL) {
		// Error al crear la Queue
		Error_Handler();
	}

	// Escáner rápido de I2C para el AS5600
	HAL_StatusTypeDef resultado;

	// Enviamos un ping de prueba para ver si el chip responde en el bus
	resultado = HAL_I2C_IsDeviceReady(&ENCODER_I2C_HANDLE, AS5600_ADDR, 3, 100);

	if (resultado != HAL_OK) {
		Error_Handler();
	}

	xTimerSensorHandle = xTimerCreate(
	        "TimerSensor",               // Nombre para debug
	        pdMS_TO_TICKS(1000),           // Período (ej: 1 s)
	        pdTRUE,                      // pdTRUE = Auto-reload (cíclico). pdFALSE = One-shot (se ejecuta una vez)
	        (void *) 0,                  // ID del timer (útil si usás el mismo callback para varios timers)
	        CallbackTimerSensor          // Función que se va a ejecutar
	    );

	if (xTimerSensorHandle != NULL) {
		BaseType_t timer_status;
		timer_status = xTimerStart(xTimerSensorHandle, portMAX_DELAY);

		if (timer_status != pdPASS) {
			Error_Handler();
		}
	}else{
		Error_Handler();
	}

	MX_USB_DEVICE_Init();

	BaseType_t status;
	status = xTaskCreate(StartDriverTask, "DriverTask", 128, NULL, 4, NULL);
	if( status != pdPASS){
	Error_Handler();
	}

	status = xTaskCreate(StartSensorTask, "SensorTask", 128, NULL, 4, NULL);
	if( status != pdPASS){
	Error_Handler();
	}

	status = xTaskCreate(StartControlTask, "ControlTask", 256, NULL, 3, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}


	status = xTaskCreate(StartInputHIDTask, "InputHIDTask", 128, NULL, 4, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}


	status = xTaskCreate(StartOutputHIDTask, "OutputHIDTask", 128, NULL, 4, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}

	status = xTaskCreate(StartMonitorTask, "MonitorTask", 128, NULL, 2, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}


	vTaskSuspend(NULL);
}



void StartDriverTask(void *argument) {
	// Inicialización del motor
	    motor.step = (Stepper_Pin){TIM2_STEP_GPIO_Port, TIM2_STEP_Pin};
	    motor.dir  = (Stepper_Pin){GPIO_DIR_GPIO_Port, GPIO_DIR_Pin};
	    motor.htim = &DRIVER_TIMER_HANDLE;
	    motor.channel = DRIVER_TIMER_CHANNEL;
	    motor.m0 = (Stepper_Pin){GPIO_M0_GPIO_Port, GPIO_M0_Pin};
	    motor.m1 = (Stepper_Pin){GPIO_M1_GPIO_Port, GPIO_M1_Pin};
	    motor.m2 = (Stepper_Pin){GPIO_M2_GPIO_Port, GPIO_M2_Pin};
	    Stepper_Init(&motor);
	    Stepper_SetMicrostepping(&motor, STEP_FULL); // 200 pasos por vuelta

	    
	    // Test: 50 pasos a 5 Hz (muy lentamente - 100ms por flanco)
	    HAL_StatusTypeDef test_status = Stepper_Move(&motor, 200, 20);
	    if (test_status != HAL_OK) {
	    	Error_Handler(); // Si falla, detener
	    }
	    
	    // Esperar a que termine la prueba (50 pasos * 2 flancos = 100 comparaciones)
	    // Cada flanco a 5 Hz = 200ms por flanco = 20 segundos totales
	    vTaskDelay(pdMS_TO_TICKS(1000));

		BaseType_t xStatus;
		int32_t target_pos;

		for(;;){
			// Le pasamos la dirección de nuestra variable local
			xStatus = xQueueReceive(Queue3_PosHandle, &target_pos, portMAX_DELAY);
			if(xStatus == pdPASS){

				// Usa 5 Hz temporalmente para debugging (muy lento, visible)
				if (Stepper_Move(&motor, target_pos, 5) != HAL_OK) {
					Error_Handler();
				}else {
					//vTaskDelay(pdMS_TO_TICKS(1000));

				}

			}
		}
}

void StartSensorTask(void *argument) {
    uint16_t last_raw = 0;
    uint32_t last_degrees = 0;
    const uint16_t dt = 1000; // 1s (frecuencia del TIM)
    uint16_t current_raw = 0;
    int16_t diff = 0;

    for(;;) {
        // 1. Esperar al DMA (Sincronizado con TIM)
        if(xSemaphoreTake(Sem2_DMA_RxHandle, portMAX_DELAY) == pdPASS) {

            // 2. Procesar datos crudos fuera del mutex para minimizar latencia
        	current_raw = ((uint16_t)sensor_data.buffer[0] << 8) | sensor_data.buffer[1];
            diff = current_raw - last_raw;
            if (diff !=0){
				// 3. Tomar Mutex para actualizar la estructura global
				if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {

					sensor_data.position_sensor = current_raw;

					// Lógica de conteo de vueltas
					if (diff > 2048)  sensor_data.rotations--;
					else if (diff < -2048) sensor_data.rotations++;

					last_raw = current_raw;

					// Calcular posición total
					sensor_data.pos_angulo = (sensor_data.rotations * 360) +
												(sensor_data.position_sensor * 360 / 4096);

					// 4. Cálculo de Velocidad (Grados por segundo)
					sensor_data.velocity_dps = (sensor_data.pos_angulo - last_degrees) * dt;
					last_degrees = sensor_data.pos_angulo;

					// 5. Liberar Mutex rápido
					xSemaphoreGive(Sem3_Mutex_Sensor);


					// 6. Encolar copia de los datos para la ControlTask (PID)
					if(xQueueSendToBack(Queue2_SensorHandle, &sensor_data, 0) == pdFAIL){
						int juan =0;
						juan++;
						Error_Handler();
					}
				}
            }
        }
    }
}


void StartControlTask(void *argument) {

	EncoderData_t Sensor_local;
	int32_t set_point_new = 0;
	int32_t set_point = 0;
	BaseType_t xStatus;
	int64_t error = 0, error_prev = 0;
	int32_t Senial_control = 0;

	int32_t integral_error;
	int32_t area=0;
	int32_t derivada_error=0;

	int32_t Kp=1, Ki=2, Kd=3;
	TickType_t tiempo_actual = 0, dt_t = 0,tiempo_prev = xTaskGetTickCount();


    for(;;){

    	xStatus = xQueueReceive(Queue2_SensorHandle, &Sensor_local, portMAX_DELAY);
		if( xStatus == pdPASS )
		{

			xStatus = xQueueReceive(Queue1_ComHandle, &set_point_new, 0);
			if( xStatus == pdPASS )
			{
				set_point = set_point_new;
			}

			tiempo_actual = xTaskGetTickCount();

			dt_t = tiempo_actual - tiempo_prev;

			error = set_point - Sensor_local.pos_angulo;

			area = ((error + error_prev) / 2.0f) * dt_t;

			integral_error += area;
			derivada_error = (error - error_prev)/dt_t;
			Senial_control = Kp*error + Ki*integral_error   + Kd*derivada_error;

			tiempo_prev = tiempo_actual;
			error_prev = error;

			xQueueSendToBack(Queue3_PosHandle, &Senial_control, 0);
		}
    }
}

void StartInputHIDTask(void *argument) {


    HID_Report_t incoming_report;
    int32_t new_setpoint;
    int32_t ticks_calculados;
    USBD_CUSTOM_HID_HandleTypeDef *hhid;

    for(;;) {
        // 1. Esperar notificación del USB (Bloqueo eficiente)
    	 if(xSemaphoreTake(Sem1_HID_RxHandle, portMAX_DELAY) == pdPASS) {

			// 2. Obtener el puntero al buffer de recepción
			hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

			// 3. Copiar de forma segura los datos al reporte local
			// hhid->Report_buf contiene el reporte recibido (incluyendo ID)
			memcpy(&incoming_report, hhid->Report_buf, sizeof(HID_Report_t));

			// 4. Procesar según el ID del reporte
			if (incoming_report.reportID == 0x02) { // Reporte de consignas

				// Ejemplo: El PC manda una nueva posición deseada
				// Suponiendo que usas el campo 'position' como Setpoint
				new_setpoint = incoming_report.position;
				ticks_calculados = ( new_setpoint * AS5600_RESOLUTION ) / 360;

				// 5. Enviar el nuevo Setpoint a la ControlTask
				if(xQueueSendToBack(Queue1_ComHandle, &ticks_calculados, 0) == pdFAIL){
					int pepe=0;
					pepe++;
					Error_Handler();
				}
			}

        }
    }
}

void StartOutputHIDTask(void *argument) {
    // 1. Variable local para recibir la COPIA de los datos del sensor
    EncoderData_t sensor_local;


    BaseType_t xStatus;

    for(;;) {
        // Bloqueo hasta que la SensorTask mande datos frescos
        // Pasamos la dirección de la variable local (&sensor_local)
        xStatus = xQueuePeek(Queue2_SensorHandle, &sensor_local, portMAX_DELAY);

        if(xStatus == pdPASS) {
        	if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {

				// 3. Mapeo de datos del sensor al reporte HID
				// Casteamos a los tipos que definimos en la struct empaquetada
        		 UpdateHIDData(sensor_local.pos_angulo, sensor_local.velocity_dps);


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
    if (htim->Instance == motor.htim->Instance) {
        if (motor.steps_to_move > 0) {
			// Hardware toglea el pin automáticamente con TOGGLE mode
            // 1. Obtener el valor actual del registro de comparación (CCR)
			uint32_t val = __HAL_TIM_GET_COMPARE(htim, motor.channel);

            // 2. Programar el próximo pulso sumando el período
			// TIM2 en la F103 es de 32 bits; no truncamos el CCR.
			__HAL_TIM_SET_COMPARE(htim, motor.channel, val + motor.period_ticks);

            motor.steps_to_move--;
        } else {
            // 3. Si terminamos, apagamos el timer de forma limpia
           	Stepper_Stop(&motor);

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

/* Función que se ejecuta cada vez que el timer vence */
void CallbackTimerSensor(TimerHandle_t xTimer){

	HAL_StatusTypeDef DMAStatus = HAL_I2C_Mem_Read_DMA(&ENCODER_I2C_HANDLE,AS5600_ADDR, ANGLE_REG_MSB, I2C_MEMADD_SIZE_8BIT, sensor_data.buffer, 2);
			// Si el bus está trabado (BUSY), reiniciamos el periférico
		if ( DMAStatus != HAL_OK) {
			__HAL_I2C_DISABLE(&ENCODER_I2C_HANDLE);
			__HAL_I2C_ENABLE(&ENCODER_I2C_HANDLE);
		}
}



