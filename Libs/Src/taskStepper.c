/*
 * taskStepper.c
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#include "taskStepper.h"
#include "usb_device.h"
#include "usbd_custom_hid_if.h"

// ============ FIXED-POINT Q16 (para PID sin floats) ============
// Q16: 16 bits decimales, 16 bits enteros
// 1.0 = 65536 (0x10000)
// 0.5 = 32768
// 0.001 = 65 (aprox)
#define Q16_SHIFT 16
#define Q16_ONE (1LL << Q16_SHIFT)     // 1.0 en Q16
#define Q16_FROM_INT(x) ((int64_t)(x) << Q16_SHIFT)
#define Q16_TO_INT(x) ((int32_t)((x) >> Q16_SHIFT))
#define Q16_MUL(a, b) (((a) * (b)) >> Q16_SHIFT)
#define Q16_DIV(a, b) (((a) << Q16_SHIFT) / (b))

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
QueueHandle_t Queue3_PosHandle;    // Control -> Driver (Pasos/Velocidad)

/* Handles de Semáforos */
SemaphoreHandle_t Sem1_HID_RxHandle;  // ISR USB -> InputHIDTask
SemaphoreHandle_t Sem2_DMA_RxHandle;  // ISR DMA -> SensorTask
SemaphoreHandle_t Sem3_Mutex_Sensor; // Mutex del sensor
SemaphoreHandle_t Sem4_Mutex_Report; // Mutex del reporte
SemaphoreHandle_t Sem5_Motor_Done;   // Motor terminó movimiento
SemaphoreHandle_t Sem6_SensorReady;  // Semáforo contador para notificar datos frescos
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

	Sem5_Motor_Done = xSemaphoreCreateBinary();
	if(Sem5_Motor_Done == NULL) {
		// Error al crear el semáforo
		Error_Handler();
	}

	// Semáforo contador: se da una vez, despierta múltiples tareas
	// Valor inicial 0, máximo 2 (para ControlTask y OutputHIDTask)
	Sem6_SensorReady = xSemaphoreCreateCounting(3, 0);
	if(Sem6_SensorReady == NULL) {
		Error_Handler();
	}

	Queue1_ComHandle = xQueueCreate(2,sizeof(int32_t));    // PC -> Control
	Queue3_PosHandle = xQueueCreate(2,sizeof(int32_t));     // Control -> Driver

	if(Queue1_ComHandle  == NULL) {
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
	        pdMS_TO_TICKS(100),           // Período: 100ms (frecuencia de lectura del AS5600)
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
	status = xTaskCreate(StartDriverTask, "DriverTask", 256, NULL, 4, NULL);
	if( status != pdPASS){
	Error_Handler();
	}

	status = xTaskCreate(StartSensorTask, "SensorTask", 256, NULL, 4, NULL);
	if( status != pdPASS){
	Error_Handler();
	}

	status = xTaskCreate(StartControlTask, "ControlTask", 512, NULL, 3, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}


	status = xTaskCreate(StartInputHIDTask, "InputHIDTask", 256, NULL, 4, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}


	status = xTaskCreate(StartOutputHIDTask, "OutputHIDTask", 256, NULL, 3, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}

	status = xTaskCreate(StartMonitorTask, "MonitorTask", 128, NULL, 2, NULL);
	if( status != pdPASS){
	  Error_Handler();
	}

	// Esperar a que todas las tareas se inicialicen
	vTaskDelay(pdMS_TO_TICKS(2000));


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
	    HAL_StatusTypeDef test_status = Stepper_Move(&motor, 200, 200);
	    if (test_status != HAL_OK) {
	    	Error_Handler(); // Si falla, detener
	    }
	    
	    // Esperar a que termine el movimiento de prueba
	    xSemaphoreTake(Sem5_Motor_Done, portMAX_DELAY);

		BaseType_t xStatus;
		int32_t target_pos;

		for(;;){
			// Esperar comando del ControlTask (bloqueante)
			xStatus = xQueueReceive(Queue3_PosHandle, &target_pos, portMAX_DELAY);
			if(xStatus == pdPASS){

				// Iniciar movimiento (no-bloqueante)
				if (Stepper_Move(&motor, target_pos, 3) != HAL_OK) {
					Error_Handler();
				}
				
				// Esperar a que el motor termine el movimiento
				// Esto bloquea la tarea hasta que el callback dispare el semáforo
				xSemaphoreTake(Sem5_Motor_Done, portMAX_DELAY);
			}
		}
}

void StartSensorTask(void *argument) {
    uint16_t last_raw = 0;
    int32_t last_degrees = 0;
    const uint16_t dt = 100; // 1s (frecuencia del TIM)
    uint16_t current_raw = 0;
    int16_t diff = 0;
    uint8_t first_read = 1;  // Flag para la primera lectura

    for(;;) {
        // 1. Esperar al DMA (Sincronizado con TIM)
        if(xSemaphoreTake(Sem2_DMA_RxHandle, portMAX_DELAY) == pdPASS) {

            // 2. Procesar datos crudos fuera del mutex para minimizar latencia
        	current_raw = ((uint16_t)sensor_data.buffer[0] << 8) | sensor_data.buffer[1];
            
            // Validar que el raw está en rango correcto (0-4095 para AS5600)
            if(current_raw > 4095) {
                continue;  // Dato corrupto, ignorar
            }

            // En la primera lectura, inicializar sin calcular diferencia
            if(first_read) {
                last_raw = current_raw;
                
				// Tomar Mutex para inicializar
				if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
					sensor_data.position_sensor = current_raw;
					sensor_data.rotations = 0;
					sensor_data.pos_angulo = (current_raw * 360) / 4096;  // Posición inicial en grados
					last_degrees = sensor_data.pos_angulo;
					xSemaphoreGive(Sem3_Mutex_Sensor);
				}
                first_read = 0;
                continue;
            }

            diff = current_raw - last_raw;

			// 3. Tomar Mutex para actualizar la estructura global
			if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {

				sensor_data.position_sensor = current_raw;

				// Lógica de conteo de vueltas (detectar transiciones)
				if (diff > 2048) {
					// Transición de 4095 -> 0 (reverse)
					sensor_data.rotations--;
				} else if (diff < -2048) {
					// Transición de 0 -> 4095 (forward)
					sensor_data.rotations++;
				}

				last_raw = current_raw;

				// Calcular posición total en grados
				// pos_angulo = rotations * 360 + (position_sensor / 4096) * 360
				int64_t pos_temp = ((int64_t)sensor_data.rotations * 360) + 
								   ((int64_t)sensor_data.position_sensor * 360 / 4096);
				sensor_data.pos_angulo = (int32_t)pos_temp;

				// 4. Cálculo de Velocidad (Grados por segundo)
				sensor_data.velocity_dps = (sensor_data.pos_angulo - last_degrees) * dt / 1000;
				last_degrees = sensor_data.pos_angulo;

				// 5. Liberar Mutex rápido
				xSemaphoreGive(Sem3_Mutex_Sensor);

				// 6. Dar el semáforo contador 2 veces (despierta ControlTask y OutputHIDTask)
				xSemaphoreGive(Sem6_SensorReady);
				xSemaphoreGive(Sem6_SensorReady);
			}
        }
    }
}


void StartControlTask(void *argument) {

	EncoderData_t Sensor_local;
	int32_t set_point_new = 0;
	int32_t set_point = 0;
	BaseType_t xStatus;

	// Todos los valores del PID en Q16 para máxima precisión
	int64_t error = 0, error_prev = 0;
	int64_t integral_error = 0;
	int64_t derivada_error = 0;
	int64_t Senial_control = 0;
	
	int64_t dt_t_q16 = 0;  // dt en Q16 (para divisiones precisas)

	// GANANCIAS EN Q16 - VALORES MUY CONSERVADORES PARA EVITAR OSCILACIONES
	// Período esperado del ControlTask: ~100ms (sincronizado con SensorTask)
	int64_t Kp = 1310;       // Kp = 0.02 (MUY bajo - integración pura casi)
	int64_t Ki = 65;         // Ki = 0.001 (integración lenta)
	int64_t Kd = 0;          // Kd = 0 (DESHABILITADO - causa oscilaciones y rebotes)
	
	// Deadband: zona muerta donde el PID no actúa
	// Si |error| < DEADBAND, el motor se detiene
	int64_t DEADBAND = Q16_FROM_INT(5);  // ±5 grados (zona de tolerancia)

	TickType_t tiempo_actual = 0, tiempo_prev = xTaskGetTickCount();

    for(;;){
    	// Esperar a que el SensorTask publique datos frescos
    	xSemaphoreTake(Sem6_SensorReady, portMAX_DELAY);

		// Tomar el mutex y leer la estructura global del sensor
		if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
			Sensor_local = sensor_data;
			xSemaphoreGive(Sem3_Mutex_Sensor);
		}

		// Revisar si hay nuevo setpoint del usuario
		xStatus = xQueueReceive(Queue1_ComHandle, &set_point_new, 0);
		if( xStatus == pdPASS )
		{
			set_point = set_point_new;
		}

		tiempo_actual = xTaskGetTickCount();
		uint32_t tick_total = tiempo_actual - tiempo_prev;  // FIJO: 100ms es el período del timer del sensor (sincronización)
		
		uint32_t dt_ms = tick_total * portTICK_PERIOD_MS;
		// Convertir dt a Q16: dt_ms en milisegundos -> Q16
		// 100ms = 0.1 segundos = Q16(0.1) = 6553 (aprox)
		dt_t_q16 = ((int64_t)dt_ms << Q16_SHIFT) / 1000;  // Convertir a segundos en Q16
		if(dt_t_q16 == 0) dt_t_q16 = 1;

		// Error en Q16 (convertir posiciones a Q16)
		error = Q16_FROM_INT(set_point) - Q16_FROM_INT(Sensor_local.position_sensor);

		// DEADBAND: Si el error es muy pequeño, ignorarlo para evitar oscilaciones
		if(error > -DEADBAND && error < DEADBAND) {
			error = 0;
			integral_error = 0;  // Resetear integral también
		}

		// INTEGRAL: solo si no estamos en zona muerta
		if(error != 0) {
			// area = (error + error_prev)/2 * dt
			// En Q16: ((error + error_prev) * dt_q16) >> (Q16_SHIFT+1)
			int64_t area = (((error + error_prev) >> 1) * dt_t_q16) >> Q16_SHIFT;
			integral_error += area;
			
			// Anti-windup: limitar el acumulador (max ±50 en Q16)
			int64_t max_integral = Q16_FROM_INT(50);
			if(integral_error > max_integral) integral_error = max_integral;
			if(integral_error < -max_integral) integral_error = -max_integral;
		}

		// DERIVADA: (error - error_prev) / dt
		// En Q16: (error_delta * Q16) / dt_q16
		int64_t error_delta = error - error_prev;
		if(dt_t_q16 > 0) {
			derivada_error = (error_delta * Q16_ONE) / dt_t_q16;
		} else {
			derivada_error = 0;
		}

		// PID: Senial = Kp*error + Ki*integral + Kd*derivada
		// Todos en Q16
		Senial_control = Q16_MUL(Kp, error) + 
						Q16_MUL(Ki, integral_error) + 
						Q16_MUL(Kd, derivada_error);

		// Calcular posición destino: set_point + corrección PID
		int32_t position_target = set_point + Q16_TO_INT(Senial_control);
		
		// Limitar la posición destino a rango razonable
		if(position_target > 3600) position_target = 3600;   // 10 vueltas máximo
		if(position_target < -3600) position_target = -3600;

		tiempo_prev = tiempo_actual;
		error_prev = error;

		// Enviar posición destino al motor
		xQueueSendToBack(Queue3_PosHandle, &position_target, 0);
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

    for(;;) {
        // 1. Esperar a que el SensorTask publique datos frescos
        xSemaphoreTake(Sem6_SensorReady, portMAX_DELAY);

        // 2. Tomar el mutex y leer la estructura global del sensor
        if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
            sensor_local = sensor_data;
            xSemaphoreGive(Sem3_Mutex_Sensor);
        }

        // 3. Mapeo de datos del sensor al reporte HID
        UpdateHIDData(sensor_local.pos_angulo, sensor_local.velocity_dps);

        // 4. Envío por USB
        // El tamaño es sizeof(HID_Report_t), que debería ser 10
        USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report_send, sizeof(HID_Report_t));
    }
}

void StartMonitorTask(void *argument) {
    EncoderData_t sensor_local;

    // Definimos el período de monitoreo (ej. cada 500ms)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(500);

    for(;;) {
        // 1. Tomar el mutex y leer la estructura global del sensor
        // MonitorTask accede directamente, sin esperar semáforo (evita agotar el contador)
        if(xSemaphoreTake(Sem3_Mutex_Sensor, pdMS_TO_TICKS(100)) == pdPASS) {
            sensor_local = sensor_data;
            xSemaphoreGive(Sem3_Mutex_Sensor);
            
            // 2. Lógica de control/diagnóstico
            if (sensor_local.status != 0) {
                // Aquí puedes activar buzzer o reportar error
                // Error_Handler();
            }
            
            // DEBUG: Podrías loguear aquí la posición y velocidad
            // printf("POS: %d°, VEL: %d°/s\r\n", sensor_local.pos_angulo, sensor_local.velocity_dps);
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
           	
           	// 4. Notificar a la DriverTask que el movimiento terminó
           	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
           	xSemaphoreGiveFromISR(Sem5_Motor_Done, &xHigherPriorityTaskWoken);
           	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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



