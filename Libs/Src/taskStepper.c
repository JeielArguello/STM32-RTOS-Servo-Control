/*
 * taskStepper.c
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#include "taskStepper.h"
#include "usb_device.h"
#include "usbd_custom_hid_if.h"
#include "tim_debug.h"
#include "motor_debug.h"
#include "hid_manager.h"
#include "debugstats.h"

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


DebugStats_t debug_stats = {0};
EncoderData_t sensor_data = {0};
Stepper_Handler motor = {0};

// ========== VARIABLES GLOBALES DE DEBUG ==========
TaskID_t current_error_task = TASK_UNKNOWN;
TaskHandle_t current_error_task_handle = NULL;
uint32_t error_line = 0;
const char* error_file = NULL;

/* Handles de Colas (Queues) */
QueueHandle_t Queue1_ComHandle;    // PC -> Control (Consignas)
QueueHandle_t Queue3_PosHandle;    // Control -> Driver (Pasos/Velocidad)

/* Handles de Tareas (para monitoreo) */
TaskHandle_t hDriverTask;
TaskHandle_t hSensorTask;
TaskHandle_t hControlTask;
TaskHandle_t hInputHIDTask;
TaskHandle_t hOutputHIDTask;
TaskHandle_t hMonitorTask;

/* Handles de Semáforos */
SemaphoreHandle_t Sem1_HID_RxHandle;  // ISR USB -> InputHIDTask
SemaphoreHandle_t Sem2_DMA_RxHandle;  // ISR DMA -> SensorTask
SemaphoreHandle_t Sem3_Mutex_Sensor; // Mutex del sensor
SemaphoreHandle_t Sem6_SensorReady;  // Semáforo contador para notificar datos frescos
TimerHandle_t xTimerSensorHandle;




/* --------------------------- Tareas ------------------------------------------------*/




void AppInit(void * pvParameters){


	Sem1_HID_RxHandle = xSemaphoreCreateBinary();
	if(Sem1_HID_RxHandle  == NULL) {
		// Error al crear el semáforo
		ERROR_TASK(TASK_APP_INIT);
	}

	Sem2_DMA_RxHandle = xSemaphoreCreateBinary();
	if(Sem2_DMA_RxHandle  == NULL) {
		// Error al crear el semáforo
		ERROR_TASK(TASK_APP_INIT);
	}

	Sem3_Mutex_Sensor = xSemaphoreCreateMutex();
	if(Sem3_Mutex_Sensor == NULL) {
		ERROR_TASK(TASK_APP_INIT);
	}

	Sem6_SensorReady = xSemaphoreCreateCounting(10, 0);  // Contador de 10 (puede haber hasta 10 datos frescos sin procesar)
	if(Sem6_SensorReady == NULL) {
		// Error al crear el semáforo
		ERROR_TASK(TASK_APP_INIT);
	}

	// Inicializar librerías de Debug y HID
	MotorDebug_Init();
	HID_Manager_Init();

	Queue1_ComHandle = xQueueCreate(2,sizeof(int32_t));    // PC -> Control
	Queue3_PosHandle = xQueueCreate(2,sizeof(int32_t));     // Control -> Driver

	if(Queue1_ComHandle  == NULL) {
		// Error al crear la Queue
		ERROR_TASK(TASK_APP_INIT);
	}

	if(Queue3_PosHandle  == NULL) {
		// Error al crear la Queue
		ERROR_TASK(TASK_APP_INIT);
	}

	// Escáner rápido de I2C para el AS5600
	HAL_StatusTypeDef resultado;

	// Enviamos un ping de prueba para ver si el chip responde en el bus
	resultado = HAL_I2C_IsDeviceReady(&ENCODER_I2C_HANDLE, AS5600_ADDR, 3, 100);

	if (resultado != HAL_OK) {
		ERROR_TASK(TASK_APP_INIT);
	}

	xTimerSensorHandle = xTimerCreate(
	        "TimerSensor",               // Nombre para debug
	        pdMS_TO_TICKS(100),            // Período: 100ms (frecuencia de lectura del AS5600 - más rápido para mejor sincronización)
	        pdTRUE,                      // pdTRUE = Auto-reload (cíclico). pdFALSE = One-shot (se ejecuta una vez)
	        (void *) 0,                  // ID del timer (útil si usás el mismo callback para varios timers)
	        CallbackTimerSensor          // Función que se va a ejecutar
	    );

	if (xTimerSensorHandle != NULL) {
		BaseType_t timer_status;
		timer_status = xTimerStart(xTimerSensorHandle, portMAX_DELAY);

		if (timer_status != pdPASS) {
			ERROR_TASK(TASK_APP_INIT);
		}
	}else{
		ERROR_TASK(TASK_APP_INIT);
	}

	MX_USB_DEVICE_Init();

	BaseType_t status;
	status = xTaskCreate(StartDriverTask, "DriverTask", 256, NULL, 4, &hDriverTask);
	if( status != pdPASS){
	ERROR_TASK(TASK_APP_INIT);
	}

	status = xTaskCreate(StartSensorTask, "SensorTask", 256, NULL, 4, &hSensorTask);
	if( status != pdPASS){
	ERROR_TASK(TASK_APP_INIT);
	}

	status = xTaskCreate(StartControlTask, "ControlTask", 512, NULL, 3, &hControlTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
	}


	status = xTaskCreate(StartInputHIDTask, "InputHIDTask", 256, NULL, 4, &hInputHIDTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
	}


	status = xTaskCreate(StartOutputHIDTask, "OutputHIDTask", 256, NULL, 3, &hOutputHIDTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
	}

	status = xTaskCreate(StartMonitorTask, "MonitorTask", 128, NULL, 2, &hMonitorTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
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

	BaseType_t xStatus;
	int32_t rpm_from_control;

	// ===== PRUEBA INICIAL: Una vuelta =====
	MotorDebug_Start(360);  // 1 vuelta = 360 grados
	Stepper_SetSteps(&motor, 360);  // 360 Hz = 1 vuelta por segundo (200 pasos/vuelta × 2 toggles/paso = 400 Hz para 1 vuelta/s, pero ajustamos a 360 para compensar ineficiencias)
	Stepper_SetSpeed(&motor, 1000);  // 500 Hz para prueba
	HAL_StatusTypeDef test_status = Stepper_Start(&motor);
	if (test_status != HAL_OK) {
		ERROR_TASK(TASK_DRIVER);
	}
	vTaskDelay(pdMS_TO_TICKS(2000));  // Dejar girar 2 segundos
	Stepper_Stop(&motor);
	MotorDebug_Stop();

	Stepper_SetSteps(&motor, 100);
	
	// ===== LOOP PRINCIPAL =====
	// Esperar RPM desde ControlTask (Queue3_PosHandle)
	// Flujo: PC -> InputHID (grados) -> Queue1 -> ControlTask (calcula RPM) -> Queue3 -> DriverTask
	for(;;){	
		xStatus = xQueueReceive(Queue3_PosHandle, &rpm_from_control, portMAX_DELAY);
		
		if(xStatus == pdPASS){
			// Registrar setpoint RPM recibido del control
			MotorDebug_Start(rpm_from_control);
			
			// Si RPM es cercana a 0, parar el motor
			if(rpm_from_control > -10 && rpm_from_control < 10) {
				Stepper_Stop(&motor);
			} else {
				// Setear velocidad en RPM (la conversión a Hz ocurre dentro de Stepper_SetSpeed)
				Stepper_SetSpeed(&motor, rpm_from_control);
				
				// Arrancar motor
				if (!motor.is_running) {  // Solo llamar Start si el motor no está corriendo (evitar reiniciar el timer innecesariamente)
					test_status = Stepper_Start(&motor);
					if (test_status != HAL_OK) {
						ERROR_TASK(TASK_DRIVER);
					}
				}
			}
		}
	}
}

void StartSensorTask(void *argument) {
    uint16_t last_raw = 0;
    int32_t last_degrees = 0;
    uint16_t current_raw = 0;
    int16_t diff = 0;
    uint8_t first_read = 1;  // Flag para la primera lectura
	int64_t pos_temp=0;

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
					sensor_data.raw_position = current_raw;
					sensor_data.rotations = 0;
					sensor_data.angle_deg = (current_raw * 360) / 4096;  // Ángulo inicial en grados
					last_degrees = sensor_data.angle_deg;
					xSemaphoreGive(Sem3_Mutex_Sensor);
				}
                first_read = 0;
                continue;
            }

            diff = current_raw - last_raw;

			// 3. Tomar Mutex para actualizar la estructura global
			if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {

				sensor_data.raw_position = current_raw;

				// Lógica de conteo de vueltas (detectar transiciones)
				if (diff > 2048) {
					// Transición de 4095 -> 0 (reverse)
					sensor_data.rotations--;
				} else if (diff < -2048) {
					// Transición de 0 -> 4095 (forward)
					sensor_data.rotations++;
				}

				last_raw = current_raw;

				// Calcular ángulo total en grados
				// angle_deg = rotations * 360 + (raw_position / 4096) * 360
				pos_temp = ((int64_t)sensor_data.rotations * 360) + 
							   ((int64_t)sensor_data.raw_position * 360 / 4096);
				sensor_data.angle_deg = (int32_t)pos_temp;


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
	int64_t set_point_ticks = 0;  // Setpoint en ticks del sensor
	BaseType_t xStatus;

	// GANANCIAS DEL PID EN Q16
	int64_t Kp = 11525;  // Proporcional: 0.176 RPM/tick
	int64_t Ki = 3;      // Integral: 0.000046 RPM/(tick·seg)

	int64_t error_ticks = 0;
	int64_t integral_error = 0;

	TickType_t tiempo_actual = 0, tiempo_prev = xTaskGetTickCount();

	// Esperar a que driver esté activa
	vTaskDelay(pdMS_TO_TICKS(2000));
    for(;;){
    	// Esperar a que el SensorTask publique datos frescos
    	xSemaphoreTake(Sem6_SensorReady, portMAX_DELAY);

		// Tomar el mutex y leer la estructura global del sensor
		if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
			Sensor_local = sensor_data;
			xSemaphoreGive(Sem3_Mutex_Sensor);
		}

		// Revisar si hay nuevo setpoint del usuario
		int64_t set_point_new = 0;
		xStatus = xQueueReceive(Queue1_ComHandle, &set_point_new, 0);
		if(xStatus == pdPASS) {
			set_point_ticks = set_point_new;
			integral_error = 0;  // Resetear integral al cambiar setpoint
		}

		// Calcular tiempo transcurrido
		tiempo_actual = xTaskGetTickCount();
		uint32_t dt_ms = (tiempo_actual - tiempo_prev) * portTICK_PERIOD_MS;
		

		// Calcular posición actual en ticks (multivuelta)
		int64_t current_ticks = ((int64_t)Sensor_local.rotations * 4096) + Sensor_local.raw_position;
		
		// Error de posición: debe ser negativo cuando está atrás (para girar atrás)
		error_ticks =  set_point_ticks - current_ticks;

		// ===== PID SIMPLE =====
		// P: proporcional al error
		int64_t p_term = Q16_MUL(Kp, Q16_FROM_INT(error_ticks));

		// I: integral del error
		if(dt_ms > 0 && error_ticks != 0) {
			int64_t dt_q16 = Q16_DIV(dt_ms, 1000);  // dt en segundos, Q16
			integral_error += Q16_MUL(Ki, Q16_MUL(Q16_FROM_INT(error_ticks), dt_q16));
			
			// Anti-windup simple
			int64_t max_integral = Q16_FROM_INT(5000);
			if(integral_error > max_integral) integral_error = max_integral;
			if(integral_error < -max_integral) integral_error = -max_integral;
		}
		
		int64_t i_term = integral_error;

		// RPM = P + I
		int64_t rpm_q16 = p_term + i_term;
		int32_t rpm_output = Q16_TO_INT(rpm_q16);

		// Limitar RPM (driver manejará frenar si RPM < 5)
		if(rpm_output > 1500) rpm_output = 1500;
		if(rpm_output < -1500) rpm_output = -1500;

		// ===== ENVIAR RPM AL DRIVER =====
		if(xQueueSendToBack(Queue3_PosHandle, &rpm_output, 0) == pdFAIL){
			ERROR_TASK(TASK_CONTROL);
		}

		tiempo_prev = tiempo_actual;
		
    }
}


void StartInputHIDTask(void *argument) {


    HID_Report_t incoming_report;
    int64_t new_setpoint_ticks;
    int64_t new_setpoint_deg;
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

				// El PC manda una nueva posición deseada en grados
				new_setpoint_deg = (int64_t)incoming_report.position;  // Posición en grados

				// Convertir grados a ticks (1 vuelta = 360 grados = 4096 ticks)
				new_setpoint_ticks = (new_setpoint_deg * 4096) / 360;
				// 5. Enviar el nuevo Setpoint en ticks a ControlTask via Queue1
				if(xQueueSendToBack(Queue1_ComHandle, &new_setpoint_ticks, 0) == pdFAIL){
					ERROR_TASK(TASK_INPUT_HID);
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

        // 3. Actualizar datos HID con info del sensor
        uint8_t status = HID_Manager_GetStatus();
        HID_Manager_Update(sensor_local.angle_deg, sensor_local.speed_rpm, status);

		// 4. Enviar reporte HID
		if(HID_Manager_SendReport() != HAL_OK) {
			ERROR_TASK(TASK_OUTPUT_HID);
		}
	}
}


void StartMonitorTask(void *argument) {
    EncoderData_t sensor_local;
    static uint32_t heap_min = 0xFFFFFFFF;
  
    // Definimos el período de monitoreo (cada 2 segundos)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2000);

    for(;;) {
        // 1. Actualizar estadísticas de memoria HEAP
        debug_stats.heap_free = xPortGetFreeHeapSize();
        if(debug_stats.heap_free < heap_min) {
            heap_min = debug_stats.heap_free;
        }
        debug_stats.heap_min = heap_min;
        
        // 2. Obtener stack high water mark de cada tarea (bytes libres mínimos)
        debug_stats.stack_driver = uxTaskGetStackHighWaterMark(hDriverTask);
        debug_stats.stack_sensor = uxTaskGetStackHighWaterMark(hSensorTask);
        debug_stats.stack_control = uxTaskGetStackHighWaterMark(hControlTask);
        debug_stats.stack_input_hid = uxTaskGetStackHighWaterMark(hInputHIDTask);
        debug_stats.stack_output_hid = uxTaskGetStackHighWaterMark(hOutputHIDTask);
        
        // 3. Determinar si hay alerta crítica
        debug_stats.memory_warning = 0;
        if(debug_stats.heap_free < 200) {                   // Crítico
            debug_stats.memory_warning = 2;
        } else if(debug_stats.heap_free < 500) {            // Advertencia
            debug_stats.memory_warning = 1;
        }
        
        // Si hay problema crítico de stack, también alertar
        if(debug_stats.stack_driver < 20 || debug_stats.stack_sensor < 20 || 
           debug_stats.stack_control < 20 || debug_stats.stack_input_hid < 20 ||
           debug_stats.stack_output_hid < 20) {
            debug_stats.memory_warning = 2;
        }
        
        // 4. Tomar el mutex y leer sensor
        if(xSemaphoreTake(Sem3_Mutex_Sensor, pdMS_TO_TICKS(100)) == pdPASS) {
            sensor_local = sensor_data;
            xSemaphoreGive(Sem3_Mutex_Sensor);
        }
        
        // Actualizar debug del Timer (frecuencia y ISRs)
        TIM2_Debug_Update(&htim2);
        TIM2_Debug_t tim_report;
        TIM2_Debug_GetReport(&tim_report);
        // Los datos están disponibles en tim_report:
        // - tim_report.current_frequency_hz: Frecuencia teórica
        // - tim_report.isr_frequency_per_sec: Frecuencia real de ISRs
        // - tim_report.error_flags: Flags de error del timer
        
        // Copiar a estructura global de stats si la tienes
        // (según tu estructura debugstats)
        debug_stats.isr_frequency = tim_report.isr_frequency_per_sec;
        debug_stats.tim_error_flags = tim_report.error_flags;
        
        // Recuperar timer si hay error
        if(tim_report.error_flags != 0) {
            // Aquí podrías tomar acciones correctivas si es necesario
            // Por ahora solo registramos el error
        }

        // 5. Alerta crítica: disparar error
        if(debug_stats.memory_warning == 2) {
            ERROR_TASK(TASK_MONITOR);
        }

        // 6. Heartbeat: Toggleamos el LED para indicar que el sistema está vivo
        HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);

        // 7. Bloqueo preciso y determinístico
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


/* ---------------------------------- CALLBACKS ----------------------------------*/


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

// ========== DEBUG: ERROR HANDLER CON INFO DE TAREA ==========
void Error_Handler_Task(TaskID_t task_id, uint32_t line, const char* file) {
	// Guardar contexto del error
	current_error_task = task_id;
	current_error_task_handle = xTaskGetCurrentTaskHandle();
	error_line = line;
	error_file = file;
	
	// Deshabilitar interrupciones
	__disable_irq();
	
	// Parpadear LED con patrón según la tarea
	// UNKNOWN=1, APP_INIT=2, DRIVER=3, SENSOR=4, CONTROL=5, INPUT_HID=6, OUTPUT_HID=7, MONITOR=8
	for(int i = 0; i < task_id; i++) {
		HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_SET);
		for(volatile int j = 0; j < 500000; j++);
		HAL_GPIO_WritePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin, GPIO_PIN_RESET);
		for(volatile int j = 0; j < 500000; j++);
	}
	
	// Esperar indefinidamente
	while(1) {
		HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);
		for(volatile int j = 0; j < 2000000; j++);
	}
}



