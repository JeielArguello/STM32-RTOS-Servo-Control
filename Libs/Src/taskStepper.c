/*
 * taskStepper.c
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#include "taskStepper.h"
#include "usb_device.h"
#include "usbd_custom_hid_if.h"
#include "hid_manager.h"
#include "pid.h"
#include "tim_debug.h"
#include "motor_debug.h"
#include "debugstats.h"

// Macro para convertir microsegundos a ticks de FreeRTOS
#define pdUS_TO_TICKS(us) ((TickType_t)(((uint64_t)(us) * configTICK_RATE_HZ) / 1000000UL))

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
QueueHandle_t Queue2_SensorControl; // Sensor -> Control (Datos del sensor)
QueueHandle_t Queue3_PosHandle;    // Control -> Driver (Pasos/Velocidad)
QueueHandle_t Queue4_SensorOutputHID; // Sensor -> OutputHID (Datos para HID)

/* Handles de Tareas (para monitoreo) */
TaskHandle_t hDriverTask;
TaskHandle_t hSensorTask;
TaskHandle_t hControlTask;
TaskHandle_t hInputHIDTask;
TaskHandle_t hOutputHIDTask;
TaskHandle_t hMonitorTask;

/* Handles de Semáforos */
// Sem1_HID_RxHandle y Sem2_DMA_RxHandle: reemplazados por notificaciones de tareas
SemaphoreHandle_t Sem3_Mutex_Sensor; // Mutex del sensor
TimerHandle_t xTimerSensorHandle;


uint32_t get_tick_ms(void){
	return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* --------------------------- Tareas ------------------------------------------------*/

void AppInit(void * pvParameters){


	// Sem1_HID_RxHandle y Sem2_DMA_RxHandle: ahora se usan notificaciones de tareas
	// No requieren creación de semáforos, las tareas tienen notificaciones nativas

	Sem3_Mutex_Sensor = xSemaphoreCreateMutex();
	if(Sem3_Mutex_Sensor == NULL) {
		ERROR_TASK(TASK_APP_INIT);
	}

	

	// Inicializar librerías de Debug y HID
	MotorDebug_Init();
	HID_Manager_Init();

	Queue1_ComHandle = xQueueCreate(2,sizeof(int32_t));    // PC -> Control
	Queue2_SensorControl = xQueueCreate(5,sizeof(EncoderData_t)); // Sensor -> Control
	Queue4_SensorOutputHID = xQueueCreate(5,sizeof(EncoderData_t)); // Sensor -> Control
	Queue3_PosHandle = xQueueCreate(2,sizeof(int32_t));     // Control -> Driver

	if(Queue1_ComHandle  == NULL) {
		// Error al crear la Queue
		ERROR_TASK(TASK_APP_INIT);
	}

	if(Queue2_SensorControl  == NULL) {
		// Error al crear la Queue
		ERROR_TASK(TASK_APP_INIT);
	}
	
	if(Queue3_PosHandle  == NULL) {
		// Error al crear la Queue
		ERROR_TASK(TASK_APP_INIT);
	}
	
	if(Queue4_SensorOutputHID  == NULL) {
		// Error al crear la Queue
		ERROR_TASK(TASK_APP_INIT);
	}

	uint8_t codigo_error = Diagnosticar_AS5600(&ENCODER_I2C_HANDLE, &sensor_data);
			
	if (codigo_error != ENCODER_OK) {
		
		// CASO CATASTRÓFICO 1: Se cortó el cable o se tildó el bus I2C
		if (codigo_error & ENCODER_ERR_I2C) {
			ERROR_TASK(TASK_APP_INIT);
		}
		
		// CASO CATASTRÓFICO 2: El imán salió volando mecánicamente
		if (codigo_error & ENCODER_ERR_NO_MAG) {
			ERROR_TASK(TASK_APP_INIT);
		}
		
		// CASOS ADVERTENCIA: El imán está descentrado, mal posicionado o zofra (causa saltos)
		if ((codigo_error & ENCODER_ERR_MAG_LOW) || (codigo_error & ENCODER_ERR_MAG_HIGH)) {
			ERROR_TASK(TASK_APP_INIT); 
		}
	}


	MX_USB_DEVICE_Init();

	BaseType_t status;
	status = xTaskCreate(StartDriverTask, "DriverTask", 256, NULL, 7, &hDriverTask);
	if( status != pdPASS){
	ERROR_TASK(TASK_APP_INIT);
	}

	status = xTaskCreate(StartSensorTask, "SensorTask", 256, NULL, 4, &hSensorTask);
	if( status != pdPASS){
	ERROR_TASK(TASK_APP_INIT);
	}

	status = xTaskCreate(StartControlTask, "ControlTask", 512, NULL, 6, &hControlTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
	}


	status = xTaskCreate(StartInputHIDTask, "InputHIDTask", 256, NULL, 3, &hInputHIDTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
	}


	status = xTaskCreate(StartOutputHIDTask, "OutputHIDTask", 256, NULL, 2, &hOutputHIDTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
	}

	status = xTaskCreate(StartMonitorTask, "MonitorTask", 128, NULL, 1 , &hMonitorTask);
	if( status != pdPASS){
	  ERROR_TASK(TASK_APP_INIT);
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
	Stepper_SetMicrostepping(&motor, STEP_HALF); // 200 pasos por vuelta

	BaseType_t xStatus;
	int32_t rpm_from_control;

	// ===== PRUEBA INICIAL: Una vuelta =====
	MotorDebug_Start(360);  // 1 vuelta = 360 grados
	Stepper_SetSteps(&motor, 360);  // 360 Hz = 1 vuelta por segundo (200 pasos/vuelta × 2 toggles/paso = 400 Hz para 1 vuelta/s, pero ajustamos a 360 para compensar ineficiencias)
	Stepper_SetSpeed(&motor, 60);  // 60 RPM para prueba
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
			if(rpm_from_control == 0) {
				Stepper_Stop(&motor);
			} else {
				// Setear velocidad en RPM (la conversión a Hz ocurre dentro de Stepper_SetSpeed)
				Stepper_SetSpeed(&motor, rpm_from_control);
				
				// Arrancar motor
				if (!motor.is_running) {  // Solo llamar Start si el motor no está corriendo (evitar reiniciar el timer innecesariamente)
					Stepper_SetSteps(&motor, 100);  
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
    uint16_t prev_raw_position = 0;
    uint16_t current_raw = 0;
    int32_t diff = 0;
    int64_t pos_temp = 0;
    
    // El filtro necesita mantener su estado en el tiempo
    int32_t posicion_absoluta_filtrada = 0; 
    uint8_t primer_ciclo = 1;

    EncoderData_t sensor_local;
    BaseType_t xStatus;

	// inicializar sensor
	// Tomar Mutex para inicializar
	if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
        sensor_data.raw_position = 0;
        sensor_data.rotations = 0;
        sensor_data.angle_deg = 0;
        sensor_data.status = 0;
        sensor_data.speed_rpm = 0;
        sensor_data.total_ticks = 0;
        xSemaphoreGive(Sem3_Mutex_Sensor);
    }

	vTaskDelay(pdMS_TO_TICKS(2000));  // Esperar a que el sistema se estabilice
	// inicializo timer de software para lectura del sensor
	xTimerSensorHandle = xTimerCreate(
	        "TimerSensor",               // Nombre para debug
	        pdMS_TO_TICKS(3),            // Período: 3ms
	        pdTRUE,                      // pdTRUE = Auto-reload (cíclico). pdFALSE = One-shot (se ejecuta una vez)
	        (void *) 0,                  // ID del timer 
	        CallbackTimerSensor          // Función que se va a ejecutar
	    );
	if (xTimerSensorHandle != NULL) {
		BaseType_t timer_status;
		timer_status = xTimerStart(xTimerSensorHandle, portMAX_DELAY);

		if (timer_status != pdPASS) {
			ERROR_TASK(TASK_SENSOR);
		}
	}else{
		ERROR_TASK(TASK_SENSOR);
	}

    for(;;) {
        // 1. Esperar al DMA (Sincronizado con TIM) usando notificación
        if(xTaskNotifyWait(0, 0x02, NULL, portMAX_DELAY) == pdPASS) {

            // 2. Procesar el dato crudo actual de 12 bits 
            
            current_raw = ((uint16_t)sensor_data.buffer[0] << 8) | sensor_data.buffer[1];
            current_raw &= 0x0FFF; // Máscara de 12 bits (0-4095) por seguridad

            if (primer_ciclo) {
                prev_raw_position = current_raw;
				if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
					// CRÍTICO: total_ticks NO puede arrancar en 0. 
					// Tiene que arrancar en la posición real del motor.
					sensor_data.total_ticks = (int32_t)current_raw; 
					xSemaphoreGive(Sem3_Mutex_Sensor);
				}
                posicion_absoluta_filtrada = current_raw; // Inicializar filtro
                primer_ciclo = 0;
            }

            // 3. CALCULAR DESPLAZAMIENTO NETO (Unwrapping sobre el valor real, sin promediar)
            diff = (int32_t)current_raw - (int32_t)prev_raw_position;
            diff = (diff + 2048) & 4095;
            diff -= 2048;
            
            prev_raw_position = current_raw; // Guardar para el próximo ms

            // 4. Tomar Mutex para actualizar los datos globales de control
            if(xSemaphoreTake(Sem3_Mutex_Sensor, portMAX_DELAY) == pdPASS) {
                
                // Acumular la posición real 
                sensor_data.total_ticks += diff; 

                // 5. FILTRO EXPONENCIAL (EMA) sobre la posición absoluta de 32 bits
                // Equivale a un promedio suave de las últimas ~8 muestras, pero sin romper el cero.
                posicion_absoluta_filtrada = ((posicion_absoluta_filtrada * 7) + sensor_data.total_ticks) >> 3;

                // Guardamos los valores filtrados en la estructura local y global
                sensor_local.total_ticks = posicion_absoluta_filtrada;
                //sensor_data.total_ticks_filtrados = posicion_absoluta_filtrada; // Opcional, si querés trackear el filtrado separado

                sensor_data.raw_position = current_raw;
                sensor_local.raw_position = current_raw;

                // Calcular vueltas basado en la posición filtrada
                sensor_data.rotations = (int32_t)(posicion_absoluta_filtrada / 4096);
                sensor_local.rotations = sensor_data.rotations;

				// Redondeo simétrico: sumamos medio divisor antes de dividir
				int64_t numerador = (int64_t)posicion_absoluta_filtrada * 360;
				if (numerador >= 0) {
					pos_temp = (numerador + 2048) / 4096;  // +0.5 * 4096
				} else {
					pos_temp = (numerador - 2048) / 4096;
				}
                sensor_data.angle_deg = (int32_t)pos_temp;
                sensor_local.angle_deg = sensor_data.angle_deg;

                // Velocidad
                sensor_data.speed_rpm = motor.current_rpm; 
                sensor_local.speed_rpm = sensor_data.speed_rpm;

                xSemaphoreGive(Sem3_Mutex_Sensor);

                // 6. Despertar a ControlTask y OutputHIDTask con datos limpios cada 1ms
                xStatus = xQueueSendToBack(Queue2_SensorControl, &sensor_local, 0);
                if(xStatus == pdFAIL) { ERROR_TASK(TASK_SENSOR); }
                
                xStatus = xQueueSendToBack(Queue4_SensorOutputHID, &sensor_local, 0);
                if(xStatus == pdFAIL) { ERROR_TASK(TASK_SENSOR); }
			}
        }
    }
}


void StartControlTask(void *argument) {

	EncoderData_t sensor_local;
	int64_t set_point_ticks = 0;  // Setpoint en ticks del sensor
	BaseType_t xStatus;
	PIDController pid;
	
	// GANANCIAS DEL PID EN Q16
	//int64_t A = Q16_DIV(255,4095);
	int64_t Kp_q16 = 46200;  // Proporcional: 
	int64_t Ki_q16 = 0;      // Integral: 
	int64_t Kd_q16 = 300;       // Derivativo: 
	
	int64_t error_ticks = 0;
	
	int32_t rpm_target = 0;  

	int64_t current_ticks = 0;
	// Revisar si hay nuevo setpoint del usuario
	int32_t set_point_new = 0;
	// Esperar a que driver esté activa
	vTaskDelay(pdMS_TO_TICKS(2000));

	PIDController_Init(&pid, Kp_q16, Ki_q16, Kd_q16, 0, 225, 0, get_tick_ms); // Ramp=0 (deshabilitada), Limit=450 RPM, Ts=0 (calculado dinámicamente), función de tiempo personalizada

    for(;;){

		// Tomar el mutex y leer la estructura global del sensor
		xStatus = xQueueReceive(Queue2_SensorControl, &sensor_local, portMAX_DELAY);
		if(xStatus == pdFAIL) {
			ERROR_TASK(TASK_CONTROL);
		}

		xStatus = xQueueReceive(Queue1_ComHandle, &set_point_new, 0);
		if(xStatus == pdPASS) {
			set_point_ticks = (int64_t)set_point_new;
			// Reiniciar PID con el nuevo setpoint para evitar overshoot
			PIDController_Reset(&pid);
		}
		int64_t DEADBAND_TICKS = 5; // Zona muerta de 10 ticks (ajustable según la resolución y el comportamiento deseado)
		
		// Calcular posición actual en ticks (multivuelta)
		current_ticks =  sensor_local.total_ticks;

		// Error de posición
		error_ticks =  set_point_ticks - current_ticks;

		if (abs(error_ticks) <= DEADBAND_TICKS) {
			// Dentro de la zona muerta: no hay movimiento posible que mejore la posición
			rpm_target = 0;
			PIDController_Reset(&pid);  // anti-windup, aunque en tu caso Ki=0 esto es preventivo
		} else {
			// Fuera de la zona muerta: PID normal
			rpm_target = PIDController_Update(&pid, error_ticks);
		}
		// ===== ENVIAR RPM AL DRIVER =====
		if(xQueueSendToBack(Queue3_PosHandle, &rpm_target, 0) == pdFAIL){
			ERROR_TASK(TASK_CONTROL);
		}
		
		
    }
}


void StartInputHIDTask(void *argument) {


    HID_Report_t incoming_report;
    int32_t new_setpoint_ticks = 0;
    int32_t new_setpoint_deg = 0;
    USBD_CUSTOM_HID_HandleTypeDef *hhid;
	int64_t new_setpoint_q16 = 0;
	int64_t new_setpoint_ticks_q16 = 0;

    for(;;) {
        // 1. Esperar notificación del USB (Bloqueo eficiente)
    	 if(xTaskNotifyWait(0, 0x01, NULL, portMAX_DELAY) == pdPASS) {

			// 2. Obtener el puntero al buffer de recepción
			hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

			// 3. Copiar de forma segura los datos al reporte local
			// hhid->Report_buf contiene el reporte recibido (incluyendo ID)
			memcpy(&incoming_report, hhid->Report_buf, sizeof(HID_Report_t));

			// 4. Procesar según el ID del reporte
			if (incoming_report.reportID == 0x02) { // Reporte de consignas

				// El PC manda una nueva posición deseada en grados
				new_setpoint_deg = -incoming_report.position;  // Posición en grados

				// Convertir grados en grados q16
				new_setpoint_q16 = Q16_FROM_INT(new_setpoint_deg);

				// Convertir grados a ticks en q16 (1 vuelta = 360 grados = 4096 ticks)
				new_setpoint_ticks_q16 = Q16_DIV(Q16_MUL(new_setpoint_q16, Q16_FROM_INT(4096)), Q16_FROM_INT(360));

				new_setpoint_ticks = Q16_TO_INT(new_setpoint_ticks_q16);
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
	BaseType_t xStatus;

    for(;;) {
        // 1. Esperar a que el SensorTask publique datos frescos
        xStatus = xQueueReceive(Queue4_SensorOutputHID, &sensor_local, portMAX_DELAY);
		if(xStatus == pdFAIL) {
			ERROR_TASK(TASK_OUTPUT_HID);
		}

        // 3. Actualizar datos HID con info del sensor
        uint8_t status = HID_Manager_GetStatus();
        HID_Manager_Update(&sensor_local, status);

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
        if(debug_stats.memory_warning == 3) {
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
        // Despertar a SensorTask mediante notificación
        xTaskNotifyFromISR(hSensorTask, 0x02, eSetBits, &xHigherPriorityTaskWoken);
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



