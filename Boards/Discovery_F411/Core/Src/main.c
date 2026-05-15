/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stepper.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Estructura para almacenar los datos procesados
typedef struct {
    uint16_t raw_angle;
    int32_t rotations;
    float total_degrees;
} EncoderData_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AS5600_ADDR (0x36 << 1)
#define ANGLE_REG_MSB 0x0E

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
EncoderData_t sensor_data = {0};
Stepper_Handler motor = {0};
SemaphoreHandle_t xBinarySemaphore = NULL;
SemaphoreHandle_t xEndMovementSemaphore = NULL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void AppInit(void * pvParameters);
void StartStepperTask(void *argument);
void StartEncoderTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  xTaskCreate(AppInit, "TaskCreate", 100, NULL, 3, NULL);
  vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void AppInit(void * pvParameters){

	xBinarySemaphore = xSemaphoreCreateBinary();
	xEndMovementSemaphore = xSemaphoreCreateBinary();

	if(xBinarySemaphore  == NULL) {
		// Error al crear el semáforo
		HAL_Delay(1000);
	}

	xTaskCreate(StartEncoderTask, "EncoderTask", 100, NULL, 2, NULL);
	xTaskCreate(StartStepperTask, "MotorTask", 100, NULL, 1, NULL);

	for(;;){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

void StartStepperTask(void *argument) {
	// Inicialización del motor (usando tu nueva librería)
	    motor.step = (Stepper_Pin){GPIOC, GPIO_PIN_2};
	    motor.dir  = (Stepper_Pin){GPIOC, GPIO_PIN_1};
	    motor.htim = &htim3;
	    motor.channel = TIM_CHANNEL_1;
	    motor.m0 = (Stepper_Pin){GPIOD, GPIO_PIN_9};
	    motor.m1 = (Stepper_Pin){GPIOD, GPIO_PIN_10};
	    motor.m2 = (Stepper_Pin){GPIOD, GPIO_PIN_11};
	    Stepper_Init(&motor);
	    Stepper_SetMicrostepping(&motor, STEP_FULL); // 200 pasos por vuelta

	    for(;;){
	    	//Me bloqueo en el semaforo hasta que presionen un boton
	    	if(xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdPASS) {
	    		Stepper_Move(&motor, 200, 500);
				vTaskDelay(pdMS_TO_TICKS(2000));
	    	}
	    }
}


/* Función que se ejecuta cuando presionas el botón */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0) // Verificamos que sea el pin PA0
    {
        // Libero el semaforo para mover el motor
    	BaseType_t xHigherPriorityTaskWoken;

    	 xHigherPriorityTaskWoken = pdFALSE;

    	 xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken );

    	 portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        if (motor.steps_to_move > 0) {
            uint32_t val = __HAL_TIM_GET_COMPARE(htim, motor.channel);
            __HAL_TIM_SET_COMPARE(htim, motor.channel, val + motor.period_ticks);
            motor.steps_to_move--;
        } else {
            HAL_TIM_OC_Stop_IT(htim, motor.channel);

            // DAMOS EL SEMÁFORO: El motor se detuvo físicamente
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(xEndMovementSemaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}



void StartEncoderTask(void *argument) {
    uint8_t buffer[2];
    uint16_t last_raw = 0;

    // El periodo de muestreo (ej: 10ms = 100Hz)
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for(;;) {
        // 1. Leer el encoder vía I2C
        if (HAL_I2C_Mem_Read(&hi2c1, AS5600_ADDR, ANGLE_REG_MSB, I2C_MEMADD_SIZE_8BIT, buffer, 2, 5) == HAL_OK) {

            sensor_data.raw_angle = ((uint16_t)buffer[0] << 8) | buffer[1];

            // 2. Lógica de conteo de vueltas
            int16_t diff = sensor_data.raw_angle - last_raw;
            if (diff > 2048) sensor_data.rotations--;
            else if (diff < -2048) sensor_data.rotations++;

            last_raw = sensor_data.raw_angle;

            // 3. Calcular ángulo acumulado
            sensor_data.total_degrees = (sensor_data.rotations * 360.0f) + (sensor_data.raw_angle * 360.0f / 4096.0f);
        }

        // Esperar de forma eficiente hasta el próximo ciclo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
