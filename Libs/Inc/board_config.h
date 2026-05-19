/*
 * board_config.h
 *
 *  Created on: May 13, 2026
 *      Author: ricardo
 */

#ifndef INC_BOARD_CONFIG_H_
#define INC_BOARD_CONFIG_H_

#include "main.h"

// --- Definiciones de Pines ---
#ifdef STM32F103xB  // Si es la Bluepill

	#define GPIO_LED_Pin GPIO_PIN_13
	#define GPIO_LED_GPIO_Port GPIOC
	#define TIM2_STEP_Pin GPIO_PIN_1
	#define TIM2_STEP_GPIO_Port GPIOA
	#define GPIO_DIR_Pin GPIO_PIN_2
	#define GPIO_DIR_GPIO_Port GPIOA
	#define GPIO_M0_Pin GPIO_PIN_3
	#define GPIO_M0_GPIO_Port GPIOA
	#define GPIO_M1_Pin GPIO_PIN_4
	#define GPIO_M1_GPIO_Port GPIOA
	#define GPIO_M2_Pin GPIO_PIN_5
	#define GPIO_M2_GPIO_Port GPIOA
	#define GPIO_EN_Pin GPIO_PIN_6
	#define GPIO_EN_GPIO_Port GPIOA


	// --- Configuración de Periféricos ---
	#define ENCODER_I2C_HANDLE  hi2c1
	#define ENCODER_I2C_INSTANCE I2C1
	#define DRIVER_TIMER_HANDLE  htim2
	#define DRIVER_TIMER_INSTANCE TIM2
	#define DRIVER_TIMER_CHANNEL TIM_CHANNEL_2
	#define SAMPLE_TIMER_HANDLE htim4
	#define SAMPLE_TIMER_INSTANCE TIM4
#elif
#ifdef STM32F411xE  // Si es la Discovery
  	#define GPIO_LED_Pin GPIO_PIN_13
	#define GPIO_LED_GPIO_Port GPIOC
	#define TIM2_STEP_Pin GPIO_PIN_1
	#define TIM2_STEP_GPIO_Port GPIOA
	#define GPIO_DIR_Pin GPIO_PIN_2
	#define GPIO_DIR_GPIO_Port GPIOA
	#define GPIO_M0_Pin GPIO_PIN_3
	#define GPIO_M0_GPIO_Port GPIOA
	#define GPIO_M1_Pin GPIO_PIN_4
	#define GPIO_M1_GPIO_Port GPIOA
	#define GPIO_M2_Pin GPIO_PIN_5
	#define GPIO_M2_GPIO_Port GPIOA
	#define GPIO_EN_Pin GPIO_PIN_6
	#define GPIO_EN_GPIO_Port GPIOA

	// --- Configuración de Periféricos ---
	#define ENCODER_I2C_HANDLE  hi2c1
	#define ENCODER_I2C_INSTANCE I2C1
	#define DRIVER_TIMER_HANDLE  htim2
	#define DRIVER_TIMER_INSTANCE TIM2
	#define DRIVER_TIMER_CHANNEL TIM_CHANNEL_1
	#define SAMPLE_TIMER_HANDLE htim1
#endif
#endif


#endif /* INC_BOARD_CONFIG_H_ */
