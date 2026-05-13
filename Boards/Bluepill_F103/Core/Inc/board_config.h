/*
 * board_config.h
 *
 *  Created on: May 13, 2026
 *      Author: ricardo
 */

#ifndef INC_BOARD_CONFIG_H_
#define INC_BOARD_CONFIG_H_

#include "main.h"

// --- Definiciones de Pines para el Driver ---
#ifdef STM32F103xB  // Si es la Bluepill
    #define DRIVER_PORT_STEP  GPIOA
    #define DRIVER_PIN_STEP   GPIO_PIN_1
    #define DRIVER_PORT_DIR   GPIOA
    #define DRIVER_PIN_DIR    GPIO_PIN_2
    #define DRIVER_PORT_EN    GPIOA
    #define DRIVER_PIN_EN     GPIO_PIN_6
#elif defined(STM32F411xE)  // Si es la Discovery
    #define DRIVER_PORT_STEP  GPIOC
    #define DRIVER_PIN_STEP   GPIO_PIN_6
    #define DRIVER_PORT_DIR   GPIOC
    #define DRIVER_PIN_DIR    GPIO_PIN_1
    #define DRIVER_PORT_EN    GPIOD
    #define DRIVER_PIN_EN     GPIO_PIN_12
#endif

// --- Configuración de Periféricos ---
#define ENCODER_I2C_HANDLE  hi2c1
#define MOTOR_TIMER_HANDLE  htim2
#define SAMPLE_TIMER_HANDLE htim1


#endif /* INC_BOARD_CONFIG_H_ */
