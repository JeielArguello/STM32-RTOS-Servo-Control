# STM32 RTOS Servo Control

Este proyecto implementa un sistema de control de posición de lazo cerrado (servomotor) utilizando un motor paso a paso (simil NEMA 17) y un encoder magnético AS5600. La arquitectura está diseñada bajo un paradigma de **Sistemas de Tiempo Real (RTOS)**, permitiendo un control determinístico y una estructura de código agnóstica al hardware.

## Características Técnicas

*   **Arquitectura RTOS:** Implementación basada en **FreeRTOS** con 6 tareas concurrentes que gestionan adquisición, control (PID), comunicación y seguridad.
*   **Muestreo Determinístico:** Adquisición de datos mediante **I2C + DMA** disparado por hardware a través del **TIM** (Update Event) a intervalos de 1ms.
*   **Lógica de Control:** Algoritmo **PID** de alta frecuencia para el posicionamiento preciso del motor.
*   **Abstracción de Hardware (HAL):** Separación de la lógica de aplicación y drivers en `/libs`, permitiendo la permutación entre diferentes MCUs mediante definiciones en el preprocesador.
*   **Interfaz USB-HID:** Telemetría y recepción de consignas en tiempo real desde una PC sin necesidad de drivers adicionales.

## Estructura del Repositorio

*   **`/Boards`**: Contiene los proyectos específicos de **STM32CubeIDE** para cada placa de desarrollo.
    *   `/Bluepill_F103`: Configuración para el STM32F103C8.
    *   `/Discovery_F411`: Configuración para el STM32F411VE.
*   **`/libs`**: Drivers para el encoder magnético AS5600 y el control del driver de motor paso a paso (DRV8825).
*   **`/Docs`**: Documentación técnica, esquemáticos y diagramas de arquitectura.
*   **`/App`**:  Aplicacion de consola para comunicarse mediante UDB HID con el micro.


## Hardware Utilizado

*   **MCU:** STM32F103C8T6 (Bluepill) o STM32F411VET6 (Discovery).
*   **Driver de Motor:** DRV8825 (Modo Step/Dir).
*   **Sensor de Posición:** Encoder magnético AS5600 conectado vía I2C.
*   **Actuador:** Motor paso a paso (simil NEMA 17).

## Diagrama de Tareas (RTOS Architecture)



1.  **Sensor Task:** Procesa el ángulo bruto del AS5600 y calcula rotaciones.
2.  **Control Task:** Calcula el error de posición y ejecuta el PID.
3.  **Driver Task:** Genera los pulsos para el DRV8825.
4.  **HID Tasks (In/Out):** Gestión de la comunicación con el host.
5.  **Monitor Task:** Supervisión de seguridad y diagnóstico del sistema.

## Cómo empezar

1.  **Clonar el repositorio:**
    ```bash
    git clone https://github.com/JeielArguello/STM32-RTOS-Servo-Control.git

   ```

## Requisitos

### Linux 
    ```bash
    sudo apt install libudev1 libudev-dev pkg-config
    sudo apt install libhidapi-dev
   ```