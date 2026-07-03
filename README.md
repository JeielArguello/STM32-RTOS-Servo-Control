# STM32 RTOS Servo Control

Sistema de control de posición en lazo cerrado para un motor paso a paso tipo NEMA 17, utilizando un encoder magnético AS5600 como realimentación, un driver DRV8825 como etapa de potencia y FreeRTOS como núcleo de concurrencia. El proyecto fue diseñado para mantener una arquitectura modular y portable entre STM32F103 y STM32F411 mediante HAL, separando claramente adquisición, control, comunicación y supervisión.

## Resumen Técnico

El firmware implementa un servo de posición que recibe consignas desde una PC por USB-HID, mide la posición real del eje mediante I2C + DMA, calcula el error respecto del setpoint y ejecuta un controlador PID en punto fijo Q16. La salida del lazo gobierna al DRV8825 por señales STEP/DIR, con microstepping fijo en 1/2 paso. El sistema opera con un período de control de 3 ms y distribuye el trabajo en seis tareas FreeRTOS con prioridades diferenciadas.

## Características Principales

- Arquitectura de tiempo real con FreeRTOS y seis tareas concurrentes.
- Adquisición no bloqueante del AS5600 mediante I2C con DMA.
- Control PID en punto fijo Q16 para posicionamiento estable y reproducible.
- Comunicación USB-HID para consignas y telemetría en tiempo real.
- Separación por capas entre aplicación, drivers reutilizables y proyectos por placa.
- Diseño portable entre STM32F103C8T6 y STM32F411VET6.
- Documentación técnica completa en la carpeta `Docs`.

## Arquitectura del Sistema

El flujo de ejecución está organizado en torno a las siguientes tareas:

| Tarea | Función |
|---|---|
| Sensor_Task | Lee el AS5600, filtra la posición y publica datos de realimentación. |
| Control_Task | Calcula error, aplica PID y traduce el resultado a velocidad/comando de movimiento. |
| Driver_Task | Genera la señal hacia el DRV8825 y administra STEP/DIR. |
| Input_HID_Task | Recibe consignas desde la PC por USB-HID. |
| Output_HID_Task | Envía telemetría y estado del sistema a la PC. |
| Monitor_Task | Supervisa el sistema, maneja errores y señalización por LEDs. |

La sincronización entre tareas se resuelve con colas y notificaciones de tarea. Las interrupciones y el software timer del sistema solo se usan para disparar la adquisición periódica y notificar la disponibilidad de datos, evitando bloqueos innecesarios.

## Hardware Soportado

- MCU STM32F103C8T6, placa Bluepill.
- MCU STM32F411VET6, placa Discovery.
- Driver de motor DRV8825 en modo Step/Dir.
- Encoder magnético AS5600 por bus I2C.
- Motor paso a paso tipo NEMA 17 o equivalente.
- Interfaz USB-HID para consola de control en PC.

## Estructura del Repositorio

| Carpeta | Contenido |
|---|---|
| `App/` | Aplicación de consola para comunicarse con el firmware por USB-HID. |
| `Boards/Bluepill_F103/` | Proyecto CubeIDE, configuración y build para STM32F103. |
| `Boards/Discovery_F411/` | Proyecto CubeIDE, configuración y build para STM32F411. |
| `Libs/` | Código reutilizable de control, driver del motor, encoder y utilidades de debug. |
| `Docs/` | Informe final, PDFs de entregas y diagramas del proyecto. |

## Requisitos

### Para compilar el firmware

- STM32CubeIDE o un entorno compatible con los proyectos generados por CubeMX.
- Toolchain ARM para STM32.
- ST-Link v2 o compatible para flasheo y depuración.

### Para la aplicación de PC en Linux

```bash
sudo apt install libudev1 libudev-dev pkg-config
sudo apt install libhidapi-dev
```

## Cómo Compilar

1. Abrir el proyecto de la placa deseada desde `Boards/Bluepill_F103/` o `Boards/Discovery_F411/`.
2. Compilar el proyecto desde STM32CubeIDE.
3. Verificar que el binario o ELF quede generado en la carpeta `Debug` del board elegido.

## Cómo Flashear la Bluepill

La Bluepill cuenta con una tarea lista para OpenOCD. Desde la raíz del proyecto también puede ejecutarse el comando equivalente:

```bash
openocd -f interface/stlink-dap.cfg -f target/stm32f1x.cfg -c "adapter speed 1000" -c "init" -c "reset halt" -c "flash write_image erase Boards/Bluepill_F103/Debug/Bluepill_F103.elf" -c "reset run" -c "exit"
```

## Cómo Correr la App de PC

La consola de `App/` está escrita en C y usa `hidapi` junto con `pthread`. El punto de entrada es [App/main.c](App/main.c) y el binario se conecta por defecto al dispositivo `0483:5750`.

### Compilación en Linux

```bash
cd App
gcc main.c -o servo_console -lhidapi-hidraw -lpthread
```

Si tu distribución expone `hidapi` con otro backend, puede ser necesario reemplazar `-lhidapi-hidraw` por la variante disponible en tu sistema.

### Ejecución

```bash
./servo_console
```

Para usar otro VID/PID, pasalos como argumentos en hexadecimal:

```bash
./servo_console 0x0483 0x5750
```

Durante la ejecución, la consola muestra la telemetría del motor y acepta comandos por stdin. `q` cierra la aplicación; también se admiten comandos de posición en grados.


## Uso General

1. Conectar la placa STM32, el AS5600 y el DRV8825 según el hardware de la configuración elegida.
2. Flashear el firmware de la placa correspondiente.
3. Ejecutar la aplicación de consola en `App/` para enviar consignas y leer telemetría.
4. Enviar el setpoint de posición deseado desde la PC.
5. Verificar la respuesta del motor y el estado del sistema en la salida HID.

## Documentación

- [Informe final](Docs/Informe_Final.md)
- [Entrega N°1 - Anteproyecto](Docs/Ricardo%20Jeiel%20Arguello%20-%20Entrega%20N%C2%B01%20modalidad%20proyecto_%20Anteproyecto.pdf)
- [Entrega N°2 - Avance de proyecto](Docs/Ricardo%20Jeiel%20Arguello%20-%20Entrega%20N%C2%B02%20modalidad%20proyecto_%20Avance%20de%20proyecto.pdf)
- Diagramas y figuras del proyecto en `Docs/img/`

## Notas de Diseño

- El período de control está dimensionado en 3 ms para mantener determinismo en lazo cerrado.
- El encoder AS5600 ofrece mayor resolución que la mecánica del actuador, por lo que el sistema prioriza estabilidad y repetibilidad por sobre microstepping dinámico.
- La separación en tareas independientes evita que la comunicación USB o el monitoreo interfieran con el lazo crítico de control.

## Estado del Proyecto

El repositorio contiene tanto la implementación del firmware como la documentación técnica que explica la arquitectura, las decisiones de diseño y la validación del sistema.
