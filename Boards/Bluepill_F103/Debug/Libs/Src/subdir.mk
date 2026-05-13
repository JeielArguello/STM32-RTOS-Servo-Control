################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Src/stepper.c \
/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Src/taskStepper.c 

OBJS += \
./Libs/Src/stepper.o \
./Libs/Src/taskStepper.o 

C_DEPS += \
./Libs/Src/stepper.d \
./Libs/Src/taskStepper.d 


# Each subdirectory must supply rules for building sources it contributes
Libs/Src/stepper.o: /home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Src/stepper.c Libs/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Src" -I"/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Inc" -I/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I"/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs" -I/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Libs/Src/taskStepper.o: /home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Src/taskStepper.c Libs/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Src" -I"/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs/Inc" -I/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc -I"/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs" -I/home/ricardo/Facultad/Sist_RT/Servo-motor/STM32-RTOS-Servo-Control/Libs -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Libs-2f-Src

clean-Libs-2f-Src:
	-$(RM) ./Libs/Src/stepper.cyclo ./Libs/Src/stepper.d ./Libs/Src/stepper.o ./Libs/Src/stepper.su ./Libs/Src/taskStepper.cyclo ./Libs/Src/taskStepper.d ./Libs/Src/taskStepper.o ./Libs/Src/taskStepper.su

.PHONY: clean-Libs-2f-Src

