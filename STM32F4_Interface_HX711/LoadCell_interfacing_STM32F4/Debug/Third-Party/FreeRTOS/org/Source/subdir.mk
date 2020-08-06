################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Third-Party/FreeRTOS/org/Source/croutine.c \
../Third-Party/FreeRTOS/org/Source/event_groups.c \
../Third-Party/FreeRTOS/org/Source/list.c \
../Third-Party/FreeRTOS/org/Source/queue.c \
../Third-Party/FreeRTOS/org/Source/stream_buffer.c \
../Third-Party/FreeRTOS/org/Source/tasks.c \
../Third-Party/FreeRTOS/org/Source/timers.c 

OBJS += \
./Third-Party/FreeRTOS/org/Source/croutine.o \
./Third-Party/FreeRTOS/org/Source/event_groups.o \
./Third-Party/FreeRTOS/org/Source/list.o \
./Third-Party/FreeRTOS/org/Source/queue.o \
./Third-Party/FreeRTOS/org/Source/stream_buffer.o \
./Third-Party/FreeRTOS/org/Source/tasks.o \
./Third-Party/FreeRTOS/org/Source/timers.o 

C_DEPS += \
./Third-Party/FreeRTOS/org/Source/croutine.d \
./Third-Party/FreeRTOS/org/Source/event_groups.d \
./Third-Party/FreeRTOS/org/Source/list.d \
./Third-Party/FreeRTOS/org/Source/queue.d \
./Third-Party/FreeRTOS/org/Source/stream_buffer.d \
./Third-Party/FreeRTOS/org/Source/tasks.d \
./Third-Party/FreeRTOS/org/Source/timers.d 


# Each subdirectory must supply rules for building sources it contributes
Third-Party/FreeRTOS/org/Source/%.o: ../Third-Party/FreeRTOS/org/Source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -DSTM32F40XX -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/StdPeriph_Driver/inc" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/SEGGER/Config" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/SEGGER/OS" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/SEGGER/SEGGER" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/FreeRTOS/org/Source/include" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Config" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/inc" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/CMSIS/device" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


