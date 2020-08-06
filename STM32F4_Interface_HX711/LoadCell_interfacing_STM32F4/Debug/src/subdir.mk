################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/main.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/main.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DSTM32F407G_DISC1 -DDEBUG -DSTM32F40XX -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/StdPeriph_Driver/inc" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/SEGGER/Config" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/SEGGER/OS" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/SEGGER/SEGGER" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/FreeRTOS/org/Source/include" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Config" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/inc" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/CMSIS/device" -I"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/CMSIS/core" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


