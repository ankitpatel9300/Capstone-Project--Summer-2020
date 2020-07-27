################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3/port.c 

OBJS += \
./Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3/port.o 

C_DEPS += \
./Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3/port.d 


# Each subdirectory must supply rules for building sources it contributes
Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3/%.o: ../Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -D__LPC17XX__ -D__REDLIB__ -I"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/lpc_board_nxp_lpcxpresso_1769/inc" -I"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/lpc_board_nxp_lpcxpresso_1769/inc" -I"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/lpc_chip_175x_6x/inc" -I"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/Interface_LPC1769_SIM7600/inc" -I"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/Interface_LPC1769_SIM7600/Third-Party/FreeRTOS/org/Source/include" -I"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/Interface_LPC1769_SIM7600/Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m3 -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


