################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include Third-Party/SEGGER/SEGGER/subdir.mk
-include Third-Party/SEGGER/OS/subdir.mk
-include Third-Party/SEGGER/Config/subdir.mk
-include Third-Party/FreeRTOS/org/Source/portable/MemMang/subdir.mk
-include Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Third-Party/FreeRTOS/org/Source/subdir.mk
-include StdPeriph_Driver/src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: LoadCell_interfacing_STM32F4.elf

# Tool invocations
LoadCell_interfacing_STM32F4.elf: $(OBJS) $(USER_OBJS) /home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/LinkerScript.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T"/home/ankit/workspace/RTOS_workspace/LoadCell_interfacing_STM32F4/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "LoadCell_interfacing_STM32F4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "LoadCell_interfacing_STM32F4.elf" "LoadCell_interfacing_STM32F4.bin"
	arm-none-eabi-size "LoadCell_interfacing_STM32F4.elf"
	arm-none-eabi-objcopy -O ihex "LoadCell_interfacing_STM32F4.elf" "LoadCell_interfacing_STM32F4.hex"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
