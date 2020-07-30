################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include Third-Party/FreeRTOS/org/Source/portable/MemMang/subdir.mk
-include Third-Party/FreeRTOS/org/Source/portable/GCC/ARM_CM3/subdir.mk
-include Third-Party/FreeRTOS/org/Source/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Interface_LPC1769_SIM7600.axf

# Tool invocations
Interface_LPC1769_SIM7600.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/lpc_board_nxp_lpcxpresso_1769/Debug" -L"/home/ankit/Documents/MCUXpresso_11.1.1_3241/workspace/lpc_chip_175x_6x/Debug" -Xlinker -Map="Interface_LPC1769_SIM7600.map" -Xlinker --cref -Xlinker --gc-sections -Xlinker -print-memory-usage -mcpu=cortex-m3 -mthumb -T "Interface_LPC1769_SIM7600_Debug.ld" -o "Interface_LPC1769_SIM7600.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) Interface_LPC1769_SIM7600.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "Interface_LPC1769_SIM7600.axf"; # arm-none-eabi-objcopy -v -O binary "Interface_LPC1769_SIM7600.axf" "Interface_LPC1769_SIM7600.bin" ; # checksum -p LPC1769 -d "Interface_LPC1769_SIM7600.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
