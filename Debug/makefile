################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
devB_prac4_canRead.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
devB_prac4_canRead.list \

OBJCOPY_BIN += \
devB_prac4_canRead.bin \


# All Target
all: devB_prac4_canRead.elf secondary-outputs

# Tool invocations
devB_prac4_canRead.elf: $(OBJS) $(USER_OBJS) C:\Users\kulal\STM32CubeIDE\workspace_1.0.0\devB_prac4_canRead\STM32F105R8TX_FLASH.ld
	arm-none-eabi-gcc -o "devB_prac4_canRead.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\kulal\STM32CubeIDE\workspace_1.0.0\devB_prac4_canRead\STM32F105R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="devB_prac4_canRead.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

devB_prac4_canRead.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "devB_prac4_canRead.list"
	@echo 'Finished building: $@'
	@echo ' '

devB_prac4_canRead.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "devB_prac4_canRead.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
