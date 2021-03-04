################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
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

BUILD_ARTIFACT_NAME := ECSE444_Lab1_G1
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
ECSE444_Lab1_G1.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ECSE444_Lab1_G1.list \

OBJCOPY_BIN += \
ECSE444_Lab1_G1.bin \


# All Target
all: main-build

# Main-build Target
main-build: ECSE444_Lab1_G1.elf secondary-outputs

# Tool invocations
ECSE444_Lab1_G1.elf: $(OBJS) $(USER_OBJS) C:\Users\Koshi\ECSE444\ECSE444---Lab\Lab\ 1\STM32L4S5VITX_FLASH.ld
	arm-none-eabi-gcc -o "ECSE444_Lab1_G1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Koshi\ECSE444\ECSE444---Lab\Lab 1\STM32L4S5VITX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ECSE444_Lab1_G1.map" -Wl,--gc-sections -static -L"C:\Users\Koshi\ECSE444\ECSE444---Lab\Lab 1\Core\Lib" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ECSE444_Lab1_G1.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ECSE444_Lab1_G1.list"
	@echo 'Finished building: $@'
	@echo ' '

ECSE444_Lab1_G1.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ECSE444_Lab1_G1.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
