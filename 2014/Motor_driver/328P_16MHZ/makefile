################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(RL_DEPS)),)
-include $(RL_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Motor_driver.lss \

FLASH_IMAGE += \
Motor_driver.hex \

EEPROM_IMAGE += \
Motor_driver.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Motor_driver.elf secondary-outputs

dependents:
	-cd C:/dev/eclipse_avr/ArduCore/328P_16MHZ && $(MAKE) all
	-cd C:/dev/eclipse_avr/ArduLibs/328P_16MHZ && $(MAKE) all

# Tool invocations
Motor_driver.elf: $(OBJS) $(USER_OBJS) C:/dev/eclipse_avr/ArduCore/328P_16MHZ/libArduCore.a C:/dev/eclipse_avr/ArduLibs/328P_16MHZ/libArduLibs.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -lc -lm --cref -s -Os -o"Motor_driver.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm -Wl,-Map,Motor_driver.map,--cref -mrelax -Wl,--gc-sections -L"C:\dev\eclipse_avr\ArduCore\328P_16MHZ" -L"C:\dev\eclipse_avr\ArduLibs\328P_16MHZ" -mmcu=atmega328p -lc
	@echo 'Finished building target: $@'
	@echo ' '

Motor_driver.lss: Motor_driver.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Motor_driver.elf  >"Motor_driver.lss"
	@echo 'Finished building: $@'
	@echo ' '

Motor_driver.hex: Motor_driver.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Motor_driver.elf  "Motor_driver.hex"
	@echo 'Finished building: $@'
	@echo ' '

Motor_driver.eep: Motor_driver.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Motor_driver.elf  "Motor_driver.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Motor_driver.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p Motor_driver.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(RL_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) Motor_driver.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
C:/dev/eclipse_avr/ArduCore/328P_16MHZ/libArduCore.a:
C:/dev/eclipse_avr/ArduLibs/328P_16MHZ/libArduLibs.a:

-include ../makefile.targets
