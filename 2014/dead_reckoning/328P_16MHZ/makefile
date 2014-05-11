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
dead_reckoning.lss \

FLASH_IMAGE += \
dead_reckoning.hex \

EEPROM_IMAGE += \
dead_reckoning.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: dead_reckoning.elf secondary-outputs

dependents:
	-cd C:/dev/eclipse_avr/ArduCore/328P_16MHZ && $(MAKE) all
	-cd C:/dev/eclipse_avr/ArduLibs/328P_16MHZ && $(MAKE) all

# Tool invocations
dead_reckoning.elf: $(OBJS) $(USER_OBJS) C:/dev/eclipse_avr/ArduCore/328P_16MHZ/libArduCore.a C:/dev/eclipse_avr/ArduLibs/328P_16MHZ/libArduLibs.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -lc -lm --cref -s -Os -o"dead_reckoning.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lm -Wl,-Map,dead_reckoning.map,--cref -mrelax -Wl,--gc-sections -L"C:\dev\eclipse_avr\ArduCore\328P_16MHZ" -L"C:\dev\eclipse_avr\ArduLibs\328P_16MHZ" -mmcu=atmega328p -lc
	@echo 'Finished building target: $@'
	@echo ' '

dead_reckoning.lss: dead_reckoning.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S dead_reckoning.elf  >"dead_reckoning.lss"
	@echo 'Finished building: $@'
	@echo ' '

dead_reckoning.hex: dead_reckoning.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex dead_reckoning.elf  "dead_reckoning.hex"
	@echo 'Finished building: $@'
	@echo ' '

dead_reckoning.eep: dead_reckoning.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex dead_reckoning.elf  "dead_reckoning.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: dead_reckoning.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p dead_reckoning.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(CPP_DEPS)$(RL_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) dead_reckoning.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
C:/dev/eclipse_avr/ArduCore/328P_16MHZ/libArduCore.a:
C:/dev/eclipse_avr/ArduLibs/328P_16MHZ/libArduLibs.a:

-include ../makefile.targets
