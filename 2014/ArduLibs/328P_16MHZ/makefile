################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/ov7670/subdir.mk
-include src/itg3200filv05/subdir.mk
-include src/adxl345driver/subdir.mk
-include src/Wire/subdir.mk
-include src/Servo/subdir.mk
-include src/LEDSensor/subdir.mk
-include src/HMC58X3/subdir.mk
-include src/FrequencyTimer2/subdir.mk
-include src/FreeIMU/subdir.mk
-include src/EEPROM/subdir.mk
-include src/DigitalIO/subdir.mk
-include src/CmdProcessor/subdir.mk
-include src/AFMotor/subdir.mk
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
libArduLibs.lss \

SIZEDUMMY += \
sizedummy \


# All Target
all: libArduLibs.a secondary-outputs

dependents:
	-cd C:/dev/eclipse_avr/ArduCore/328P_16MHZ && $(MAKE) all

# Tool invocations
libArduLibs.a: $(OBJS) C:/dev/eclipse_avr/ArduCore/328P_16MHZ/libArduCore.a
	@echo 'Building target: $@'
	@echo 'Invoking: AVR Archiver'
	avr-ar -r  "libArduLibs.a" $(OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

libArduLibs.lss: libArduLibs.a
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S libArduLibs.a  >"libArduLibs.lss"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: libArduLibs.a
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega328p libArduLibs.a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(ARCHIVES)$(CPP_DEPS)$(RL_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) libArduLibs.a
	-@echo ' '

secondary-outputs: $(LSS) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:
C:/dev/eclipse_avr/ArduCore/328P_16MHZ/libArduCore.a:

-include ../makefile.targets
