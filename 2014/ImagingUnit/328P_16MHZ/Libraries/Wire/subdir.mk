################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libraries/Wire/Wire.cpp 

OBJS += \
./Libraries/Wire/Wire.o 

CPP_DEPS += \
./Libraries/Wire/Wire.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/Wire/%.o: ../Libraries/Wire/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\dev\eclipse_avr\UnoCore\src" -I"C:\dev\eclipse_avr\LineFollower\Libraries" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


