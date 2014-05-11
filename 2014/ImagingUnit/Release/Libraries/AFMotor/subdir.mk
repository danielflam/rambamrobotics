################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Libraries/AFMotor/AFMotor.cpp 

OBJS += \
./Libraries/AFMotor/AFMotor.o 

CPP_DEPS += \
./Libraries/AFMotor/AFMotor.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/AFMotor/%.o: ../Libraries/AFMotor/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\dev\eclipse_avr\UnoCore\src" -I"C:\dev\eclipse_avr\LineFollower\Libraries" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


