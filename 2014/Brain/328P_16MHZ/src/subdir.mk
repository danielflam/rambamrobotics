################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/arduinoMain.cpp \
../src/main.cpp 

OBJS += \
./src/arduinoMain.o \
./src/main.o 

CPP_DEPS += \
./src/arduinoMain.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\dev\eclipse_avr\ArduCore\include" -I"C:\dev\eclipse_avr\ArduLibs\include\Wire" -I"C:\dev\eclipse_avr\ArduLibs\include" -I"C:\dev\eclipse_avr\Brain\include" -D__AVR_ATmega328P__ -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


