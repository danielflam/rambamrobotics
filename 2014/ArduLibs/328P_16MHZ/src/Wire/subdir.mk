################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Wire/Wire.cpp \
../src/Wire/twi.cpp 

OBJS += \
./src/Wire/Wire.o \
./src/Wire/twi.o 

CPP_DEPS += \
./src/Wire/Wire.d \
./src/Wire/twi.d 


# Each subdirectory must supply rules for building sources it contributes
src/Wire/%.o: ../src/Wire/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\dev\eclipse_avr\ArduLibs\include\ov7670" -I"C:\dev\eclipse_avr\ArduLibs\src\EEPROM" -I"C:\dev\eclipse_avr\ArduLibs\src\HMC58X3" -I"C:\dev\eclipse_avr\ArduLibs\src\itg3200filv05" -I"C:\dev\eclipse_avr\ArduLibs\include\adxl345driver" -I"C:\dev\eclipse_avr\ArduLibs\include\FreeIMU" -I"C:\dev\eclipse_avr\ArduLibs\include\CmdProcessor" -I"C:\dev\eclipse_avr\ArduLibs\include" -I"C:\dev\eclipse_avr\ArduLibs\include\DigitalIO" -I"C:\dev\eclipse_avr\ArduLibs\include\FrequencyTimer2" -I"C:\dev\eclipse_avr\ArduLibs\include\LEDSensor" -I"C:\dev\eclipse_avr\ArduLibs\include\Servo" -I"C:\dev\eclipse_avr\ArduLibs\include\Wire" -I"C:\dev\eclipse_avr\ArduLibs\include\AFMotor" -I"C:\dev\eclipse_avr\ArduCore\include" -Wall -Os -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


