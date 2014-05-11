################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/HC_SR04.cpp \
../src/SoftI2CUtils.cpp \
../src/arduinoMain.cpp \
../src/fastRandom.cpp \
../src/main.cpp \
../src/rambamIMU.cpp \
../src/serialCommands.cpp \
../src/softi2c.cpp 

OBJS += \
./src/HC_SR04.o \
./src/SoftI2CUtils.o \
./src/arduinoMain.o \
./src/fastRandom.o \
./src/main.o \
./src/rambamIMU.o \
./src/serialCommands.o \
./src/softi2c.o 

CPP_DEPS += \
./src/HC_SR04.d \
./src/SoftI2CUtils.d \
./src/arduinoMain.d \
./src/fastRandom.d \
./src/main.d \
./src/rambamIMU.d \
./src/serialCommands.d \
./src/softi2c.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: AVR C++ Compiler'
	avr-g++ -I"C:\dev\eclipse_avr\ArduCore\include" -I"C:\dev\eclipse_avr\ArduLibs\src\HMC58X3" -I"C:\dev\eclipse_avr\ArduLibs\src\itg3200filv05" -I"C:\dev\eclipse_avr\ArduLibs\include\FrequencyTimer2" -I"C:\dev\eclipse_avr\ArduLibs\src\EEPROM" -I"C:\dev\eclipse_avr\ArduLibs\include\FreeIMU" -I"C:\dev\eclipse_avr\ArduLibs\include\adxl345driver" -I"C:\dev\eclipse_avr\ArduLibs\src\HMC58X3" -I"C:\dev\eclipse_avr\ArduLibs\src\itg3200filv05" -I"C:\dev\eclipse_avr\ArduLibs\include\Wire" -I"C:\dev\eclipse_avr\ArduLibs\include" -I"C:\dev\eclipse_avr\dead_reckoning\include" -D__AVR_ATmega328P__ -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -fno-exceptions -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


