################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../include/avr-libc/malloc.c \
../include/avr-libc/realloc.c 

OBJS += \
./include/avr-libc/malloc.o \
./include/avr-libc/realloc.o 

C_DEPS += \
./include/avr-libc/malloc.d \
./include/avr-libc/realloc.d 


# Each subdirectory must supply rules for building sources it contributes
include/avr-libc/%.o: ../include/avr-libc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -I"C:\dev\eclipse_avr\ArduCore\include" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


