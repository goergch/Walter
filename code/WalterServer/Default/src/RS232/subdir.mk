################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/RS232/rs232-linux.c \
../src/RS232/rs232-win.c 

OBJS += \
./src/RS232/rs232-linux.o \
./src/RS232/rs232-win.o 

C_DEPS += \
./src/RS232/rs232-linux.d \
./src/RS232/rs232-win.d 


# Each subdirectory must supply rules for building sources it contributes
src/RS232/%.o: ../src/RS232/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"E:\Projects\Arm\code\WalterServer\src\RS232" -O0 -g3 -Wall -c -fmessage-length=0  -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


