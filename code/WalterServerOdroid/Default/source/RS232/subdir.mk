################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Projects/Arm/code/WalterServer/src/RS232/rs232-linux.c \
E:/Projects/Arm/code/WalterServer/src/RS232/rs232-win.c 

OBJS += \
./source/RS232/rs232-linux.o \
./source/RS232/rs232-win.o 

C_DEPS += \
./source/RS232/rs232-linux.d \
./source/RS232/rs232-win.d 


# Each subdirectory must supply rules for building sources it contributes
source/RS232/rs232-linux.o: E:/Projects/Arm/code/WalterServer/src/RS232/rs232-linux.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include-fixed" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -O2 -g -Wall -c -fmessage-length=0 -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/RS232/rs232-win.o: E:/Projects/Arm/code/WalterServer/src/RS232/rs232-win.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include-fixed" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -O2 -g -Wall -c -fmessage-length=0 -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


