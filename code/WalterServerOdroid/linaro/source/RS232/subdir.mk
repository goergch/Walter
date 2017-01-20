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
	arm-linux-gnueabihf-gcc -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\libc\usr\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include-fixed" -O0 -g3 -Wall -c -fmessage-length=0  -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/RS232/rs232-win.o: E:/Projects/Arm/code/WalterServer/src/RS232/rs232-win.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-linux-gnueabihf-gcc -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\libc\usr\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include-fixed" -O0 -g3 -Wall -c -fmessage-length=0  -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


