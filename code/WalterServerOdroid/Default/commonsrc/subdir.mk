################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
E:/Projects/Arm/code/WalterCommon/src/ActuatorProperty.cpp \
E:/Projects/Arm/code/WalterCommon/src/CommDef.cpp \
E:/Projects/Arm/code/WalterCommon/src/core.cpp 

OBJS += \
./commonsrc/ActuatorProperty.o \
./commonsrc/CommDef.o \
./commonsrc/core.o 

CPP_DEPS += \
./commonsrc/ActuatorProperty.d \
./commonsrc/CommDef.d \
./commonsrc/core.d 


# Each subdirectory must supply rules for building sources it contributes
commonsrc/ActuatorProperty.o: E:/Projects/Arm/code/WalterCommon/src/ActuatorProperty.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

commonsrc/CommDef.o: E:/Projects/Arm/code/WalterCommon/src/CommDef.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

commonsrc/core.o: E:/Projects/Arm/code/WalterCommon/src/core.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


