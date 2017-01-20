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
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

commonsrc/CommDef.o: E:/Projects/Arm/code/WalterCommon/src/CommDef.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

commonsrc/core.o: E:/Projects/Arm/code/WalterCommon/src/core.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


