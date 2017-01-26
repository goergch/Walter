################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
E:/Projects/Arm/code/WalterServer/src/CmdDispatcher.cpp \
E:/Projects/Arm/code/WalterServer/src/CortexController.cpp \
E:/Projects/Arm/code/WalterServer/src/SerialPort.cpp \
E:/Projects/Arm/code/WalterServer/src/TrajectoryExecution.cpp \
E:/Projects/Arm/code/WalterServer/src/main.cpp 

C_SRCS += \
E:/Projects/Arm/code/WalterServer/src/mongoose.c 

OBJS += \
./source/CmdDispatcher.o \
./source/CortexController.o \
./source/SerialPort.o \
./source/TrajectoryExecution.o \
./source/main.o \
./source/mongoose.o 

CPP_DEPS += \
./source/CmdDispatcher.d \
./source/CortexController.d \
./source/SerialPort.d \
./source/TrajectoryExecution.d \
./source/main.d 

C_DEPS += \
./source/mongoose.d 


# Each subdirectory must supply rules for building sources it contributes
source/CmdDispatcher.o: E:/Projects/Arm/code/WalterServer/src/CmdDispatcher.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src\RS232" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/CortexController.o: E:/Projects/Arm/code/WalterServer/src/CortexController.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src\RS232" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/SerialPort.o: E:/Projects/Arm/code/WalterServer/src/SerialPort.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src\RS232" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/TrajectoryExecution.o: E:/Projects/Arm/code/WalterServer/src/TrajectoryExecution.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src\RS232" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/main.o: E:/Projects/Arm/code/WalterServer/src/main.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"E:\Projects\Arm\code\WalterServer\src\RS232" -I"E:\Projects\Arm\code\WalterServer\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\include\c++\4.8.3\arm-none-eabi" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

source/mongoose.o: E:/Projects/Arm/code/WalterServer/src/mongoose.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-linux-gnueabihf-gcc -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\arm-none-eabi\libc\usr\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include" -I"D:\Programme\gcc-linaro-6.2.1-2016.11-i686-mingw32_arm-linux-gnueabihf\lib\gcc\arm-none-eabi\4.8.3\include-fixed" -O0 -g3 -Wall -c -fmessage-length=0  -std=c99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


