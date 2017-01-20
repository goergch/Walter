################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/CmdDispatcher.cpp \
../src/CortexController.cpp \
../src/SerialPort.cpp \
../src/TrajectoryExecution.cpp \
../src/main.cpp 

C_SRCS += \
../src/mongoose.c 

OBJS += \
./src/CmdDispatcher.o \
./src/CortexController.o \
./src/SerialPort.o \
./src/TrajectoryExecution.o \
./src/main.o \
./src/mongoose.o 

CPP_DEPS += \
./src/CmdDispatcher.d \
./src/CortexController.d \
./src/SerialPort.d \
./src/TrajectoryExecution.d \
./src/main.d 

C_DEPS += \
./src/mongoose.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\workspace/../WalterCommon/src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


