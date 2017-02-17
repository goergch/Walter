################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BotDrawer.cpp \
../src/BotView.cpp \
../src/BotWindowCtrl.cpp \
../src/CADSTLObject.cpp \
../src/ExecutionInvoker.cpp \
../src/Hanoi.cpp \
../src/TrajectorySimulation.cpp \
../src/TrajectoryView.cpp \
../src/main.cpp 

OBJS += \
./src/BotDrawer.o \
./src/BotView.o \
./src/BotWindowCtrl.o \
./src/CADSTLObject.o \
./src/ExecutionInvoker.o \
./src/Hanoi.o \
./src/TrajectorySimulation.o \
./src/TrajectoryView.o \
./src/main.o 

CPP_DEPS += \
./src/BotDrawer.d \
./src/BotView.d \
./src/BotWindowCtrl.d \
./src/CADSTLObject.d \
./src/ExecutionInvoker.d \
./src/Hanoi.d \
./src/TrajectorySimulation.d \
./src/TrajectoryView.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -DFREEGLUT_STATIC -DPOCO_STATIC -I"E:\Projects\Arm\code\WalterPlanner\src" -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\GLUI\src\include" -I"E:\Projects\Arm\code\poco-1.7.6\Net\include" -I"E:\Projects\Arm\code\poco-1.7.6\Foundation\include" -I"E:\Projects\Arm\code\freeglut\include" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


