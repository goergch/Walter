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
../src/TrajectorySimulation.cpp \
../src/TrajectoryView.cpp \
../src/main.cpp 

OBJS += \
./src/BotDrawer.o \
./src/BotView.o \
./src/BotWindowCtrl.o \
./src/CADSTLObject.o \
./src/ExecutionInvoker.o \
./src/TrajectorySimulation.o \
./src/TrajectoryView.o \
./src/main.o 

CPP_DEPS += \
./src/BotDrawer.d \
./src/BotView.d \
./src/BotWindowCtrl.d \
./src/CADSTLObject.d \
./src/ExecutionInvoker.d \
./src/TrajectorySimulation.d \
./src/TrajectoryView.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -DPOCO_STATIC -I"E:\Projects\Arm\code\workspace/../poco-1.7.6/Net/include" -I"E:\Projects\Arm\code\workspace/../poco-1.7.6/Foundation/include" -I"E:\Projects\Arm\code\workspace/../WalterCommon/src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterPlanner\src" -I"E:\Projects\Arm\code\GLUI\src\include" -I"E:\Projects\Arm\code\workspace/freeglut/include" -I"D:\Programme\mingw-w64\i686-5.1.0-posix-dwarf-rt_v4-rev0\mingw32\include" -O0 -g3 -Wall -c -fmessage-length=0  -std=c++11 -DFREEGLUT_STATIC -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/ExecutionInvoker.o: ../src/ExecutionInvoker.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -DPOCO_STATIC -I"E:\Projects\Arm\code\workspace/../poco-1.7.6/Net/include" -I"E:\Projects\Arm\code\workspace/../poco-1.7.6/Foundation/include" -I"E:\Projects\Arm\code\workspace/../WalterCommon/src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"E:\Projects\Arm\code\WalterPlanner\src" -I"E:\Projects\Arm\code\GLUI\src\include" -I"E:\Projects\Arm\code\workspace/freeglut/include" -I"D:\Programme\mingw-w64\i686-5.1.0-posix-dwarf-rt_v4-rev0\mingw32\include" -O0 -g3 -Wall -c -fmessage-length=0  -DFREEGLUT_STATIC -std=c++11 -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/ExecutionInvoker.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


