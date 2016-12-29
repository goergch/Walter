################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BezierCurve.cpp \
../src/DenavitHardenbergParam.cpp \
../src/Kinematics.cpp \
../src/SpeedProfile.cpp \
../src/Trajectory.cpp \
../src/TrajectoryPlayer.cpp \
../src/Util.cpp \
../src/logger.cpp \
../src/spatial.cpp 

OBJS += \
./src/BezierCurve.o \
./src/DenavitHardenbergParam.o \
./src/Kinematics.o \
./src/SpeedProfile.o \
./src/Trajectory.o \
./src/TrajectoryPlayer.o \
./src/Util.o \
./src/logger.o \
./src/spatial.o 

CPP_DEPS += \
./src/BezierCurve.d \
./src/DenavitHardenbergParam.d \
./src/Kinematics.d \
./src/SpeedProfile.d \
./src/Trajectory.d \
./src/TrajectoryPlayer.d \
./src/Util.d \
./src/logger.d \
./src/spatial.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -O0 -g3 -Wall -c -fmessage-length=0  -std=c++11 -U__STRICT_ANSI__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


