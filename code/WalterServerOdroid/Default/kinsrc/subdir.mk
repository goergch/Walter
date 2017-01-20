################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
E:/Projects/Arm/code/WalterKinematics/src/BezierCurve.cpp \
E:/Projects/Arm/code/WalterKinematics/src/DenavitHardenbergParam.cpp \
E:/Projects/Arm/code/WalterKinematics/src/Kinematics.cpp \
E:/Projects/Arm/code/WalterKinematics/src/SpeedProfile.cpp \
E:/Projects/Arm/code/WalterKinematics/src/Trajectory.cpp \
E:/Projects/Arm/code/WalterKinematics/src/TrajectoryPlayer.cpp \
E:/Projects/Arm/code/WalterKinematics/src/Util.cpp \
E:/Projects/Arm/code/WalterKinematics/src/logger.cpp \
E:/Projects/Arm/code/WalterKinematics/src/spatial.cpp 

OBJS += \
./kinsrc/BezierCurve.o \
./kinsrc/DenavitHardenbergParam.o \
./kinsrc/Kinematics.o \
./kinsrc/SpeedProfile.o \
./kinsrc/Trajectory.o \
./kinsrc/TrajectoryPlayer.o \
./kinsrc/Util.o \
./kinsrc/logger.o \
./kinsrc/spatial.o 

CPP_DEPS += \
./kinsrc/BezierCurve.d \
./kinsrc/DenavitHardenbergParam.d \
./kinsrc/Kinematics.d \
./kinsrc/SpeedProfile.d \
./kinsrc/Trajectory.d \
./kinsrc/TrajectoryPlayer.d \
./kinsrc/Util.d \
./kinsrc/logger.d \
./kinsrc/spatial.d 


# Each subdirectory must supply rules for building sources it contributes
kinsrc/BezierCurve.o: E:/Projects/Arm/code/WalterKinematics/src/BezierCurve.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/DenavitHardenbergParam.o: E:/Projects/Arm/code/WalterKinematics/src/DenavitHardenbergParam.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/Kinematics.o: E:/Projects/Arm/code/WalterKinematics/src/Kinematics.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/SpeedProfile.o: E:/Projects/Arm/code/WalterKinematics/src/SpeedProfile.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/Trajectory.o: E:/Projects/Arm/code/WalterKinematics/src/Trajectory.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/TrajectoryPlayer.o: E:/Projects/Arm/code/WalterKinematics/src/TrajectoryPlayer.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/Util.o: E:/Projects/Arm/code/WalterKinematics/src/Util.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/logger.o: E:/Projects/Arm/code/WalterKinematics/src/logger.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

kinsrc/spatial.o: E:/Projects/Arm/code/WalterKinematics/src/spatial.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2\include" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\arm-linux-gnueabihf\arm-linux-gnueabi" -I"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\include\c++\4.8.2\tr1" -O0 -g3 -Wall -c -fmessage-length=0 -std=c++11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


