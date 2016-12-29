################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
E:/Projects/Arm/code/WalterCommon/src/ActuatorProperty.cpp \
E:/Projects/Arm/code/WalterCommon/src/CommDef.cpp \
E:/Projects/Arm/code/WalterCommon/src/core.cpp 

OBJS += \
./src/WalterCommon/ActuatorProperty.o \
./src/WalterCommon/CommDef.o \
./src/WalterCommon/core.o 

CPP_DEPS += \
./src/WalterCommon/ActuatorProperty.d \
./src/WalterCommon/CommDef.d \
./src/WalterCommon/core.d 


# Each subdirectory must supply rules for building sources it contributes
src/WalterCommon/ActuatorProperty.o: E:/Projects/Arm/code/WalterCommon/src/ActuatorProperty.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -O0 -g3 -Wall -c -fmessage-length=0  -std=c++11 -U__STRICT_ANSI__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/WalterCommon/CommDef.o: E:/Projects/Arm/code/WalterCommon/src/CommDef.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -O0 -g3 -Wall -c -fmessage-length=0  -std=c++11 -U__STRICT_ANSI__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/WalterCommon/core.o: E:/Projects/Arm/code/WalterCommon/src/core.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"E:\Projects\Arm\code\WalterCommon\src" -I"E:\Projects\Arm\code\WalterKinematics\src" -O0 -g3 -Wall -c -fmessage-length=0  -std=c++11 -U__STRICT_ANSI__ -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


