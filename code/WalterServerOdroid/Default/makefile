################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := del

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include source/subdir.mk
-include source/RS232/subdir.mk
-include kinsrc/subdir.mk
-include commonsrc/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: server.exe

# Tool invocations
server.exe: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++ -L"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\lib\gcc\arm-linux-gnueabihf\4.8.2" -L"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\libc\usr\lib\arm-linux-gnueabihf" -L"D:\Programme\gcc-linaro-arm-linux-gnueabihf-4.8-2013.09_win32\arm-linux-gnueabihf\lib\arm-linux-gnueabi" -o"server.exe" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) server.exe
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
