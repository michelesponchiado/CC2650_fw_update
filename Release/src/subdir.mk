################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/sblAppEx.cpp \
../src/sbl_device.cpp \
../src/sbl_device_cc2650.cpp \
../src/sbllib.cpp 

OBJS += \
./src/sblAppEx.o \
./src/sbl_device.o \
./src/sbl_device_cc2650.o \
./src/sbllib.o 

CPP_DEPS += \
./src/sblAppEx.d \
./src/sbl_device.d \
./src/sbl_device_cc2650.d \
./src/sbllib.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


