################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/device.cpp \
../src/main.cpp \
../src/pack.cpp \
../src/rx_stream.cpp 

CC_SRCS += \
../src/rfnm_sink_impl.cc 

CC_DEPS += \
./src/rfnm_sink_impl.d 

OBJS += \
./src/device.o \
./src/main.o \
./src/pack.o \
./src/rfnm_sink_impl.o \
./src/rx_stream.o 

CPP_DEPS += \
./src/device.d \
./src/main.d \
./src/pack.d \
./src/rx_stream.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C++ Compiler'
	aarch64-none-linux-gnu-g++ -mcpu=generic+simd -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -std=gnu++17 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cc src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU Arm Cross C++ Compiler'
	aarch64-none-linux-gnu-g++ -mcpu=generic+simd -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -g3 -std=gnu++17 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


