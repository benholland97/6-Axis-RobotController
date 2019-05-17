################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/DHParams.cpp \
../src/ImageProcessor.cpp \
../src/Kinematics.cpp \
../src/MovementController.cpp \
../src/Positions.cpp \
../src/SerialPort.cpp \
../src/Utils.cpp \
../src/main.cpp 

OBJS += \
./src/DHParams.o \
./src/ImageProcessor.o \
./src/Kinematics.o \
./src/MovementController.o \
./src/Positions.o \
./src/SerialPort.o \
./src/Utils.o \
./src/main.o 

CPP_DEPS += \
./src/DHParams.d \
./src/ImageProcessor.d \
./src/Kinematics.d \
./src/MovementController.d \
./src/Positions.d \
./src/SerialPort.d \
./src/Utils.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/matrix/include -I/usr/local/include/opencv4 -O0 -g3 -Wall -c -fmessage-length=0 -isystem /usr/local/include/matrix/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


