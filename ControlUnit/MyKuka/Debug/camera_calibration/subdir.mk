################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../camera_calibration/calibrate.cpp 

OBJS += \
./camera_calibration/calibrate.o 

CPP_DEPS += \
./camera_calibration/calibrate.d 


# Each subdirectory must supply rules for building sources it contributes
camera_calibration/%.o: ../camera_calibration/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/matrix/include -I/usr/local/include/opencv4 -O0 -g3 -Wall -c -fmessage-length=0 -isystem /usr/local/include/matrix/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


