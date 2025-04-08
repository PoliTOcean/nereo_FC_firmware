################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/navigation/cs_control_stabilize_mode.cpp \
../Core/Src/navigation/full_state_feedback_control.cpp 

C_SRCS += \
../Core/Src/navigation/ThrusterConfigurationMatrix.c \
../Core/Src/navigation/navigation.c \
../Core/Src/navigation/stabilize_mode.c 

C_DEPS += \
./Core/Src/navigation/ThrusterConfigurationMatrix.d \
./Core/Src/navigation/navigation.d \
./Core/Src/navigation/stabilize_mode.d 

OBJS += \
./Core/Src/navigation/ThrusterConfigurationMatrix.o \
./Core/Src/navigation/cs_control_stabilize_mode.o \
./Core/Src/navigation/full_state_feedback_control.o \
./Core/Src/navigation/navigation.o \
./Core/Src/navigation/stabilize_mode.o 

CPP_DEPS += \
./Core/Src/navigation/cs_control_stabilize_mode.d \
./Core/Src/navigation/full_state_feedback_control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/navigation/%.o Core/Src/navigation/%.su Core/Src/navigation/%.cyclo: ../Core/Src/navigation/%.c Core/Src/navigation/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/navigation/%.o Core/Src/navigation/%.su Core/Src/navigation/%.cyclo: ../Core/Src/navigation/%.cpp Core/Src/navigation/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DARM_MATH_CM4 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-navigation

clean-Core-2f-Src-2f-navigation:
	-$(RM) ./Core/Src/navigation/ThrusterConfigurationMatrix.cyclo ./Core/Src/navigation/ThrusterConfigurationMatrix.d ./Core/Src/navigation/ThrusterConfigurationMatrix.o ./Core/Src/navigation/ThrusterConfigurationMatrix.su ./Core/Src/navigation/cs_control_stabilize_mode.cyclo ./Core/Src/navigation/cs_control_stabilize_mode.d ./Core/Src/navigation/cs_control_stabilize_mode.o ./Core/Src/navigation/cs_control_stabilize_mode.su ./Core/Src/navigation/full_state_feedback_control.cyclo ./Core/Src/navigation/full_state_feedback_control.d ./Core/Src/navigation/full_state_feedback_control.o ./Core/Src/navigation/full_state_feedback_control.su ./Core/Src/navigation/navigation.cyclo ./Core/Src/navigation/navigation.d ./Core/Src/navigation/navigation.o ./Core/Src/navigation/navigation.su ./Core/Src/navigation/stabilize_mode.cyclo ./Core/Src/navigation/stabilize_mode.d ./Core/Src/navigation/stabilize_mode.o ./Core/Src/navigation/stabilize_mode.su

.PHONY: clean-Core-2f-Src-2f-navigation

