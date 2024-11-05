################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/navigation/mixing_matrix.c \
../Core/Src/navigation/navigation.c \
../Core/Src/navigation/pid.c \
../Core/Src/navigation/stabilize_mode.c 

OBJS += \
./Core/Src/navigation/mixing_matrix.o \
./Core/Src/navigation/navigation.o \
./Core/Src/navigation/pid.o \
./Core/Src/navigation/stabilize_mode.o 

C_DEPS += \
./Core/Src/navigation/mixing_matrix.d \
./Core/Src/navigation/navigation.d \
./Core/Src/navigation/pid.d \
./Core/Src/navigation/stabilize_mode.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/navigation/%.o Core/Src/navigation/%.su Core/Src/navigation/%.cyclo: ../Core/Src/navigation/%.c Core/Src/navigation/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DARM_MATH_CM4 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F469xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/ARM/DSP/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-navigation

clean-Core-2f-Src-2f-navigation:
	-$(RM) ./Core/Src/navigation/mixing_matrix.cyclo ./Core/Src/navigation/mixing_matrix.d ./Core/Src/navigation/mixing_matrix.o ./Core/Src/navigation/mixing_matrix.su ./Core/Src/navigation/navigation.cyclo ./Core/Src/navigation/navigation.d ./Core/Src/navigation/navigation.o ./Core/Src/navigation/navigation.su ./Core/Src/navigation/pid.cyclo ./Core/Src/navigation/pid.d ./Core/Src/navigation/pid.o ./Core/Src/navigation/pid.su ./Core/Src/navigation/stabilize_mode.cyclo ./Core/Src/navigation/stabilize_mode.d ./Core/Src/navigation/stabilize_mode.o ./Core/Src/navigation/stabilize_mode.su

.PHONY: clean-Core-2f-Src-2f-navigation

