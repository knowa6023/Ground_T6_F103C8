################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/AD7792.c \
../User/SPI_communication.c \
../User/application.c \
../User/mcu_communication.c \
../User/timer.c 

OBJS += \
./User/AD7792.o \
./User/SPI_communication.o \
./User/application.o \
./User/mcu_communication.o \
./User/timer.o 

C_DEPS += \
./User/AD7792.d \
./User/SPI_communication.d \
./User/application.d \
./User/mcu_communication.d \
./User/timer.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/FM/STM32CubeIDE/workspace_1.7.0/Ground_T6_F103C8/User" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

