################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/control.c \
../Src/main.c \
../Src/setup.c \
../Src/stm32f0xx_hal_msp.c \
../Src/stm32f0xx_it.c \
../Src/system_stm32f0xx.c 

OBJS += \
./Src/control.o \
./Src/main.o \
./Src/setup.o \
./Src/stm32f0xx_hal_msp.o \
./Src/stm32f0xx_it.o \
./Src/system_stm32f0xx.o 

C_DEPS += \
./Src/control.d \
./Src/main.d \
./Src/setup.d \
./Src/stm32f0xx_hal_msp.d \
./Src/stm32f0xx_it.d \
./Src/system_stm32f0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -mfloat-abi=soft -D__weak=__attribute__((weak)) -D__packed=__attribute__((__packed__)) -DUSE_HAL_DRIVER -DSTM32F030x6 -I"D:/Dropbox/eagle/linefollower/GetInLine/Inc" -I"D:/Dropbox/eagle/linefollower/GetInLine/Drivers/STM32F0xx_HAL_Driver/Inc" -I"D:/Dropbox/eagle/linefollower/GetInLine/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy" -I"D:/Dropbox/eagle/linefollower/GetInLine/Drivers/CMSIS/Device/ST/STM32F0xx/Include" -I"D:/Dropbox/eagle/linefollower/GetInLine/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


