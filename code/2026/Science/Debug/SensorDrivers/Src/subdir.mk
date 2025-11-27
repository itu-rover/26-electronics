################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../SensorDrivers/Src/BMP180.c \
../SensorDrivers/Src/DTH11.c \
../SensorDrivers/Src/MICS-4514.c \
../SensorDrivers/Src/MQ135.c \
../SensorDrivers/Src/MQ4.c 

OBJS += \
./SensorDrivers/Src/BMP180.o \
./SensorDrivers/Src/DTH11.o \
./SensorDrivers/Src/MICS-4514.o \
./SensorDrivers/Src/MQ135.o \
./SensorDrivers/Src/MQ4.o 

C_DEPS += \
./SensorDrivers/Src/BMP180.d \
./SensorDrivers/Src/DTH11.d \
./SensorDrivers/Src/MICS-4514.d \
./SensorDrivers/Src/MQ135.d \
./SensorDrivers/Src/MQ4.d 


# Each subdirectory must supply rules for building sources it contributes
SensorDrivers/Src/%.o SensorDrivers/Src/%.su SensorDrivers/Src/%.cyclo: ../SensorDrivers/Src/%.c SensorDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/ALPEREN/Documents/STM/Science/SensorDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-SensorDrivers-2f-Src

clean-SensorDrivers-2f-Src:
	-$(RM) ./SensorDrivers/Src/BMP180.cyclo ./SensorDrivers/Src/BMP180.d ./SensorDrivers/Src/BMP180.o ./SensorDrivers/Src/BMP180.su ./SensorDrivers/Src/DTH11.cyclo ./SensorDrivers/Src/DTH11.d ./SensorDrivers/Src/DTH11.o ./SensorDrivers/Src/DTH11.su ./SensorDrivers/Src/MICS-4514.cyclo ./SensorDrivers/Src/MICS-4514.d ./SensorDrivers/Src/MICS-4514.o ./SensorDrivers/Src/MICS-4514.su ./SensorDrivers/Src/MQ135.cyclo ./SensorDrivers/Src/MQ135.d ./SensorDrivers/Src/MQ135.o ./SensorDrivers/Src/MQ135.su ./SensorDrivers/Src/MQ4.cyclo ./SensorDrivers/Src/MQ4.d ./SensorDrivers/Src/MQ4.o ./SensorDrivers/Src/MQ4.su

.PHONY: clean-SensorDrivers-2f-Src

