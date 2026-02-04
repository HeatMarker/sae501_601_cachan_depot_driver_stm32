################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Bosch/BMI088/bmi088_anymotiona.c \
../Drivers/Bosch/BMI088/bmi088_mma.c \
../Drivers/Bosch/BMI088/bmi08a.c \
../Drivers/Bosch/BMI088/bmi08g.c \
../Drivers/Bosch/BMI088/bmi08xa.c 

OBJS += \
./Drivers/Bosch/BMI088/bmi088_anymotiona.o \
./Drivers/Bosch/BMI088/bmi088_mma.o \
./Drivers/Bosch/BMI088/bmi08a.o \
./Drivers/Bosch/BMI088/bmi08g.o \
./Drivers/Bosch/BMI088/bmi08xa.o 

C_DEPS += \
./Drivers/Bosch/BMI088/bmi088_anymotiona.d \
./Drivers/Bosch/BMI088/bmi088_mma.d \
./Drivers/Bosch/BMI088/bmi08a.d \
./Drivers/Bosch/BMI088/bmi08g.d \
./Drivers/Bosch/BMI088/bmi08xa.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Bosch/BMI088/%.o Drivers/Bosch/BMI088/%.su Drivers/Bosch/BMI088/%.cyclo: ../Drivers/Bosch/BMI088/%.c Drivers/Bosch/BMI088/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G0B1xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/Bosch/BMI088 -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Bosch-2f-BMI088

clean-Drivers-2f-Bosch-2f-BMI088:
	-$(RM) ./Drivers/Bosch/BMI088/bmi088_anymotiona.cyclo ./Drivers/Bosch/BMI088/bmi088_anymotiona.d ./Drivers/Bosch/BMI088/bmi088_anymotiona.o ./Drivers/Bosch/BMI088/bmi088_anymotiona.su ./Drivers/Bosch/BMI088/bmi088_mma.cyclo ./Drivers/Bosch/BMI088/bmi088_mma.d ./Drivers/Bosch/BMI088/bmi088_mma.o ./Drivers/Bosch/BMI088/bmi088_mma.su ./Drivers/Bosch/BMI088/bmi08a.cyclo ./Drivers/Bosch/BMI088/bmi08a.d ./Drivers/Bosch/BMI088/bmi08a.o ./Drivers/Bosch/BMI088/bmi08a.su ./Drivers/Bosch/BMI088/bmi08g.cyclo ./Drivers/Bosch/BMI088/bmi08g.d ./Drivers/Bosch/BMI088/bmi08g.o ./Drivers/Bosch/BMI088/bmi08g.su ./Drivers/Bosch/BMI088/bmi08xa.cyclo ./Drivers/Bosch/BMI088/bmi08xa.d ./Drivers/Bosch/BMI088/bmi08xa.o ./Drivers/Bosch/BMI088/bmi08xa.su

.PHONY: clean-Drivers-2f-Bosch-2f-BMI088

