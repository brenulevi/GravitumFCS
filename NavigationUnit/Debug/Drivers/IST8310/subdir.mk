################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IST8310/IST8310.c 

OBJS += \
./Drivers/IST8310/IST8310.o 

C_DEPS += \
./Drivers/IST8310/IST8310.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IST8310/%.o Drivers/IST8310/%.su Drivers/IST8310/%.cyclo: ../Drivers/IST8310/%.c Drivers/IST8310/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/TMP100 -I../Drivers/BMP390 -I../Drivers/IST8310 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-IST8310

clean-Drivers-2f-IST8310:
	-$(RM) ./Drivers/IST8310/IST8310.cyclo ./Drivers/IST8310/IST8310.d ./Drivers/IST8310/IST8310.o ./Drivers/IST8310/IST8310.su

.PHONY: clean-Drivers-2f-IST8310

