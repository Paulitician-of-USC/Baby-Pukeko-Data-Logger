################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/LSM6DSO32TR/lsm6dso32_reg.c 

OBJS += \
./Core/Inc/LSM6DSO32TR/lsm6dso32_reg.o 

C_DEPS += \
./Core/Inc/LSM6DSO32TR/lsm6dso32_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/LSM6DSO32TR/%.o Core/Inc/LSM6DSO32TR/%.su Core/Inc/LSM6DSO32TR/%.cyclo: ../Core/Inc/LSM6DSO32TR/%.c Core/Inc/LSM6DSO32TR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-LSM6DSO32TR

clean-Core-2f-Inc-2f-LSM6DSO32TR:
	-$(RM) ./Core/Inc/LSM6DSO32TR/lsm6dso32_reg.cyclo ./Core/Inc/LSM6DSO32TR/lsm6dso32_reg.d ./Core/Inc/LSM6DSO32TR/lsm6dso32_reg.o ./Core/Inc/LSM6DSO32TR/lsm6dso32_reg.su

.PHONY: clean-Core-2f-Inc-2f-LSM6DSO32TR

