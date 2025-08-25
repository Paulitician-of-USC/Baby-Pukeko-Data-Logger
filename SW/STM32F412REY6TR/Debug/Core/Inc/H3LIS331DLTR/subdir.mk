################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/H3LIS331DLTR/h3lis331dl_reg.c 

OBJS += \
./Core/Inc/H3LIS331DLTR/h3lis331dl_reg.o 

C_DEPS += \
./Core/Inc/H3LIS331DLTR/h3lis331dl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/H3LIS331DLTR/%.o Core/Inc/H3LIS331DLTR/%.su Core/Inc/H3LIS331DLTR/%.cyclo: ../Core/Inc/H3LIS331DLTR/%.c Core/Inc/H3LIS331DLTR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-H3LIS331DLTR

clean-Core-2f-Inc-2f-H3LIS331DLTR:
	-$(RM) ./Core/Inc/H3LIS331DLTR/h3lis331dl_reg.cyclo ./Core/Inc/H3LIS331DLTR/h3lis331dl_reg.d ./Core/Inc/H3LIS331DLTR/h3lis331dl_reg.o ./Core/Inc/H3LIS331DLTR/h3lis331dl_reg.su

.PHONY: clean-Core-2f-Inc-2f-H3LIS331DLTR

