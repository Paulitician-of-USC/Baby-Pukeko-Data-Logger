################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/MS560702BA03-50/MS5607.c 

OBJS += \
./Core/Inc/MS560702BA03-50/MS5607.o 

C_DEPS += \
./Core/Inc/MS560702BA03-50/MS5607.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/MS560702BA03-50/%.o Core/Inc/MS560702BA03-50/%.su Core/Inc/MS560702BA03-50/%.cyclo: ../Core/Inc/MS560702BA03-50/%.c Core/Inc/MS560702BA03-50/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso32x -I../Drivers/BSP/Components/h3lis331dl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-MS560702BA03-2d-50

clean-Core-2f-Inc-2f-MS560702BA03-2d-50:
	-$(RM) ./Core/Inc/MS560702BA03-50/MS5607.cyclo ./Core/Inc/MS560702BA03-50/MS5607.d ./Core/Inc/MS560702BA03-50/MS5607.o ./Core/Inc/MS560702BA03-50/MS5607.su

.PHONY: clean-Core-2f-Inc-2f-MS560702BA03-2d-50

