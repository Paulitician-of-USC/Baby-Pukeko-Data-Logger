################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.c 

OBJS += \
./Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.o 

C_DEPS += \
./Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/MT29F1G01ABAFDWB/%.o Core/Inc/MT29F1G01ABAFDWB/%.su Core/Inc/MT29F1G01ABAFDWB/%.cyclo: ../Core/Inc/MT29F1G01ABAFDWB/%.c Core/Inc/MT29F1G01ABAFDWB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso32x -I../Drivers/BSP/Components/h3lis331dl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-MT29F1G01ABAFDWB

clean-Core-2f-Inc-2f-MT29F1G01ABAFDWB:
	-$(RM) ./Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.cyclo ./Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.d ./Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.o ./Core/Inc/MT29F1G01ABAFDWB/drv_spi_flash.su

.PHONY: clean-Core-2f-Inc-2f-MT29F1G01ABAFDWB

