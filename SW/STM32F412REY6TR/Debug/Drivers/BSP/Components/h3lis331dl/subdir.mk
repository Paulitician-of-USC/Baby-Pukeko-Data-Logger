################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/h3lis331dl/h3lis331dl.c \
../Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.c 

OBJS += \
./Drivers/BSP/Components/h3lis331dl/h3lis331dl.o \
./Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.o 

C_DEPS += \
./Drivers/BSP/Components/h3lis331dl/h3lis331dl.d \
./Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/h3lis331dl/%.o Drivers/BSP/Components/h3lis331dl/%.su Drivers/BSP/Components/h3lis331dl/%.cyclo: ../Drivers/BSP/Components/h3lis331dl/%.c Drivers/BSP/Components/h3lis331dl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F412Rx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso32x -I../Drivers/BSP/Components/h3lis331dl -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-h3lis331dl

clean-Drivers-2f-BSP-2f-Components-2f-h3lis331dl:
	-$(RM) ./Drivers/BSP/Components/h3lis331dl/h3lis331dl.cyclo ./Drivers/BSP/Components/h3lis331dl/h3lis331dl.d ./Drivers/BSP/Components/h3lis331dl/h3lis331dl.o ./Drivers/BSP/Components/h3lis331dl/h3lis331dl.su ./Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.cyclo ./Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.d ./Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.o ./Drivers/BSP/Components/h3lis331dl/h3lis331dl_reg.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-h3lis331dl

