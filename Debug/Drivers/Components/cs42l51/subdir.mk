################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/cs42l51/cs42l51.c 

OBJS += \
./Drivers/Components/cs42l51/cs42l51.o 

C_DEPS += \
./Drivers/Components/cs42l51/cs42l51.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/cs42l51/cs42l51.o: ../Drivers/Components/cs42l51/cs42l51.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32L476G-Discovery -I../Drivers/Components -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Components/cs42l51/cs42l51.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

