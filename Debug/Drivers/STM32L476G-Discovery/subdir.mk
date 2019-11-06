################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery.c \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery_compass.c \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.c \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.c \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery_idd.c \
../Drivers/STM32L476G-Discovery/stm32l476g_discovery_qspi.c 

OBJS += \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery.o \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_compass.o \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.o \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.o \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_idd.o \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_qspi.o 

C_DEPS += \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery.d \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_compass.d \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.d \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.d \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_idd.d \
./Drivers/STM32L476G-Discovery/stm32l476g_discovery_qspi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32L476G-Discovery/stm32l476g_discovery.o: ../Drivers/STM32L476G-Discovery/stm32l476g_discovery.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32L476G-Discovery -I../Drivers/CMSIS/Include -I../Drivers/Components -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32L476G-Discovery/stm32l476g_discovery.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32L476G-Discovery/stm32l476g_discovery_compass.o: ../Drivers/STM32L476G-Discovery/stm32l476g_discovery_compass.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32L476G-Discovery -I../Drivers/CMSIS/Include -I../Drivers/Components -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32L476G-Discovery/stm32l476g_discovery_compass.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.o: ../Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32L476G-Discovery -I../Drivers/CMSIS/Include -I../Drivers/Components -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.o: ../Drivers/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32L476G-Discovery -I../Drivers/CMSIS/Include -I../Drivers/Components -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32L476G-Discovery/stm32l476g_discovery_idd.o: ../Drivers/STM32L476G-Discovery/stm32l476g_discovery_idd.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32L476G-Discovery -I../Drivers/CMSIS/Include -I../Drivers/Components -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32L476G-Discovery/stm32l476g_discovery_idd.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/STM32L476G-Discovery/stm32l476g_discovery_qspi.o: ../Drivers/STM32L476G-Discovery/stm32l476g_discovery_qspi.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L476xx -DDEBUG -c -I../USB_HOST/App -I../Drivers/STM32L476G-Discovery -I../Drivers/CMSIS/Include -I../Drivers/Components -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../USB_HOST/Target -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/STM32L476G-Discovery/stm32l476g_discovery_qspi.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

