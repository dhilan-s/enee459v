################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/B-L4S5I-IOT01A/b_l4s5i_iot01a_bus.c 

OBJS += \
./Drivers/BSP/B-L4S5I-IOT01A/b_l4s5i_iot01a_bus.o 

C_DEPS += \
./Drivers/BSP/B-L4S5I-IOT01A/b_l4s5i_iot01a_bus.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B-L4S5I-IOT01A/%.o Drivers/BSP/B-L4S5I-IOT01A/%.su: ../Drivers/BSP/B-L4S5I-IOT01A/%.c Drivers/BSP/B-L4S5I-IOT01A/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../BlueNRG-MS/Target -I../Drivers/BSP/B-L4S5I-IOT01A -I../Middlewares/ST/BlueNRG-MS/utils -I../Middlewares/ST/BlueNRG-MS/includes -I../Middlewares/ST/BlueNRG-MS/hci/hci_tl_patterns/Basic -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01A

clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01A:
	-$(RM) ./Drivers/BSP/B-L4S5I-IOT01A/b_l4s5i_iot01a_bus.d ./Drivers/BSP/B-L4S5I-IOT01A/b_l4s5i_iot01a_bus.o ./Drivers/BSP/B-L4S5I-IOT01A/b_l4s5i_iot01a_bus.su

.PHONY: clean-Drivers-2f-BSP-2f-B-2d-L4S5I-2d-IOT01A

