################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/Src/ds1307.c \
../Bsp/Src/lcd.c 

OBJS += \
./Bsp/Src/ds1307.o \
./Bsp/Src/lcd.o 

C_DEPS += \
./Bsp/Src/ds1307.d \
./Bsp/Src/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/Src/%.o Bsp/Src/%.su Bsp/Src/%.cyclo: ../Bsp/Src/%.c Bsp/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/Swastika/OneDrive/Documents/Udemy Courses/Embedded-C/my_workspace/target/STM32F4xx_Driver/Bsp/Inc" -I"C:/Users/Swastika/OneDrive/Documents/Udemy Courses/Embedded-C/my_workspace/target/STM32F4xx_Driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bsp-2f-Src

clean-Bsp-2f-Src:
	-$(RM) ./Bsp/Src/ds1307.cyclo ./Bsp/Src/ds1307.d ./Bsp/Src/ds1307.o ./Bsp/Src/ds1307.su ./Bsp/Src/lcd.cyclo ./Bsp/Src/lcd.d ./Bsp/Src/lcd.o ./Bsp/Src/lcd.su

.PHONY: clean-Bsp-2f-Src

