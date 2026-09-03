################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/board_battery.c \
../Core/Src/board_relays.c \
../Core/Src/cart_param.c \
../Core/Src/cart_persist.c \
../Core/Src/fw_image.c \
../Core/Src/brake_can.c \
../Core/Src/dla_cart_app.c \
../Core/Src/foc_can.c \
../Core/Src/imu_lsm6dsv.c \
../Core/Src/main.c \
../Core/Src/motor_control.c \
../Core/Src/rc_input.c \
../Core/Src/screen.c \
../Core/Src/selftest.c \
../Core/Src/servo_can.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim_capture.c \
../Core/Src/train_uart.c \
../Core/Src/wheel_speed.c 

OBJS += \
./Core/Src/board_battery.o \
./Core/Src/board_relays.o \
./Core/Src/cart_param.o \
./Core/Src/cart_persist.o \
./Core/Src/fw_image.o \
./Core/Src/brake_can.o \
./Core/Src/dla_cart_app.o \
./Core/Src/foc_can.o \
./Core/Src/imu_lsm6dsv.o \
./Core/Src/main.o \
./Core/Src/motor_control.o \
./Core/Src/rc_input.o \
./Core/Src/screen.o \
./Core/Src/selftest.o \
./Core/Src/servo_can.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim_capture.o \
./Core/Src/train_uart.o \
./Core/Src/wheel_speed.o 

C_DEPS += \
./Core/Src/board_battery.d \
./Core/Src/board_relays.d \
./Core/Src/cart_param.d \
./Core/Src/cart_persist.d \
./Core/Src/fw_image.d \
./Core/Src/brake_can.d \
./Core/Src/dla_cart_app.d \
./Core/Src/foc_can.d \
./Core/Src/imu_lsm6dsv.d \
./Core/Src/main.d \
./Core/Src/motor_control.d \
./Core/Src/rc_input.d \
./Core/Src/screen.d \
./Core/Src/selftest.d \
./Core/Src/servo_can.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim_capture.d \
./Core/Src/train_uart.d \
./Core/Src/wheel_speed.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/board_battery.cyclo ./Core/Src/board_battery.d ./Core/Src/board_battery.o ./Core/Src/board_battery.su ./Core/Src/board_relays.cyclo ./Core/Src/board_relays.d ./Core/Src/board_relays.o ./Core/Src/board_relays.su ./Core/Src/cart_param.cyclo ./Core/Src/cart_param.d ./Core/Src/cart_param.o ./Core/Src/cart_param.su ./Core/Src/brake_can.cyclo ./Core/Src/brake_can.d ./Core/Src/brake_can.o ./Core/Src/brake_can.su ./Core/Src/dla_cart_app.cyclo ./Core/Src/dla_cart_app.d ./Core/Src/dla_cart_app.o ./Core/Src/dla_cart_app.su ./Core/Src/foc_can.cyclo ./Core/Src/foc_can.d ./Core/Src/foc_can.o ./Core/Src/foc_can.su ./Core/Src/imu_lsm6dsv.cyclo ./Core/Src/imu_lsm6dsv.d ./Core/Src/imu_lsm6dsv.o ./Core/Src/imu_lsm6dsv.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor_control.cyclo ./Core/Src/motor_control.d ./Core/Src/motor_control.o ./Core/Src/motor_control.su ./Core/Src/rc_input.cyclo ./Core/Src/rc_input.d ./Core/Src/rc_input.o ./Core/Src/rc_input.su ./Core/Src/screen.cyclo ./Core/Src/screen.d ./Core/Src/screen.o ./Core/Src/screen.su ./Core/Src/selftest.cyclo ./Core/Src/selftest.d ./Core/Src/selftest.o ./Core/Src/selftest.su ./Core/Src/servo_can.cyclo ./Core/Src/servo_can.d ./Core/Src/servo_can.o ./Core/Src/servo_can.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim_capture.cyclo ./Core/Src/tim_capture.d ./Core/Src/tim_capture.o ./Core/Src/tim_capture.su ./Core/Src/train_uart.cyclo ./Core/Src/train_uart.d ./Core/Src/train_uart.o ./Core/Src/train_uart.su ./Core/Src/wheel_speed.cyclo ./Core/Src/wheel_speed.d ./Core/Src/wheel_speed.o ./Core/Src/wheel_speed.su

.PHONY: clean-Core-2f-Src

