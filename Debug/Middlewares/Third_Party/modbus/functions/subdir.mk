################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/modbus/functions/mbfunccoils.c \
../Middlewares/Third_Party/modbus/functions/mbfuncdiag.c \
../Middlewares/Third_Party/modbus/functions/mbfuncdisc.c \
../Middlewares/Third_Party/modbus/functions/mbfuncfile.c \
../Middlewares/Third_Party/modbus/functions/mbfuncholding.c \
../Middlewares/Third_Party/modbus/functions/mbfuncinput.c \
../Middlewares/Third_Party/modbus/functions/mbfuncother.c \
../Middlewares/Third_Party/modbus/functions/mbutils.c 

OBJS += \
./Middlewares/Third_Party/modbus/functions/mbfunccoils.o \
./Middlewares/Third_Party/modbus/functions/mbfuncdiag.o \
./Middlewares/Third_Party/modbus/functions/mbfuncdisc.o \
./Middlewares/Third_Party/modbus/functions/mbfuncfile.o \
./Middlewares/Third_Party/modbus/functions/mbfuncholding.o \
./Middlewares/Third_Party/modbus/functions/mbfuncinput.o \
./Middlewares/Third_Party/modbus/functions/mbfuncother.o \
./Middlewares/Third_Party/modbus/functions/mbutils.o 

C_DEPS += \
./Middlewares/Third_Party/modbus/functions/mbfunccoils.d \
./Middlewares/Third_Party/modbus/functions/mbfuncdiag.d \
./Middlewares/Third_Party/modbus/functions/mbfuncdisc.d \
./Middlewares/Third_Party/modbus/functions/mbfuncfile.d \
./Middlewares/Third_Party/modbus/functions/mbfuncholding.d \
./Middlewares/Third_Party/modbus/functions/mbfuncinput.d \
./Middlewares/Third_Party/modbus/functions/mbfuncother.d \
./Middlewares/Third_Party/modbus/functions/mbutils.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/modbus/functions/%.o Middlewares/Third_Party/modbus/functions/%.su Middlewares/Third_Party/modbus/functions/%.cyclo: ../Middlewares/Third_Party/modbus/functions/%.c Middlewares/Third_Party/modbus/functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/compat/posix -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/arpa -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/net -I../Middlewares/Third_Party/LwIP/src/include/compat/posix/sys -I../Middlewares/Third_Party/LwIP/src/include/compat/stdc -I../Middlewares/Third_Party/LwIP/system/arch -I../Middlewares/Third_Party/modbus/include -I../Middlewares/Third_Party/modbus/tcp -I../Middlewares/Third_Party/modbus/rtu -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-modbus-2f-functions

clean-Middlewares-2f-Third_Party-2f-modbus-2f-functions:
	-$(RM) ./Middlewares/Third_Party/modbus/functions/mbfunccoils.cyclo ./Middlewares/Third_Party/modbus/functions/mbfunccoils.d ./Middlewares/Third_Party/modbus/functions/mbfunccoils.o ./Middlewares/Third_Party/modbus/functions/mbfunccoils.su ./Middlewares/Third_Party/modbus/functions/mbfuncdiag.cyclo ./Middlewares/Third_Party/modbus/functions/mbfuncdiag.d ./Middlewares/Third_Party/modbus/functions/mbfuncdiag.o ./Middlewares/Third_Party/modbus/functions/mbfuncdiag.su ./Middlewares/Third_Party/modbus/functions/mbfuncdisc.cyclo ./Middlewares/Third_Party/modbus/functions/mbfuncdisc.d ./Middlewares/Third_Party/modbus/functions/mbfuncdisc.o ./Middlewares/Third_Party/modbus/functions/mbfuncdisc.su ./Middlewares/Third_Party/modbus/functions/mbfuncfile.cyclo ./Middlewares/Third_Party/modbus/functions/mbfuncfile.d ./Middlewares/Third_Party/modbus/functions/mbfuncfile.o ./Middlewares/Third_Party/modbus/functions/mbfuncfile.su ./Middlewares/Third_Party/modbus/functions/mbfuncholding.cyclo ./Middlewares/Third_Party/modbus/functions/mbfuncholding.d ./Middlewares/Third_Party/modbus/functions/mbfuncholding.o ./Middlewares/Third_Party/modbus/functions/mbfuncholding.su ./Middlewares/Third_Party/modbus/functions/mbfuncinput.cyclo ./Middlewares/Third_Party/modbus/functions/mbfuncinput.d ./Middlewares/Third_Party/modbus/functions/mbfuncinput.o ./Middlewares/Third_Party/modbus/functions/mbfuncinput.su ./Middlewares/Third_Party/modbus/functions/mbfuncother.cyclo ./Middlewares/Third_Party/modbus/functions/mbfuncother.d ./Middlewares/Third_Party/modbus/functions/mbfuncother.o ./Middlewares/Third_Party/modbus/functions/mbfuncother.su ./Middlewares/Third_Party/modbus/functions/mbutils.cyclo ./Middlewares/Third_Party/modbus/functions/mbutils.d ./Middlewares/Third_Party/modbus/functions/mbutils.o ./Middlewares/Third_Party/modbus/functions/mbutils.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-modbus-2f-functions

