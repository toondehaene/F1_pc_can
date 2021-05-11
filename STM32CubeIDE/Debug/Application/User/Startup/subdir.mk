################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Application/User/Startup/startup_stm32f103rbtx.s 

OBJS += \
./Application/User/Startup/startup_stm32f103rbtx.o 

S_DEPS += \
./Application/User/Startup/startup_stm32f103rbtx.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Startup/startup_stm32f103rbtx.o: ../Application/User/Startup/startup_stm32f103rbtx.s
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Application/User/Startup/startup_stm32f103rbtx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

