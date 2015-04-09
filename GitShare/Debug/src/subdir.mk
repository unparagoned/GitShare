################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/SPI_Flash.c \
../src/_write.c \
../src/dosfs.c \
../src/fonts.c \
../src/main.c \
../src/menu.c \
../src/myMaths.c \
../src/newlibStubs.c \
../src/optimalPWM.c \
../src/sound.c \
../src/stm3210e_eval.c \
../src/stm3210e_eval_fsmc_nand.c \
../src/stm3210e_eval_fsmc_nor.c \
../src/stm3210e_eval_fsmc_sram.c \
../src/stm3210e_eval_lcd.c \
../src/stm32_eval_sdio_sd.c \
../src/stm32_eval_spi_flash.c \
../src/stm32f10x_it.c \
../src/storage.c \
../src/voltageControl.c 

OBJS += \
./src/SPI_Flash.o \
./src/_write.o \
./src/dosfs.o \
./src/fonts.o \
./src/main.o \
./src/menu.o \
./src/myMaths.o \
./src/newlibStubs.o \
./src/optimalPWM.o \
./src/sound.o \
./src/stm3210e_eval.o \
./src/stm3210e_eval_fsmc_nand.o \
./src/stm3210e_eval_fsmc_nor.o \
./src/stm3210e_eval_fsmc_sram.o \
./src/stm3210e_eval_lcd.o \
./src/stm32_eval_sdio_sd.o \
./src/stm32_eval_spi_flash.o \
./src/stm32f10x_it.o \
./src/storage.o \
./src/voltageControl.o 

C_DEPS += \
./src/SPI_Flash.d \
./src/_write.d \
./src/dosfs.d \
./src/fonts.d \
./src/main.d \
./src/menu.d \
./src/myMaths.d \
./src/newlibStubs.d \
./src/optimalPWM.d \
./src/sound.d \
./src/stm3210e_eval.d \
./src/stm3210e_eval_fsmc_nand.d \
./src/stm3210e_eval_fsmc_nor.d \
./src/stm3210e_eval_fsmc_sram.d \
./src/stm3210e_eval_lcd.d \
./src/stm32_eval_sdio_sd.d \
./src/stm32_eval_spi_flash.d \
./src/stm32f10x_it.d \
./src/storage.d \
./src/voltageControl.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F10X_HD -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f1-stdperiph" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


