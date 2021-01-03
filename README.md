# NucleoTNC Firmware

Source code for STM32L432KC Nucleo32-based TNC (PCB & breadboard version).

http://www.mobilinkd.com/2019/06/24/nucleotnc/

# Building

Use Eclipse with CDT and the GNU MCU Eclipse plugins.

If you are porting this to another build platform, you will need to
build the firmware using the same compiler and linker options.  As
with most firmware projects, there is a linker script with defines
the memory layout for for the Flash and SRAM.

Below are example compilation and linking lines for reference:

    arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-inline-functions -fsingle-precision-constant -fstack-usage -fstrict-aliasing -ffast-math -Wall -Wextra -Wlogical-op -Wfloat-equal -g -D__FPU_PRESENT=1 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DSTM32L432xx -D__weak=__attribute__((weak)) -DNUCLEOTNC=1 -I../Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I/home/rob/workspace/Nucleo_L432KC_TNC/Inc -I/home/rob/workspace/Nucleo_L432KC_TNC/Drivers/STM32L4xx_HAL_Driver/Inc -I/home/rob/workspace/Nucleo_L432KC_TNC/Drivers/CMSIS/Include -I/home/rob/workspace/Nucleo_L432KC_TNC/Drivers/CMSIS/Device/ST/STM32L4xx/Include -I/home/rob/workspace/Nucleo_L432KC_TNC/Middlewares/Third_Party/FreeRTOS/Source/include -I/home/rob/workspace/Nucleo_L432KC_TNC/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I/home/rob/workspace/Nucleo_L432KC_TNC/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I/home/rob/workspace/Nucleo_L432KC_TNC/TNC -I/usr/arm-none-eabi/include -std=gnu++1z -fabi-version=9 -fno-exceptions -fno-rtti -fno-use-cxa-atexit -fno-threadsafe-statics -Wno-register -c -o TNC/HdlcFrame.o ../TNC/HdlcFrame.cpp 
    arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-inline-functions -fsingle-precision-constant -fstack-usage -fstrict-aliasing -ffast-math -Wall -Wextra -Wlogical-op -Wfloat-equal -g -T /home/rob/workspace/Nucleo_L432KC_TNC/STM32L432KC_FLASH.ld -Xlinker --gc-sections -Wl,-Map,firmware.map --specs=nano.specs -o firmware.elf Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_adc_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_crc_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dac_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_flash_ramfunc.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_opamp_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rng.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rtc_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_tim_ex.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.o Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.o Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o Middlewares/Third_Party/FreeRTOS/Source/croutine.o Middlewares/Third_Party/FreeRTOS/Source/event_groups.o Middlewares/Third_Party/FreeRTOS/Source/list.o Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.o Middlewares/Third_Party/FreeRTOS/Source/queue.o Middlewares/Third_Party/FreeRTOS/Source/tasks.o Middlewares/Third_Party/FreeRTOS/Source/timers.o Src/arm_fir_f32.o Src/arm_fir_fast_q15.o Src/arm_fir_init_f32.o Src/arm_fir_init_q15.o Src/arm_fir_interpolate_init_q15.o Src/arm_fir_interpolate_q15.o Src/arm_offset_q15.o Src/arm_q15_to_float.o Src/freertos.o Src/main.o Src/stm32l4xx_hal_msp.o Src/stm32l4xx_hal_timebase_TIM.o Src/stm32l4xx_it.o Src/system_stm32l4xx.o TNC/AFSKModulator.o TNC/AFSKTestTone.o TNC/Afsk1200Demodulator.o TNC/AfskDemodulator.o TNC/AudioInput.o TNC/AudioLevel.o TNC/DCD.o TNC/Demodulator.o TNC/FilterCoefficients.o TNC/Fsk9600Demodulator.o TNC/Fsk9600Modulator.o TNC/Goertzel.o TNC/Golay24.o TNC/HdlcDecoder.o TNC/HdlcFrame.o TNC/IOEventTask.o TNC/Kiss.o TNC/KissHardware.o TNC/KissTask.o TNC/LEDIndicator.o TNC/Log.o TNC/M17.o TNC/M17Demodulator.o TNC/M17Encoder.o TNC/M17Modulator.o TNC/ModulatorTask.o TNC/NullPort.o TNC/PortInterface.o TNC/SerialPort.o newlib/_exit.o newlib/_sbrk.o newlib/_syscalls.o startup/startup_stm32l432xx.o 

All of the macros defined on the compiler line are important in order
to properly build the firmware.

# Debugging

Logging is enabled in debug builds and is output via ITM (SWO).  The
firmware is distributed with an openocd stlink config file that enables
ITM output to a named pipe -- `swv`.  You must create this pipe in the
top level directory.

To read from this pipe, open a terminal and run:

`while true; do tr -d '\01' < swv; done`

If you change the MCU's core clock, you need to adjust the timing in the
`stlink-tnc3.cfg` config file.

The TNC3 runs at 48MHz on startup.  It may switch to 80MHz for modulation
types (9600, M17) which require more speed.  This makes using SWO challenging
as it cannot handle changes in core speeds.

# Installing firmware

Firmware can be installed via the on-board ST/LINK port or via USB DFU.

## USB DFU

 1. Download the STM32CubeProgrammer.
    https://s3.amazonaws.com/mobilinkd/en.stm32cubeprog-1.4.0.zip
    This programmer will work on Linux, OS X, and Windows.

 2. Download the ELF file from the release (or that you have built from source).

 3. Plug the TNC into a USB port and turn the TNC on.  You should see a USB serial port enumerated.

 4. Put the TNC into DFU mode by pressing the DFU button on the side.  The TNC
will only enter DFU mode when plugged into a USB port.

 ![TNC3 Diagram](https://s3.amazonaws.com/mobilinkd/TNC3/TNC3_Diagram.png)

***There is no visible indication on the TNC that it is in DFU mode***

 5. You should see the serial port device go away and a new DFU device appear.

 6. Run the STM32CubeProgrammer from the command-line. (Replace "firmware.elf" with the appropriate firmware filename.)

    ./STM32_Programmer_CLI -c port=USB1 -d firmware.elf -v -g 0x8000000

 7. When that is complete, the DFU device will disappear and the serial port
device will re-appear.

----

## Important Note
If you regenerate the STM32 code, please note that the LL ADC driver in
the 1.12.0 version of the HAL driver is buggy.

   Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h

This file has been modified to address the defect.  If it is replaced by
STM32CubeMX, please ensure that the defect has been fixed.

Details of the defect are available [on the ST community site](https://community.st.com/s/question/0D50X00009bLP0eSAG/adc-init-bug-with-optimization-o1-stm32l4 ADC init bug with optimization >= -O1).

----

## TNC Build Instructions

Please go here: [TNC Build Instructions](Build/NucleoTNC.ipynb)