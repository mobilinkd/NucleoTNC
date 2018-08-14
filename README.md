# nucleo32-l432kc-tnc-firmware
Source code for breadboard TNC based on STM32L432KC Nucleo32 dev board

## Important Note
If you regenerate the STM32 code, please note that the LL ADC driver in
the 1.12.0 version of the HAL driver is buggy.

   Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_ll_adc.h

This file has been modified to address the defect.  If it is replaced by
STM32CubeMX, please ensure that the defect has been fixed.

Details of the defect are available [on the ST community site](https://community.st.com/s/question/0D50X00009bLP0eSAG/adc-init-bug-with-optimization-o1-stm32l4 ADC init bug with optimization >= -O1).

