# ADCS Mockup

Simple mockup up ADCS firmware without using external components

## Note
This project currrently uses a mutex to prevent the serial line from being used by multiple threads at once.
Your version of HALCoGen might have a bug with generating FreeRTOS code that enables mutexes.

This bug is documented [here](http://e2e.ti.com/support/microcontrollers/hercules/f/312/p/626490/2320355).

The workaround is to copy the `mpu_wrappers.c` file in `HALCoGen_fix` to `<Your HALCoGen Install>/drivers/FreeRTOS/portable/CCS/Coretex-R4/`.
You may want to rename the original file to something like `mpu_wrappers.c.old` just in case you want to restore it.
