# X-NUCLEO-LED61A1

The X-NUCLEO-LED61A1 is an expansion board based on LED6001. The expansion board is equipped with a single-channel, 
constant-current LED driver for boost or SEPIC topologies. The X-NUCLEO interfaces with the STM32 microcontroller. 
It is compatible with the Arduino™ UNO R3 connector. The brightness of the LED string connected to its output can be 
controlled through a PWM signal (0 % - 100 % dimming) or a control voltage (analog dimming). Open/Short LED fault, 
feedback disconnection, LED overcurrent and output-to-ground  short-circuit (SEPIC only) faults are detected and managed 
through the LED driver. The expansion board is designed to provide examples for applications involving several LEDs 
arranged is a single string (e.g., indoor and architectural LED lighting, off-grid street lighting, emergency LED lighting, 
white goods, gaming, etc.).

# Examples

There is 1 example with the X-NUCLEO-LED61A1 library.
* X_NUCLEO_LED61A1_HelloWorld: This application provides a simple example of usage of the X-NUCLEO-LED61A1 
LED Control Expansion Board. It shows how to control a LED stripe load connected to the board by means 
of a sinusoidal wave form injected into the PWM dimming control pin.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-LED61A1

The LED6001 datasheet is available at  
http://www.st.com/content/st_com/en/products/power-management/led-drivers/boost-current-regulators-for-led/led6001.html