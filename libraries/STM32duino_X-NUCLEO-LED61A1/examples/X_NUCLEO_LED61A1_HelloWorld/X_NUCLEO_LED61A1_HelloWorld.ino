/**
 ******************************************************************************
 * @file    X_NUCLEO_LED61A1_HelloWorld.ino
 * @author  Davide Aliprandi, STMicroelectronics
 * @version V1.0.0
 * @date    October 17h, 2017
 * @brief   Arduino test application for the STMicroelectronics X-NUCLEO-LED61A1
 *          LED expansion board.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

/* Arduino specific header files. */
#include "Arduino.h"

/* Component specific header files. */
#include "Led6001.h"


/* Definitions ---------------------------------------------------------------*/

/* PI. */
#ifndef M_PI
    #define M_PI                          (3.14159265358979323846f)
#endif

/* Loop period in micro-seconds. */
#define LOOP_PERIOD_ms                    (5E2) /* 0.5 seconds. */

/* Sin period in micro-seconds. */
#define LED_SIN_PERIOD_ms                 (1E4) /* 10 seconds. */

#define SerialPort Serial

/* Variables -----------------------------------------------------------------*/

/* Main loop's ticker. */
unsigned long current_time, old_time;

/* LED Control Component. */
Led6001 *led;

/* Interrupt flags. */
static volatile bool xfault_irq_triggered = false;


/* Functions -----------------------------------------------------------------*/

/**
 * @brief  Handling the LED capabilities.
 * @param  None.
 * @retval None.
 */
void led_handler(void)
{
    static int tick = 0;

    /* Handling the LED dimming when powered ON. */
    float dimming = 0.5f * sin(2 * M_PI * (tick++ * LOOP_PERIOD_ms) / LED_SIN_PERIOD_ms) + 0.5f;
    tick %= (int) (LED_SIN_PERIOD_ms / LOOP_PERIOD_ms);

    SerialPort.print("Sinusoidal PWM Dimming --> ");
    SerialPort.print(dimming);
    SerialPort.print("\r");

    /*
       Writing PWM dimming values to the LED.

       Notes:
         + Use "set_pwm_dimming()" for a PWM control, or "set_analog_dimming()"
           for an analog control.
     */
    led->set_analog_dimming(dimming);
}

/**
 * @brief  Interrupt Request for the component's XFAULT interrupt.
 * @param  None.
 * @retval None.
 */
void xfault_irq(void)
{
    xfault_irq_triggered = true;
}

/**
 * @brief  Interrupt Handler for the component's XFAULT interrupt.
 * @param  None.
 * @retval None.
 */
void xfault_handler(void)
{
    /* Printing to the console. */
    SerialPort.print("XFAULT Interrupt detected! Re-initializing LED driver...");

    /* Re-starting-up LED Control Component. */
    led->start_up();

    /* Printing to the console. */
    SerialPort.print("Done.\r\n\n");
}

/**
 * @brief  Initialization.
 * @param  None.
 * @retval None.
 */
void setup()
{
    /* Printing to the console. */
    SerialPort.begin(115200);
    SerialPort.print("LED Control Application Example\r\n\n");

    /* Initializing LED Control Component. */
    led = new Led6001(D4, A3, D6, D5);
    if (led->init() != COMPONENT_OK) {
        exit(EXIT_FAILURE);
    }

    /* Attaching interrupt request functions. */
    led->attach_xfault_irq(&xfault_irq);

    /* Starting-up LED Control Component. */
    led->start_up();
    old_time = millis();
}

/**
 * @brief  Main loop.
 * @param  None.
 * @retval None.
 */
void loop()
{
    current_time = millis();

    /* Either performing the component handler, interrupt handlers, or waiting for events. */
    if (current_time - old_time >= LOOP_PERIOD_ms) {
        led_handler();
        old_time = current_time;
    } else if (xfault_irq_triggered) {
        xfault_irq_triggered = false;
        xfault_handler();
    }
}
