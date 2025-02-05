/**
 ******************************************************************************
 * @file    Led6001.h
 * @author  Davide Aliprandi, STMicroelectronics
 * @version V1.0.0
 * @date    December 9th, 2015
 * @brief   This file contains the class of an LED6001 component.
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


/* Generated with STM32CubeTOO -----------------------------------------------*/


/* Revision ------------------------------------------------------------------*/
/*
    Repository:       http://svn.x-nucleodev.codex.cro.st.com/svnroot/X-NucleoDev
    Branch/Trunk/Tag: trunk
    Based on:         X-CUBE-LED1/trunk/Drivers/BSP/Components/led6001/led6001.h
    Revision:         0
*/


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __LED6001_CLASS_H
#define __LED6001_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/        
#include "Arduino.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/        
#include "component_def.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "HumiditySensor.h"                                              *
 *   #include "TemperatureSensor.h"                                           *
 *----------------------------------------------------------------------------*/
#include "Led.h"


/* Definitions ---------------------------------------------------------------*/

/* Duration of initialization interval in milli-seconds. */
#define INITIALIZATION_INTERVAL_ms (2E3)

/* Duty Cycle of PWM and Analog Dimming modes's PWM signal to power the LED on. */
#define PWM_DIMMING_ON             (255)

/* Duty Cycle of PWM and Analog Dimming modes's PWM signal to power the LED off. */
#define PWM_DIMMING_OFF            (0)

/* Duty Cycle Range of Analog Dimming. */
#define MAX_PWM_ANALOG_DIMMING     (1.2f/3.3f)
#define MIN_PWM_ANALOG_DIMMING     (0.3f/3.3f)

/* Maximum current flowing into the LED. */
#define LED_CURRENT_MAX            (1023.0f)


/* Classes -------------------------------------------------------------------*/

/**
 * @brief Class representing a LED6001 component.
 */
class Led6001 : public Led
{
public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param xfault_irq pin name of the XFAULT pin of the component.
     * @param current    pin name of the ADC pin of the component responsible
     *                   for sensing the current flowing through the LED.
     * @param pwm        pin name of the PWM pin of the component.
     * @param analog     pin name of the Analog pin of the component.
     */
    Led6001(uint8_t xfault_irq, uint8_t current, uint8_t pwm, uint8_t analog) : Led(), xfault_irq(xfault_irq), current(current), pwm(pwm), analog(analog)
    {
        /* ACTION 4 ----------------------------------------------------------*
         * Initialize here the component's member variables, one variable per *
         * line.                                                              *
         *                                                                    *
         * Example:                                                           *
         *   measure = 0;                                                     *
         *   instance_id = number_of_instances++;                             *
         *--------------------------------------------------------------------*/
		pinMode(xfault_irq, INPUT_PULLUP);
		pinMode(pwm, OUTPUT);
		pinMode(analog, OUTPUT);
    }
    
    /**
     * @brief Destructor.
     */
    virtual ~Led6001(void) {}
    

    /*** Public Component Related Methods ***/

    /* ACTION 5 --------------------------------------------------------------*
     * Implement here the component's public methods, as wrappers of the C    *
     * component's functions.                                                 *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2).                                     *
     *                                                                        *
     * Example:                                                               *
     *   virtual int get_value(float *p_data) //(1)                           *
     *   {                                                                    *
     *     return COMPONENT_get_value(float *pf_data);                        *
     *   }                                                                    *
     *                                                                        *
     *   virtual int enable_feature(void) //(2)                               *
     *   {                                                                    *
     *     return COMPONENT_enable_feature();                                 *
     *   }                                                                    *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Initializing the component.
     * @param  init Pointer to device specific initalization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int init(void *init = NULL)
    {
        return (int) COMPONENT_OK;
    }

    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int read_id(uint8_t *id = NULL)
    {
        return (int) COMPONENT_OK;
    }

    /**
     * @brief  Getting the current flowing through the LED.
     * @param  None.
     * @retval The current flowing through the LED, represented as a floating
     *         point number in the range [0.0, 1.0].
     */
    virtual float get_current(void)
    {
        return (float) (analogRead(current) / LED_CURRENT_MAX);
    }

    /**
     * @brief  Setting PWM dimming value.
     * @param  dimming PWM dimming value, represented as a floating
     *         point number in the range [0.0, 1.0].
     * @retval None.
     */
    virtual void set_pwm_dimming(float dimming)
    {
        if (dimming >= 0.0f && dimming <= 1.0f) {
            analogWrite(pwm, (uint8_t) (dimming * PWM_DIMMING_ON));
        }
    }

    /**
     * @brief  Setting Analog dimming value.
     * @param  dimming Analog dimming value, represented as a floating
     *         point number in the range [0.0, 1.0].
     * @retval None.
     */
    virtual void set_analog_dimming(float dimming)
    {
        if (dimming >= 0.0f && dimming <= 1.0f) {
            analogWrite(analog, (uint8_t) (((float) (dimming * (MAX_PWM_ANALOG_DIMMING - MIN_PWM_ANALOG_DIMMING) + MIN_PWM_ANALOG_DIMMING)) * PWM_DIMMING_ON));
        }
    }

    /**
     * @brief  Powering ON at maximum light intensity.
     * @param  None.
     * @retval None.
     */
    virtual void power_on(void)
    {
        analogWrite(pwm, PWM_DIMMING_ON);
        analogWrite(analog, PWM_DIMMING_ON);
    }

    /**
     * @brief  Powering OFF.
     * @param  None.
     * @retval None.
     */
    virtual void power_off(void)
    {
        analogWrite(pwm, PWM_DIMMING_OFF);
        analogWrite(analog, PWM_DIMMING_OFF);
    }

    /**
     * @brief  Starting-up the component.
     * @param  None.
     * @retval None.
     */
    virtual void start_up(void)
    {
        /* Start-up sequence. */
        power_off();
        delay(INITIALIZATION_INTERVAL_ms);
        power_on();
    }


    /*** Public Interrupt Related Methods ***/

    /* ACTION 6 --------------------------------------------------------------*
     * Implement here interrupt related methods, if any.                      *
     * Note that interrupt handling is platform dependent, e.g.:              *
     *   + mbed:                                                              *
     *     InterruptIn feature_irq(pin); //Interrupt object.                  *
     *     feature_irq.rise(callback);   //Attach a callback.                 *
     *     feature_irq.mode(PullNone);   //Set interrupt mode.                *
     *     feature_irq.enable_irq();     //Enable interrupt.                  *
     *     feature_irq.disable_irq();    //Disable interrupt.                 *
     *   + Arduino:                                                           *
     *     attachInterrupt(pin, callback, RISING); //Attach a callback.       *
     *     detachInterrupt(pin);                   //Detach a callback.       *
     *                                                                        *
     * Example (mbed):                                                        *
     *   void attach_feature_irq(void (*fptr) (void))                         *
     *   {                                                                    *
     *     feature_irq.rise(fptr);                                            *
     *   }                                                                    *
     *                                                                        *
     *   void enable_feature_irq(void)                                        *
     *   {                                                                    *
     *     feature_irq.enable_irq();                                          *
     *   }                                                                    *
     *                                                                        *
     *   void disable_feature_irq(void)                                       *
     *   {                                                                    *
     *     feature_irq.disable_irq();                                         *
     *   }                                                                    *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Attaching an interrupt handler to the XFAULT interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void attach_xfault_irq(void (*fptr)(void))
    {
        attachInterrupt(xfault_irq, fptr, FALLING);
    }


protected:

    /*** Protected Component Related Methods ***/

    /* ACTION 7 --------------------------------------------------------------*
     * Declare here the component's specific methods.                         *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2);                                     *
     *   + Helper methods, if any, like functions declared in the component's *
     *     source files but not pointed by the component's virtual table (3). *
     *                                                                        *
     * Example:                                                               *
     *   status_t COMPONENT_get_value(float *f);   //(1)                      *
     *   status_t COMPONENT_enable_feature(void);  //(2)                      *
     *   status_t COMPONENT_compute_average(void); //(3)                      *
     *------------------------------------------------------------------------*/


    /*** Component's I/O Methods ***/

    /* ACTION 8 --------------------------------------------------------------*
     * Implement here other I/O methods beyond those already implemented      *
     * above, which are declared extern within the component's header file.   *
     *------------------------------------------------------------------------*/


    /*** Component's Instance Variables ***/

    /* ACTION 9 --------------------------------------------------------------*
     * Declare here interrupt related variables, if needed.                   *
     * Note that interrupt handling is platform dependent, see                *
     * "Interrupt Related Methods" above.                                     *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     InterruptIn feature_irq;                                           *
     *------------------------------------------------------------------------*/
    /* XFault Interrupt. */
    uint8_t xfault_irq;

    /* ACTION 10 -------------------------------------------------------------*
     * Declare here other pin related variables, if needed.                   *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     DigitalOut standby_reset;                                          *
     *------------------------------------------------------------------------*/
    /* Pin to sense the current flowing through the LED. */
    uint32_t current;

    /* Pulse Width Modulation dimming pin. */
    uint32_t pwm;

    /* Analog dimming pin. */
    uint32_t analog;

    /* ACTION 11 -------------------------------------------------------------*
     * Declare here communication related variables, if needed.               *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     DigitalOut address;                                                *
     *     DevI2C &dev_i2c;                                                   *
     *------------------------------------------------------------------------*/

    /* ACTION 12 -------------------------------------------------------------*
     * Declare here identity related variables, if needed.                    *
     * Note that there should be only a unique identifier for each component, *
     * which should be the "who_am_i" parameter.                              *
     *------------------------------------------------------------------------*/

    /* ACTION 13 -------------------------------------------------------------*
     * Declare here the component's static and non-static data, one variable  *
     * per line.                                                              *
     *                                                                        *
     * Example:                                                               *
     *   float measure;                                                       *
     *   int instance_id;                                                     *
     *   static int number_of_instances;                                      *
     *------------------------------------------------------------------------*/
};

#endif // __LED6001_CLASS_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
