#ifndef __FSL_GPIO_PINS_H__
#define __FSL_GPIO_PINS_H__
#include "fsl_gpio_driver.h"

/*
 *	On Warp, these are the alternative functions we have chosen:
 *
 *	Pin Name			Default		Configuration
 *	=========================	========	=============
 *	PTA0/IRQ_0/LLWU_P7		SWD_CLK		ALT1
 *	PTA1/IRQ_1/LPTMR0_ALT1		RESET_b		ALT1
 *	PTA2				SWD_DIO		ALT1
 *	PTA3				EXTAL0		DEFAULT
 *	PTA4				XTAL0		DEFAULT
 *	PTA5/RTC_CLK_IN			DISABLED	ALT1
 *	PTA6				DISABLED	ALT3
 *	PTA7/IRQ_4			DISABLED	ALT3
 *	PTA8				ADC0_SE2	ALT1
 *	PTA9				ADC0_SE2	ALT1
 *	PTA12/IRQ_13/LPTMR0_ALT2	ADC0_SE0	ALT1
 *
 *	PTB0/IRQ_5/LLWU_P4		ADC0_SE9	ALT3
 *	PTB1/IRQ_6			ADC0_SE8	DEFAULT
 *	PTB2/IRQ_7			VREF_OUT	ALT1
 *	PTB3/IRQ_10			DISABLED	ALT2
 *	PTB4/IRQ_11			DISABLED	ALT2
 *	PTB5/IRQ_12			NMI_b		ALT1
  *	PTB6				DISABLED	ALT1
 *	PTB7				DISABLED	ALT1
 *	PTB10				DISABLED	ALT1
 *	PTB11				DISABLED	ALT1
 *	PTB13/CLKOUT32K			DISABLED	ALT1 
 */

enum _gpio_pins
{
	kWarpPinUnusedPTA0			= GPIO_MAKE_PIN(HW_GPIOA, 0),	/*	PTA0: Reserved for SWD CLK	*/
	kWarpPinUnusedPTA1			= GPIO_MAKE_PIN(HW_GPIOA, 1),	/*	PTA1: Reserved for SWD RESET_B	*/
	kWarpPinUnusedPTA2			= GPIO_MAKE_PIN(HW_GPIOA, 2),	/*	PTA2: Reserved for SWD DIO	*/
	kWarpPinEXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 3),	/*	PTA3: Reserved for EXTAL0	*/
	kWarpPinXTAL0				= GPIO_MAKE_PIN(HW_GPIOA, 4),	/*	PTA4: Reserved for XTAL0	*/

    kWarpPinSPI_R			        = GPIO_MAKE_PIN(HW_GPIOB, 0),
    kWarpPinSPI_DC             		= GPIO_MAKE_PIN(HW_GPIOB, 1),
    i2cSCL                  		= GPIO_MAKE_PIN(HW_GPIOB, 3),
    i2cSDA                  		= GPIO_MAKE_PIN(HW_GPIOB, 4),
    kWarpPinSPI_OC			        = GPIO_MAKE_PIN(HW_GPIOB, 11),

    kWarpPinSPI_MOSI			    = GPIO_MAKE_PIN(HW_GPIOA, 8),
    kWarpPinSPI_CK		            = GPIO_MAKE_PIN(HW_GPIOA, 9),
    kWarpPinMMA8451QINT2		    = GPIO_MAKE_PIN(HW_GPIOA, 12),
    kWarpPinL3GD20HINTDRDY		    = GPIO_MAKE_PIN(HW_GPIOB, 6),

};

extern gpio_input_pin_user_config_t	    inputPins[];
extern gpio_output_pin_user_config_t	outputPins[];
extern gpio_input_pin_user_config_t	    wakeupPins[];
#endif /* __FSL_GPIO_PINS_H__ */
