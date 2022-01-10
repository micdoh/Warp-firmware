/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "config.h"

#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"
#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
//#include "fsl_lpuart_driver.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "devSSD1331.h"
#include "devINA219.h"
#include "devMMA8451Q.h"
#include "devL3GD20H.h"
volatile WarpI2CDeviceState		deviceL3GD20HState;
volatile WarpI2CDeviceState		deviceINA219State;
volatile WarpI2CDeviceState		deviceMMA8451QState;

#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"

volatile i2c_master_state_t			i2cMasterState;
volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;
//volatile lpuart_user_config_t		lpuartUserConfig;
//volatile lpuart_state_t				lpuartState;

volatile bool						gWarpBooted				                = false;
volatile uint32_t					gWarpI2cBaudRateKbps			        = kWarpDefaultI2cBaudRateKbps;
//volatile uint32_t					gWarpUartBaudRateBps			        = kWarpDefaultUartBaudRateBps;
volatile uint32_t					gWarpSpiBaudRateKbps			        = kWarpDefaultSpiBaudRateKbps;
volatile uint32_t					gWarpSleeptimeSeconds			        = kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask				gWarpMode				                = kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpI2cTimeoutMilliseconds		        = kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t					gWarpSpiTimeoutMicroseconds		        = kWarpDefaultSpiTimeoutMicroseconds;
//volatile uint32_t					gWarpUartTimeoutMilliseconds		    = kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		    = kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t					gWarpCurrentSupplyVoltage		        = kWarpDefaultSupplyVoltageMillivolts;
char							    gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];
uint8_t							    gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							    gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

volatile bool dataReady = false;
const uint8_t nSamples = kWarpSizesI2cBufferBytes/6;

int16_t readingsMMA8451QFIFO[kWarpSizesI2cBufferBytes/2];
int16_t readingsL3GD20HFIFO[kWarpSizesI2cBufferBytes/2];
uint8_t statusRegisterValueGyro, statusRegisterValueAccel;
int16_t zeroRateGyroX = 0, zeroRateGyroY = 0, zeroRateGyroZ = 0;
int16_t zeroRateAccelX = 0, zeroRateAccelY = 0, zeroRateAccelZ = 0;
int16_t xGyro, yGyro, zGyro, xAccel, yAccel, zAccel;
uint8_t i;
uint8_t j;
uint8_t k;
uint32_t currTime, timeStart;
uint8_t digits[6] = {0, 0, 0, 0, 0, 0};
uint8_t digitsPrev[6] = {1, 9, 9, 9, 9, 9};


static void						lowPowerPinStates(void);
static void						disableTPS62740(void);
static void						enableTPS62740(uint16_t voltageMillivolts);
static void						setTPS62740CommonControlLines(uint16_t voltageMillivolts);
uint16_t                        avg(uint16_t *, uint8_t length);
uint16_t                        iterativeAvg(uint16_t prev_avg, uint16_t cur_elem, uint8_t n);
WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);
void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);


/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
/*
void
LLWU_IRQHandler(void)
{

	 //	BOARD_* defines are defined in warp.h

	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}
*/
/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
/*
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{

	 //	BOARD_* defines are defined in warp.h

	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}
*/
/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}


void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);
	#endif

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}


void
warpDisableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	#endif

//TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}


void
debugPrintSPIsinkBuffer(void)
{
	for (int i = 0; i < kWarpMemoryCommonSpiBufferBytes; i++)
	{
		warpPrint("\tgWarpSpiCommonSinkBuffer[%d] = [0x%02X]\n", i, gWarpSpiCommonSinkBuffer[i]);
	}
	warpPrint("\n");
}


void
warpEnableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}


void
warpDisableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	CLOCK_SYS_DisableI2cClock(0);
}


void
lowPowerPinStates(void)
{
    /*
     *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
     *	we configure all pins as output and set them to a known state. We choose
     *	to set them all to '0' since it happens that the devices we want to keep
     *	deactivated (SI4705) also need '0'.
     */

    /*
     *			PORT A
     */
    /*
     *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

    /*
     *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
     *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
     *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
     *	functionality.
     *
     *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

    /*
     *	Disable PTA5
     *
     *	NOTE: Enabling this significantly increases current draw
     *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
     *
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

    /*
     *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
     *
     *		"Unused pins should be configured in the disabled state, mux(0),
     *		to prevent unwanted leakage (potentially caused by floating inputs)."
     *
     *	However, other documents advice to place pin as GPIO and drive low or high.
     *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

    /*
     *	NOTE: The KL03 has no PTA10 or PTA11
     */
    /*
     * Configure PTA12 for interrupt on falling edge
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
    PORT_HAL_SetPinIntMode(PORTA_BASE, 12, kPortIntEitherEdge);//kPortIntFallingEdge);



    /*
     *			PORT B
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
}


void
disableTPS62740(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_REGCTRL);
	#endif
}


void
enableTPS62740(uint16_t voltageMillivolts)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*
		 *	By default, assumme pins are currently disabled (e.g., by a recent lowPowerPinStates())
		 *
		 *	Setup:
		 *		PTB5/kWarpPinTPS62740_REGCTRL for GPIO
		 *		PTB6/kWarpPinTPS62740_VSEL4 for GPIO
		 *		PTB7/kWarpPinTPS62740_VSEL3 for GPIO
		 *		PTB10/kWarpPinTPS62740_VSEL2 for GPIO
		 *		PTB11/kWarpPinTPS62740_VSEL1 for GPIO
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

		setTPS62740CommonControlLines(voltageMillivolts);
		GPIO_DRV_SetPinOutput(kWarpPinTPS62740_REGCTRL);
	#endif
}


void
setTPS62740CommonControlLines(uint16_t voltageMillivolts)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		switch(voltageMillivolts)
		{
			case 1800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 1900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2400:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2500:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2600:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2700:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			/*
			 *	Should never happen, due to previous check in warpScaleSupplyVoltage()
			 */
			default:
			{
				warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
			}
		}

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}


void
warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == gWarpCurrentSupplyVoltage)
	{
		return;
	}

	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
		{
			enableTPS62740(voltageMillivolts);
			gWarpCurrentSupplyVoltage = voltageMillivolts;
		}
		else
		{
			warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
		}
	#endif
}


void
warpDisableSupplyVoltage(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT)
		disableTPS62740();

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}


void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	WarpStatus	status = kWarpStatusOK;

	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
	if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}

	status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}
}


uint16_t
iterativeAvg(uint16_t prev_avg, uint16_t cur_elem, uint8_t n) {
    uint16_t  result;
    result = (((prev_avg * (n-1)) + cur_elem) / n );
    return result;
}


void
warpPrint(const char *fmt, ...)
{
	int	fmtlen;
	va_list	arg;

	/*
	 *	We use an ifdef rather than a C if to allow us to compile-out
	 *	all references to SEGGER_RTT_*printf if we don't want them.
	 *
	 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
	 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
	 *	also takes our print buffer which we will eventually send over
	 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
	 *	2kB flash and removes the use of malloc so we can keep heap
	 *	allocation to zero.
	 */
	#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
		/*
		 *	We can't use SEGGER_RTT_vprintf to format into a buffer
		 *	since SEGGER_RTT_vprintf formats directly into the special
		 *	RTT memory region to be picked up by the RTT / SWD mechanism...
		 */
		va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
		va_end(arg);

		if (fmtlen < 0)
		{
			SEGGER_RTT_WriteString(0, gWarpEfmt);

			return;
		}

	#else
		/*
		 *	If we are not compiling in the SEGGER_RTT_printf,
		 *	we just send the format string of warpPrint()
		 */
		SEGGER_RTT_WriteString(0, fmt);


	#endif

	return;
}


int
warpWaitKey(void)
{
	/*
	 *	SEGGER'S implementation assumes the result of
	 *	SEGGER_RTT_GetKey() is an int, so we play along.
	 */
	int		rttKey, bleChar = kWarpMiscMarkerForAbsentByte;

	/*
	 *	Set the UART buffer to 0xFF and then wait until either the
	 *	UART RX buffer changes or the RTT icoming key changes.
	 *
	 *	The check below on rttKey is exactly what SEGGER_RTT_WaitKey()
	 *	does in SEGGER_RTT.c.
	 */

	do
	{
		rttKey	= SEGGER_RTT_GetKey();

		/*
		 *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
		 */
		if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
		{
			bleChar = kWarpMiscMarkerForAbsentByte;
		}
	} while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));


	return rttKey;
}


void PORTA_IRQHandler(void)
{
    //readSensorRegisterMMA8451Q(0x00, 1); // Clear interrupt flag
    PORT_HAL_ClearPortIntFlag(PORTA_BASE); // Lower interrupt pin
    dataReady = 1;
}


int
main(void) {
    WarpStatus status;
    rtc_datetime_t warpBootDate;
    power_manager_user_config_t warpPowerModeWaitConfig;
    power_manager_user_config_t warpPowerModeStopConfig;
    power_manager_user_config_t warpPowerModeVlpwConfig;
    power_manager_user_config_t warpPowerModeVlpsConfig;
    power_manager_user_config_t warpPowerModeVlls0Config;
    power_manager_user_config_t warpPowerModeVlls1Config;
    power_manager_user_config_t warpPowerModeVlls3Config;
    power_manager_user_config_t warpPowerModeRunConfig;

    /*
     *	We use this as a template later below and change the .mode fields for the different other modes.
     */
    const power_manager_user_config_t warpPowerModeVlprConfig = {
            .mode            = kPowerManagerVlpr,
            .sleepOnExitValue    = false,
            .sleepOnExitOption    = false
    };

    power_manager_user_config_t const *powerConfigs[] = {
            /*
             *	NOTE: POWER_SYS_SetMode() depends on this order
             *
             *	See KSDK13APIRM.pdf Section 55.5.3
             */
            &warpPowerModeWaitConfig,
            &warpPowerModeStopConfig,
            &warpPowerModeVlprConfig,
            &warpPowerModeVlpwConfig,
            &warpPowerModeVlpsConfig,
            &warpPowerModeVlls0Config,
            &warpPowerModeVlls1Config,
            &warpPowerModeVlls3Config,
            &warpPowerModeRunConfig,
    };

    WarpPowerManagerCallbackStructure powerManagerCallbackStructure;

    /*
     *	Callback configuration structure for power manager
     */
    const power_manager_callback_user_config_t callbackCfg0 = {
            callback0,
            kPowerManagerCallbackBeforeAfter,
            (power_manager_callback_data_t * ) & powerManagerCallbackStructure};

    /*
     *	Pointers to power manager callbacks.
     */
    power_manager_callback_user_config_t const *callbacks[] = {
            &callbackCfg0
    };

    /*
     *	Enable clock for I/O PORT A and PORT B
     */
    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);

    /*
     *	Set board crystal value (Warp revB and earlier).
     */
    g_xtal0ClkFreq = 32768U;

    /*
     *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
     */
    OSA_Init();

    /*
     *	Setup SEGGER RTT to output as much as fits in buffers.
     *
     *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
     *	we might have SWD disabled at time of blockage.
     */
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

    /*
     *	When booting to CSV stream, we wait to be up and running as soon as possible after
     *	a reset (e.g., a reset due to waking from VLLS0)
     */
    if (!WARP_BUILD_BOOT_TO_CSVSTREAM) {
        warpPrint("\n\n\n\rBooting Warp, in 3... ");
        OSA_TimeDelay(1000);
        warpPrint("2... ");
        OSA_TimeDelay(1000);
        warpPrint("1...\n\n\n\r");
        OSA_TimeDelay(1000);
    }

    /*
     *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
     *
     *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
     */
    CLOCK_SYS_Init(g_defaultClockConfigurations,
                   CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
                   &clockCallbackTable,
                   ARRAY_SIZE(clockCallbackTable)
    );
    CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

    /*
     *	Initialize RTC Driver (not needed on Glaux, but we enable it anyway for now
     *	as that lets us use the current sleep routines). NOTE: We also don't seem to
     *	be able to go to VLPR mode unless we enable the RTC.
     */
    RTC_DRV_Init(0);

    /*
     *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
     */
    warpBootDate.year = 2016U;
    warpBootDate.month = 1U;
    warpBootDate.day = 1U;
    warpBootDate.hour = 0U;
    warpBootDate.minute = 0U;
    warpBootDate.second = 0U;
    RTC_DRV_SetDatetime(0, &warpBootDate);

    /*
     *	Setup Power Manager Driver
     */
    memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

    warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

    warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
    warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

    warpPowerModeWaitConfig = warpPowerModeVlprConfig;
    warpPowerModeWaitConfig.mode = kPowerManagerWait;

    warpPowerModeStopConfig = warpPowerModeVlprConfig;
    warpPowerModeStopConfig.mode = kPowerManagerStop;

    warpPowerModeVlls0Config = warpPowerModeVlprConfig;
    warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

    warpPowerModeVlls1Config = warpPowerModeVlprConfig;
    warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

    warpPowerModeVlls3Config = warpPowerModeVlprConfig;
    warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

    warpPowerModeRunConfig.mode = kPowerManagerRun;

    POWER_SYS_Init(&powerConfigs,
                   sizeof(powerConfigs) / sizeof(power_manager_user_config_t * ),
                   &callbacks,
                   sizeof(callbacks) / sizeof(power_manager_callback_user_config_t * )
    );

    /*
     *	Switch CPU to Very Low Power Run (VLPR) mode
     */
    if (WARP_BUILD_BOOT_TO_VLPR) {
        warpPrint("About to switch CPU to VLPR mode... ");
        status = warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
        if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr)) {
            warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR() failed...\n");
        }
        warpPrint("done.\n\r");
    }

    /*
     *	Initialize the GPIO pins with the appropriate pull-up, etc.,
     *	defined in the inputPins and outputPins arrays (gpio_pins.c).
     *
     *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
     */
    warpPrint("About to GPIO_DRV_Init()... ");
    GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
    warpPrint("done.\n");



    /*
     *	Note that it is lowPowerPinStates() that sets the pin mux mode,
     *	so until we call it pins are in their default state.
     */
    warpPrint("About to lowPowerPinStates()... ");
    lowPowerPinStates();
    warpPrint("done.\n");

    /*
     *	At this point, we consider the system "booted" and, e.g., warpPrint()s
     *	will also be sent to the BLE if that is compiled in.
     */
    gWarpBooted = true;


    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
    PORT_HAL_SetPinIntMode(PORTA_BASE, 12, kPortIntEitherEdge);//kPortIntFallingEdge);


    initMMA8451Q(0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
    initL3GD20H(0x6B, kWarpDefaultSupplyVoltageMillivoltsL3GD20H);
    initSSD1331(); // Initialize SSD1331 OLED Display

    uint8_t linesL[8] = {0, 0, 0, 20, 0, 0, 10, 0};
    uint8_t linese[28] = {10, 0 , 3, 0, 3, 0, 0, 3, 0, 3, 0, 6, 0, 6, 3, 10, 3, 10, 7, 10, 7, 10, 10, 6, 10, 6, 0, 6};
    uint8_t linest[16] = {0, 20, 0, 3, 0, 3, 3, 0, 3, 0, 10, 0, 0, 10, 10, 10};
    uint8_t linesApost[4] = {10, 20, 7, 17};
    uint8_t liness[28] = {7, 10, 3, 10, 3, 10, 0, 7, 0, 7, 3, 5, 3, 5, 7, 5, 7, 5, 10, 3, 10, 3, 7, 0, 7, 0, 3, 0};
    uint8_t linesR[28] = {0, 0, 0, 20, 0, 20, 8, 20, 8, 20, 10, 18, 10, 18, 10, 12, 10, 12, 8, 10, 8, 10, 0, 10, 7, 10, 10, 0};
    uint8_t linesi[8] = {0, 0, 0, 10, 0, 12, 0, 14};
    uint8_t linesd[16] = {10, 20, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 10, 10};
    drawChar(0, 20, 255, 255, 255, linesL, 2, 2);
    drawChar(12, 20, 255, 255, 255, linese, 7,2);
    drawChar(24, 20, 255, 255, 255, linest, 4,2);
    drawChar(34, 20, 255, 255, 255, linesApost, 1,2);
    drawChar(40, 20, 255, 255, 255, liness, 7,2);
    drawChar(40, 50, 255, 255, 255, linesR, 7,2);
    drawChar(55, 50, 255, 255, 255, linesi, 2,2);
    drawChar(60, 50, 255, 255, 255, linesd, 4, 2);
    drawChar(73, 50, 255, 255, 255, linese, 7,2);

        /* Configuration value 0b000001000101:
            - 0-16V
            - PGA Gain 1
            - +/- 40mV range
            - 12-bit ADC resolution
            - 532 us conversion time
            - 1 sample per measurement
        */
    //initINA219(0x40, 3300);
    //status = writeSensorRegisterINA219(0x00, 0b000001000101);
    //if (status != kWarpStatusOK) {
    //    warpPrint("\rINA219 configuration failed...\n");
    //}

    /* Checking register values */
    /*
	printSensorDataINA219(false, 0x00);  // Configuration
    printSensorDataINA219(false, 0x01);  // Shunt voltage
    printSensorDataINA219(false, 0x02);  // Bus Voltage
    printSensorDataINA219(false, 0x03);  // Power
    printSensorDataINA219(false, 0x04);  // Current
    printSensorDataINA219(false, 0x05);  // Calibration
*/
    /* Print 1000 current (mA) readings */
    //for (int i = 0; i < 1000; i++)
    //{
    //    printSensorDataINA219(true, 0x01);  // Read V_shunt and convert to current (R=0.1 Ohm)
    //}

    configureSensorMMA8451Q(kWarpRegisterF_SETUPValueMMA8451Q,
                            kWarpRegisterCTRL_REG1ValueMMA8451Q,
                            kWarpRegisterXYZ_DATA_CFGValueMMA8451Q
                            );
    configureSensorL3GD20H(kWarpRegisterCTRL1ValueL3GD20H,
                           kWarpRegisterCTRL2ValueL3GD20H,
                           kWarpRegisterCTRL5ValueL3GD20H,
                           kWarpRegisterFIFO_CTRLValueL3GD20H
                           );
    /*
    uint8_t digits[6];
    int16_t test = 0b1111111111111111;
    convertFromRawMMA8451Q(test, digits);
    warpPrint("Test digits: %d %d %d %d %d %d\n", digits[0], digits[1], digits[2], digits[3], digits[4], digits[5]);
    */
    /*
     *             warpPrint("I2C Buffer post-read: %d, %d, %d, %d, %d, %d\n",
                      deviceMMA8451QState.i2cBuffer[0],
                      deviceMMA8451QState.i2cBuffer[1],
                      deviceMMA8451QState.i2cBuffer[2],
                      deviceMMA8451QState.i2cBuffer[3],
                      deviceMMA8451QState.i2cBuffer[4],
                      deviceMMA8451QState.i2cBuffer[5]);
     */

/*
    j = 0;
    while (j < 100) {

        readSensorRegisterL3GD20H(0x2F, 1);
        statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];

        if ((statusRegisterValueGyro & 0b10000000) == 0b10000000) {

            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytes);
            k = 0;

            for (i = 0; i < nSamples; i++) {
                zeroRateGyroX = iterativeAvg(zeroRateGyroX, readingsL3GD20HFIFO[k], j + i + 1);
                zeroRateGyroY = iterativeAvg(zeroRateGyroY, readingsL3GD20HFIFO[k + 1], j + i + 1);
                zeroRateGyroZ = iterativeAvg(zeroRateGyroZ, readingsL3GD20HFIFO[k + 2], j + i + 1);
                k += 3;
            }
            j += nSamples;
        }
    }


    j = 0;
    while (j < 100) {

        readSensorRegisterMMA8451Q(0x00, 1);
        statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

        if ((statusRegisterValueAccel & 0b01000000) == 0b01000000) {

            returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytes);
            k = 0;

            for (i = 0; i < nSamples; i++) {
                zeroRateAccelX = iterativeAvg(zeroRateAccelX, readingsMMA8451QFIFO[k], j + i + 1);
                zeroRateAccelY = iterativeAvg(zeroRateAccelY, readingsMMA8451QFIFO[k + 1], j + i + 1);
                zeroRateAccelZ = iterativeAvg(zeroRateAccelZ, readingsMMA8451QFIFO[k + 2], j + i + 1);
                k += 3;
            }
            j += nSamples;
        }
    }
    //zeroRateAccelX = ~zeroRateAccelX + 1
    int8_t X_OFFSET = (zeroRateAccelX/8) * 1;
    int8_t Y_OFFSET = (zeroRateAccelY/8) * 1;
    int8_t Z_OFFSET = ((4096-zeroRateAccelZ)/8) * 1;
    warpPrint("ZERO: %d, %d, %d\n", zeroRateAccelX, zeroRateAccelY, zeroRateAccelZ);
    warpPrint("OFFSETS: %d, %d, %d\n", X_OFFSET, Y_OFFSET, Z_OFFSET);
    writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1, 0x00); // Standby
    writeSensorRegisterMMA8451Q(0x2F, X_OFFSET); // x_OFFSET
    writeSensorRegisterMMA8451Q(0x30, Y_OFFSET); // Y_OFFSET
    writeSensorRegisterMMA8451Q(0x31, Z_OFFSET); // Z_OFFSET
    writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1, kWarpRegisterCTRL_REG1ValueMMA8451Q); // Active
*/
    writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1, 0x00); // Standby
    writeSensorRegisterMMA8451Q(0x2D, 0x40); // FIFO interrupt to INT2 (PTA12)
    writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1, kWarpRegisterCTRL_REG1ValueMMA8451Q); // Active

    writeSensorRegisterL3GD20H(0x22, 0b10000100);
    clearScreen();

    warpPrint("xGyro, yGyro, zGyro, xAccel, yAccel, zAccel\n");
    timeStart = OSA_TimeGetMsec();
    drawPoint(35, 25, 8, 255, 255, 255);

    //warpPrint("AAA");
    //INT_SYS_EnableIRQ(PORTA_IRQn);
    //warpPrint("BBB");
    //OSA_InstallIntHandler(PORTA_IRQn, PORTA_IRQHandler);
    //warpPrint("CCC");

    while (1) {

        if (dataReady) {
            readSensorRegisterMMA8451Q(0x00, 1); // Clear interrupt flag
            warpPrint("READY\n");
        }
        //warpPrint("%d", dataReady);

        currTime = OSA_TimeGetMsec();

        readSensorRegisterMMA8451Q(0x00, 1);
        statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

        if ((statusRegisterValueAccel & 0b01000000) == 0b01000000) {

            returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytes);
            j = 0;
            for (i = 0; i < nSamples; i++) {
                //warpPrint("%d, %d, %d\n", readingsMMA8451QFIFO[j], readingsMMA8451QFIFO[j+1], readingsMMA8451QFIFO[j+2]);
                j+=3;
            }
            xAccel = readingsMMA8451QFIFO[0];
            yAccel = readingsMMA8451QFIFO[1];
            zAccel = readingsMMA8451QFIFO[2];
        }

        readSensorRegisterL3GD20H(0x2F, 1); // FIFO
        statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];

        if ((statusRegisterValueGyro & 0b10000000) == 0b10000000) {

            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytes);
            j = 0;
            for (i = 0; i < nSamples; i++) {
                //warpPrint("%d, %d, %d\n", readingsL3GD20HFIFO[j], readingsL3GD20HFIFO[j+1], readingsL3GD20HFIFO[j+2]);
                j+=3;
            }
            xGyro = readingsL3GD20HFIFO[0];
            yGyro = readingsL3GD20HFIFO[1];
            zGyro = readingsL3GD20HFIFO[2];
        }

        //warpPrint("regValues: %d %d\n", statusRegisterValueGyro, statusRegisterValueAccel);

        if (currTime - timeStart >= 1000) {
            convertFromRawMMA8451Q(zAccel, digits);
            if (digitsPrev[0] != digits[0]) {
                if (digits[0] == 1) {
                    drawMinus(10, 25, 8, 255, 255, 255);
                    digitsPrev[0] = digits[0];
                }
                else {
                    drawMinus(10, 25, 8, 0, 0, 0);
                    digitsPrev[0] = digits[0];
                }
            }
            if (digitsPrev[1] != digits[1]) {
                drawGlyph(22, 25, 8, 255, 255, 255, digits[1]);
                digitsPrev[1] = digits[1];
            }
            if (digitsPrev[2] != digits[2]) {
                drawGlyph(42, 25, 8, 255, 255, 255, digits[2]);
                digitsPrev[2] = digits[2];
            }
            if (digitsPrev[3] != digits[3]) {
                drawGlyph(55, 25, 8, 255, 255, 255, digits[3]);
                digitsPrev[3] = digits[3];
            }
            if (digitsPrev[4] != digits[4]) {
                drawGlyph(67, 25, 8, 255, 255, 255, digits[4]);
                digitsPrev[4] = digits[4];
            }
            if (digitsPrev[5] != digits[5]) {
                drawGlyph(80, 25, 8, 255, 255, 255, digits[5]);
                digitsPrev[5] = digits[5];
            }
            timeStart = currTime;
        }

    }
    return 0;
}

    //warpPrint("%d, %d, %d, %d, %d, %d\n", xGyro, yGyro, zGyro, xAccel, yAccel, zAccel);

/*
    while (j < 3)
    {
        uint32_t currTime, timeStart;

        timeStart = OSA_TimeGetMsec();

        do {
            currTime = OSA_TimeGetMsec(); // Get current time stamp
            returnSensorDataMMA8451Q(readingsMMA8451Q);
            returnSensorDataL3GD20H(readingsL3GD20H);
            warpPrint("%d %d %d %d %d %d\n", readingsMMA8451Q[0], readingsMMA8451Q[1], readingsMMA8451Q[2], readingsL3GD20H[0], readingsL3GD20H[1], readingsL3GD20H[2]);
        } while (1000 >= (currTime - timeStart));

        j++;
    }
    */

    //warpPrint("A:");
    //printSensorDataL3GD20H(0);
    //printSensorDataMMA8451Q(0);
    //warpPrint("\n");

    //readSensorRegisterL3GD20H(0x27, 1); // No FIFO

/*
        if (((statusRegisterValueGyro & 0b00001000) == 8)
        && ((statusRegisterValueAccel & 0b00001000) == 8)) {

            returnSensorDataL3GD20H(readingsL3GD20H);
            xGyro = readingsL3GD20H[0];// - zeroRateGyroX;
            yGyro = readingsL3GD20H[1];// - zeroRateGyroY;
            zGyro = readingsL3GD20H[2];// - zeroRateGyroZ;

            returnSensorDataMMA8451Q(readingsMMA8451Q);
            xAccel = readingsMMA8451Q[0];
            yAccel = readingsMMA8451Q[1];
            zAccel = readingsMMA8451Q[2];

        }
*/
