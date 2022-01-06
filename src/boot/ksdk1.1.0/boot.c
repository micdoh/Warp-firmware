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

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
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
volatile lpuart_user_config_t		lpuartUserConfig;
volatile lpuart_state_t				lpuartState;

volatile bool						gWarpBooted				                = false;
volatile uint32_t					gWarpI2cBaudRateKbps			        = kWarpDefaultI2cBaudRateKbps;
volatile uint32_t					gWarpUartBaudRateBps			        = kWarpDefaultUartBaudRateBps;
volatile uint32_t					gWarpSpiBaudRateKbps			        = kWarpDefaultSpiBaudRateKbps;
volatile uint32_t					gWarpSleeptimeSeconds			        = kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask				gWarpMode				                = kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpI2cTimeoutMilliseconds		        = kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t					gWarpSpiTimeoutMicroseconds		        = kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t					gWarpUartTimeoutMilliseconds		    = kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		    = kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t					gWarpCurrentSupplyVoltage		        = kWarpDefaultSupplyVoltageMillivolts;
char							    gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];
uint8_t							    gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							    gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void						sleepUntilReset(void);
static void						lowPowerPinStates(void);
static void						disableTPS62740(void);
static void						enableTPS62740(uint16_t voltageMillivolts);
static void						setTPS62740CommonControlLines(uint16_t voltageMillivolts);
static void						dumpProcessorState(void);
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
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

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
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */



void
sleepUntilReset(void)
{
	while (1)
	{
		#if (WARP_BUILD_ENABLE_DEVSI4705)
			GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
		#endif

		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);

		#if (WARP_BUILD_ENABLE_DEVSI4705)
			GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
		#endif

		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}


void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX

//TODO: we don't use hw flow control so don't need RTS/CTS
 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

//TODO: we don't use hw flow control so don't need RTS/CTS
//	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
//	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}


void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX

//TODO: we don't use the HW flow control and that messes with the SPI any way
 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

//TODO: we don't use flow-control
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	*/
	CLOCK_SYS_DisableLpuartClock(0);
}



WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
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
warpDeasserAllSPIchipSelects(void)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Drive all chip selects high to disable them. Individual drivers call this routine before
	 *	appropriately asserting their respective chip selects.
	 *
	 *	Setup:
	 *		PTA12/kWarpPinISL23415_SPI_nCS	for GPIO
	 *		PTA9/kWarpPinAT45DB_SPI_nCS	for GPIO
	 *		PTA8/kWarpPinADXL362_SPI_nCS	for GPIO
	 *		PTB1/kWarpPinFPGA_nCS		for GPIO
	 *
	 *		On Glaux
	 		PTB2/kGlauxPinFlash_SPI_nCS for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	#endif

	#if (WARP_BUILD_ENABLE_DEVISL23415)
		GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_DEVAT45DB)
		GPIO_DRV_SetPinOutput(kWarpPinAT45DB_SPI_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_DEVADXL362)
		GPIO_DRV_SetPinOutput(kWarpPinADXL362_SPI_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_DEVICE40)
		GPIO_DRV_SetPinOutput(kWarpPinFPGA_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
	#endif
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
    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


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
		 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
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

/*
void
printPinDirections(void)
{
	warpPrint("I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA_UART_RX));
	OSA_TimeDelay(100);
	warpPrint("I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL_UART_TX));
	OSA_TimeDelay(100);
	warpPrint("SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI_UART_CTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO_UART_RTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
	OSA_TimeDelay(100);
	warpPrint("ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
	OSA_TimeDelay(100);
}
*/



void
dumpProcessorState(void)
{
	uint32_t	cpuClockFrequency;

	CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
	warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
	warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
	warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
	warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
	warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
	warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
	warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
	warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
	warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
	warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
	warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
	warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
	warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
	warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
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

			#if (WARP_BUILD_ENABLE_DEVBGX)
				if (gWarpBooted)
				{
					WarpStatus	status;

					enableLPUARTpins();
					initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
					status = sendBytesToUART((uint8_t *)gWarpEfmt, strlen(gWarpEfmt)+1);
					if (status != kWarpStatusOK)
					{
						SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
					}
					disableLPUARTpins();

					/*
					 *	We don't want to deInit() the BGX since that would drop
					 *	any remote terminal connected to it.
					 */
					//deinitBGX();
				}
			#endif

			return;
		}

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);

				status = sendBytesToUART((uint8_t *)gWarpPrintBuffer, max(fmtlen, kWarpDefaultPrintBufferSizeBytes));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#else
		/*
		 *	If we are not compiling in the SEGGER_RTT_printf,
		 *	we just send the format string of warpPrint()
		 */
		SEGGER_RTT_WriteString(0, fmt);

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
				status = sendBytesToUART(fmt, strlen(fmt));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#endif

	return;
}

int
warpWaitKey(void)
{
	/*
	 *	SEGGER'S implementation assumes the result of result of
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
	#if (WARP_BUILD_ENABLE_DEVBGX)
		deviceBGXState.uartRXBuffer[0] = kWarpMiscMarkerForAbsentByte;
		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
	#endif

	do
	{
		rttKey	= SEGGER_RTT_GetKey();

		#if (WARP_BUILD_ENABLE_DEVBGX)
			bleChar	= deviceBGXState.uartRXBuffer[0];
		#endif

		/*
		 *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
		 */
		if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
		{
			bleChar = kWarpMiscMarkerForAbsentByte;
		}
	} while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));

	#if (WARP_BUILD_ENABLE_DEVBGX)
		if (bleChar != kWarpMiscMarkerForAbsentByte)
		{
			/*
			 *	Send a copy of incoming BLE chars to RTT
			 */
			SEGGER_RTT_PutChar(0, bleChar);
			disableLPUARTpins();

			/*
			 *	We don't want to deInit() the BGX since that would drop
			 *	any remote terminal connected to it.
			 */
			//deinitBGX();

			return (int)bleChar;
		}

		/*
		 *	Send a copy of incoming RTT chars to BLE
		 */
		WarpStatus status = sendBytesToUART((uint8_t *)&rttKey, 1);
		if (status != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
		}

		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
		//deinitBGX();
	#endif

	return rttKey;
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
     *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
     *
     *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

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
    warpPrint("Boot done.\n");
    initMMA8451Q(0x1D    /* i2cAddress */, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
    initL3GD20H(0x6B    /* i2cAddress */, kWarpDefaultSupplyVoltageMillivoltsL3GD20H);
    initINA219(0x40 /* i2cAddress */, 3300 /* Operating voltage (mV) */);
    devSSD1331init(); /* Initialize SSD1331 OLED Display */

    draw0(2, 20, 6, 255, 255, 255);
    draw1(7, 20, 6, 255, 255, 255);
    draw2(17, 20, 6, 255, 255, 255);
    draw3(27, 20, 6, 255, 255, 255);
    draw4(37, 20, 6, 255, 255, 255);
    draw5(47, 20, 6, 255, 255, 255);
    draw6(57, 20, 6, 255, 255, 255);
    draw7(67, 20, 6, 255, 255, 255);
    draw8(77, 20, 6, 255, 255, 255);
    draw9(87, 20, 6, 255, 255, 255);


    /* Configuration value 0b000001000101:
        - 0-16V
        - PGA Gain 1
        - +/- 40mV range
        - 12-bit ADC resolution
        - 532 us conversion time
        - 1 sample per measurement
    */

    status = writeSensorRegisterINA219(0x00, 0b000001000101);
    if (status != kWarpStatusOK) {
        warpPrint("\rINA219 configuration failed...\n");
    }

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
    /*
    activeMMA8451Q();
    warpPrint("MMA8451Q Activated\n");
    for ( i = 0; i < 5; ++i) {
        readingsMMA8451Q = returnSensorDataMMA8451Q();
        OSA_TimeDelay(1000);
        warpPrint("X, %d, ", readingsMMA8451Q[0]);
        warpPrint("Y, %d, ", readingsMMA8451Q[1]);
        warpPrint("Z, %d\n", readingsMMA8451Q[2]);
        OSA_TimeDelay(1000);
    }
     */
    configureSensorMMA8451Q(0x00,/* Payload: Disable FIFO */
                            0x05/* Normal read 8bit, 800Hz, low noise ( limited to +/-4g), active mode */
    );
    configureSensorL3GD20H(0b11101111,/* ODR 800Hz, No cut-off, see table 21, normal mode, x,y,z enable */
                           0b00100000,
                           0b00000000/* normal mode, disable FIFO, disable high pass filter */
    );

    clearScreen();
    uint16_t readingsMMA8451Q[3];
    uint16_t readingsL3GD20H[3];

    uint8_t statusRegisterValue;
    uint16_t xAVG;
    uint16_t yAVG;
    uint16_t zAVG;
    int j = 0;
    while (j < 100) {
        readSensorRegisterL3GD20H(0x27, 1);
        statusRegisterValue = deviceL3GD20HState.i2cBuffer[0];
        warpPrint("Status register value: %d\n\n", statusRegisterValue);
        if ((statusRegisterValue & 0b00001000) == 8) {
            returnSensorDataL3GD20H(readingsL3GD20H);
            if (j == 0){
                xAVG = readingsL3GD20H[j];
                yAVG = readingsL3GD20H[j+1];
                zAVG = readingsL3GD20H[j+2];
            }
            else {
                xAVG = iterativeAvg(xAVG, readingsL3GD20H[0], j+1);
                yAVG = iterativeAvg(xAVG, readingsL3GD20H[1], j+1);
                zAVG = iterativeAvg(xAVG, readingsL3GD20H[2], j+1);
            }
            j++;
        }
    }

    drawGlyph(20, 2, 6, 255, 255, 255, (xAVG/10000)%10);
    drawGlyph(30, 3, 6, 255, 255, 255, (xAVG/1000)%10);
    drawGlyph(40, 4, 6, 255, 255, 255, (xAVG/100)%10);
    drawGlyph(50, 5, 6, 255, 255, 255, (xAVG/10)%10);
    drawGlyph(60, 6, 6, 255, 255, 255, xAVG%10);

    while (1) {
        warpPrint("Avg values: %d, %d, %d\n", xAVG, yAVG, zAVG);
    }

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
    return 0;
}