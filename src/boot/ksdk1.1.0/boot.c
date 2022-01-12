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
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "devSSD1331.h"
#include "devINA219.h"
#include "devMMA8451Q.h"
#include "devL3GD20H.h"
volatile L3GD20H		deviceL3GD20HState;
volatile INA219		deviceINA219State;
volatile MMA8451Q		deviceMMA8451QState;

#define							kWarpConstantStringI2cFailure		"\rI2C failed"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid V"
#define							kWarpConstantStringErrorSanity		"\rCheck fail"

volatile i2c_master_state_t			i2cMasterState;
volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;

//volatile bool						gWarpBooted				                = false;
const uint32_t					gWarpI2cBaudRateKbps			        = kWarpDefaultI2cBaudRateKbps;
const uint32_t					gWarpSpiBaudRateKbps			        = kWarpDefaultSpiBaudRateKbps;
//volatile uint32_t					gWarpSleeptimeSeconds			        = kWarpDefaultSleeptimeSeconds;
const uint32_t					gWarpI2cTimeoutMilliseconds		        = kWarpDefaultI2cTimeoutMilliseconds;
//volatile uint32_t					gWarpSpiTimeoutMicroseconds		        = kWarpDefaultSpiTimeoutMicroseconds;
//volatile uint32_t					gWarpMenuPrintDelayMilliseconds		    = kWarpDefaultMenuPrintDelayMilliseconds;
//volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
char							    gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];
uint8_t							    gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							    gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

volatile bool tap = false;
volatile uint8_t tapCount = 0;
const uint8_t nSamplesL3GD20H = kWarpSizesI2cBufferBytesL3GD20H/6;
const uint8_t nSamplesMMA8451Q = kWarpSizesI2cBufferBytesMMA8451Q/6;
int16_t readingsMMA8451QFIFO[kWarpSizesI2cBufferBytesMMA8451Q/2];
int16_t readingsL3GD20HFIFO[kWarpSizesI2cBufferBytesL3GD20H/2];
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
uint16_t sampleCount = 0;




static void					   lowPowerPinStates(void);
int16_t                        iterativeAvg(int16_t prev_avg, int16_t cur_elem, uint8_t n);
uint16_t                       gapsqrt32(uint32_t a);
uint16_t                       getRmsXyz(int16_t x, int16_t y, int16_t z);
//WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
//WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);
//void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);


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


void
PORTA_IRQHandler(void)
{
    PORT_HAL_ClearPortIntFlag(PORTA_BASE); // Lower interrupt pin
    tap = true;
}


int16_t
iterativeAvg(int16_t prev_avg, int16_t cur_elem, uint8_t n) {
    uint16_t  result;
    result = (((prev_avg * (n-1)) + cur_elem) / n );
    return result;
}

// From https://gist.github.com/foobaz/3287f153d125277eefea
uint16_t
gapsqrt32(uint32_t a) {
    uint32_t rem = 0, root = 0;

    for (int i = 32 / 2; i > 0; i--) {
        root <<= 1;
        rem = (rem << 2) | (a >> (32 - 2));
        a <<= 2;
        if (root < rem) {
            rem -= root | 1;
            root += 2;
        }
    }
    return root >> 1;
}


uint16_t
getRmsXyz(int16_t x, int16_t y, int16_t z) {
    int16_t     result;
    uint32_t    sum;

    sum = (x*x) + (y*y) + (z*z);
    result = gapsqrt32(sum);

    return result;
}


int16_t
getGradient(int16_t x, int16_t y, int16_t z) {
    int16_t     result;
    uint16_t    rms;
    int16_t    aRes;

    rms = getRmsXyz(x, y, z);
    aRes = 4096 - rms;
    result = rms;
    return result;
}


int
main(void) {
    WarpStatus status;

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
        warpPrint("\n\n\n\rBooting\n\n");
        OSA_TimeDelay(1000);
    }

    /*
     *	Initialize the GPIO pins with the appropriate pull-up, etc.,
     *	defined in the inputPins and outputPins arrays (gpio_pins.c).
     *
     *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
     */
    GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);

    /*
     *	Note that it is lowPowerPinStates() that sets the pin mux mode,
     *	so until we call it pins are in their default state.
     */
    lowPowerPinStates();

    /*
     *	At this point, we consider the system "booted" and, e.g., warpPrint()s
     *	will also be sent to the BLE if that is compiled in.
     */
    //gWarpBooted = true;

    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
    PORT_HAL_SetPinIntMode(PORTA_BASE, 12, kPortIntFallingEdge);

    // Initialise sensors and screen
    initMMA8451Q(0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
    OSA_TimeDelay(100);
    initL3GD20H(0x6B, kWarpDefaultSupplyVoltageMillivoltsL3GD20H);
    OSA_TimeDelay(100);
    initSSD1331();
    OSA_TimeDelay(100);
    warpPrint("sqrt 121: %d", gapsqrt32(121));
    warpPrint("RMS 100, 100, 100: %d", getRmsXyz(100, 100, 100));
    OSA_TimeDelay(1000);

    uint8_t linesL[8] = {0, 0, 0, 20, 0, 0, 10, 0};
    uint8_t linese[28] = {10, 0 , 3, 0, 3, 0, 0, 3, 0, 3, 0, 6, 0, 6, 3, 10, 3, 10, 7, 10, 7, 10, 10, 6, 10, 6, 0, 6};
    uint8_t linest[16] = {0, 20, 0, 3, 0, 3, 3, 0, 3, 0, 10, 0, 0, 10, 10, 10};
    uint8_t linesApost[4] = {10, 20, 7, 17};
    uint8_t liness[28] = {7, 10, 3, 10, 3, 10, 0, 7, 0, 7, 3, 5, 3, 5, 7, 5, 7, 5, 10, 3, 10, 3, 7, 0, 7, 0, 3, 0};
    uint8_t linesR[28] = {0, 0, 0, 20, 0, 20, 8, 20, 8, 20, 10, 18, 10, 18, 10, 12, 10, 12, 8, 10, 8, 10, 0, 10, 7, 10, 10, 0};
    uint8_t linesi[8] = {0, 0, 0, 10, 0, 12, 0, 14};
    uint8_t linesd[16] = {10, 20, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 10, 10};
    drawChar(0, 20, 255, linesL, 2, 2);
    drawChar(12, 20, 255, linese, 7,2);
    drawChar(24, 20, 255, linest, 4,2);
    drawChar(34, 20, 255, linesApost, 1,2);
    drawChar(40, 20, 255, liness, 7,2);
    drawChar(40, 50, 255, linesR, 7,2);
    drawChar(55, 50, 255, linesi, 2,2);
    drawChar(60, 50, 255, linesd, 4, 2);
    drawChar(73, 50, 255, linese, 7,2);

    status = configureSensorMMA8451Q();
    if (status != kWarpStatusOK) {
       warpPrint("\rMMA8451Q configuration failed...\n");
    }
    OSA_TimeDelay(1000);
    status = configureSensorL3GD20H();
    if (status != kWarpStatusOK) {
        warpPrint("\rIL3GD20H configuration failed...\n");
    }
    OSA_TimeDelay(1000);

    j = 0;
    while (j < 100) {

        readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1);
        statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];
        warpPrint("Gyro %d\n", statusRegisterValueGyro);
        if ((statusRegisterValueGyro & 0b11000000) > 0) {

            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);
            k = 0;

            for (i = 0; i < nSamplesL3GD20H; i++) {
                zeroRateGyroX = iterativeAvg(zeroRateGyroX, readingsL3GD20HFIFO[k], j + i + 1);
                zeroRateGyroY = iterativeAvg(zeroRateGyroY, readingsL3GD20HFIFO[k + 1], j + i + 1);
                zeroRateGyroZ = iterativeAvg(zeroRateGyroZ, readingsL3GD20HFIFO[k + 2], j + i + 1);
                k += 3;
            }
            j += nSamplesL3GD20H;
        }
    }


    j = 0;
    while (j < 100) {

        readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
        statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];
        warpPrint("Accel %d\n", statusRegisterValueAccel);
        if ((statusRegisterValueAccel & 0b11000000) > 0) {

            returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytesMMA8451Q);
            k = 0;

            for (i = 0; i < nSamplesMMA8451Q; i++) {
                zeroRateAccelX = iterativeAvg(zeroRateAccelX, readingsMMA8451QFIFO[k], j + i + 1);
                zeroRateAccelY = iterativeAvg(zeroRateAccelY, readingsMMA8451QFIFO[k + 1], j + i + 1);
                zeroRateAccelZ = iterativeAvg(zeroRateAccelZ, readingsMMA8451QFIFO[k + 2], j + i + 1);
                k += 3;
                OSA_TimeDelay(2);
            }
            j += nSamplesMMA8451Q;
        }
    }
    int8_t X_OFFSET = (zeroRateAccelX/8) * 1;
    int8_t Y_OFFSET = (zeroRateAccelY/8) * 1;
    int8_t Z_OFFSET = ((4096-zeroRateAccelZ)/8) * 1;
    warpPrint("ZERO: %d, %d, %d\n", zeroRateAccelX, zeroRateAccelY, zeroRateAccelZ);
    warpPrint("OFFSETS: %d, %d, %d\n", X_OFFSET, Y_OFFSET, Z_OFFSET);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1, 0x00); // Standby
    writeSensorRegisterMMA8451Q(0x2F, X_OFFSET); // x_OFFSET
    writeSensorRegisterMMA8451Q(0x30, Y_OFFSET); // Y_OFFSET
    writeSensorRegisterMMA8451Q(0x31, Z_OFFSET); // Z_OFFSET
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1, val_MMA8451Q_CTRL_REG1); // Active

    clearScreen();

    //warpPrint("sampleCount, xGyro, yGyro, zGyro\n");
    timeStart = OSA_TimeGetMsec();
    drawPoint(35, 25, 8, 255);

    while (1) {

        if (tap) {
            tapCount++;
            tapCount %= 3;
            readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
            readSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_SRC, 1); // Clear interrupt flag
            warpPrint("TAP\n");
            tap = false;
        }

        currTime = OSA_TimeGetMsec();

        switch(tapCount) {

            case 1: // Gradient only
                break;

            case 2: // Cadence only
                break;

            default: // Both
                break;
        }

        readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
        statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

        if ((statusRegisterValueAccel & 0b11000000) > 0) {

            returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytesMMA8451Q);
            j = 0;
            for (i = 0; i < nSamplesMMA8451Q; i++) {
                //warpPrint("%d, %d, %d\n", readingsMMA8451QFIFO[j], readingsMMA8451QFIFO[j+1], readingsMMA8451QFIFO[j+2]);
                j+=3;
            }
            xAccel = readingsMMA8451QFIFO[0];
            yAccel = readingsMMA8451QFIFO[1];
            zAccel = readingsMMA8451QFIFO[2];
        }

        readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1); // FIFO
        statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];

        if ((statusRegisterValueGyro & 0b11000000) > 0) {

            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);
            j = 0;
            for (i = 0; i < nSamplesL3GD20H; i++) {
                sampleCount += 1;
                //warpPrint("%u, %d, %d, %d\n", sampleCount, readingsL3GD20HFIFO[j], readingsL3GD20HFIFO[j+1], readingsL3GD20HFIFO[j+2]);
                j+=3;
            }
            xGyro = readingsL3GD20HFIFO[0];
            yGyro = readingsL3GD20HFIFO[1];
            zGyro = readingsL3GD20HFIFO[2];
        }

        if (currTime - timeStart >= 1000) {
            convertFromRawMMA8451Q(zAccel, digits);
            if (digitsPrev[0] != digits[0]) {
                if (digits[0] == 1) {
                    drawMinus(10, 25, 8, 255);
                    digitsPrev[0] = digits[0];
                }
                else {
                    drawMinus(10, 25, 8, 0);
                    digitsPrev[0] = digits[0];
                }
            }
            if (digitsPrev[1] != digits[1]) {
                drawGlyph(22, 25, 8, 255, digits[1]);
                digitsPrev[1] = digits[1];
            }
            if (digitsPrev[2] != digits[2]) {
                drawGlyph(42, 25, 8, 255, digits[2]);
                digitsPrev[2] = digits[2];
            }
            if (digitsPrev[3] != digits[3]) {
                drawGlyph(55, 25, 8, 255, digits[3]);
                digitsPrev[3] = digits[3];
            }
            if (digitsPrev[4] != digits[4]) {
                drawGlyph(67, 25, 8, 255, digits[4]);
                digitsPrev[4] = digits[4];
            }
            if (digitsPrev[5] != digits[5]) {
                drawGlyph(80, 25, 8, 255, digits[5]);
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
//    p


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