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
#include "sin1.h"

#define							kWarpConstantStringI2cFailure		"\rI2C failed"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid V"
#define							kWarpConstantStringErrorSanity		"\rCheck fail"

volatile L3GD20H		            deviceL3GD20HState;
volatile INA219		                deviceINA219State;
volatile MMA8451Q		            deviceMMA8451QState;

volatile i2c_master_state_t			i2cMasterState;
volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t	spiUserConfig;

const uint32_t					    gWarpI2cBaudRateKbps			        = kWarpDefaultI2cBaudRateKbps;
const uint32_t					    gWarpSpiBaudRateKbps			        = kWarpDefaultSpiBaudRateKbps;
const uint32_t					    gWarpI2cTimeoutMilliseconds		        = kWarpDefaultI2cTimeoutMilliseconds;
char							    gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];
uint8_t							    gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							    gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

volatile bool dataReady = false;
volatile bool point = false;
volatile bool tap = false;
volatile bool changeIcon = true;
volatile uint8_t tapCount = 0;
volatile uint8_t strokeCount = 0;
volatile uint8_t strokeCountPrev = 0;
volatile uint8_t strokeCount6 = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t cadence = 0;
volatile uint8_t printIndicator = 1;
volatile uint8_t strokeCount4SecsAgo = 0;
volatile uint8_t strokeCount8SecsAgo = 0;
volatile uint8_t time = 0;
volatile int16_t Accel;
volatile int16_t currAvgGyro = 0;
volatile int16_t prevAvgGyro = 0;
volatile int16_t currDerivGyro = 0;
volatile int16_t prevDerivGyro = 0;
volatile uint8_t statusRegisterL3GD20HFIFO;
const uint8_t nSamplesL3GD20H = kWarpSizesI2cBufferBytesL3GD20H/2; // only y-axis enabled
const uint8_t nSamplesMMA8451Q = kWarpSizesI2cBufferBytesMMA8451Q/6; // all three axes
int16_t readingsMMA8451QFIFO[kWarpSizesI2cBufferBytesMMA8451Q/2];
int16_t readingsL3GD20HFIFO[kWarpSizesI2cBufferBytesL3GD20H/2];
uint8_t statusRegisterValueGyro, statusRegisterValueAccel;
int16_t zeroRateAccelX = 0, zeroRateAccelY = 0, zeroRateAccelZ = 0;
int16_t xGyro, yGyro, zGyro, xAccel, yAccel, zAccel;
uint8_t i;
uint8_t j;
uint8_t k;
uint32_t currTime, timeStart;
uint8_t cadenceDigits[3] = {0, 0, 0};
uint8_t cadenceDigitsPrev[3] = {10, 10, 10};
uint8_t digits[6] = {0, 0, 0, 0, 0, 0};
uint8_t digitsPrev[6] = {1, 9 ,9 ,9 ,9 ,9};
uint8_t linesL[8] = {0, 0, 0, 20, 0, 0, 10, 0};
uint8_t linese[28] = {10, 0 , 3, 0, 3, 0, 0, 3, 0, 3, 0, 6, 0, 6, 3, 10, 3, 10, 7, 10, 7, 10, 10, 6, 10, 6, 0, 6};
uint8_t linest[16] = {0, 20, 0, 3, 0, 3, 3, 0, 3, 0, 10, 0, 0, 10, 10, 10};
uint8_t linesApost[4] = {10, 20, 7, 17};
uint8_t liness[28] = {7, 10, 3, 10, 3, 10, 0, 7, 0, 7, 3, 5, 3, 5, 7, 5, 7, 5, 10, 3, 10, 3, 7, 0, 7, 0, 3, 0};
uint8_t linesR[28] = {0, 0, 0, 20, 0, 20, 8, 20, 8, 20, 10, 18, 10, 18, 10, 12, 10, 12, 8, 10, 8, 10, 0, 10, 7, 10, 10, 0};
uint8_t linesi[8] = {0, 0, 0, 10, 0, 12, 0, 14};
uint8_t linesd[16] = {10, 20, 10, 0, 10, 0, 0, 0, 0, 0, 0, 10, 0, 10, 10, 10};
uint8_t linesRs[28] = {0, 0, 0, 10, 0, 10, 4, 10, 4, 10, 5, 9, 5, 9, 5, 6, 5, 6, 4, 5, 4, 5, 0, 5, 4, 5, 5, 0};
uint8_t linesPs[24] = {0, 0, 0, 10, 0, 10, 4, 10, 4, 10, 5, 9, 5, 9, 5, 6, 5, 6, 4, 5, 4, 5, 0, 5};
uint8_t linesMs[16] = {0, 0, 0, 10, 0, 10, 5, 5, 5, 5, 10, 10, 10, 10, 10, 0};
uint8_t linesX[8] = {0, 10, 10, 0, 0, 0, 10, 10};
uint8_t linesY[8] = {0, 10, 5, 5, 0, 0, 10, 10};
uint8_t linesZ[12] = {0, 10, 10, 10, 10, 10, 0, 0, 0, 0, 10, 0};

static void					   lowPowerPinStates(void);
int16_t                        iterativeAvg(int16_t prev_avg, int16_t cur_elem, uint8_t n);

int16_t
iterativeAvg(int16_t prev_avg, int16_t cur_elem, uint8_t n) {
    uint16_t  result;
    result = (((prev_avg * (n-1)) + cur_elem) / n );
    return result;
}

void
printCadence(uint8_t cad, uint8_t * cadDigits, uint8_t * cadDigitsPrev) {
    cadDigits[0] = (cad / 100) % 10;
    cadDigits[1] = (cad / 10) % 10;
    cadDigits[2] = cad % 10;

    // Remove decimal point
    if (point) {
        clearScreen();
        point = false;
    }

    // Print RPM in top right
    if (changeIcon) {
        printRPM();
        changeIcon = false;
    }

    // Check if digit already displayed onscreen, to save time re-printing
    if (cadDigits[0] != 0) {
        drawGlyph(20, 30, 12, 255, cadDigits[0]);
        cadDigitsPrev[0] = cadDigits[0];
    }
    else {
        drawGlyph(20, 30, 12, 0, cadDigitsPrev[0]);
    }
    if (cadDigitsPrev[1] != cadDigits[1]) {
        drawGlyph(40, 30, 12, 255, cadDigits[1]);
        cadDigitsPrev[1] = cadDigits[1];
    }
    if (cadDigitsPrev[2] != cadDigits[2]) {
        drawGlyph(60, 30, 12, 255, cadDigits[2]);
        cadDigitsPrev[2] = cadDigits[2];
    }
}

void bootSplash(void) {
    drawChar(0, 20, 255, linesL, 2, 2);
    drawChar(12, 20, 255, linese, 7, 2);
    drawChar(24, 20, 255, linest, 4, 2);
    drawChar(34, 20, 255, linesApost, 1, 2);
    drawChar(40, 20, 255, liness, 7, 2);
    drawChar(40, 50, 255, linesR, 7, 2);
    drawChar(55, 50, 255, linesi, 2, 2);
    drawChar(60, 50, 255, linesd, 4, 2);
    drawChar(73, 50, 255, linese, 7, 2);
}

void resetIcon(void) {
    clearScreen();
    digitsPrev[0] = 10;
    digitsPrev[1] = 10;
    digitsPrev[2] = 10;
    digitsPrev[3] = 10;
    digitsPrev[4] = 10;
    digitsPrev[5] = 10;
}

void printRPM(void) {
    drawChar(0, 12, 255, linesRs, 7, 2);
    drawChar(12, 12, 255, linesPs, 6, 2);
    drawChar(24, 12, 255, linesMs, 4, 2);
}

void printX(void) {
    resetIcon();
    drawChar(0, 12, 255, linesX, 2, 2);
}

void printY(void) {
    resetIcon();
    drawChar(0, 12, 255, linesY, 2, 2);
}

void printZ(void) {
    resetIcon();
    drawChar(0, 12, 255, linesZ, 3, 2);
}

// To be used if WARP_BUILD_BOOT_TO_ACCELSTREAM is enabled
void printMMA8451QValues(void) {
    warpPrint("xAccel, yAccel, zAccel");
    while (1) {
        readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
        statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

        if ((statusRegisterValueAccel & 0b11000000) > 0) {

            returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytesMMA8451Q);
            j = 0;
            for (i = 0; i < nSamplesMMA8451Q; i++) {
                warpPrint("%d, %d, %d\n", readingsMMA8451QFIFO[j], readingsMMA8451QFIFO[j + 1],readingsMMA8451QFIFO[j + 2]);
                j += 3;
            }
        }
    }
}

// To be used if WARP_BUILD_BOOT_TO_GYROSTREAM is enabled
void printL3GD20HValues(void) {
    warpPrint("yGyro, ");
    while (1) {
        readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1); // FIFO
        statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];

        if ((statusRegisterValueGyro & 0b11000000) > 0) {

            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);

            for (i = 0; i < nSamplesL3GD20H; i++) {
                warpPrint("%d\n", readingsL3GD20HFIFO[i]);
            }
        }
    }
}

void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

    /*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
    PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			          calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		    = kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}

void
warpEnableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0, (i2c_master_state_t *)&i2cMasterState);
}

void
lowPowerPinStates(void)
{

    PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

    /*
     * Configure PTA12 for interrupt on rising edge
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
    PORT_HAL_SetPinIntMode(PORTA_BASE, 12, kPortIntRisingEdge);

    /*
     * Configure PTB6 for interrupt on rising edge
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
    PORT_HAL_SetPinIntMode(PORTB_BASE, 6, kPortIntRisingEdge);

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
    tap = true; // Tap detected
}

void
PORTB_IRQHandler(void)
{
    PORT_HAL_ClearPortIntFlag(PORTB_BASE); // Lower interrupt pin
    dataReady = true; // FIFO interrupt from gyro (to be implemented)
}

int
main(void) {
    WarpStatus status;


    // Enable clock for I/O PORT A and PORT B
    CLOCK_SYS_EnablePortClock(0);
    CLOCK_SYS_EnablePortClock(1);

    // Initialize KSDK Operating System Abstraction layer (OSA) layer.
    OSA_Init();

    /*
     *	Setup SEGGER RTT to output as much as fits in buffers.
     *
     *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
     *	we might have SWD disabled at time of blockage.
     */
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

    //warpPrint("\n\n\n\rBooting\n\n");
    //OSA_TimeDelay(1000);

    // Initialize the GPIO pins
    GPIO_DRV_Init(inputPins, outputPins);

    // Set pin mux mode,
    lowPowerPinStates();

    // Initialise sensors and screen
    initMMA8451Q(0x1D, kWarpDefaultSupplyVoltageMillivoltsMMA8451Q);
    OSA_TimeDelay(100);
    initL3GD20H(0x6B, kWarpDefaultSupplyVoltageMillivoltsL3GD20H);
    OSA_TimeDelay(100);
    initSSD1331();
    OSA_TimeDelay(100);

    // Display "Let's Ride""
    bootSplash();

    // Configure sensors
    status = configureSensorMMA8451Q();
    if (status != kWarpStatusOK) {
       warpPrint("\rMMA8451Q configuration failed...\n");
    }
    OSA_TimeDelay(1000);
    status = configureSensorL3GD20H();
    if (status != kWarpStatusOK) {
        warpPrint("\rL3GD20H configuration failed...\n");
    }
    OSA_TimeDelay(1000);

    // Perform 100 reads of the MMA8451Q FIFO, average the values on each axis and set as the offset
    j = 0;
    while (j < 100) {

        readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
        statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];
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
    // Calculations are correct for +/-4g
    int8_t X_OFFSET = (zeroRateAccelX/8) * 1;
    int8_t Y_OFFSET = (zeroRateAccelY/8) * 1;
    int8_t Z_OFFSET = ((2098-zeroRateAccelZ)/8) * 1;

    // Write offsets to dedicated registers
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1, 0x00); // Standby
    writeSensorRegisterMMA8451Q(0x2F, X_OFFSET);
    writeSensorRegisterMMA8451Q(0x30, Y_OFFSET);
    writeSensorRegisterMMA8451Q(0x31, Z_OFFSET);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1, val_MMA8451Q_CTRL_REG1); // Active

    // Remove boot screen
    clearScreen();

    // Print output of MMA8451Q indefinitely (for evaluation/debug purposes)
    if (WARP_BUILD_BOOT_TO_ACCELSTREAM) {
        printMMA8451QValues();
    }

    // Print output of L3GD20H indefinitely (for evaluation/debug purposes)
    if (WARP_BUILD_BOOT_TO_GYROSTREAM) {
        printL3GD20HValues();
    }

    if (WARP_BUILD_BOOT_INA219) {
        /* Configuration value 0b000001000101:
         * - 0-16V
         * - PGA Gain 1
         * - +/- 40mV range
         * - 12-bit ADC resolution
         * - 532 us conversion time
         * - 1 sample per measurement
        */
        initINA219(0x40, 3300);
        status = writeSensorRegisterINA219(0x00, 0b000001000101);
        if (status != kWarpStatusOK) {
            warpPrint("\rINA219 configuration failed...\n");
        }

        // Checking register values
        printSensorDataINA219(false, 0x00);  // Configuration
        printSensorDataINA219(false, 0x01);  // Shunt voltage
        printSensorDataINA219(false, 0x02);  // Bus Voltage
        printSensorDataINA219(false, 0x03);  // Power
        printSensorDataINA219(false, 0x04);  // Current
        printSensorDataINA219(false, 0x05);  // Calibration
    }

    if (WARP_BUILD_BOOT_TO_CADENCESTREAM) {
        warpPrint("currAvgGyro,currDerivGyro,strokeCount,cadence\n");
    }

    timeStart = OSA_TimeGetMsec();

    tap = false;

    while (1) {

        // If tap detected, increment counter to alter onscreen display
        if (tap) {
            tap = false;
            changeIcon = true;
            tapCount++;
            tapCount %= 4;
        }

        // Check status of gyro FIFO
        readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1);
        statusRegisterL3GD20HFIFO = deviceL3GD20HState.i2cBuffer[0];

        // If the FIFO is overrunning, threshold is met or 32 samples available
        if (
                ((statusRegisterL3GD20HFIFO & 0b1000000) == 0b1000000) ||
                ((statusRegisterL3GD20HFIFO & 0b0100000) == 0b0100000) ||
                (statusRegisterL3GD20HFIFO == 32)) {

            // Get values of angular velocity from values in FIFO (default 32)
            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);

            // Average all values that were in FIFO. Use iterative average to avoid overflow when summing.
            for (i = 0; i < nSamplesL3GD20H; i++) {
                currAvgGyro = iterativeAvg(currAvgGyro, readingsL3GD20HFIFO[i], i+1);
            }

            // Get gradient compared to previous and take average of 2 most recent gradients
            currDerivGyro = (currDerivGyro + currAvgGyro - prevAvgGyro) / 2;
            if (WARP_BUILD_BOOT_TO_CADENCESTREAM) {
                warpPrint("%d,%d,,\n", currAvgGyro, currDerivGyro);
            }

            // Every 3 FIFO reads, set previous gradient to current
            sampleCounter++;
            sampleCounter %= 3;
            if (sampleCounter == 0) {
                prevDerivGyro = currDerivGyro;
            }

            // 2 FIFO reads after measuring previous gradient, compare against current gradient
            // (gap of 1 FIFO read to smooth noise and ensure change of polarity in ang. velocity is sustained)
            if (sampleCounter == 2) {
                // Gradients must have opposite sign and absolute difference must exceed 10
                if (((currDerivGyro > 0) && (prevDerivGyro < 0)) || ((currDerivGyro < 0) && (prevDerivGyro > 0))) {
                    if (((currDerivGyro-prevDerivGyro)*(currDerivGyro-prevDerivGyro)) > 100) {
                        strokeCount++;
                        if (WARP_BUILD_BOOT_TO_CADENCESTREAM) {
                            warpPrint(",,%d,\n", strokeCount);
                        }
                    }
                }
            prevDerivGyro = currDerivGyro;
            prevAvgGyro = currAvgGyro;
            }
        }

        currTime = OSA_TimeGetMsec();

        // Every 1 second...
        if (currTime - timeStart >= 1000) {

            time ++;
            time %= 4;

            // Calculate cadence based on 3 most recent 4 second intervals
            // Most recent interval is weighted more heavily (2:3:5 ratio)
            // Multiplicative factor to ensure it is per minute
            if (time == 0) {
                cadence = ((2 * strokeCount8SecsAgo) + (3 * strokeCount4SecsAgo) + (5 * strokeCount)) * 3 / 2;
                strokeCount8SecsAgo = strokeCount4SecsAgo;
                strokeCount4SecsAgo = strokeCount;
                strokeCount = 0;
                if (WARP_BUILD_BOOT_TO_CADENCESTREAM) {
                    warpPrint(",,,%d\n", cadence);
                }
            }

            // Update screen every 2 secs
            if (printIndicator == 1) {

                readSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_SRC, 1); // Clear interrupt flag

                // Display cadence calculation
                if (tapCount == 0) {
                    printCadence(cadence, cadenceDigits, cadenceDigitsPrev);
                }

                // Display acceleration on selected axis
                else {

                    readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
                    statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

                    if ((statusRegisterValueAccel & 0b11000000) > 0) {

                        returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytesMMA8451Q);
                        j = 0;
                        for (i = 0; i < nSamplesMMA8451Q; i++) {
                            xAccel = iterativeAvg(xAccel, readingsMMA8451QFIFO[j], i+1);
                            yAccel = iterativeAvg(yAccel, readingsMMA8451QFIFO[j+1], i+1);
                            zAccel = iterativeAvg(zAccel, readingsMMA8451QFIFO[j+2], i+1);
                            j+=3;
                        }
                    }

                    // Set display option based on number of taps
                    switch(tapCount) {
                        case 1:
                            Accel = zAccel;
                            if (changeIcon) {
                                printZ();
                                drawPoint(35, 30, 8, 255);
                                point = true;
                                changeIcon = false;
                            }
                            break;
                        case 2:
                            Accel = xAccel;
                            if (changeIcon) {
                                printX();
                                drawPoint(35, 30, 8, 255);
                                point = true;
                                changeIcon = false;
                            }
                            break;
                        case 3:
                            Accel = yAccel;
                            if (changeIcon) {
                                printY();
                                drawPoint(35, 30, 8, 255);
                                point = true;
                                changeIcon = false;
                            }
                            break;
                        default:
                            Accel = zAccel;
                            if (changeIcon) {
                                printZ();
                                drawPoint(35, 30, 8, 255);
                                point = true;
                                changeIcon = false;
                            }
                            break;
                    }

                    // Convert from counts into g's and return array of digits for printing onscreen
                    convertFromRawMMA8451Q(Accel, digits);

                    if (digitsPrev[0] != digits[0]) {
                        if (digits[0] == 1) {
                            drawMinus(10, 30, 8, 255);
                            digitsPrev[0] = digits[0];
                        }
                        else {
                            drawMinus(10, 30, 8, 0);
                            digitsPrev[0] = digits[0];
                        }
                    }
                    if (digitsPrev[1] != digits[1]) {
                        drawGlyph(22, 30, 8, 255, digits[1]);
                        digitsPrev[1] = digits[1];
                    }
                    if (digitsPrev[2] != digits[2]) {
                        drawGlyph(42, 30, 8, 255, digits[2]);
                        digitsPrev[2] = digits[2];
                    }
                    if (digitsPrev[3] != digits[3]) {
                        drawGlyph(55, 30, 8, 255, digits[3]);
                        digitsPrev[3] = digits[3];
                    }
                    if (digitsPrev[4] != digits[4]) {
                        drawGlyph(67, 30, 8, 255, digits[4]);
                        digitsPrev[4] = digits[4];
                    }
                    if (digitsPrev[5] != digits[5]) {
                        drawGlyph(80, 30, 8, 255, digits[5]);
                        digitsPrev[5] = digits[5];
                    }
                }
            }

            printIndicator *= -1;
            timeStart = currTime;
        }

        // If characterising current consumption
        if (WARP_BUILD_BOOT_INA219) {
            printSensorDataINA219(true, 0x01);  // Print current (mA) reading
        }
    }
    return 0;
}