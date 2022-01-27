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
volatile bool tap = false;
volatile uint8_t tapCount = 0;
volatile uint8_t strokeCount = 0;
volatile uint8_t strokeCountPrev = 0;
volatile uint8_t strokeCount6 = 0;
volatile uint8_t sampleCounter = 0;
volatile uint8_t cadence = 0;
volatile uint8_t time = 0;
volatile printIndicator = 1;
const uint8_t nSamplesL3GD20H = kWarpSizesI2cBufferBytesL3GD20H/2;
const uint8_t nSamplesMMA8451Q = kWarpSizesI2cBufferBytesMMA8451Q/6;
int16_t readingsMMA8451QFIFO[kWarpSizesI2cBufferBytesMMA8451Q/2];
int16_t readingsL3GD20HFIFO[kWarpSizesI2cBufferBytesL3GD20H/2];
uint8_t statusRegisterValueGyro, statusRegisterValueAccel;
int16_t zeroRateAccelX = 0, zeroRateAccelY = 0, zeroRateAccelZ = 0;
int16_t xGyro, yGyro, zGyro, xAccel, yAccel, zAccel;
uint8_t i;
uint8_t j;
uint8_t k;
uint32_t currTime, timeStart;
uint8_t gradDigits[5] = {0, 0, 0, 0, 0};
uint8_t gradDigitsPrev[5] = {1, 9, 9, 9, 9};
uint8_t digits[6] = {0, 0, 0, 0, 0, 0};
uint8_t digitsPrev[6] = {1, 9, 9, 9, 9, 9};

int16_t sin1(int16_t angle);
int16_t cos1(int16_t angle);
static void					   lowPowerPinStates(void);
int16_t                        iterativeAvg(int16_t prev_avg, int16_t cur_elem, uint8_t n);
uint16_t                       gapsqrt32(uint32_t a);
uint16_t                       getRmsXyz(int16_t x, int16_t y, int16_t z);
//WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
//WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);
//void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);





/**
 * Example for a sine/cosine table lookup
 * Implementation of sin1() / cos1().
 * We "outsource" this implementation so that the precompiler constants/macros
 * are only defined here.
 *
 * @file sin1.c
 * @author stfwi
 **/

#include "sin1.h"

/*
 * The number of bits of our data type: here 16 (sizeof operator returns bytes).
 */
#define INT16_BITS  (8 * sizeof(int16_t))
#ifndef INT16_MAX
#define INT16_MAX   ((1<<(INT16_BITS-1))-1)
#endif

/*
 * "5 bit" large table = 32 values. The mask: all bit belonging to the table
 * are 1, the all above 0.
 */
#define TABLE_BITS  (5)
#define TABLE_SIZE  (1<<TABLE_BITS)
#define TABLE_MASK  (TABLE_SIZE-1)

/*
 * The lookup table is to 90DEG, the input can be -360 to 360 DEG, where negative
 * values are transformed to positive before further processing. We need two
 * additional bits (*4) to represent 360 DEG:
 */
#define LOOKUP_BITS (TABLE_BITS+2)
#define LOOKUP_MASK ((1<<LOOKUP_BITS)-1)
#define FLIP_BIT    (1<<TABLE_BITS)
#define NEGATE_BIT  (1<<(TABLE_BITS+1))
#define INTERP_BITS (INT16_BITS-1-LOOKUP_BITS)
#define INTERP_MASK ((1<<INTERP_BITS)-1)

/**
 * "5 bit" lookup table for the offsets. These are the sines for exactly
 * at 0deg, 11.25deg, 22.5deg etc. The values are from -1 to 1 in Q15.
 */
static int16_t sin90[TABLE_SIZE+1] = {
        0x0000,0x0647,0x0c8b,0x12c7,0x18f8,0x1f19,0x2527,0x2b1e,
        0x30fb,0x36b9,0x3c56,0x41cd,0x471c,0x4c3f,0x5133,0x55f4,
        0x5a81,0x5ed6,0x62f1,0x66ce,0x6a6c,0x6dc9,0x70e1,0x73b5,
        0x7640,0x7883,0x7a7c,0x7c29,0x7d89,0x7e9c,0x7f61,0x7fd7,
        0x7fff
};

/**
 * Sine calculation using interpolated table lookup.
 * Instead of radians or degrees we use "turns" here. Means this
 * sine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * See the full description at www.AtWillys.de for the detailed
 * explanation.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t sin1(int16_t angle)
{
    int16_t v0, v1;
    if(angle < 0) { angle += INT16_MAX; angle += 1; }
    v0 = (angle >> INTERP_BITS);
    if(v0 & FLIP_BIT) { v0 = ~v0; v1 = ~angle; } else { v1 = angle; }
    v0 &= TABLE_MASK;
    v1 = sin90[v0] + (int16_t) (((int32_t) (sin90[v0+1]-sin90[v0]) * (v1 & INTERP_MASK)) >> INTERP_BITS);
    if((angle >> INTERP_BITS) & NEGATE_BIT) v1 = -v1;
    return v1;
}

/**
 * Cosine calculation using interpolated table lookup.
 * Instead of radians or degrees we use "turns" here. Means this
 * cosine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t cos1(int16_t angle)
{
    if(angle < 0) { angle += INT16_MAX; angle += 1; }
    return sin1(angle - (int16_t)(((int32_t)INT16_MAX * 270) / 360));
}

void bootSplash(void)
{
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
    tap = true;
}

void
PORTB_IRQHandler(void)
{
    PORT_HAL_ClearPortIntFlag(PORTB_BASE); // Lower interrupt pin
    dataReady = true;
}

int16_t
iterativeAvg(int16_t prev_avg, int16_t cur_elem, uint8_t n) {
    uint16_t  result;
    result = (((prev_avg * (n-1)) + cur_elem) / n );
    return result;
}

int16_t
sumAvg(int16_t * array, uint8_t n) {
    int index;
    uint16_t  result = 0;
    for (index = 0; index < n; index++) {
        result = result + array[index];
    }
    result = result / n;
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

void
printCadence(uint8_t startCol, uint8_t scale, uint8_t * cadDigits, uint8_t * cadDigitsPrev) {
    if (digitsPrev[0] != cadDigits[0]) {
        drawGlyph(67, 25, 8, 255, cadDigits[4]);
        digitsPrev[4] = cadDigits[4];
    }
    if (digitsPrev[1] != cadDigits[1]) {
        drawGlyph(80, 25, 8, 255, cadDigits[5]);
        digitsPrev[5] = cadDigits[5];
    }
}


/*
 * Source: https://www.nullhardware.com/blog/fixed-point-sine-and-cosine-for-embedded-systems/
Implements the 5-order polynomial approximation to sin(x).
@param i   angle (with 2^15 units/circle)
@return    16 bit fixed point Sine value (4.12) (ie: +4096 = +1 & -4096 = -1)

The result is accurate to within +- 1 count. ie: +/-2.44e-4.
*/
int16_t fpsin(int16_t i)
{
    /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which is the region of the curve fit). */
    /* ------------------------------------------------------------------- */
    i <<= 1;
    uint8_t c = i<0; //set carry for output pos/neg

    if(i == (i|0x4000)) // flip input value to corresponding value in range [0..8192)
        i = (1<<15) - i;
    i = (i & 0x7FFF) >> 1;
    /* ------------------------------------------------------------------- */

    /* The following section implements the formula:
     = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1 * y]) * 2^(a-q)
    Where the constants are defined as follows:
    */
    enum {A1=3370945099UL, B1=2746362156UL, C1=292421UL};
    enum {n=13, p=32, q=31, r=3, a=12};

    uint32_t y = (C1*((uint32_t)i))>>n;
    y = B1 - (((uint32_t)i*y)>>r);
    y = (uint32_t)i * (y>>n);
    y = (uint32_t)i * (y>>n);
    y = A1 - (y>>(p-q));
    y = (uint32_t)i * (y>>n);
    y = (y+(1UL<<(q-a-1)))>>(q-a); // Rounding

    return c ? -y : y;
}

//Cos(x) = sin(x + pi/2)
#define fpcos(i) fpsin((int16_t)(((uint16_t)(i)) + 8192U))

uint16_t
acos_fp(int16_t x /* 0<x<2098 */) {
    return gapsqrt32((6*2098*4) - gapsqrt32((12*2098*2098*16) + (24*x*2098*16)));
}


int
getGradient(int16_t xAc, int16_t yAc, int16_t zAc, uint8_t * display_digits) {
    //uint16_t    rms;
    //int16_t     aRes;
    uint16_t    gradientRads;
    uint32_t    gradientPercent;

    //rmsAc = getRmsXyz(xAc, yAc, zAc);
    //aRes = rms - 2098;

    // If zAc greater than 1g it is due to noise
    // Exit and digits are 0
    if (zAc >= 2098) {
        return 0;
    }

    // Check if x-axis acceleration is
    //    positive (downhill)
    //    negative (uphill)
    if (xAc < 0) {
        display_digits[0] = 1;
        zAc = zAc * -1;
    }
    gradientRads = acos_fp(zAc);
    gradientPercent = 1000 *  sin1(gradientRads*4) / cos1(gradientRads*4);
    warpPrint("Grad: %d\n", gradientPercent);
    OSA_TimeDelay(10);
    warpPrint("Sin: %d\n", sin1(gradientRads));
    OSA_TimeDelay(10);
    warpPrint("Cos: %d\n", cos1(gradientRads));
    OSA_TimeDelay(10);
    display_digits[3] = (gradientPercent / 100) % 10;
    display_digits[4] = (gradientPercent / 10) % 10;
    display_digits[5] = gradientPercent % 10;

    return 0;
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

    warpPrint("\n\n\n\rBooting\n\n");
    OSA_TimeDelay(1000);

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
    int8_t Z_OFFSET = ((2098-zeroRateAccelZ)/8) * 1;
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1, 0x00); // Standby
    writeSensorRegisterMMA8451Q(0x2F, X_OFFSET); // x_OFFSET
    writeSensorRegisterMMA8451Q(0x30, Y_OFFSET); // Y_OFFSET
    writeSensorRegisterMMA8451Q(0x31, Z_OFFSET); // Z_OFFSET
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1, val_MMA8451Q_CTRL_REG1); // Active

    clearScreen();

    // Print output of MMA8451Q indefinitely (for evaluation/debug purposes)
    if (WARP_BUILD_BOOT_TO_ACCELSTREAM) {
        warpPrint("xAccel, yAccel, zAccel");
        while (1) {
            readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
            statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

            if ((statusRegisterValueAccel & 0b11000000) > 0) {

                returnSensorDataMMA8451QFIFO(readingsMMA8451QFIFO, kWarpSizesI2cBufferBytesMMA8451Q);

                for (i = 0; i < nSamplesMMA8451Q; i++) {
                    warpPrint("%d, %d, %d\n", readingsMMA8451QFIFO[i], readingsMMA8451QFIFO[i + 1],
                              readingsMMA8451QFIFO[i + 2]);
                }
            }
        }
    }

    // Print output of L3GD20H indefinitely (for evaluation/debug purposes)
    if (WARP_BUILD_BOOT_TO_GYROSTREAM) {
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

    drawPoint(35, 25, 8, 255);

    volatile uint8_t statusRegisterPulse;
    volatile uint8_t statusRegisterInt;
    volatile int16_t Accel;
    volatile int16_t currAvgGyro = 0;
    volatile int16_t prevAvgGyro = 0;
    volatile int16_t currDerivGyro = 0;
    volatile int16_t prevDerivGyro = 0;
    uint8_t reg_val;

    timeStart = OSA_TimeGetMsec();

    tap = false;
    dataReady = false;

    while (1) {

        if (tap) {
            tap = false;
            tapCount++;
            tapCount %= 3;
            warpPrint("TAP\n");
            warpPrint("%d\n", statusRegisterPulse);
        }

        readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1);
        reg_val = deviceL3GD20HState.i2cBuffer[0];
        //warpPrint("Reg1: %d\n", reg_val); // Clear interrupt flag

        if (dataReady) {
            dataReady = false;
            warpPrint("READY\n");
        }
        if (((reg_val & 0b1000000) == 0b1000000) || ((reg_val & 0b0100000) == 0b0100000) || (reg_val == 32)) {

            // Get average value of angular velocity from values in FIFO (default 32)
            returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);

            for (i = 0; i < nSamplesL3GD20H; i++) {
                currAvgGyro = iterativeAvg(currAvgGyro, readingsL3GD20HFIFO[i], i+1);
            }

            currDerivGyro = (currDerivGyro + currAvgGyro - prevAvgGyro) / 2;
            prevAvgGyro = currAvgGyro;

            sampleCounter++;
            sampleCounter %= 3;
            if (sampleCounter == 0) {
                prevDerivGyro = currDerivGyro;
            }

            if (sampleCounter == 2) {
                if (((currDerivGyro > 0) && (prevDerivGyro < 0)) || ((currDerivGyro < 0) && (prevDerivGyro > 0))) {
                    if (((currDerivGyro-prevDerivGyro)*(currDerivGyro-prevDerivGyro)) > 100) {
                        strokeCount++;
                        warpPrint("!!!!!!! %d\n", strokeCount);
                    }
                }
            }
        }

        switch(tapCount) {

            case 1: // Gradient only
                Accel = xAccel;
                break;

            case 2: // Cadence only
                Accel = yAccel;
                break;

            default: // Both
                Accel = zAccel;
                break;
        }

        currTime = OSA_TimeGetMsec();

        if (currTime - timeStart >= 1000) {

            readSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_SRC, 1); // Clear interrupt flag

            time ++;
            time %= 6;
            //strokeCount3 += strokeCount;
            //strokeCount12 += strokeCount;
            //if (time == 0) {
                //cadence = ((5 * strokeCount12) + (20 * strokeCount3)) / 2;
                //strokeCount12 = 0;
                //strokeCount12 = iterativeAvg(strokeCount12, strokeCount3, )
                //warpPrint("CADENCE: %d\n", cadence);
            //}
            if (time == 0) {
                strokeCount6 -= strokeCountPrev;
            }
            if (time % 3 == 0) {
                strokeCount6 += strokeCount;
                cadence = ((5 * strokeCount6) + (20 * strokeCount)) / 2;
                strokeCountPrev = strokeCount;
                warpPrint("CADENCE: %d\n", cadence);
                strokeCount = 0;
            }

            readSensorRegisterMMA8451Q(reg_MMA8451Q_STATUS, 1);
            statusRegisterValueAccel = deviceMMA8451QState.i2cBuffer[0];

            // Update screen every 2 secs
            if (printIndicator == 1) {

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

                convertFromRawMMA8451Q(Accel, digits);

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
            }
            printIndicator *= -1;
            timeStart = currTime;
        }
    }
    return 0;
}

//warpPrint("PrevAvg: %d\n", prevAvgGyro);
//warpPrint("CurrAvg: %d\n", currAvgGyro);
//warpPrint("PrevDeriv: %d\n", prevDerivGyro);
//warpPrint("CurrDeriv: %d\n", currDerivGyro);
//warpPrint("SampleCounter: %d\n", sampleCounter);

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

            volatile uint8_t statusRegisterPulse;
            volatile uint8_t statusRegisterInt;
            readSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_SRC, 1); // Clear interrupt flag
            statusRegisterPulse = deviceMMA8451QState.i2cBuffer[0];
            readSensorRegisterMMA8451Q(reg_MMA8451Q_INT_SOURCE, 1);
            statusRegisterInt = deviceMMA8451QState.i2cBuffer[0];
 */

/*
j = 0;
while (j < 100) {

    readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1);
    statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];
    warpPrint("Gyro %d\n", statusRegisterValueGyro);
    if ((statusRegisterValueGyro & 0b11000000) > 0) {

        returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);
        k = 0;

        for (i = 0; i < nSamplesL3GD20H; i++) {
            //zeroRateGyroX = iterativeAvg(zeroRateGyroX, readingsL3GD20HFIFO[k], j + i + 1);
            zeroRateGyroY = iterativeAvg(zeroRateGyroY, readingsL3GD20HFIFO[k + 1], j + i + 1);
            //zeroRateGyroZ = iterativeAvg(zeroRateGyroZ, readingsL3GD20HFIFO[k + 2], j + i + 1);
            k += 3;
        }
        j += nSamplesL3GD20H;
    }
}
*/
/*
 *     //warpPrint("ZERO: %d, %d, %d\n", zeroRateAccelX, zeroRateAccelY, zeroRateAccelZ);
    //warpPrint("OFFSETS: %d, %d, %d\n", X_OFFSET, Y_OFFSET, Z_OFFSET);
 */

/*
readSensorRegisterL3GD20H(reg_L3GD20H_FIFO_SRC, 1); // FIFO
statusRegisterValueGyro = deviceL3GD20HState.i2cBuffer[0];

if ((statusRegisterValueGyro & 0b11000000) > 0) {

    returnSensorDataL3GD20HFIFO(readingsL3GD20HFIFO, kWarpSizesI2cBufferBytesL3GD20H);
    j = 0;
    for (i = 0; i < nSamplesL3GD20H; i++) {
        sampleCounter += 1;
        //warpPrint("%u, %d, %d, %d\n", sampleCount, readingsL3GD20HFIFO[j], readingsL3GD20HFIFO[j+1], readingsL3GD20HFIFO[j+2]);
        j+=3;
    }
    xGyro = readingsL3GD20HFIFO[0];
    yGyro = readingsL3GD20HFIFO[1];
    zGyro = readingsL3GD20HFIFO[2];
}
 */