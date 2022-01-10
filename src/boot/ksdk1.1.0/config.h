/*
	Authored 2021, Phillip Stanley-Marbell.

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

#define		WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF		1
#define		WARP_BUILD_BOOT_TO_CSVSTREAM			0
#define		WARP_BUILD_BOOT_TO_VLPR				0
#define		WARP_BUILD_DISABLE_SUPPLIES_BY_DEFAULT		0

#define		WARP_BUILD_ENABLE_FRDMKL03			1

#define		WARP_BUILD_ENABLE_DEVL3GD20H		1
#define		WARP_BUILD_ENABLE_DEVMMA8451Q		1


typedef enum
{
	/*
	 *	Speeds
	 */
	kWarpDefaultI2cBaudRateKbps				= 400,
	kWarpDefaultUartBaudRateBps				= 115200,
	kWarpDefaultSpiBaudRateKbps				= 10000,

	/*
	 *	Times
	 */
	kWarpDefaultSleeptimeSeconds				= 0,
	kWarpDefaultI2cTimeoutMilliseconds			= 10,
	kWarpDefaultUartTimeoutMilliseconds			= 1000,
	kWarpDefaultSpiTimeoutMicroseconds			= 5,
	kWarpDefaultMenuPrintDelayMilliseconds			= 10,
	kWarpDefaultSupplySettlingDelayMilliseconds		= 1,

	/*
	 *	Sizes
	*/
	kWarpDefaultPrintBufferSizeBytes			= 64,
	kWarpMemoryCommonSpiBufferBytes				= 64,
	kWarpSizesI2cBufferBytes				= 192, // 6x the FIFO sample watermark
	kWarpSizesSpiBufferBytes				= 7,
	kWarpSizesUartBufferBytes				= 8,

	/*
	 *	Voltages
	 */
	kWarpDefaultSupplyVoltageMillivolts			= 3000,
	kWarpDefaultSupplyVoltageMillivoltsMMA8451Q		= 3000,
	kWarpDefaultSupplyVoltageMillivoltsL3GD20H		= 3000,

    /*
     *  Configuration registers
     */
    kWarpRegisterCTRL1ValueL3GD20H = 0b11111111,  // ODR 800Hz, 100Hz cut-off, see table 21, normal mode, x,y,z enable
    kWarpRegisterCTRL2ValueL3GD20H = 0b00100000,
    kWarpRegisterCTRL5ValueL3GD20H = 0b01100000,//0b00000000,  // normal mode, disable FIFO, disable high pass filter
    kWarpRegisterFIFO_CTRLValueL3GD20H = 0b11001000,

    kWarpRegisterF_SETUPValueMMA8451Q = 0b01000000,//0b01001000,   // Enable FIFO, 8 sample watermark (8 best so far)
    kWarpRegisterCTRL_REG1ValueMMA8451Q = 0b00010101,//0x05,   // Normal read 14-bit, 800Hz, low noise (limited to +/-4g), active mode
    kWarpRegisterXYZ_DATA_CFGValueMMA8451Q = 0x01,  // No high-pass filter, +/-4g range

} WarpDefaults;
