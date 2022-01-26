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

#define		WARP_BUILD_ENABLE_FRDMKL03			1

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
	kWarpDefaultPrintBufferSizeBytes		= 64,
	kWarpMemoryCommonSpiBufferBytes			= 64,
    kWarpSizesI2cBufferBytesINA219			= 2,  // Default
    kWarpSizesI2cBufferBytesMMA8451Q		= 6,//48, // 6x the FIFO sample watermark
    kWarpSizesI2cBufferBytesL3GD20H         = 64,//192,//48, //192, // 8 samples
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
    val_L3GD20H_CTRL_1 = 0b00001010, // ODR 100Hz, 20Hz LPF cut-off, see table 21, normal mode, y enable
    //val_L3GD20H_CTRL_1 = 0b10001010, // ODR 400Hz, 20Hz LPF cut-off, see table 21, normal mode, y enable
    val_L3GD20H_CTRL_2 = 0b00000000, // No HPF
    val_L3GD20H_CTRL_3 = 0b00000010, // FIFO overrun interrupt on DRDY pin
    val_L3GD20H_CTRL_4 = 0b10000000, // Block data update, 245 dps scale
    val_L3GD20H_CTRL_5 = 0b01001010, // Enable FIFO, enable LPF2 (20Hz cut-off), no threshold
    val_L3GD20H_FIFO_CTRL = 0b01000000, // FIFO stream mode, 32 sample threshold (default)
    val_L3GD20H_LOW_ODR = 0b00000001, // Low speed Output Data Rate enabled

    val_MMA8451Q_CTRL_REG1 = 0b00001101, // 400Hz, low noise (limited to +/-4g), active mode
    val_MMA8451Q_CTRL_REG2 = 0b00000000, // Normal power mode
    val_MMA8451Q_CTRL_REG3 = 0b00000000, // No interrupt waking from sleep
    //val_MMA8451Q_CTRL_REG3 = 0b00000011, // Interrupt HIGH and OPEN DRAIN
    val_MMA8451Q_CTRL_REG4 = 0b00001000, // Enable pulse interrupt
    val_MMA8451Q_CTRL_REG5 = 0b00000000, // Interrupts to INT2
    val_MMA8451Q_F_SETUP = 0b01000110, // FIFO circular buffer, 6 samples threshold
    val_MMA8451Q_TRIG_CFG = 0b00000000, // No FIFO trigger
    val_MMA8451Q_XYZ_DATA_CFG = 0b00000001,  // No high-pass filter, +/-4g range
    val_MMA8451Q_PULSE_CFG = 0b01010000, // Single pulse enabled for z axis. Pulse event flags are latched into the PULSE_SRC register. Reading of the PULSE_SRC register clears the event flag.
    val_MMA8451Q_PULSE_THSX = 0x19, // Set X Threshold to 1.575g (LSB is 8g/127 = 0.063g)
    val_MMA8451Q_PULSE_THSY = 0x19, // 0x019 Set Y Threshold to 1.575g
    val_MMA8451Q_PULSE_THSZ = 0x2F, // Set Z Threshold to 2.961g (LSB is 8g/127 = 0.063g)
    val_MMA8451Q_PULSE_TMLT = 0x50, // 50ms (80 x 0.625ms) time limit for pulse to exceed threshold and drop below
    val_MMA8451Q_HP_FILTER_CUTOFF = 0b00010000, // Enable LPF for pulse detection. Pulse time limit step set to 0.625ms (otherwise 2.5ms)
    val_MMA8451Q_PULSE_LTCY = 0x50, // 300ms window until next pulse detected

} WarpDefaults;
