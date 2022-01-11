/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

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
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
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

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


void
initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress			= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	//warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1, uint8_t payloadXYZ_DATA_CFG)
{
    WarpStatus	status;

    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1,val_MMA8451Q_CTRL_REG1);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG2,val_MMA8451Q_CTRL_REG2);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG3,val_MMA8451Q_CTRL_REG3);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG4,val_MMA8451Q_CTRL_REG4);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG5,val_MMA8451Q_CTRL_REG5);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_F_SETUP,val_MMA8451Q_F_SETUP);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_TRIG_CFG,val_MMA8451Q_TRIG_CFG);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_XYZ_DATA_CFG,val_MMA8451Q_XYZ_DATA_CFG);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_CFG,val_MMA8451Q_PULSE_CFG);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_THSX,val_MMA8451Q_PULSE_THSX);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_THSY,val_MMA8451Q_PULSE_THSY);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_THSZ,val_MMA8451Q_PULSE_THSZ);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_TMLT,val_MMA8451Q_PULSE_TMLT);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_PULSE_LTCY,val_MMA8451Q_PULSE_LTCY);
    writeSensorRegisterMMA8451Q(reg_MMA8451Q_HP_FILTER_CUTOFF,val_MMA8451Q_HP_FILTER_CUTOFF);
    status = writeSensorRegisterMMA8451Q(reg_MMA8451Q_CTRL_REG1,val_MMA8451Q_CTRL_REG1);

    return status;
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMMA8451QState.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
        warpPrint("MMA8451Q read failed\n");
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMMA8451Q(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	//warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	//i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[2];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[3];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	//i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[4];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[5];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
}

/*
int
returnSensorDataMMA8451Q(int16_t * readings)
{
    uint16_t	    readSensorRegisterValueLSB;
    uint16_t	    readSensorRegisterValueMSB;
    int16_t		    readSensorRegisterValueCombined;
    WarpStatus	    i2cReadStatus;
    int             i;
    int             j = 0;

    warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 6);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint("MMA8451Q read failed\n");
    }
    for (i = 0; i < 6; i+=2) {
        readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[i];
        readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[i+1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
        //	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
        readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
        readings[j] = readSensorRegisterValueCombined;
        j++;
    }

    return 0;
}
*/

int
returnSensorDataMMA8451QFIFO(int16_t * readings, uint8_t nBytes)
{
    uint16_t	    readSensorRegisterValueLSB;
    uint16_t	    readSensorRegisterValueMSB;
    int16_t		    readSensorRegisterValueCombined;
    WarpStatus	    i2cReadStatus;
    int             i;
    int             j = 0;


    i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, nBytes);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint("MMA8451Q read failed\n");
    }
    for (i = 0; i < nBytes; i+=2) {
        readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[i];
        readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[i+1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);
        //	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
        readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);
        readings[j] = readSensorRegisterValueCombined;
        j++;
    }

    return 0;
}



int
convertFromRawMMA8451Q(int16_t raw, uint8_t * digits) {
    uint8_t sign = 0;
    uint8_t digit;
    uint16_t fracDigits = 0;
    uint16_t frac = 5000;
    int i;

    if (raw < 0) {
        sign = 1;  // Indicates negative number
        raw &= 0xFFFC;
        raw = (~raw) + 1;
    }
    
        raw = raw << 1;
        digit = (raw & 0x7000) >> 13; // Get first digit
        raw = raw << 3;
        
    for (i = 0; i < 12; i++) {
        if ((raw & 0x8000) == 0x8000) {
            fracDigits += frac;
        }
        frac = (frac+1) / 2;
        raw = raw << 1;
    }
    
    digits[0] = sign;
    digits[1] = digit;
    digits[2] = (fracDigits / 1000) % 10;
    digits[3] = (fracDigits / 100) % 10;
    digits[4] = (fracDigits / 10) % 10;
    digits[5] = fracDigits % 10;

    return 0;
}