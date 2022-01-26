#include "fsl_spi_master_driver.h"

#define	min(x,y)	((x) < (y) ? (x) : (y))
#define	max(x,y)	((x) > (y) ? (x) : (y))
#define	USED(x)		(void)(x)

/*
 *	On Glaux, we use PTA0/IRQ0/LLWU_P7 (SWD_CLK) as the interrupt line
 *	for the RV8803C7 RTC. The original version of this function for the
 *	FRDMKL03 was using PTB0.
 *
 *	The following taken from KSDK_1.1.0//boards/frdmkl03z/board.h. We
 *	don't include that whole file verbatim since we have a custom board.
 */
typedef enum
{
	kWarpStatusOK			= 0,

	kWarpStatusDeviceNotInitialized,
	kWarpStatusDeviceCommunicationFailed,
	kWarpStatusBadDeviceCommand,
	kWarpStatusCommsError,

	kWarpStatusMax
} WarpStatus;


typedef enum
{
	kWarpSensorMMA8451Q,
	kWarpSensorL3GD20H,
	kWarpSensorINA219,
} WarpSensorDevice;


typedef enum
{
	kWarpModeDisableAdcOnSleep		= (1 << 0),
} WarpModeMask;


typedef enum
{

    reg_L3GD20H_CTRL_1 = 0x20,
    reg_L3GD20H_CTRL_2 = 0x21,
    reg_L3GD20H_CTRL_3 = 0x22,
    reg_L3GD20H_CTRL_4 = 0x23,
    reg_L3GD20H_CTRL_5 = 0x24,
    reg_L3GD20H_FIFO_CTRL = 0x2E,
    reg_L3GD20H_FIFO_SRC = 0x2F, // Read-only
    reg_L3GD20H_LOW_ODR = 0x39, // Read-only

    reg_MMA8451Q_STATUS = 0x00,
    reg_MMA8451Q_INT_SOURCE = 0x0C, // Read-only
    reg_MMA8451Q_CTRL_REG1 = 0x2A,
    reg_MMA8451Q_CTRL_REG2 = 0x2B,
    reg_MMA8451Q_CTRL_REG3 = 0x2C,
    reg_MMA8451Q_CTRL_REG4 = 0x2D,
    reg_MMA8451Q_CTRL_REG5 = 0x2E,
    reg_MMA8451Q_F_SETUP = 0x09,
    reg_MMA8451Q_TRIG_CFG = 0x0A,
    reg_MMA8451Q_XYZ_DATA_CFG = 0x0E,
    reg_MMA8451Q_PULSE_CFG = 0x21,
    reg_MMA8451Q_PULSE_SRC = 0x22, // Read-only
    reg_MMA8451Q_PULSE_THSX = 0x23,
    reg_MMA8451Q_PULSE_THSY = 0x24,
    reg_MMA8451Q_PULSE_THSZ = 0x25,
    reg_MMA8451Q_PULSE_TMLT = 0x26,
    reg_MMA8451Q_PULSE_LTCY = 0x27,
    reg_MMA8451Q_HP_FILTER_CUTOFF = 0x0F,

} WarpSensorConfigurationRegister;


typedef enum
{
	kWarpSensorOutputRegisterMMA8451QOUT_X_MSB			= 0x01,
	kWarpSensorOutputRegisterMMA8451QOUT_X_LSB			= 0x02,
	kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB			= 0x03,
	kWarpSensorOutputRegisterMMA8451QOUT_Y_LSB			= 0x04,
	kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB			= 0x05,
	kWarpSensorOutputRegisterMMA8451QOUT_Z_LSB			= 0x06,

	kWarpSensorOutputRegisterL3GD20HOUT_TEMP			= 0x26,
	kWarpSensorOutputRegisterL3GD20HOUT_X_L				= 0x28,
	kWarpSensorOutputRegisterL3GD20HOUT_X_H				= 0x29,
	kWarpSensorOutputRegisterL3GD20HOUT_Y_L				= 0x2A,
	kWarpSensorOutputRegisterL3GD20HOUT_Y_H				= 0x2B,
	kWarpSensorOutputRegisterL3GD20HOUT_Z_L				= 0x2C,
	kWarpSensorOutputRegisterL3GD20HOUT_Z_H				= 0x2D,

} WarpSensorOutputRegister;


typedef enum
{
	kWarpMiscMarkerForAbsentByte					= 0xFF,
} WarpMisc;


typedef struct
{
	bool			isInitialized;

	/*
	 *	For holding the SPI CS I/O pin identifier to make
	 *	the driver independent of board config.
	 */
	int			chipSelectIoPinID;

	uint8_t *		spiSourceBuffer;
	uint8_t *		spiSinkBuffer;
	size_t			spiBufferLength;
	uint16_t		operatingVoltageMillivolts;
} WarpSPIDeviceState;


typedef struct
{
	uint8_t			errorCount;
} WarpPowerManagerCallbackStructure;


typedef struct
{
    uint8_t     i2cAddress;
    uint8_t     i2cBuffer[kWarpSizesI2cBufferBytesMMA8451Q];
    uint16_t	operatingVoltageMillivolts;
} MMA8451Q;


typedef struct
{
    uint8_t     i2cAddress;
    uint8_t     i2cBuffer[kWarpSizesI2cBufferBytesL3GD20H];
    uint16_t	operatingVoltageMillivolts;
} L3GD20H;


typedef struct
{
    uint8_t     i2cAddress;
    uint8_t     i2cBuffer[kWarpSizesI2cBufferBytesINA219];
    uint16_t	operatingVoltageMillivolts;
} INA219;


void		warpEnableI2Cpins(void);

void		warpEnableSPIpins(void);

void		warpPrint(const char *fmt, ...);

