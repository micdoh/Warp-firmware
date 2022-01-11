#include <stdint.h>

#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];
volatile uint8_t	commandBytes[11];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 11),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOB, 1),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(2);  //Reduce delay to speed writes
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

static int
writeMultipleCommand(uint8_t * commandBytes, uint8_t transferSize)
{

    /*
     *	Drive /CS low.
     *
     *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
     */
    GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
    OSA_TimeDelay(2);  //Reduce delay to speed writes
    GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

    /*
     *	Drive DC low (command).
     */
    GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

    int i;
    for (i = 0; i < transferSize; ++i)
    {
        SPI_DRV_MasterTransferBlocking(0	/* master instance */,
                NULL		/* spi_master_user_config_t */,
                (const uint8_t * restrict)&commandBytes[i],
                (uint8_t * restrict)&inBuffer[0],
                1		/* transfer size */,
                1000		/* timeout in microseconds (unlike I2C which is ms) */);
    }
    /*
     *	Drive /CS high
     */
    GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return 0;
}



int
initSSD1331(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n\n");


	return 0;
}

int
clearScreen(void){
    uint8_t commands[5] = {
            kSSD1331CommandCLEAR, 0x00, 0x00, 0x5F, 0x3F
    };
    writeMultipleCommand(commands, 5);
    return 0;
}

int
drawChar(uint8_t startCol, uint8_t startRow, uint8_t brightness, uint8_t * lines, uint8_t nLines, uint8_t weight){
    int j = 0;
    int i;
    int offset;
    uint8_t commands[nLines*8*weight];
    for (offset = 0; offset < weight; offset++) {
        for (i = 0; i < nLines*4; i+=4) {
            commands[j] = kSSD1331CommandDRAWLINE;
            commands[j+1] = startCol + lines[i] + offset;
            commands[j+2] = startRow - lines[i+1] + offset;
            commands[j+3] = startCol + lines[i+2] + offset;
            commands[j+4] = startRow - lines[i+3] + offset;
            commands[j+5] = brightness;
            commands[j+6] = brightness;
            commands[j+7] = brightness;
            j+=8;
        }
    }
    writeMultipleCommand(commands, nLines*8*weight);
    return 0;
}

int
drawPoint(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[11] = {
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale-(scale/4), startCol+(scale/4), startRow+scale+scale,
            brightness, brightness, brightness, brightness, brightness, brightness,
    };
    writeMultipleCommand(commands, 11);
    return 0;
}

int
drawMinus(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[11] = {
            kSSD1331CommandDRAWRECT, startCol+1, startRow+scale-1, startCol+scale-1, startRow+scale+1,
            brightness, brightness, brightness, 0, 0, 0,
    };
    writeMultipleCommand(commands, 11);
    return 0;
}

int
draw0(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[66] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1,
            brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0, // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // All right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    };
    writeMultipleCommand(commands, 66);
    return 0;
}

int
draw1(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[33] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    };
    writeMultipleCommand(commands, 33);
    return 0;
}

int
draw2(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale+1, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Top right
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // Bottom left
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw3(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[77] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // All right
    writeMultipleCommand(commands, 77);
    return 0;
}

int
draw4(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[66] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+2, startRow-1, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0, // Top left
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // All right
    writeMultipleCommand(commands, 66);
    return 0;
}

int
draw5(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale+1, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0, // Top left
            kSSD1331CommandDRAWRECT, startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // Bottom right
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw6(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale+1, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // All left
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw7(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[44] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // All right
    writeMultipleCommand(commands, 44);
    return 0;
}

int
draw8(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // All right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    }; // All left
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw9(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, brightness, brightness, brightness, brightness, brightness, brightness,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0
    };
    writeMultipleCommand(commands, 88);
    return 0;
}

int
drawGlyph(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t brightness, uint8_t glyph) {

    switch(glyph)
    {
        case 0:
            draw0(startCol, startRow, scale, brightness);
            break;
        case 1:
            draw1(startCol, startRow, scale, brightness);
            break;
        case 2:
            draw2(startCol, startRow, scale, brightness);
            break;
        case 3:
            draw3(startCol, startRow, scale, brightness);
            break;
        case 4:
            draw4(startCol, startRow, scale, brightness);
            break;
        case 5:
            draw5(startCol, startRow, scale, brightness);
            break;
        case 6:
            draw6(startCol, startRow, scale, brightness);
            break;
        case 7:
            draw7(startCol, startRow, scale, brightness);
            break;
        case 8:
            draw8(startCol, startRow, scale, brightness);
            break;
        case 9:
            draw9(startCol, startRow, scale, brightness);
            break;
        default:
            break;
    }
    return 0;
}
