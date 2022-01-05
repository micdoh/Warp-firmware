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
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
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
    spi_status_t status;
    warpPrint("%d\n", transferSize);

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
    warpPrint("Starting write\n");
    for (i = 0; i < transferSize; ++i)
    {
        status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
                NULL		/* spi_master_user_config_t */,
                (const uint8_t * restrict)&commandBytes[i],
                (uint8_t * restrict)&inBuffer[0],
                1		/* transfer size */,
                1000		/* timeout in microseconds (unlike I2C which is ms) */);
        warpPrint("%d\n", i);

    }
    /*
 *	Drive /CS high
 */
    GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

    return status;
}



int
devSSD1331init(void)
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
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
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
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");


	return 0;
}

int
clearScreen(void){
    uint8_t commands[5] = {
            kSSD1331CommandCLEAR, 0x00, 0x00, 0x5F, 0x3F};
    writeMultipleCommand(commands, 5);
    return 0;
}

int
draw0(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[66] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0, // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // All right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0};
    writeMultipleCommand(commands, 66);
    return 0;
}

int
draw1(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[22] = {
            kSSD1331CommandDRAWRECT, startCol+scale-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0};
    writeMultipleCommand(commands, 22);
    return 0;
}

int
draw2(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale+1, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Top right
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // Bottom left
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw3(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[77] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,}; // All right
    writeMultipleCommand(commands, 77);
    return 0;
}

int
draw4(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[66] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow-1, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0, // Top left
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All right
    writeMultipleCommand(commands, 66);
    return 0;
}

int
draw5(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale+1, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0, // Top left
            kSSD1331CommandDRAWRECT, startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // Bottom right
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw6(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale+1, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All left
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw7(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[44] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All right
    writeMultipleCommand(commands, 44);
    return 0;
}

int
draw8(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // All right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All left
    writeMultipleCommand(commands, 88);
    return 0;
}

int
draw9(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b){
    uint8_t commands[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0};
    writeMultipleCommand(commands, 88);
    return 0;
}

/*
int
devSSD1331drawGlyph(uint8_t startCol, uint8_t startRow, uint8_t scale, uint8_t r, uint8_t g, uint8_t b, uint8_t glyph) {

    //uint8_t * commands;
    uint8_t commands0[66] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0, // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // All right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0};
    uint8_t commands1[22] = {
            kSSD1331CommandDRAWRECT, startCol+scale-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0};
    uint8_t commands2[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale+1, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Top right
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // Bottom left
    uint8_t commands3[77] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,}; // All right
    uint8_t commands4[66] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow-1, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0, // Top left
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All right
    uint8_t commands5[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale+1, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0, // Top left
            kSSD1331CommandDRAWRECT, startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // Bottom right
    uint8_t commands6[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale+1, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,  // Top
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0, // Middle
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom
            kSSD1331CommandDRAWRECT, startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // Bottom right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All left
    uint8_t commands7[44] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+2, startCol+scale-2, startRow+scale+scale+1, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All right
    uint8_t commands8[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0, // All right
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0, 0, 0, 0}; // All left
    uint8_t commands9[88] = {
            kSSD1331CommandDRAWRECT, startCol-1, startRow-1, startCol+scale+1, startRow+scale+scale+1, r, g, b, r, g, b,
            kSSD1331CommandDRAWRECT, startCol+2, startRow+2, startCol+scale-2, startRow+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol-1, startRow+scale+2, startCol+scale-2, startRow+scale+scale-2, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol+scale, startRow, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol, startRow, startCol, startRow+scale, 0, 0, 0, 0, 0, 0,
            kSSD1331CommandDRAWRECT, startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0, 0, 0, 0};
    drawRect(0, 0, 95, 63, 255, 0 ,0);
    switch(glyph)
    {
        case 0:
            warpPrint("writing 0");
            //commands = commands0(startCol, startRow, scale, r, g, b);
            writeMultipleCommand(commands0, 66);
            break;
        case 1:
            warpPrint("writing 1");
            writeMultipleCommand(commands1, 22);
            break;
        case 2:
            writeMultipleCommand(commands2, 88);
            break;
        case 3:
            //writeMultipleCommand(commands3, 77);
            break;
        case 4:
            //writeMultipleCommand(commands4, 66);
            break;
        case 5:
            //writeMultipleCommand(commands5, 88);
            break;
        case 6:
            //writeMultipleCommand(commands6, 88);
            break;
        case 7:
            //writeMultipleCommand(commands7, 44);
            break;
        case 8:
            //writeMultipleCommand(commands8, 88);
            break;
        case 9:
            //writeMultipleCommand(commands9, 88);
            break;
    }
    return 0;
}


int
drawRect(uint8_t startCol, uint8_t startRow, uint8_t endCol, uint8_t endRow, uint8_t r, uint8_t g, uint8_t b) {
    /*
     *	Initialise the

uint8_t commands[11]; /*= {kSSD1331CommandDRAWRECT, startCol, startRow, endCol, endRow, r, g, b, r, g, b};
*//*
commands[0] = kSSD1331CommandDRAWRECT;
commands[1] = startCol;
commands[2] = startRow;
commands[3] = endCol;
commands[4] = endRow;
commands[5] = r;
commands[6] = g;
commands[7] = b;
commands[8] = r;
commands[9] = g;
commands[10] = b;
warpPrint("drawing rect\n");
//writeMultipleCommand(commands, 11);

writeCommand(kSSD1331CommandDRAWRECT);  // Initiate "draw rectangle" mode
writeCommand(startCol);              // Start column
writeCommand(startRow);              // Start row
writeCommand(endCol);                // End column
writeCommand(endRow);                // End row
writeCommand(r);                     // Outline blue contrast
writeCommand(g);                     // Outline green contrast (max)
writeCommand(b);                     // Outline red contrast
writeCommand(r);                     // Fill blue contrast
writeCommand(g);                     // Fill green contrast
writeCommand(b);                     // Fill red contrast

return 0;
}

int
drawLine(uint8_t startCol, uint8_t startRow, uint8_t endCol, uint8_t endRow, uint8_t r, uint8_t g, uint8_t b) {
    /*
     *	Initialise the
     */
/*
    writeCommand(kSSD1331CommandDRAWLINE);  // Initiate "draw line" mode
    writeCommand(startCol);              // Start column
    writeCommand(startRow);              // Start row
    writeCommand(endCol);                // End column
    writeCommand(endRow);                // End row
    writeCommand(r);                     // Outline red contrast
    writeCommand(g);                     // Outline green contrast (max)
    writeCommand(b);                     // Outline blue contrast

    return 0;
}



/*
case 8:
drawRect(startCol-1, startRow-1, startCol+scale+1, startRow+1, r, g, b); // ^-
drawRect(startCol-1, startRow+scale-1, startCol+scale+1, startRow+scale+1, r, g, b); // -
drawRect(startCol-1, startRow+scale+scale-1, startCol+scale+1, startRow+scale+scale+1, r, g, b); // _
drawRect(startCol-1, startRow-1, startCol+1, startRow+scale+1, r, g, b); // ^<|
drawRect(startCol+scale-1, startRow-1, startCol+scale+1, startRow+scale+1, r, g, b); // ^>|
drawRect(startCol-1, startRow+scale-1, startCol+1, startRow+scale+scale+1, r, g, b); // <|
drawRect(startCol+scale-1, startRow+scale-1, startCol+scale+1, startRow+scale+scale+1, r, g, b); //>|

drawRect(startCol, startRow, startCol+scale, startRow, 0, 0, 0);  // Top
drawRect(startCol, startRow+scale, startCol+scale, startRow+scale, 0, 0, 0); // Middle
drawRect(startCol, startRow+scale+scale, startCol+scale, startRow+scale+scale, 0, 0, 0); // Bottom
drawRect(startCol, startRow, startCol, startRow+scale, 0, 0, 0); // Top left
drawRect(startCol+scale, startRow, startCol+scale, startRow+scale, 0, 0, 0); // Top right
drawRect(startCol, startRow+scale, startCol, startRow+scale+scale, 0, 0, 0); // Bottom left
drawRect(startCol+scale, startRow+scale, startCol+scale, startRow+scale+scale, 0, 0, 0); // Bottom right
drawRect(startCol+scale, startRow, startCol+scale, startRow+scale+scale, 0, 0, 0); // All right
drawRect(startCol, startRow, startCol, startRow+scale+scale, 0, 0, 0); // All left

break;

*/

