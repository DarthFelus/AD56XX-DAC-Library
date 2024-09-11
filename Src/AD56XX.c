/*
 * AD5623R.c
 *
 *  Created on: Jul 15, 2024
 *      Author: d.arseniuk
 *		ver 1.0.3 23.07.2024
 */

/*
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Library for using in STM32 series microcontroller
 *
 *
 *
 * Analog Devices:
 * Low power, smallest pin-compatible, dual nanoDAC
 * 		@ AD5663R: 16 bits
 *		@ AD5643R: 14 bits
 *		@ AD5623R: 12 bits
 *
 *
 * Datasheet link: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5623R_43R_63R.pdf
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */



#include <AD56XX.h>

/*
 * Function to initialize the DAC
 *
 * Setting SPI line and GPIO configuration
 * 		 @ had: pointer to specified DAC information
 *       @ hspi: pointer to specified SPI parameters. MOSI (DIN) - serial data input; SCLK - serial clock input; SS (SYNC) - chip select
 *       @ LDAC: GPIO port and pin
 *       @ CLR: GPIO port and pin
 *       @ Model: DAC model to be used
 */
void AD56XX_Init(AD56XX_HandleTypeDef *had, SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx_SYNC, uint16_t GPIO_Pin_SYNC,
		GPIO_TypeDef *GPIOx_LDAC, uint16_t GPIO_Pin_LDAC, GPIO_TypeDef *GPIOx_CLR,
		uint16_t GPIO_Pin_CLR, AD56XX_Model model)
{

	had->hspi = hspi;
	had->sync_gpio_port = GPIOx_SYNC;
	had->sync_gpio_pin = GPIO_Pin_SYNC;

	had->ldac_gpio_port = GPIOx_LDAC;
	had->ldac_gpio_pin = GPIO_Pin_LDAC;

	had->clr_gpio_port = GPIOx_CLR;
	had->clr_gpio_pin = GPIO_Pin_CLR;


	had->model = model;

/*
 *	Set LDAC and CLR to their initial state
 *	Set LDAC as LOW to allow any DAC register to be updated if input have new data; While LDAC is HIGH
 *	Set CLR as HIGH to clear all input and DAC registers, this clears the output to 0 V; When CLR is LOW all LDAC pulses ignores.
 */
	AD56XX_ControlLDAC(had, GPIO_PIN_RESET);	//LDAC as LOW at start. When LDAC is HIGH then DAC Register is updated
	AD56XX_ControlCLR(had, GPIO_PIN_SET);		//CLR as HIGH at start. Indicates that DAC Register is cleared
}

/*
 *	Function to transmit 24-bit data to the DAC via SPI interface
 *		@had: pointer to specified DAC information
 *		@data: data to transmit
 */
void AD56XX_Transmit(AD56XX_HandleTypeDef *had, uint32_t data)
{
	uint8_t tx_buffer[3];	//buffer to hold 24-bit data

	/*
	 * Serialization of data by 8 bit
	 */
	tx_buffer[0] = (data >> 16) & 0xFF;
	tx_buffer[1] = (data >> 8) & 0xFF;
	tx_buffer[2] = data & 0xFF;


	HAL_GPIO_WritePin(had->sync_gpio_port, had->sync_gpio_pin, GPIO_PIN_RESET); //Pull SYNC (SS) to LOW

	HAL_SPI_Transmit(had->hspi, tx_buffer, sizeof(tx_buffer), HAL_MAX_DELAY);	//SPI Transmit function
	HAL_GPIO_WritePin(had->sync_gpio_port, had->sync_gpio_pin, GPIO_PIN_SET);
}
/*
 * Function to control the state of LDAC pin
 * 		@had: pointer to specified DAC information
 * 		@state: LDAC pin state
 *
 * Access to the DAC registers is controlled by the LDAC pin.
 * When the LDAC pin is HIGH, the DAC registers are latched and the input registers can change state without
 * affecting the contents of the DAC registers. When LDAC is brought LOW, however, the DAC registers become
 * transparent and the contents of the input registers are transferred to them.
 */
void AD56XX_ControlLDAC(AD56XX_HandleTypeDef *had, GPIO_PinState state)
{

	HAL_GPIO_WritePin(had->ldac_gpio_port, had->ldac_gpio_pin, state);	//Set the LDAC pin state

}

/*
 * Function of control the state of CLR pin
 * 		@had: pointer to specified DAC information
 * 		@state: CLR pin state
 *
 * While CLR is LOW, all LDAC pulses are ignored. When CLR is HIGH, zero scale is loaded to all input
 * and DAC registers. This clears the output to 0 V. The part exits clear code mode on the 24th falling edge
 * of the next write to the part. If CLR is activated during a write sequence, the write is aborted.
 */
void AD56XX_ControlCLR(AD56XX_HandleTypeDef *had, GPIO_PinState state)
{

	HAL_GPIO_WritePin(had->clr_gpio_port, had->clr_gpio_pin, state);	//Set the CLR pin state
}

/*
 * Function to write a command and data to DAC
 * 		@had: pointer to specified DAC information
 * 		@command: command to be send to DAC. Referenced to the DAC Commands definition
 * 		@address: Address of the DAC register. Referenced to the Address Command of the DAC register
 * 		@data: data to be written to DAC
 */
void AD56XX_Write(AD56XX_HandleTypeDef *had, uint8_t command, uint8_t address, uint16_t data)
{
	/*
	 * The input shift register is 24 bit wide;
	*/

	/*
	 * First 2 MSB don't cares; C2-C0 are Command bits; A2-A0 are Address bits; Others are data bits, depends on DAC model
	*/
	/*
	 * 16 bit:
	 * --------------------------------------------------
	 * | X | X | C2 | C1 | C0 | A2 | A1 | A0 | D15 - DB0|
	 * --------------------------------------------------
	 * 	14 bit:
	 * --------------------------------------------------
	 * | X | X | C2 | C1 | C0 | A2 | A1 | A0 | D13 - DB0|
	 * --------------------------------------------------
	 * 	12 bit:
	 * --------------------------------------------------
	 * | X | X | C2 | C1 | C0 | A2 | A1 | A0 | D11 - DB0|
	 * --------------------------------------------------
	 */
	uint32_t tx_data = ((uint32_t)command & 0x07) << 19 | ((uint32_t)address & 0x07) << 16;	//Set command and address

	/*
	 * Next are 16-,14-,12-bit data-words; Data followed by 0, 2 or 4 don't care bits for depends on DAC model
	 * Binary code that is loading to DAC register depends on model
	 */
	switch (had->model)
	{
		case AD5623:		//12-bit DAC (0 to 4095)
			tx_data |= ((data & 0xFFF) << 4);
			if (data > 0xFFF) Error_Handler();	//Check data range
			break;
		case AD5643:		//14-bit DAC (0 to 16383)
			tx_data |= ((data & 0x3FFF) << 2);
			if (data > 0x3FFF) Error_Handler();	//Check data range
			break;
		case AD5663:		//16-bit DAC (0 to 65535)
			tx_data |= data & 0xFFFF;
			if (data > 0xFFFF) Error_Handler();	//Check data range
			break;
	}
	//The data bits are transferred to the DAC register on the 24th falling edge of SCLK.
	AD56XX_Transmit(had, tx_data);	//Transmit the data
}


/*
 * Function resets the DAC according to the specific reset mode
 * 		@had: pointer to specified DAC information
 * 		@mode: reset mode to be applied. Reference to the Software Reset Modes
 *
 * Command 0x05 (101) contains 2 reset modes that are setting by DB0 (LSB) bit in the control register
 * State of the bit corresponds to the operation of the device:
 *
 * ----------------------------------------------------
 * | 	DB0		| 	Registers Reset to Zero		      |
 * ----------------------------------------------------
 * |     0      |  DAC Registers				      |
 * |			|  Input Registers                    |
 * ----------------------------------------------------
 * |	 1	    |	DAC Registers                     |
 * |            |   Input Registers                   |
 * |            |   LDAC Registers                    |
 * |            |   Power-down Registers              |
 * |            |   Internal reference setup registers|
 * ----------------------------------------------------
 *
 * 24-bit Input Shift Register for Software Reset Command:
 * MSB                                                                        LSB
 * ------------------------------------------------------------------------------
 * | DB23 to DB22 | DB21 | DB20 | DB19 | DB18 | DB17 | DB16 | DB15 to DB1 | DB0 |
 * ------------------------------------------------------------------------------
 * |     x        |  1   |  0   |   1  |   x  |  x   |   x  |      x      | 1/0 |
 * ------------------------------------------------------------------------------
 * |  Don't care  |   Command Bits     |    Address Bits    |  Don't care | Mode|
 * ------------------------------------------------------------------------------
 */
void AD56XX_Reset(AD56XX_HandleTypeDef *had, AD56XX_ResetModes mode)
{
	uint32_t tx_data = ((uint32_t) AD56XX_CMD_RESET_DAC & 0x07) << 19 | (mode & 0x03); //Set command and mode
	//The data bits are transferred to the DAC register on the 24th falling edge of SCLK.
	AD56XX_Transmit(had, tx_data); //Transmit the data
}

/*
 * Function to configure the internal reference mode for the DAC
 * 		@had: pointer to specified DAC information
 * 		@mode: reference mode to be set. Reference to the Reference Setup Register Modes
 *
 * Command 0x07 (111) is reserved for setting up the internal reference
 *
 * ------------------------------------------------------
 * | Internal Reference       |                         |
 * | Setup Register (DB0)     |      Action             |
 * ------------------------------------------------------
 * | 0                        | Reference off (default) |
 * -----------------------------------------------------
 * | 1                        | Reference on            |
 * ------------------------------------------------------
 *
 * 24-bit Input Shift Register for Reference Setup Function:
 * MSB                                                                        LSB
 * ------------------------------------------------------------------------------
 * | DB23 to DB22 | DB21 | DB20 | DB19 | DB18 | DB17 | DB16 | DB15 to DB1 | DB0 |
 * ------------------------------------------------------------------------------
 * |     x        |  1   |  1   |   1  |   x  |  x   |   x  |      x      | 1/0 |
 * ------------------------------------------------------------------------------
 * |  Don't care  |   Command Bits     |    Address Bits    |  Don't care | Mode|
 * ------------------------------------------------------------------------------
 */
void AD56XX_SetRef(AD56XX_HandleTypeDef *had, AD56XX_ReferenceSetup mode)
{
	uint32_t tx_data = ((uint32_t) AD56XX_CMD_REF_SETUP & 0x07) << 19 | (mode & 0x03); //Set the command and mode
	//The data bits are transferred to the DAC register on the 24th falling edge of SCLK.
	AD56XX_Transmit(had, tx_data); //Transmit the data
}

/*
 *	Function to power down the specific DAC channel(s) according to selected power mode
 *		@had: pointer to specified DAC information
 *		@mode: power down mode to be applied. Referenced to Operating Modes
 *		@dac: DAC channel(s) to be powered down. Referenced to DAC channel selection
 *
 *		Command 0x04 (100) setting bit DB5 and DB4 to control register, their states corresponds to the mode of operation of the device:
 *	-------------------------------------------------
 *	|    DB5    |    DB4     |    Operation Mode    |
 *	-------------------------------------------------
 *	|     0		|      0     |  Normal operation    |
 *	-------------------------------------------------
 *	|			|			 |  Power-down modes:   |
 *	-------------------------------------------------
 *	|     0     |     1      |   1kOhm to GND       |
 *	-------------------------------------------------
 *	|     1     |     0      |   100kOhm to GND     |
 *	-------------------------------------------------
 *	|     1     |     1      |    Three-state       |
 *	-------------------------------------------------
 *
 *	24-bit Input Shift Register for Power Up/Power Down Function
 * 	MSB                                                                                                      LSB
 * 	------------------------------------------------------------------------------------------------------------
 * 	| DB23 to DB22 | DB21 | DB20 | DB19 | DB18 | DB17 | DB16 | DB15 to DB6 | DB5 | DB4 | DB3 | DB2 | DB1 | DB0 |
 * 	------------------------------------------------------------------------------------------------------------
 * 	|     x        |  1   |  1   |   1  |   x  |  x   |   x  |      x      | PD1 | PD2 |  x  |  x  |DAC B|DAC A|
 * 	------------------------------------------------------------------------------------------------------------
 * 	|  Don't care  |   Command Bits     |    Address Bits    |  Don't care |  PD Mode  | Don't care| Channels  |
 * 	------------------------------------------------------------------------------------------------------------
 */
void AD56XX_PowerDown(AD56XX_HandleTypeDef *had, AD56XX_PowerModes mode, AD56XX_ChannelSelection dac)
{
	uint32_t tx_data = ((uint32_t) AD56XX_CMD_POWER_DOWN_DAC & 0x07) << 19 | (mode & 0x03) << 4 | (dac & 0x03); //Set the command, mode and DAC channel(s)
	//The data bits are transferred to the DAC register on the 24th falling edge of SCLK.
	AD56XX_Transmit(had, tx_data); //Transmit the data
}

/*
 *	Function configures the hardware LDAC Register to select which combination of DAC channels to simultaneously update
 *	 	@had: pointer to specified DAC information
 *	 	@dac: DAC channel(s) for LDAC configuration. Referenced to DAC channel selection
 *
 *	Setting the command 0x06 (110) load LDAC register to select which combination of DAC channels to simultaneously update
 *	LDAC bit register to 0 for a DAC channel means that the update of this channel is controlled by the LDAC pin.
 *	If this bit is set to 1, means the DAC register is updated, regardless of the state of the LDAC pin
 *
 *	--------------------------------------------------------------------------
 *	|   LDAC Bits    |    LDAC Pin      |       LDAC Operation               |
 *	|  (DB1 to DB0)  |                  |                                    |
 *	--------------------------------------------------------------------------
 *	|     0          |       1/0        |      Determined by LDAC pin        |
 *	--------------------------------------------------------------------------
 *	|     1          |      Don't care  |    The DAC register are updated    |
 *	|                |                  |    after new data is read in on    |
 *	|                |                  |    falling edge of 24th SLCK pulse |
 *  --------------------------------------------------------------------------
 *
 *  24-bit Input Shift Register for LDAC Setup Command
 * MSB                                                                              LSB
 * ------------------------------------------------------------------------------------
 * | DB23 to DB22 | DB21 | DB20 | DB19 | DB18 | DB17 | DB16 | DB15 to DB2 | DB1 | DB0 |
 * ------------------------------------------------------------------------------------
 * |     x        |  1   |  1   |   0  |   x  |  x   |   x  |      x      |DAC A|DAC B|
 * ------------------------------------------------------------------------------------
 * |  Don't care  |   Command Bits     |    Address Bits    |  Don't care | LDAC mode |
 * ------------------------------------------------------------------------------------
 */
void AD56XX_SetLDAC(AD56XX_HandleTypeDef *had, AD56XX_ChannelSelection dac)
{

	uint32_t tx_data = ((uint32_t) AD56XX_CMD_LDAC_SETUP & 0x07) << 19 | (dac & 0x03); //Set the command and DAC channel
	//The data bits are transferred to the DAC register on the 24th falling edge of SCLK.
	AD56XX_Transmit(had, tx_data); //Transfer the data
}

/*
 * Function writes the data to specified DAC Input Register
 * 		@had: pointer to specified DAC information
 * 		@add: address of the DAC register. Referenced to Address Command
 * 		@value: data to be written to the DAC
 */
void AD56XX_WriteToReg(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value)
{
	AD56XX_Write(had ,AD56XX_CMD_WRITE_INPUT_REG, add, value);	//Write to DAC
}

/*
 * Function updates specific DAC Input Register with new data
 * 		@had: pointer to specified DAC information
 * 		@add: address of the DAC register. Referenced to Address Command
 * 		@value: data to be written to the DAC
 */
void AD56XX_UpdateReg(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value)
{
	AD56XX_Write(had ,AD56XX_CMD_UPDATE_DAC_REG, add, value);	//Write to DAC
}
/*
 * Function performs software control LDAC operation write a specified DAC Input Register and update All
 * 		@had: pointer to specified DAC information
 * 		@add: address of the DAC register. Referenced to Address Command
 * 		@value: data to be written and updated to DAC
 */
void AD56XX_LDACSoftware(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value)
{
	AD56XX_Write(had ,AD56XX_CMD_WRITE_INPUT_REG_UPDATE, add, value);	//Write to DAC
}

/*
 * Function writes and updates specified DAC channel with new data
 * 		@had: pointer to specified DAC information
 * 		@add: address of the DAC register. Referenced to Address Command
 * 		@value: data to be written and updated to DAC
 */
void AD56XX_WriteUpdateChannel(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value)
{
	AD56XX_Write(had, AD56XX_CMD_WRITE_UPDATE_DAC_CHANNEL, add, value);	//Write to DAC
}



