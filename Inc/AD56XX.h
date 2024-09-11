/*
 * AD5623R.h
 *
 *  Created on: Jul 15, 2024
 *      Author: d.arseniuk
 *		ver 1.0.3 23.07.2024
 */

#ifndef INC_AD56XX_H_
#define INC_AD56XX_H_

#ifdef __cplusplus
extern "C" {
#endif


/************************************************ INCLUDES ************************************************/
/*
 * Define the target microcontroller series
 */
#define STM32L4
// #define STM32F1



#if defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#else
#error "AD56XX worked only on STM32L4, STM32F4 MCU series"
#endif

/************************************************ DEFINES ************************************************/

/*
 * DAC Commands Definition
 */
#define AD56XX_CMD_WRITE_INPUT_REG 		  	0x00	//(000) Write to Input Register n
#define AD56XX_CMD_UPDATE_DAC_REG 		  	0x01	//(001) Update DAC Register n
#define AD56XX_CMD_WRITE_INPUT_REG_UPDATE 	0x02	//(010) Write to Input Register n, update all (software LDAC)
#define AD56XX_CMD_WRITE_UPDATE_DAC_CHANNEL 0x03	//(011) Write to and update DAC Channel n
#define AD56XX_CMD_POWER_DOWN_DAC			0x04	//(100) Power down DAC (power up)
#define AD56XX_CMD_RESET_DAC				0x05	//(101) Reset
#define AD56XX_CMD_LDAC_SETUP				0x06	//(110) LDAC register setup
#define AD56XX_CMD_REF_SETUP				0x07	//(111) Internal reference setup (on/off)


/************************************************ TYPES ************************************************/
/*
 * Selection of using DAC model
 */
typedef enum {
	AD5623,
	AD5643,
	AD5663
}AD56XX_Model;

/*
 * Structure to hold DAC pinout and model information
 */
typedef struct {
	SPI_HandleTypeDef *hspi;		//SPI handle
	GPIO_TypeDef *sync_gpio_port;	//SYNC (For SPI - SS) GPIO port
	uint16_t sync_gpio_pin;			//SYNC GPIO pin
	GPIO_TypeDef *ldac_gpio_port;	//LDAC GPIO port
	uint16_t ldac_gpio_pin;			//LDAC GPIO pin
	GPIO_TypeDef *clr_gpio_port;	//CLR GPIO port
	uint16_t clr_gpio_pin;			//CLR GPIO pin
	AD56XX_Model model;				//DAC model
} AD56XX_HandleTypeDef;


/*
 * Address Command
 *
 * (010) and (011) Reserved
 */
typedef enum {
	AD56XX_ADR_DAC_A		= 0x00,		//(000) DAC A
	AD56XX_ADR_DAC_B		= 0x01,		//(001) DAC B
	AD56XX_ADR_DAC_ALL    	= 0x07		//(111) All DACs
} AD56XX_Address;

/*
 * Software Reset Modes
 */
typedef enum {					//Registers Reset to Zero:
	PartReset = 0x00,			//DAC Register; Input Register
	FullReset = 0x01			//DAC Register; Input Register; LDAC register; Power Down Register; Internal reference setup register
} AD56XX_ResetModes;

/*
 * Operating Modes
 */
typedef enum {
	NormalOperation 		= 0x00,		//(00) Normal operation
	PowerDown1kOhm 			= 0x01,		//(01) Power Down: 1kOhm to GND
	PowerDown100kOhm 		= 0x02,		//(10) Power Down: 100kOhm to GND
	ThreeState 				= 0x03		//(11) Power Down: Three-state
}AD56XX_PowerModes;

/*
 * Reference Setup Register
 */
typedef enum {
	ReferenceOff  			= 0x00,		//(0) Reference off (default)
	ReferenceOn				= 0x01		//(1) Reference on
}AD56XX_ReferenceSetup;

/*
 * DAC channel selection
 */
typedef enum {
	DAC_A					= 0x01,		//DAC channel A
	DAC_B					= 0x02,		//DAC channel B
	DAC_ALL					= 0x03		//DAC channel A and B
}AD56XX_ChannelSelection;



/************************************************ FUNCTION PROTOTYPES ************************************************/


void AD56XX_Init(AD56XX_HandleTypeDef *had, SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx_SYNC, uint16_t GPIO_Pin_SYNC,
		GPIO_TypeDef *GPIOx_LDAC, uint16_t GPIO_Pin_LDAC, GPIO_TypeDef *GPIOx_CLR,
		uint16_t GPIO_Pin_CLR, AD56XX_Model model);

void AD56XX_Transmit(AD56XX_HandleTypeDef *had, uint32_t data);

void AD56XX_ControlLDAC(AD56XX_HandleTypeDef *had, GPIO_PinState state);

void AD56XX_ControlCLR(AD56XX_HandleTypeDef *had, GPIO_PinState state);

void AD56XX_Write(AD56XX_HandleTypeDef *had, uint8_t command, uint8_t address, uint16_t data);

void AD56XX_WriteToReg(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value);

void AD56XX_UpdateReg(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value);

void AD56XX_LDACSoftware(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value);

void AD56XX_WriteUpdateChannel(AD56XX_HandleTypeDef *had, AD56XX_Address add, uint16_t value);

void AD56XX_PowerDown(AD56XX_HandleTypeDef *had, AD56XX_PowerModes mode, AD56XX_ChannelSelection dac);

void AD56XX_SetLDAC(AD56XX_HandleTypeDef *had, AD56XX_ChannelSelection dac);

void AD56XX_SetRef(AD56XX_HandleTypeDef *had, AD56XX_ReferenceSetup mode);

void AD56XX_Reset(AD56XX_HandleTypeDef *had, AD56XX_ResetModes mode);

#ifdef __cplusplus
}
#endif


#endif /* INC_AD56XX_H_ */
