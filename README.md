
# AD56XX DAC Library

## Overview

This library is designed for STM32 microcontrollers to interface with the Analog Devices AD56XX family of DACs. It supports DAC models like AD5623R (12-bit), AD5643R (14-bit), and AD5663R (16-bit) via SPI communication. The library provides functions to initialize the DAC, transmit data, control LDAC and CLR pins, and perform various DAC operations such as reset, power down or internal reference.

## Features

- Support for AD56XX family of DACs
- SPI communication with STM32
- Control for LDAC and CLR pins
- Functions for writing, updating, and resetting DAC registers
- Power-down mode and internal reference configuration
- Example code for quick setup

## Requirements

- STM32 microcontroller (tested with STM32L4 series)
- HAL (Hardware Abstraction Layer) drivers
- SPI peripheral configured for communication
- Compatible DACs: AD5623R (12-bit), AD5643R(14-bit), AD5663R(16-bit)

## Installation

Clone the repository and include it in your STM32 project:

```bash
git clone https://github.com/DarthFelus/AD56XX-DAC-Library.git
```

## Usage

1. Initialization

	Include the header file AD56XX.h in your project:
	
    ```c
    #include "AD56XX.h"
    ```

	Initialize the DAC by configuring the SPI and GPIO settings. You must specify the SPI handle, GPIO pins for SYNC, LDAC, and CLR, and the DAC model.

    ```c
	AD56XX_HandleTypeDef dac_handle;

	AD56XX_Init(&dac_handle, &hspi1, GPIOB, GPIO_PIN_6,  // SYNC
							GPIOB, GPIO_PIN_7,  // LDAC
							GPIOB, GPIO_PIN_8,  // CLR
							AD5663);            // DAC model (16-bit)
    ```

2. Writing and Updating DAC Values
   
	To send data to the DAC, use the AD56XX_Write() function. This function allows you to specify the DAC command, address, and data.

    ```c
    // Write data to the DAC
	AD56XX_Write(&dac_handle, AD56XX_CMD_WRITE_INPUT_REG, DAC_A, 2048);

	// Update the DAC register
	AD56XX_UpdateReg(&dac_handle, DAC_A, 2048);
    ```
	
	To write and update the DAC channel in one command:
	```c
	// Write and update DAC channel
	AD56XX_WriteUpdateChannel(&dac_handle, DAC_A, 4096);
	```
	
3. LDAC and CLR Pin Control

	Control the LDAC and CLR pins to latch or clear the DAC registers:
	
	```c
	// Set LDAC pin to low to allow DAC register update
	AD56XX_ControlLDAC(&dac_handle, GPIO_PIN_RESET);

	// Set CLR pin high to clear DAC registers
	AD56XX_ControlCLR(&dac_handle, GPIO_PIN_SET);
	```
	
4. Resetting the DAC
	
	You can reset the DAC registers using the AD56XX_Reset() function with the desired reset mode.
	```c
	// Reset DAC registers
	AD56XX_Reset(&dac_handle, FullReset);
	```
	
5. Power Down Mode

	You can set the DAC to enter a power-down mode using AD56XX_PowerDown():
	```c
	// Power down DAC channel A to 1kOhm to GND mode
	AD56XX_PowerDown(&dac_handle, PowerDown1kOhm, DAC_A);
	```

6. Reference mode
   
	You can configure the internal reference mode for the DAC using AD56XX_SetRef():
	
	```c
	// Set Reference mode on
	AD56XX_SetRef(&dac_handle, ReferenceOn);
	```

 And other specified functions, described on datasheets work here. See details on AD56XX.c.
### Example Code

```c
#include "AD56XX.h"

// SPI handle declaration
SPI_HandleTypeDef hspi1;
AD56XX_HandleTypeDef dac_handle;

int main(void) {
    // Initialize system and peripherals
    HAL_Init();
    
    // Initialize DAC with specified model and GPIO settings
    AD56XX_Init(&dac_handle, &hspi1, GPIOB, GPIO_PIN_6,  // SYNC
                              GPIOB, GPIO_PIN_7,  // LDAC
                              GPIOB, GPIO_PIN_8,  // CLR
                              AD5663);            // DAC model (16-bit)
    
    // Write a value to DAC channel A
    AD56XX_Write(&dac_handle, AD56XX_CMD_WRITE_INPUT_REG, DAC_A, 2048);
    
    // Update DAC channel A with a new value
    AD56XX_UpdateReg(&dac_handle, DAC_A, 4096);
    
    // Main loop
    while (1) {
        // Do something
    }
}

```

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Contributing

Feel free to contribute by opening issues or submitting pull requests.

