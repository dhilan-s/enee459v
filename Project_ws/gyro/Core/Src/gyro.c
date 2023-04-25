/*
 * gyro.c
 *
 *  Created on: Apr 24, 2023
 *      Author: PC
 */

#include "main.h"

extern hi2c2;
const uint8_t gyroSAD = (0x6A << 1);
const uint16_t WHO_AM_I = 0x0F;
const uint16_t CTRL2_G = 0x11;

uint8_t data[2];
uint8_t ODR_config = (0b0110 << 4);
uint8_t config_check[2];


//function which reports some unique signature of the target peripheral (WHO_AM_I or equivalent).
void what_is_it(){
	HAL_I2C_Mem_Read (&hi2c2, gyroSAD, WHO_AM_I, 1, data, 1, HAL_MAX_DELAY);
}


//function which changes content of some data or control register of the peripheral.
void write_location(){
	HAL_I2C_Mem_Write (hi2c2, gyroSAD, CTRL2_G, 1, &ODR_config, 2, HAL_MAX_DELAY);
}

//function which reports contents of some data or control register of the peripheral.
void read_location(){
	HAL_I2C_Mem_Read (hi2c2, gyroSAD, CTRL2_G, 1, config_check, 2, HAL_MAX_DELAY);
}
