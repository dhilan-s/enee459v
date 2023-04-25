/*
 * gyro.c
 *
 *  Created on: Apr 24, 2023
 *      Author: PC
 */


//function which reports some unique signature of the target peripheral (WHO_AM_I or equivalent).
void what_is_it(){

}


//function which changes content of some data or control register of the peripheral.
void write_location(){
	HAL_I2C_Mem_Write (I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout);
}

//function which reports contents of some data or control register of the peripheral.
void read_location(){
	HAL_I2C_Mem_Read (I2C_HandleTypeDef * hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t * pData, uint16_t Size, uint32_t Timeout);
}
