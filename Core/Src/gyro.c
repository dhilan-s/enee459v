#include "main.h"
#include "lsm6dsl.h"
#include "b_l4s5i_iot01a_bus.h"
#include <stdio.h>

extern LSM6DSL_Object_t Gyro;
extern uint8_t command[];
extern uint8_t result[];
extern uint32_t dataRdyIntReceived;

//For testing, uncomment puts (result) to see result buffer values in terminal.

//Returns the current position
void read_cur_position(void){
	if (dataRdyIntReceived != 0) {
		dataRdyIntReceived = 0;
		LSM6DSL_Axes_t gyro_axes;
		LSM6DSL_GYRO_GetAxes(&Gyro, &gyro_axes);
		sprintf(result, "% 5d, % 5d, % 5d\r\n",  (int) gyro_axes.x, (int) gyro_axes.y, (int) gyro_axes.z);
		//puts (result)
	 }
}

//Returns the device id
void what_is_it(void){
	uint8_t id;
	LSM6DSL_ReadID(&Gyro, &id);
	sprintf(result, "%x \r\n", id);
	//puts (result)
}


//function which changes content of some data or control register of the peripheral.
//Odr - refers to a float that is indicated by the user. The gyroscope has a set of pre-configured
//		ODR, as a result this function sets the user input to the nearest possible ODR value.
void write_location(float Odr){
	char unit[10] = "";
	uint32_t freq;
	LSM6DSL_GYRO_SetOutputDataRate(&Gyro, Odr);
		if(Odr == 0) {
			freq = 0;
			strcat(unit, "Hz");
		}
		else if(Odr <= 12.5){
			freq = 12.5;
			strcat(unit, "Hz");
		}
		else if (Odr <= 26){
			freq = 26;
			strcat(unit, "Hz");
		}
		else if ( Odr <= 52) {
			freq = 52;
			strcat(unit, "Hz");
		}
		else if ( Odr <= 104) {
			freq = 104;
			strcat(unit, "Hz");
		}
		else if ( Odr <= 208) {
			freq = 208;
			strcat(unit, "Hz");
		}
		else if ( Odr <= 416) {
			freq = 416;
			strcat(unit, "Hz");
		}
		else if ( Odr <= 833) {
			freq = 833;
			strcat(unit, "Hz");
		}
		else if ( Odr <= 1660) {
			freq = 1.66;
		  	strcat(unit, "kHz");
		}
		else if ( Odr <= 3330) {
			freq = 3.33;
			strcat(unit, "kHz");
		}
		else if ( Odr <= 6660) {
			freq = 6.66;
			strcat(unit, "kHz");
		}
		sprintf(result, "%5d %s \r\n", (int)freq, unit);
		//puts (result)
}

//function which reports contents of some data or control register of the peripheral.
void read_location(void){
	float Odr;
	LSM6DSL_GYRO_GetOutputDataRate(&Gyro, &Odr);
	sprintf(result, "%d \r\n", (int)Odr);
	//puts (result)
}
