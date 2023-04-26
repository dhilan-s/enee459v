



#include "main.h"
#include "lsm6dsl.h"
#include "b_l4s5i_iot01a_bus.h"
#include <stdio.h>

extern LSM6DSL_Object_t Gyro;

void what_is_it(void){
	uint8_t id;
	LSM6DSL_ReadID(&Gyro, &id);
	printf("WHO_AM_I: %5x \r\n", (int)id);
}


//function which changes content of some data or control register of the peripheral.
void write_location(float Odr){
	char unit[10] = "";
	uint32_t freq;
	LSM6DSL_GYRO_SetOutputDataRate(&Gyro, Odr);
	switch((int)Odr){
		case 0:
				freq = 0;
				strcat(unit, "Power Down");
				break;
		case 1:
				freq = 12.5;
				strcat(unit, "Hz");
				break;
		case 2:
				freq = 26;
				strcat(unit, "Hz");
				break;
		case 3:
				freq = 52;
				strcat(unit, "Hz");
				break;
		case 4:
				freq = 104;
				strcat(unit, "Hz");
				break;
		case 5:
				freq = 208;
				strcat(unit, "Hz");
				break;
		case 6:
				freq = 416;
				strcat(unit, "Hz");
				break;
		case 7:
				freq = 833;
				strcat(unit, "Hz");
				break;
		case 8:
				freq = 1.66;
				strcat(unit, "kHz");
				break;
		case 9:
				freq = 3.33;
				strcat(unit, "kHz");
				break;
		case 10:
				freq = 6.66;
				strcat(unit, "kHz");
				break;
		default:
				freq = 0;
				strcat(unit, "Invalid");
				break;
	}
	printf("Setting ODR: %5d %s \r\n", (int)freq, unit);
}

//function which reports contents of some data or control register of the peripheral.
void read_location(void){
	float Odr;
	LSM6DSL_GYRO_GetOutputDataRate(&Gyro, &Odr);
	printf("Current ODR: %5d \r\n", (int)Odr);
}
