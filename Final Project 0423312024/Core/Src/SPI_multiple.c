#include "SPI_multiple.h"
#define SPI_Port GPIOA
#define clock_Pin GPIO_PIN_5
#define cs_Pin GPIO_PIN_4
#define data_Pin GPIO_PIN_7

extern SPI_HandleTypeDef hspi1;

void tiny_dly(void) {
}
void SPI_byte (uint8_t byte)
{
	HAL_SPI_Transmit(&hspi1, &byte, 1, 1);

}


void SPI_7219_SEND (uint8_t address, uint8_t data, int n)
{
	HAL_GPIO_WritePin (SPI_Port, cs_Pin, 0);  // pull the CS pin LOW

	SPI_byte (address);
	SPI_byte (data);

	for (int k=0; k<(n-1); k++)
		{
			SPI_byte (0x00); //Address of No Operation
			SPI_byte (0x00);
		}

	HAL_GPIO_WritePin (SPI_Port, cs_Pin, 1);  // pull the CS pin HIGH

}


