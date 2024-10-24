#pragma once
#include <inttypes.h>
#include "stm32f1xx_hal.h"

void HAL_SPI_ReadFast(SPI_HandleTypeDef *hspi, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi->Instance;
	uint32_t tickstart = HAL_GetTick();
	
	__HAL_SPI_ENABLE(hspi);
	while(Size--)
	{
		while( (SPIx->SR & SPI_FLAG_TXE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*(__IO uint8_t *)&SPIx->DR = 0xFF;
		
		while( (SPIx->SR & SPI_FLAG_RXNE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*pRxData++ = *(__IO uint8_t *)&SPIx->DR;
	}
	
	exit:
	
	return;
}

void HAL_SPI_WriteFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint16_t Size, uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi->Instance;
	uint32_t tickstart = HAL_GetTick();
	uint8_t RXdummy;
	
	__HAL_SPI_ENABLE(hspi);
	while(Size--)
	{
		while( (SPIx->SR & SPI_FLAG_TXE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*(__IO uint8_t *)&SPIx->DR = *pTxData++;

		while( (SPIx->SR & SPI_FLAG_RXNE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		RXdummy = *(__IO uint8_t *)&SPIx->DR;
	}
	
	exit:
	
	return;
}

void HAL_SPI_WriteReadFast(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
{
	SPI_TypeDef *SPIx= hspi->Instance;
	uint32_t tickstart = HAL_GetTick();

	__HAL_SPI_ENABLE(hspi);
	while(Size--)
	{
		while( (SPIx->SR & SPI_FLAG_TXE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*(__IO uint8_t *)&SPIx->DR = *pTxData++;

		while( (SPIx->SR & SPI_FLAG_RXNE) == 0 )
		{
			if( (HAL_GetTick() - tickstart) >= Timeout ) goto exit;
		}
		*pRxData++ = *(__IO uint8_t *)&SPIx->DR;
	}
	
	exit:
	
	return;
}
