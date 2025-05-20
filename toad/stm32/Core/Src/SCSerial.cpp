#include "SCSerial.h"

SCSerial::SCSerial(): SCS()
{
	IOTimeOut = 100;
	pUart = nullptr;
}

SCSerial::SCSerial(u8 End): SCS(End)
{
	IOTimeOut = 100;
	pUart = nullptr;
}

SCSerial::SCSerial(u8 End, u8 Level): SCS(End, Level)
{
	IOTimeOut = 100;
	pUart = nullptr;
}

void SCSerial::begin(UART_HandleTypeDef *huart)
{
	pUart = huart;
}

int SCSerial::readSCS(unsigned char *nDat, int nLen)
{
	uint32_t startTick = HAL_GetTick();
	int received = 0;

	while ((HAL_GetTick() - startTick) < IOTimeOut) {
		uint8_t byte;
		if (HAL_UART_Receive(pUart, &byte, 1, 1) == HAL_OK) {
			if (nDat) nDat[received] = byte;
			received++;
			if (received >= nLen) break;
		}
	}

	return received;
}

int SCSerial::writeSCS(unsigned char *nDat, int nLen)
{
	if (pUart == nullptr || nDat == nullptr) return 0;
	HAL_UART_Transmit(pUart, nDat, nLen, HAL_MAX_DELAY);
	return nLen;
}

int SCSerial::writeSCS(unsigned char bDat)
{
	if (pUart == nullptr) return 0;
	return HAL_UART_Transmit(pUart, &bDat, 1, HAL_MAX_DELAY) == HAL_OK ? 1 : 0;
}

void SCSerial::rFlushSCS()
{
	__HAL_UART_FLUSH_DRREGISTER(pUart); // 수신 버퍼 플러시
}

void SCSerial::wFlushSCS()
{
	// STM32 HAL은 전송 대기 없음
}
