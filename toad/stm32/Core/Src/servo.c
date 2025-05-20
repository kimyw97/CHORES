#include "stm32f4xx_hal.h"
#include "servo.h"

void STS3032_WritePosition(uint8_t id, uint16_t position, UART_HandleTypeDef *huart) {
    uint8_t packet[10];
    uint8_t checksum;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 0x07;           // Length
    packet[4] = 0x03;           // Instruction: WRITE
    packet[5] = 0x2A;           // Address: Goal Position Low
    packet[6] = 0x00;
    packet[7] = position & 0xFF;
    packet[8] = (position >> 8) & 0xFF;

    checksum = ~(packet[2] + packet[3] + packet[4] + packet[5] + packet[6] + packet[7] + packet[8]) & 0xFF;
    packet[9] = checksum;

    HAL_UART_Transmit(huart, packet, 10, 100);
}
