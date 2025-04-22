#include "stm32f4xx_hal.h"
#include "servo.h"

void STS3032_WritePosition(uint8_t id, uint16_t position, UART_HandleTypeDef *huart) {
    uint8_t packet[10];
    uint8_t checksum;

    uint8_t pos_l = position & 0xFF;
    uint8_t pos_h = (position >> 8) & 0xFF;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = 5;                 // Length = parameter 개수(3) + 2
    packet[4] = 0x03;              // WRITE command
    packet[5] = 0x1E;              // Goal Position 주소 (0x1E)
    packet[6] = pos_l;
    packet[7] = pos_h;

    checksum = ~(id + packet[3] + packet[4] + packet[5] + packet[6] + packet[7]);
    packet[8] = checksum;

    HAL_UART_Transmit(huart, packet, 9, HAL_MAX_DELAY);
}
