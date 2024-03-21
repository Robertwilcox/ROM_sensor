/**
 * @file esp32_BLE_uart.h
 * @brief Header file for interfacing with an ESP32 over UART for BLE communication.
 *
 * @author Robert Wilcox
 * @version 1.0
 * @date 3.12.2024
 */

#ifndef ESP32_BLE_UART_H
#define ESP32_BLE_UART_H

#include <stdbool.h>
#include <stdint.h>

void sendByteToESP32(uint8_t byte);
bool receiveByteFromESP32(uint8_t* byte);

#endif // ESP32_BLE_UART_H
