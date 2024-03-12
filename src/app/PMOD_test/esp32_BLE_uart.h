#ifndef ESP32_BLE_UART_H
#define ESP32_BLE_UART_H

#include <stdint.h>
#include <stdbool.h>

// Function to send a single byte over UART1
void sendByteToESP32(uint8_t byte);

// Function to receive a single byte from UART1 (non-blocking)
bool receiveByteFromESP32(uint8_t* byte);

#endif // ESP32_BLE_UART_H
