/**
 * @file esp32_BLE_uart.c
 * @brief Header file for interfacing with an ESP32 over UART for BLE communication.
 *
 * @author Robert Wilcox, Ibrahim Binmahfood
 * @version 1.0
 * @date 3.12.2024
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

#include "xuartlite_l.h"
#include "xuartlite.h"

#include "esp32_BLE_uart.h"

// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

// Function to send a single byte over UART1
void sendByteToESP32(uint8_t byte) {
    XUartLite_SendByte(UART1_BASEADDR, byte);
}

// Function to receive a single byte from UART1 (non-blocking)
bool receiveByteFromESP32(uint8_t* byte) {
    // Check if there is data available
    if (XUartLite_IsReceiveEmpty(UART1_BASEADDR) == FALSE) {
        *byte = XUartLite_RecvByte(UART1_BASEADDR);
        return true;  // Data was received
    }
    return false;  // No data available
}
