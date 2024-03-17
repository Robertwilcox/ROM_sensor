/**
 * @file esp32_BLE_uart.h
 * @brief Header file for interfacing with an ESP32 over UART for BLE communication.
 *
 * This header file contains declarations for handling BLE data communication
 * between an FPGA and an ESP32 module. It defines structures for Quaternion,
 * Linear Acceleration, and Gravity data received from the ESP32 and declares
 * functions for sending and receiving bytes over UART1, as well as processing
 * the received sensor data into structured form.
 *
 * @author Robert Wilcox
 * @version 1.0
 * @date 3.12.2024
 */

#ifndef ESP32_BLE_UART_H
#define ESP32_BLE_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float w, x, y, z;
} QuaternionData;

typedef struct {
    float x, y, z;
} LinearAccData;

typedef struct {
    float x, y, z;
} GravityData;

extern QuaternionData qData;
extern LinearAccData lData;
extern GravityData gData;

void processData(void);
void readData(void);
void sendByteToESP32(uint8_t byte);
bool receiveByteFromESP32(uint8_t* byte);

#endif // ESP32_BLE_UART_H
