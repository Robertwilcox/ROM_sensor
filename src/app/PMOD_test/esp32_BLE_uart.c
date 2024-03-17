/**
 * @file esp32_BLE_uart.c
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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include "xuartlite_l.h"
#include "xuartlite.h"

// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

//  Enum to manage to state of the BLE transactions
typedef enum {
    WAITING_FOR_DATA,
    READING_QUATERNION,
    READING_LINEAR_ACCEL,
    READING_GRAVITY
} DataReadState;

DataReadState currentState = WAITING_FOR_DATA;


typedef struct {
    float w, x, y, z;
} QuaternionData;

typedef struct {
    float x, y, z;
} LinearAccData;

typedef struct {
    float x, y, z;
} GravityData;

QuaternionData qData;
LinearAccData lData;
GravityData gData;

#define BUFFER_SIZE 128 // overkill but that's ok
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

bool receiveByteFromESP32(uint8_t* byte);
// Function to parse data and put into structs
void processData() {
    
    sscanf(buffer, "Q:%f,%f,%f,%f L:%f,%f,%f G:%f,%f,%f",
           &qData.w, &qData.x, &qData.y, &qData.z,
           &lData.x, &lData.y, &lData.z,
           &gData.x, &gData.y, &gData.z);

    // Reset buffer index after processing
    bufferIndex = 0;
    currentState = WAITING_FOR_DATA; // Reset state
}

// Function to read data
void readData() {
    
    uint8_t byte;

    while (receiveByteFromESP32(&byte)) {
        switch (currentState) {
            case WAITING_FOR_DATA:
                if (byte == 'Q') {
                    currentState = READING_QUATERNION;
                    bufferIndex = 0; // Reset buffer index
                    buffer[bufferIndex++] = byte; // Include 'Q' in the buffer for processing
                }
                break;

            case READING_QUATERNION:
                if (byte == 'L') { // Transition from Quaternion to Linear Acceleration
                    currentState = READING_LINEAR_ACCEL;
                    buffer[bufferIndex++] = byte; // Include 'L' in the buffer for processing
                } else {
                    // Continue reading Quaternion data
                    if (bufferIndex < BUFFER_SIZE - 1) buffer[bufferIndex++] = byte;
                }
                break;

            case READING_LINEAR_ACCEL:
                if (byte == 'G') { // Transition from Linear Acceleration to Gravity
                    currentState = READING_GRAVITY;
                    buffer[bufferIndex++] = byte; // Include 'G' in the buffer for processing
                } else {
                    // Continue reading Linear Acceleration data
                    if (bufferIndex < BUFFER_SIZE - 1) buffer[bufferIndex++] = byte;
                }
                break;

            case READING_GRAVITY:
                if (byte == '\n') { // End of packet
                    buffer[bufferIndex] = '\0'; // Null-terminate the string
                    processData(); // Process the complete packet
                    currentState = WAITING_FOR_DATA; // Reset state to wait for the next data packet
                    bufferIndex = 0; // Reset buffer index for the next packet
                } else {
                    // Continue reading Gravity data
                    if (bufferIndex < BUFFER_SIZE - 1) buffer[bufferIndex++] = byte;
                }
                break;
        }

        // Safety check to prevent buffer overflow
        if (bufferIndex >= BUFFER_SIZE - 1) {
            buffer[BUFFER_SIZE - 1] = '\0'; // Ensure buffer is null-terminated
            processData(); // Process what we have as a precaution
            currentState = WAITING_FOR_DATA; // Reset state
            bufferIndex = 0; // Reset buffer index
        }
    }
}


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
