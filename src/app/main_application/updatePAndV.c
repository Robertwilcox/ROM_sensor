/**
 * @file updataPandV.c
 * @brief file for updatting global struct with position and velocity
 * data from packetReader.h
 *
 * @author Robert Wilcox, Ibrahim Binmahfood
 * @version 1.0
 * @date 3.21.2024
 */

#include "updatePandV.h"
#include <stdint.h>
#include "xil_printf.h"
#include "esp32_BLE_uart.h"
#include "packetReader.h"

// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

Position myPosition;
Velocity myVelocity;

void getPositionAndVelocity(DataPoint *dataPoint) {

    processPacket(&myVelocity, &myPosition);

    dataPoint->x_pos = myPosition.xPos;
    dataPoint->y_pos = myPosition.yPos;
    dataPoint->z_pos = myPosition.zPos;

    dataPoint->x_veloc = myVelocity.xVel;
    dataPoint->y_veloc = myVelocity.yVel;
    dataPoint->z_veloc = myVelocity.zVel;

}
