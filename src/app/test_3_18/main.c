#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <stddef.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "xintc.h"
#include "xuartlite_l.h"
#include "xuartlite.h"
#include "xstatus.h"
#include "xparameters.h"
#include "platform.h"
#include "microblaze_sleep.h"
#include "xil_printf.h"
#include "esp32_BLE_uart.h"
#include "packetReader.h"
#include "updatePandV.h"


// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

#define MAX_PACKET_SIZE 20
#define START_MARKER 0x02
#define END_MARKER 0x03


void reverseString(char* str, int length);
int intToStr(int x, char str[], int d);
void floatToString(float n, char *res, int afterpoint);



DataPoint myDataPoint;

void vUARTCommunicationTask(void *pvParameters) {
    xil_printf("UART Communication Task Started\r\n");

    // Buffer to hold the string representation of floats
    char floatStr[20]; // Adjust the size based on your needs

    while (1) {
        if (readPacket()) { // Check if a packet was read successfully

            /*
            xil_printf("Valid packet received (%d bytes): \r\n", length);
            for (int i = 0; i < length; i++) {
                xil_printf("%02X ", packet[i]);
            }
            */

            xil_printf("\r\n");

            getPositionAndValocity(&myDataPoint);

			// Convert position values to strings and print
			floatToString(myDataPoint.x_pos, floatStr, 2);
			xil_printf("Position X: %s\r\n", floatStr);

			floatToString(myDataPoint.y_pos, floatStr, 2);
			xil_printf("Position Y: %s\r\n", floatStr);

			floatToString(myDataPoint.z_pos, floatStr, 2);
			xil_printf("Position Z: %s\r\n", floatStr);

			// Convert velocity values to strings and print
			floatToString(myDataPoint.x_veloc, floatStr, 2);
			xil_printf("Velocity X: %s\r\n", floatStr);

			floatToString(myDataPoint.y_veloc, floatStr, 2);
			xil_printf("Velocity Y: %s\r\n", floatStr);

			floatToString(myDataPoint.z_veloc, floatStr, 2);
			xil_printf("Velocity Z: %s\r\n", floatStr);

			/*
            // Convert quaternion values to strings and print
            floatToString((float)myQuaternion.w, floatStr, 2);
            xil_printf("Quaternion w: %s\r\n", floatStr);

            floatToString((float)myQuaternion.x, floatStr, 2);
            xil_printf("Quaternion x: %s\r\n", floatStr);

            floatToString((float)myQuaternion.y, floatStr, 2);
            xil_printf("Quaternion y: %s\r\n", floatStr);

            floatToString((float)myQuaternion.z, floatStr, 2);
            xil_printf("Quaternion z: %s\r\n", floatStr);

            // Convert linear acceleration values to strings and print
            floatToString((float)myLinearAccel.linAccX, floatStr, 2);
            xil_printf("Linear Acceleration X: %s\r\n", floatStr);

            floatToString((float)myLinearAccel.linAccY, floatStr, 2);
            xil_printf("Linear Acceleration Y: %s\r\n", floatStr);

            floatToString((float)myLinearAccel.linAccZ, floatStr, 2);
            xil_printf("Linear Acceleration Z: %s\r\n", floatStr);

            // Convert gravity vector values to strings and print
            floatToString((float)myGravityVector.gravX, floatStr, 2);
            xil_printf("Gravity Vector X: %s\r\n", floatStr);

            floatToString((float)myGravityVector.gravY, floatStr, 2);
            xil_printf("Gravity Vector Y: %s\r\n", floatStr);

            floatToString((float)myGravityVector.gravZ, floatStr, 2);
            xil_printf("Gravity Vector Z: %s\r\n", floatStr);
            */
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before trying again
    }
}



void reverseString(char* str, int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

int intToStr(int x, char str[], int d) {
    int i = 0;
    bool isNegative = false;

    if (x < 0) {
        isNegative = true;
        x = -x;
    }

    // Extract characters for the string
    do {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    } while (x);

    // If number of digits required is more, then add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    if (isNegative)
        str[i++] = '-';

    reverseString(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void floatToString(float n, char *res, int afterpoint) {
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // Convert integer part to string
    int i = intToStr(ipart, res, 0);

    // Check for display option after point
    if (afterpoint != 0) {
        res[i] = '.';  // add dot

        // Get the value of fraction part up to given no. of points after dot.
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

int main(void) {
    init_platform();  // Initialize platform
    xil_printf("Program started. Communicating with ESP32...\r\n");

    // Create a FreeRTOS task for UART communication
    xTaskCreate(vUARTCommunicationTask, "UART Communication Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();  // Start the scheduler

    // Cleanup and exit (should never reach here in normal operation)
    cleanup_platform();
    return 0;
}

