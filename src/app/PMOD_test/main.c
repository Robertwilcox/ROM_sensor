#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

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

// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

void vUARTCommunicationTask(void *pvParameters) {
    (void)pvParameters;  // Unused parameter
    uint8_t receivedByte;
    bool byteReceived;

    while (1) {

        sendByteToESP32('a');  // Send a test character to ESP32
        xil_printf("Sending byte to ESP32\r\n");

        byteReceived = false;

        while(!byteReceived) {
        	// Receive a char if available
        	if (receiveByteFromESP32(&receivedByte)) {
        	    xil_printf("Received from ESP32: %c\r\n", receivedByte);
        	    byteReceived = true;
        	} else {
        		vTaskDelay(pdMS_TO_TICKS(10)); // Wait for 10ms before checking again
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay
     }
    }


int main(void) {
    init_platform();  // Initialize platform
    xil_printf("Program started. Communicating with ESP32...\r\n");

    // Create a FreeRTOS task for UART communication
    xTaskCreate(vUARTCommunicationTask, "UART Communication Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();  // Start the scheduler

    // Cleanup and exit (should never reach here in normal operation)
    cleanup_platform();
    return 0;
}

