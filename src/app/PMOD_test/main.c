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
#include "positionCalculator.h"

// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

void vPositionUpdateTask(void *pvParameters) {
    (void)pvParameters; // Unused parameter

    Position currentPosition; // Structure to hold the current position

    while (1) {

        // Fetch the latest 3D position and update currentPosition
        get3Dposition(&currentPosition);

        // Print the current position to the console
        xil_printf("Current Position: X=%f, Y=%f, Z=%f\r\n", 
                    currentPosition.xPos, currentPosition.yPos, currentPosition.zPos);

        // Delay before the next update
        vTaskDelay(pdMS_TO_TICKS(1000)); // Adjust delay as needed
    }
}


int main(void) {
    init_platform();  // Initialize platform
    xil_printf("Program started. Updating 3D position...\r\n");

    // Create a FreeRTOS task for position updates
    xTaskCreate(vPositionUpdateTask, "Position Update Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();  // Start the scheduler

    // Cleanup and exit (should never reach here in normal operation)
    cleanup_platform();
    return 0;
}

