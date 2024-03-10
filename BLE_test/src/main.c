#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "xuartlite.h"
#include "xparameters.h"
#include "platform.h"

// Alias for UART Lite Peripheral 
#define UART_BASEADDR           XPAR_AXI_UARTLITE_0_BASEADDR

/**
 * @brief Task for reading from the UART and printing the received characters.
 * 
 * This task continuously reads characters from the UART input and echoes them back to the UART output.
 * It uses polling to check for new characters, which simplifies the implementation at the cost of CPU efficiency.
 * 
 * @param pvParameters Parameters for the task, not used.
 */
void vUARTReadAndPrintTask(void *pvParameters) {
    (void)pvParameters; // Unused parameter

    char ch;

    for (;;) {
        // Check if data is available in the UART receive FIFO
        if (XUartLite_IsReceiveEmpty(UART_BASEADDR) == FALSE) {
            // Read a character from UART
            ch = XUartLite_RecvByte(UART_BASEADDR);

            // Echo the character back to UART
            XUartLite_SendByte(UART_BASEADDR, ch);
        }

        // Yield to other tasks
        taskYIELD();
    }
}

/**
 * @brief Main function where execution begins.
 *
 * This function initializes the platform, creates the UART read-and-print task, and starts the FreeRTOS scheduler.
 *
 * @return Should never return under normal operation.
 */
int main(void) {
    // Initialize platform
    init_platform();

    // Create the UART read-and-print task
    xTaskCreate(vUARTReadAndPrintTask, "UART Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // In case the scheduler returns for some reason, clean up and exit
    cleanup_platform();
    return 0;
}
