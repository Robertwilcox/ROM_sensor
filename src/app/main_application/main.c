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

// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

/**
 * @brief Task for reading from UART0 and sending the received characters to UART1.
 *
 * @param pvParameters Parameters for the task, not used.
 */
/*void vUART0ToUART1TransferTask(void *pvParameters) {
    (void)pvParameters; // Unused parameter
    char ch;

    for (;;) {
        if (XUartLite_IsReceiveEmpty(UART0_BASEADDR) == FALSE) {
            ch = XUartLite_RecvByte(UART0_BASEADDR);
            XUartLite_SendByte(UART1_BASEADDR, ch);
        }
        taskYIELD();
    }
}*/

/**
 * @brief Task for monitoring UART1 and printing any received data.
 *
 * @param pvParameters Parameters for the task, not used.
 */
void vMonitorUART1Task(void *pvParameters) {
    (void)pvParameters; // Unused parameter
    char ch, c;
    char test1 = 'a';
    char test2 = 'b';
    char newline = '\n';
    xil_printf("Monitoring\r\n");
    for (;;) {

    	XUartLite_SendByte(UART1_BASEADDR, test1);
    	xil_printf("Sending UART1: %c\r\n", test1);

    	XUartLite_SendByte(UART1_BASEADDR, newline);
    	xil_printf("Sending UART1: %c\r\n", newline);

    	XUartLite_SendByte(UART0_BASEADDR, test2);
    	xil_printf("Sending UART0: %c\r\n", test2);

    	XUartLite_SendByte(UART0_BASEADDR, newline);
    	xil_printf("Sending UART0: %c\r\n", newline);

    	if (XUartLite_IsReceiveEmpty(UART1_BASEADDR) == FALSE) {
    	   ch = XUartLite_RecvByte(UART1_BASEADDR);
    	   xil_printf("UART1 Received: %c\r\n", ch);
    	}
    	if (XUartLite_IsReceiveEmpty(UART0_BASEADDR) == FALSE) {
    	   c = XUartLite_RecvByte(UART0_BASEADDR);
    	   xil_printf("UART0 Received: %c\r\n", c);
    	}
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Main function where execution begins.
 */
int main(void) {
    // Initialize platform
    init_platform();
    //int c = 8;
    xil_printf("Program started. Awaiting UART0 input...\r\n");
    //XUartLite_SendByte(UART0_BASEADDR, c);

    // Create tasks
   // xTaskCreate(vUART0ToUART1TransferTask, "UART0toUART1 Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vMonitorUART1Task, "Monitor UART1 Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    cleanup_platform();
    return 0;
}
