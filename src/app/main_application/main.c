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
#include "semphr.h"

// Alias for Interrupt Controller Peripheral
#define INTC_DEVICE_ID          XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID

// Alias for USB UART Lite Peripheral
#define USB_UART_BASEADDR 		XPAR_AXI_UARTLITE_0_BASEADDR
#define AXI_USB_UART_INTR_NUM   XPAR_MICROBLAZE_0_AXI_INTC_AXI_UARTLITE_0_INTERRUPT_INTR

// Alias for PMOD UART Lite Peripheral
#define PMOD_UART_BASEADDR 		XPAR_AXI_UARTLITE_1_BASEADDR
#define AXI_PMOD_UART_INTR_NUM  XPAR_MICROBLAZE_0_AXI_INTC_AXI_UARTLITE_1_INTERRUPT_INTR

// Alias for AXI Timer Peripheral
#define AXI_TIMER_INTR_NUM      XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR

int init_sys(void);

void vMenuTask(void *pvParameters);

void vDataTask(void *pvParameters);

void vWarmUpTask(void *pvParameters);

void vWorkSetTask(void *pvParameters);

typedef struct {
	uint8_t curr_lift_type;
	uint8_t curr_warmup_mode;
	uint8_t prev_lift_type;
	uint8_t prev_warmup_mode;
	bool    warmup_done;
    bool    workset_done;
} UserInput, *UserInputPtr;

TaskHandle_t xMenu    = NULL;
TaskHandle_t xData    = NULL;
TaskHandle_t xWarmUp  = NULL;
TaskHandle_t xWorkSet = NULL;

xSemaphoreHandle data_lck = 0;

XIntc        irq;

UserInputPtr user_input;
WarmUpSetPtr warmup_set;
WorkSetPtr   work_set;

int main(void) {
    // Initialize platform
    init_platform();

    if (init_sys() != XST_SUCCESS) {
    	cleanup_platform();
    	return -1;
    }

    print("BarBuddy Initializing...\r\n");

    data_lck = xSemaphoreCreateMutex();

    xTaskCreate(vMenuTask, "MENU TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xMenu);
    xTaskCreate(vDataTask, "DATA TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xData);
    xTaskCreate(vWarmUpTask, "WARMUP TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xWarmUp);
    xTaskCreate(vWorkSetTask,"WORKSET TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xWorkSet);


    vTaskSuspend(xData);
    vTaskSuspend(xWarmUp);
    vTaskSuspend(xWorkSet);


    vTaskResume(xMenu);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    cleanup_platform();
    return 0;
}

int init_sys(void) {
	int status;

	// Initialize the Interrupt Controller
	status = XIntc_Initialize(&irq, INTC_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Start the Interrupt Controller such that interrupts are enabled for all
	// devices that cause interrupts
	status = XIntc_Start(&irq, XIN_REAL_MODE);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	// Enable the AXI Timer and UART lite interrupts
	XIntc_Enable(&irq, AXI_TIMER_INTR_NUM);
	XIntc_Enable(&irq, AXI_USB_UART_INTR_NUM);
	XIntc_Enable(&irq, AXI_PMOD_UART_INTR_NUM);

	return XST_SUCCESS;
}

void vMenuTask(void *pvParameters) {
	print("DEBUG: Entered vMenuTask()\r\n");

	for (;;){
		print("INFO:vMenuTask()\tECE 544 Final Project\r\n");
		print("INFO:vMenuTask()\tBy Ibrahim Binmahfood (ibrah5@pdx.edu)\r\n");
		print("INFO:vMenuTask()\tand Robert Wilcox     (wilcox6@pdx.edu)\r\n");

		// Prompt User to select a lift out of list
		print("Select a Lift from the list by the number it is associated with: \r\n");
		print("0 [Squat], 1 [Bench], 2 [Bicep Curl]...\r\n");
		user_input->curr_lift_type = XUartLite_RecvByte(USB_UART_BASEADDR);

		if (isdigit(user_input->curr_lift_type)) {
			// Check if the previous lift type was called again
			if (user_input->prev_lift_type == user_input->curr_lift_type) {
				xil_printf("DEBUG:Lift Type: %d\tWarmUp Set Done: %s\r\n",
						user_input->prev_lift_type,
						user_input->warmup_done ? "true" : "false");

				vTaskResume(xWorkSet);

				vTaskSuspend(xWarmUp);
				vTaskSuspend(xData);

				vTaskSuspend(NULL); // Suspend vMenuTask
			}

			// Prompt User to select Warm Up MODE for that lift
			print("Select WarmUp MODE [y/n]: \r\n");
			user_input->curr_warmup_mode = XUartLite_RecvByte(USB_UART_BASEADDR);

			if (user_input->curr_warmup_mode == 'y') {
				// Set current values to previous values
			    user_input->prev_lift_type   = user_input->curr_lift_type;
				user_input->prev_warmup_mode = user_input->curr_warmup_mode;

				vTaskResume(xWarmUp);

				vTaskSuspend(xData);
				vTaskSuspend(xWorkSet);

				vTaskSuspend(NULL);	// Suspend vMenuTask
			}

			else {

				print("Require to Select 'y' for WarmUp MODE to record a Lift's WarmUp Set\r\n");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vDataTask(void *pvParameters) {
	print("DEBUG:Entered vDataTask()\r\n");

	for (;;){
		if (xSemaphoreTake(data_lck, 1000)) {
            print("DEBUG:Mutex Taken\r\n");

        }

        else {
	        print("DEBUG:Mutex In Use\r\n"); 
        }


		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vWarmUpTask(void *pvParameters) {
	print("DEBUG:Entered vWarmUpTask()\r\n");

	for (;;){
        if (xSemaphoreTake(data_lck, 1000)) {
            print("DEBUG:Mutex Taken\r\n");
            WarmUpSet_LogRep(warmup_set);
		    user_input->warmup_done = true;
            xSemaphoreGive(data_lck); 
        }
        else {
	        print("DEBUG:Mutex In Use\r\n"); 
        }
		
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vWorkSetTask(void *pvParameters) {
	print("DEBUG:Entered vWorkSetTask()\r\n");

	for (;;){
        if (xSemaphoreTake(data_lck, 1000)) {
	        print("DEBUG:Mutex Taken\r\n");
            WarmUpSet_LogRep(work_set);
            xSemaphoreGive(data_lck);
        }
        else {
	        print("DEBUG:Mutex In Use\r\n"); 
        }
		

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
