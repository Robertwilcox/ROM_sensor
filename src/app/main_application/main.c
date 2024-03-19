#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
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

typedef struct {
	uint8_t curr_lift_type;
	uint8_t curr_warmup_mode;
	uint8_t prev_lift_type;
	uint8_t prev_warmup_mode;
    uint8_t workset_deviate;

	bool    warmup_done;
    bool    workset_done;
} UserInput, *UserInputPtr;

typedef struct {
	float x_pos;
	float y_pos;
	float z_pos;

	float x_veloc;
	float y_veloc;
	float z_veloc;
} DataPoint, *DataPointPtr;

typedef struct {
	float x_pos_max;
	float x_pos_min;

	float y_pos_max;
	float y_pos_min;

	float z_pos_max;
	float z_pos_min;

	float x_veloc_max;
	float x_veloc_min;

	float y_veloc_max;
	float y_veloc_min;

	float z_veloc_max;
	float z_veloc_min;
} MaxMinData, *MaxMinDataPtr;

int init_sys(void);

void reverseString(char* str, int length);

int intToStr(int x, char str[], int d);

void floatToString(float n, char *res, int afterpoint);

void getCurrentDataPoints(DataPoint* inst_ptr);

void vMenuTask(void *pvParameters);

void vWarmUpTask(void *pvParameters);

void vWorkSetTask(void *pvParameters);

TaskHandle_t xMenu    = NULL;
TaskHandle_t xWarmUp  = NULL;
TaskHandle_t xWorkSet = NULL;

xSemaphoreHandle data_lck = 0;

XIntc         irq;

UserInputPtr  user_input;
DataPointPtr  warmup_set;
DataPointPtr  work_set;
MaxMinDataPtr warmup_max_min;
MaxMinDataPtr work_set_max_min;

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
    xTaskCreate(vWarmUpTask, "WARMUP TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xWarmUp);
    xTaskCreate(vWorkSetTask,"WORKSET TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xWorkSet);

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

void getCurrentDataPoints(DataPoint* inst_ptr, MaxMinData* cmp_inst_ptr){
	print("DEBUG: Entered getCurrentDataPoint\r\n");
    
    // Update the DataPoint struct with current position and velocity
    getPositionandVelocity(&inst_ptr);

    // Find the max and min of the positions and velocities
    cmp_inst_ptr->x_pos_max = (cmp_inst_ptr->x_pos_max > inst_ptr->x_pos) ?
                               cmp_inst_ptr->x_pos_max : inst_ptr->x_pos;
    cmp_inst_ptr->y_pos_max = (cmp_inst_ptr->y_pos_max > inst_ptr->y_pos) ?
                               cmp_inst_ptr->y_pos_max : inst_ptr->y_pos;
    cmp_inst_ptr->z_pos_max = (cmp_inst_ptr->z_pos_max > inst_ptr->z_pos) ?
                               cmp_inst_ptr->z_pos_max : inst_ptr->z_pos;
    
    cmp_inst_ptr->x_pos_min = (cmp_inst_ptr->x_pos_min < inst_ptr->x_pos) ?
                               cmp_inst_ptr->x_pos_min : inst_ptr->x_pos;
    cmp_inst_ptr->y_pos_min = (cmp_inst_ptr->y_pos_min < inst_ptr->y_pos) ?
                               cmp_inst_ptr->y_pos_min : inst_ptr->y_pos;
    cmp_inst_ptr->z_pos_min = (cmp_inst_ptr->z_pos_min < inst_ptr->z_pos) ?
                               cmp_inst_ptr->z_pos_min : inst_ptr->z_pos;

    cmp_inst_ptr->x_veloc_max = (cmp_inst_ptr->x_veloc_max > inst_ptr->x_veloc) ?
                                 cmp_inst_ptr->x_veloc_max : inst_ptr->x_veloc;
    cmp_inst_ptr->y_veloc_max = (cmp_inst_ptr->y_veloc_max > inst_ptr->y_veloc) ?
                                 cmp_inst_ptr->y_veloc_max : inst_ptr->y_veloc;
    cmp_inst_ptr->z_veloc_max = (cmp_inst_ptr->z_veloc_max > inst_ptr->z_veloc) ?
                                 cmp_inst_ptr->z_veloc_max : inst_ptr->z_veloc;

    cmp_inst_ptr->x_veloc_min = (cmp_inst_ptr->x_veloc_min < inst_ptr->x_veloc) ?
                                 cmp_inst_ptr->x_veloc_min : inst_ptr->x_veloc;
    cmp_inst_ptr->y_veloc_min = (cmp_inst_ptr->y_veloc_min < inst_ptr->y_veloc) ?
                                 cmp_inst_ptr->y_veloc_min : inst_ptr->y_veloc;
    cmp_inst_ptr->z_veloc_min = (cmp_inst_ptr->z_veloc_min < inst_ptr->z_veloc) ?
                                 cmp_inst_ptr->z_veloc_min : inst_ptr->z_veloc;

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
				xil_printf("DEBUG:Lift Type: %d\tWarmUp Set Done: %s\tWork Set Done: %s\r\n",
						user_input->prev_lift_type,
						user_input->warmup_done ? "true" : "false",
						user_input->workset_done ? "true" : "false");

                // Prompt User to select amount of deviation for Work Set
		        print("Select a Deviation from the range below: \r\n");
		        print("1%% <= Deviation <= 5%%\nNOTE: Enter as a number\r\n");
		        user_input->workset_deviate = XUartLite_RecvByte(USB_UART_BASEADDR);
                
                // Check if it is a digit and deviation range acceptable
                if (isdigit(user_input->workset_deviate)      && 
                    ((atoi(user_input->workset_deviate) >= 1) ||
                    (atoi(user_input->workset_deviate) < = 5))  ) {
                    xil_printf("DEBUG:User Entered Work Set Deviation: %d\r\n", 
                            user_input->workset_deviate);
                    
                    vTaskResume(xWorkSet);
				    vTaskSuspend(xWarmUp);

				    vTaskSuspend(NULL); // Suspend vMenuTask
                }

                else {
                    print("Deviation value must be a digit from [1, 5] inclusive r\n");
                }	
			}

			// Prompt User to select Warm Up MODE for that lift
			print("Select WarmUp MODE [y/n]: \r\n");
			user_input->curr_warmup_mode = XUartLite_RecvByte(USB_UART_BASEADDR);

			if (user_input->curr_warmup_mode == 'y') {
				// Set current values to previous values
			    user_input->prev_lift_type   = user_input->curr_lift_type;
				user_input->prev_warmup_mode = user_input->curr_warmup_mode;

				vTaskResume(xWarmUp);
				vTaskSuspend(xWorkSet);

				vTaskSuspend(NULL);	// Suspend vMenuTask
			}

			else {
				user_input->warmup_done = false;
				user_input->workset_done = false;
				print("Require to Select 'y' for WarmUp MODE to record a Lift's WarmUp Set\r\n");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vWarmUpTask(void *pvParameters) {
	print("DEBUG:Entered vWarmUpTask()\r\n");

	for (;;){
        if (xSemaphoreTake(data_lck, 1000)) {
            print("DEBUG:Mutex Taken\r\n");
            getCurrentDataPoints(warmup_set, warmup_max_min);
		    user_input->warmup_done = true;
            xSemaphoreGive(data_lck);

            vTaskResume(xMenu);
            vTaskSuspend(NULL);	// Suspend vWarmUpTask
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
	        getCurrentDataPoints(work_set, work_set_max_min);
	        user_input->workset_done = true;
            xSemaphoreGive(data_lck);

            vTaskResume(xMenu);
            vTaskSuspend(NULL);	// Suspend vWorkSetTask
        }
        else {
	        print("DEBUG:Mutex In Use\r\n"); 
        }
		
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
