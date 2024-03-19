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
#include "esp32_BLE_uart.h"
#include "packetReader.h"
#include "updatePandV.h"
#include "nexys4IO.h"

#include "semphr.h"



// Alias for Interrupt Controller Peripheral
#define INTC_DEVICE_ID          XPAR_MICROBLAZE_0_AXI_INTC_DEVICE_ID

// Alias for USB UART Lite Peripheral
#define USB_UART_BASEADDR       XPAR_AXI_UARTLITE_0_BASEADDR
#define AXI_USB_UART_INTR_NUM   XPAR_MICROBLAZE_0_AXI_INTC_AXI_UARTLITE_0_INTERRUPT_INTR

// Alias for PMOD UART Lite Peripheral
#define PMOD_UART_BASEADDR      XPAR_AXI_UARTLITE_1_BASEADDR
#define AXI_PMOD_UART_INTR_NUM  XPAR_MICROBLAZE_0_AXI_INTC_AXI_UARTLITE_1_INTERRUPT_INTR

// Alias for AXI Timer Peripheral
#define AXI_TIMER_INTR_NUM      XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMER_0_INTERRUPT_INTR


// Alias for UART Lite Peripheral
#define UART0_BASEADDR XPAR_AXI_UARTLITE_0_BASEADDR
#define UART1_BASEADDR XPAR_AXI_UARTLITE_1_BASEADDR

#define MAX_PACKET_SIZE 20
#define START_MARKER 0x02
#define END_MARKER 0x03

// Aliases for NEXYS4IO Peripheral
#define N4IO_DEVICE_ID          XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR           XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR           XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

volatile bool transmitData = true; // Global flag to control data transmission

void reverseString(char* str, int length);
int intToStr(int x, char str[], int d);
void floatToString(float n, char *res, int afterpoint);

DataPoint myDataPoint;

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

void Exercise(DataPoint* inst_ptr, MaxMinData* cmp_inst_ptr);

void vMenuTask(void *pvParameters);

void vWarmUpTask(void *pvParameters);

void vWorkSetTask(void *pvParameters);

void vSwitchMonitorTask(void *pvParameters);

void vUARTCommunicationTask(void *pvParameters);

TaskHandle_t xMenu    = NULL;
TaskHandle_t xWarmUp  = NULL;
TaskHandle_t xWorkSet = NULL;
TaskHandle_t xSwitchMonitor = NULL;
TaskHandle_t xUartTest = NULL;

xSemaphoreHandle data_lck = 0;

XIntc         irq;

UserInputPtr user_input;
DataPointPtr warmup_set;
DataPointPtr work_set;
MaxMinDataPtr warmup_max_min;
MaxMinDataPtr work_set_max_min;


static float      prev_velocity            = 0.0f; // Previous velocity of the primary axis (z-axis)
static TickType_t repStartTime             = 0;
static int        repCount                 = 0;
static int        directionChangeCount     = 0; // Counts consecutive velocity readings in the new direction
static const int  directionChangeThreshold = 3; // Threshold for confirming a direction change
static TickType_t totalRepDuration         = 0; // Accumulates total duration of all reps
static TickType_t avgRepDuration           = 0; // Avg duration of all reps

DataPoint myDataPoint;


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

void LogRepCount(DataPoint* inst_ptr) {
    xil_printf("DEBUG: Entered LogRepCount()\r\n");

    // Buffer to hold the string representation of floats
    char floatStr[20];

    if(transmitData){
    	xil_printf("Should print packet\r\n");

		if (readPacket()) { // Check if a packet was read successfully

			xil_printf("\r\n");

			getPositionAndVelocity(&myDataPoint);

			// Convert position values to strings and print
			floatToString(myDataPoint.x_pos, floatStr, 2);
			xil_printf("%s ", floatStr);

			floatToString(myDataPoint.y_pos, floatStr, 2);
			xil_printf("%s ", floatStr);

			floatToString(myDataPoint.z_pos, floatStr, 2);
			xil_printf("%s\r", floatStr);

		}
		if(!readPacket()) {
			xil_printf("PACKET NOT READ\r\n");
		}
	}


    // Directly use z-axis velocity, as it's the primary axis for all exercises
    float primary_velocity = inst_ptr->z_veloc;

    // Check for a potential change in direction by looking for a zero crossing in velocity
    if ((prev_velocity <= 0 && primary_velocity > 0) || (prev_velocity > 0 && primary_velocity <= 0)) {

        // Increment the count of potential direction changes
        directionChangeCount++;
    } else {

        // If the velocity continues in the same direction, reset the potential direction change count
        if (directionChangeCount > 0 && ((primary_velocity > 0 && prev_velocity > 0) || (primary_velocity <= 0 && prev_velocity <= 0))) {
            directionChangeCount = 0; // Reset count if we continue in the same direction
        }
    }

    // Confirm the change in direction if we have seen enough consecutive readings
    if (directionChangeCount >= directionChangeThreshold) {

        // Confirmed direction change; handle as a rep
        if (repStartTime != 0) { // Ensure this isn't the first data point

            TickType_t currentTick = xTaskGetTickCount();
            TickType_t repDuration = currentTick - repStartTime;

            // Accumulate total rep duration
            totalRepDuration += repDuration;
            repCount++;

            // Calculate the average rep duration
            avgRepDuration = totalRepDuration / repCount;

            char debugMessage[100];
            xil_printf(debugMessage, sizeof(debugMessage), "Rep %d detected. Duration: %lu ticks. Average Duration: %lu ticks.\r\n", repCount, repDuration, avgRepDuration);
            print(debugMessage);

            // Reset rep start time for the next rep
            repStartTime = currentTick;
        } else {
            // First confirmed direction change, start tracking time
            repStartTime = xTaskGetTickCount();
        }
        directionChangeCount = 0; // Reset count after confirming a rep
    }

    // Update for next call
    prev_velocity = primary_velocity;
    //xil_printf("DEBUG: After logRepCount()\r\n");
}

void vUARTCommunicationTask(void *pvParameters) {
    xil_printf("UART Communication Task Started\r\n");

    // Buffer to hold the string representation of floats
    char floatStr[20]; // Adjust the size based on your needs

    while (1) {
        if(transmitData){
            if (readPacket()) { // Check if a packet was read successfully

                xil_printf("\r\n");

                getPositionAndVelocity(&myDataPoint);

                // Convert position values to strings and print
                floatToString(myDataPoint.x_pos, floatStr, 2);
                xil_printf("%s ", floatStr);

                floatToString(myDataPoint.y_pos, floatStr, 2);
                xil_printf("%s ", floatStr);

                floatToString(myDataPoint.z_pos, floatStr, 2);
                xil_printf("%s\r", floatStr);

                /*-
                // Convert velocity values to strings and print
                floatToString(myDataPoint.x_veloc, floatStr, 2);
                xil_printf("Velocity X: %s\r\n", floatStr);

                floatToString(myDataPoint.y_veloc, floatStr, 2);
                xil_printf("Velocity Y: %s\r\n", floatStr);

                floatToString(myDataPoint.z_veloc, floatStr, 2);
                xil_printf("Velocity Z: %s\r\n", floatStr);
                */
            }
        }


        vTaskDelay(pdMS_TO_TICKS(10)); // Wait a bit before trying again
    }
}


void Exercise(DataPoint* inst_ptr, MaxMinData* cmp_inst_ptr){
    xil_printf("DEBUG: Entered Exercise()\r\n");

    MaxMinData deviate; // store the deviated positions and velocities locally

    // Update the DataPoint struct with current position and velocity
    // Logging the Rep
    LogRepCount(inst_ptr);

    /*

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

    // Multiply the deviation for the velocities and positions
    // Let the deviation be split by 2 for the range in mind (min to max)
    // deviated max or min = ((deivation / 2) + 1) * current_max or current_min
    deviate.x_pos_max = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->x_pos_max);
    deviate.y_pos_max = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->y_pos_max);
    deviate.z_pos_max = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->z_pos_max);

    deviate.x_veloc_max = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->x_veloc_max);
    deviate.y_veloc_max = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->y_veloc_max);
    deviate.z_veloc_max = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->z_veloc_max);

    deviate.x_pos_min = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->x_pos_min);
    deviate.y_pos_min = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->y_pos_min);
    deviate.z_pos_min = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->z_pos_min);

    deviate.x_veloc_min = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->x_veloc_min);
    deviate.y_veloc_min = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->y_veloc_min);
    deviate.z_veloc_min = (atoi((char*)user_input->workset_deviate) / 2 + 1) * (cmp_inst_ptr->z_veloc_min);

    // Check the user's current {x, y, z} positions
    // align within the range of the deviated max/min positions
    if (inst_ptr->x_pos < deviate.x_pos_min) {
        xil_printf("Position is under the selected deviation in the x axis\r\n");
    }
    else if (inst_ptr->x_pos > deviate.x_pos_max) {
        xil_printf("Position is over the selected deviation in the x axis\r\n");
    }
    else if (inst_ptr->y_pos < deviate.y_pos_min) {
        xil_printf("Position is under the selected deviation in the y axis\r\n");
    }
    else if (inst_ptr->y_pos > deviate.y_pos_max) {
        xil_printf("Position is over the selected deviation in the y axis\r\n");
    }
    else if (inst_ptr->z_pos < deviate.z_pos_min) {
        xil_printf("Position is under the selected deviation in the z axis\r\n");
    }
    else if (inst_ptr->z_pos > deviate.z_pos_max) {
        xil_printf("Position is over the selected deviation in the z axis\r\n");
    }
    else {
        xil_printf("Positions within the range of the selected deviation!\r\n");
    }

    // Check the user's current {x, y, z} velocities
    // align within the range of the deviated max/min velocities
    if (inst_ptr->x_veloc < deviate.x_veloc_min) {
        xil_printf("Velocity is under the selected deviation in the x axis\r\n");
    }
    else if (inst_ptr->x_veloc > deviate.x_veloc_max) {
        xil_printf("Velocity is over the selected deviation in the x axis\r\n");
    }
    else if (inst_ptr->y_veloc < deviate.y_veloc_min) {
        xil_printf("Velocity is under the selected deviation in the y axis\r\n");
    }
    else if (inst_ptr->y_veloc > deviate.y_veloc_max) {
        xil_printf("Velocity is over the selected deviation in the y axis\r\n");
    }
    else if (inst_ptr->z_veloc < deviate.z_veloc_min) {
        xil_printf("Velocity is under the selected deviation in the z axis\r\n");
    }
    else if (inst_ptr->z_veloc > deviate.z_veloc_max) {
        xil_printf("Velocity is over the selected deviation in the z axis\r\n");
    }
    else {
        xil_printf("Velocities within the range of the selected deviation!\r\n");
    }
    */
}

void vMenuTask(void *pvParameters) {
    xil_printf("DEBUG: Entered vMenuTask()\r\n");

    for (;;){
        xil_printf("INFO:vMenuTask()\tECE 544 Final Project\r\n");
        xil_printf("INFO:vMenuTask()\tBy Ibrahim Binmahfood (ibrah5@pdx.edu)\r\n");
        xil_printf("INFO:vMenuTask()\tand Robert Wilcox     (wilcox6@pdx.edu)\r\n");

        // Prompt User to select a lift out of list
        xil_printf("Select a Lift from the list by the number it is associated with: \r\n");
        xil_printf("0 [Squat], 1 [Bench], 2 [Bicep Curl]...\r\n");
        user_input->curr_lift_type = XUartLite_RecvByte(USB_UART_BASEADDR);

        switch(user_input->curr_lift_type) {
            case '0': // SQUAT
                const char* squatCommand = "SQUAT\n";
                for (int i = 0; squatCommand[i] != '\0'; i++) {
                    sendByteToESP32(squatCommand[i]);
                }
                break;
            case '1': // BENCH_PRESS
                const char* benchCommand = "BENCH_PRESS\n";
                for (int i = 0; benchCommand[i] != '\0'; i++) {
                    sendByteToESP32(benchCommand[i]);
                }
                break;
            case '2': // BICEP_CURL
                const char* curlCommand = "BICEP_CURL\n";
                for (int i = 0; curlCommand[i] != '\0'; i++) {
                    sendByteToESP32(curlCommand[i]);
                }
                break;
            default:
                xil_printf("Invalid selection. Please select 0, 1, or 2.\r\n");
                break;
        }

        if (isdigit(user_input->curr_lift_type)) {
            // Check if the previous lift type was called again
            if (user_input->prev_lift_type == user_input->curr_lift_type) {
                xil_printf("DEBUG:Lift Type: %d\tWarmUp Set Done: %s\tWork Set Done: %s\r\n",
                        user_input->prev_lift_type,
                        user_input->warmup_done ? "true" : "false",
                        user_input->workset_done ? "true" : "false");

                // Prompt User to select amount of deviation for Work Set
                xil_printf("Select a Deviation from the range below: \r\n");
                xil_printf("1%% <= Deviation <= 5%%\nNOTE: Enter as a number\r\n");
                user_input->workset_deviate = XUartLite_RecvByte(USB_UART_BASEADDR);

                // Check if it is a digit and deviation range acceptable
                if (isdigit(user_input->workset_deviate)             &&
                    ((atoi((char*)user_input->workset_deviate) >= 1) ||
                    (atoi((char*)user_input->workset_deviate) <= 5))  ) {
                    xil_printf("DEBUG:User Entered Work Set Deviation: %d\r\n",
                            user_input->workset_deviate);

                    vTaskResume(xWorkSet);
                    vTaskSuspend(xWarmUp);

                    vTaskSuspend(NULL); // Suspend vMenuTask
                }

                else {
                    xil_printf("Deviation value must be a digit from [1, 5] inclusive r\n");
                }
            }

            // Prompt User to select Warm Up MODE for that lift
            xil_printf("Select WarmUp MODE [y/n]: \r\n");
            user_input->curr_warmup_mode = XUartLite_RecvByte(USB_UART_BASEADDR);

            if (user_input->curr_warmup_mode == 'y') {
                // Set current values to previous values
                user_input->prev_lift_type   = user_input->curr_lift_type;
                user_input->prev_warmup_mode = user_input->curr_warmup_mode;

                vTaskResume(xUartTest);
                vTaskSuspend(xWorkSet);

                vTaskSuspend(NULL); // Suspend vMenuTask
            }

            else {
                user_input->warmup_done = false;
                user_input->workset_done = false;
                xil_printf("Require to Select 'y' for WarmUp MODE to record a Lift's WarmUp Set\r\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

void vWarmUpTask(void *pvParameters) {
    xil_printf("DEBUG:Entered vWarmUpTask()\r\n");

    for (;;){
        if(transmitData) {
        	/*
            // Send start command to ESP32
            const char* startCommand = "START\n";
            for (int i = 0; startCommand[i] != '\0'; i++) {
                sendByteToESP32(startCommand[i]);
            }*/
            if (1) {
                //print("DEBUG:Mutex Taken\r\n");
                Exercise(warmup_set, warmup_max_min);
                user_input->warmup_done = true;
                //xSemaphoreGive(data_lck);
            }
            else {
                xil_printf("DEBUG:Mutex In Use\r\n");
            }
        } else {
            xil_printf("DEBUG:Data collection paused, returning to menu\r\n");
            vTaskResume(xMenu);
            vTaskSuspend(NULL); // Suspend vWorkSetTask
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void vWorkSetTask(void *pvParameters) {
    xil_printf("DEBUG:Entered vWorkSetTask()\r\n");
    for (;;){
        if(transmitData) {
            // Send start command to ESP32
            const char* startCommand = "START\n";
            for (int i = 0; startCommand[i] != '\0'; i++) {
                sendByteToESP32(startCommand[i]);
            }
            if (xSemaphoreTake(data_lck, 1000)) {
                xil_printf("DEBUG:Mutex Taken\r\n");
                Exercise(work_set, work_set_max_min);
                user_input->workset_done = true;
                xSemaphoreGive(data_lck);

                vTaskResume(xMenu);
                vTaskSuspend(NULL); // Suspend vWorkSetTask
            }
            else {
                xil_printf("DEBUG:Mutex In Use\r\n");
            }
        } else {
            xil_printf("DEBUG:Data collection paused, returning to menu\r\n");
            vTaskResume(xMenu);
            vTaskSuspend(NULL); // Suspend vWorkSetTask
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

// Task to monitor switch 0 to control data transmission
void vSwitchMonitorTask(void *pvParameters) {
    uint16_t switchStatePrev = 0;
    uint16_t switchState = 0;

    while (1) {
        switchState = NX4IO_getSwitches(); // Get the current state of the switches

        // Handle START/STOP data transmission based on switch 0
        if ((switchState & 0x0001) != (switchStatePrev & 0x0001)) {
            if (switchState & 0x0001) { // If switch 0 is ON (STOP)
                const char* stopCommand = "STOP\n";
                for (int i = 0; stopCommand[i] != '\0'; i++) {
                    sendByteToESP32(stopCommand[i]);
                }
                transmitData = false; // Stop data transmission
                xil_printf("Data transmission stopped.\n");
            } else { // If switch 0 is OFF (START)
                const char* startCommand = "START\n";
                for (int i = 0; startCommand[i] != '\0'; i++) {
                    sendByteToESP32(startCommand[i]);
                }
                transmitData = true; // Resume data transmission
                xil_printf("Data transmission resumed.\n");
            }
        }

        // Handle data type change based on switch 1 without affecting data transmission
        if ((switchState & 0x0002) != (switchStatePrev & 0x0002)) {
            if (switchState & 0x0002) { // If switch 1 is ON (WORKING mode)
                const char* workingCommand = "WORKING\n";
                for (int i = 0; workingCommand[i] != '\0'; i++) {
                    sendByteToESP32(workingCommand[i]);
                }
                xil_printf("Switched to WORKING data mode.\n");
            } else { // If switch 1 is OFF (WARMUP mode)
                const char* warmupCommand = "WARMUP\n";
                for (int i = 0; warmupCommand[i] != '\0'; i++) {
                    sendByteToESP32(warmupCommand[i]);
                }
                xil_printf("Switched to WARMUP data mode.\n");
            }
        }

        switchStatePrev = switchState; // Update previous state
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for debouncing
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

    // Handle negative numbers
    bool isNegative = n < 0;
    if (isNegative) {
        n = -n; // Make n positive
    }

    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // Convert integer part to string
    int i = intToStr(ipart, res, 0);

    // If the number is negative, shift everything one place to the right
    // to insert the negative sign at the beginning
    if (isNegative) {
        for (int j = i; j >= 0; j--) {
            res[j + 1] = res[j];
        }
        res[0] = '-';
        i++; // Increase string length by one
    }

    // Check for display option after point
    if (afterpoint != 0) {
        res[i] = '.'; // add dot

        // Get the value of fraction part up to given no. of points after dot.
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)(fpart + 0.5), res + i + 1, afterpoint); // Adding 0.5 for rounding off purpose
    }
}

int main(void) {

	/* ROBERTS MAIN CODE
    init_platform();  // Initialize platform
    xil_printf("Program started. Communicating with ESP32...\r\n");

    // Initialize NX4IO
    int status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Example: Send a "BENCH_PRESS" command to start data flow
    const char* initCommand = "BENCH_PRESS\n";
    for (int i = 0; initCommand[i] != '\0'; i++) {
        sendByteToESP32(initCommand[i]);
    }

    // Create a FreeRTOS task for UART communication
    xTaskCreate(vUARTCommunicationTask, "UART Communication Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Create a FreeRTOS task for monitoring the switch state
    xTaskCreate(vSwitchMonitorTask, "Switch Monitor Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();  // Start the scheduler

    // Cleanup and exit (should never reach here in normal operation)
    cleanup_platform();
    return 0;
    */

    // IBS MAIN CODE
    // Initialize platform
    init_platform();

    if (init_sys() != XST_SUCCESS) {
        cleanup_platform();
        return -1;
    }

    // Iniitialize NX4IO
    int status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    xil_printf("BarBuddy Initializing...\r\n");

    const char* initCommand = "BENCH_PRESS\n";
    for (int i = 0; initCommand[i] != '\0'; i++) {
        sendByteToESP32(initCommand[i]);
    }


    data_lck = xSemaphoreCreateMutex();

    xTaskCreate(vMenuTask, "MENU TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xMenu);
    //xTaskCreate(vWarmUpTask, "WARMUP TASK", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xWarmUp);
    //xTaskCreate(vWorkSetTask,"WORKSET TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xWorkSet);
    xTaskCreate(vUARTCommunicationTask, "UART Communication Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &xUartTest);
    xTaskCreate(vSwitchMonitorTask, "Switch Monitor Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &xSwitchMonitor);

    //vTaskSuspend(xWarmUp);
   // vTaskSuspend(xWorkSet);
      vTaskSuspend(xUartTest);



    vTaskResume(xMenu);

    // Start the scheduler
    vTaskStartScheduler();

    // Should never reach here
    cleanup_platform();
    return 0;

}
