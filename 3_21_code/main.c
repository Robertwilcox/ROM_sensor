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
#include "queue.h"

#define USB_UART_BASEADDR       XPAR_AXI_UARTLITE_0_BASEADDR

// Aliases for NEXYS4IO Peripheral
#define N4IO_DEVICE_ID          XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR           XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR           XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

volatile bool transmitData = false; // Global flag to control data transmission
volatile bool WarmUp_flag = true;


void reverseString(char* str, int length);
int intToStr(int x, char str[], int d);
void floatToString(float n, char *res, int afterpoint);


void reverseString(char* str, int length);

int intToStr(int x, char str[], int d);

void floatToString(float n, char *res, int afterpoint);


void vMenuTask(void *pvParameters);

void vUARTCommunicationTask(void *pvParameters);

typedef struct {
    uint8_t curr_lift_type;
    uint8_t curr_warmup_mode;

} UserInput, *UserInputPtr;

typedef enum {
    STATE_DATA_GATHERING,
    STATE_WARM_UP,
    STATE_MENU,
    STATE_REPCOUNT,
    STATE_WORKING
} SystemState;

typedef struct {
    float prev_velocity;                // Last velocity to detect changes in direction
    int directionChangeCount;           // Count of potential direction changes
    int repCount;                       // Total number of repetitions
    TickType_t repStartTime;            // Start time of the current rep
    unsigned long totalRepDuration;     // Total duration of all reps
    unsigned long avgRepDuration;       // Average duration per rep
} RepCounter;

typedef struct {
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    float minVelX, maxVelX;
    float minVelY, maxVelY;
    float minVelZ, maxVelZ;
} Extremes;

volatile Extremes dataExtremes = {
    .minX = 10000, .maxX = -1000,
    .minY = 10000, .maxY = -1000,
    .minZ = 10000, .maxZ = -1000,
    .minVelX = 10000, .maxVelX = -1000,
    .minVelY = 10000, .maxVelY = -1000,
    .minVelZ = 10000, .maxVelZ = -1000
};


volatile SystemState currentState = STATE_MENU;

DataPoint currentSensorData;    // The most current sensor data

QueueHandle_t dataPointQueue0;   // Queue for sensor data
//QueueHandle_t dataPointQueue1;   // Queue for sensor data

UserInput user_input;           // Struct for user input

RepCounter repCounter = {0}; // Initialize all members to zero

static const int  directionChangeThreshold = 3; // Threshold for confirming a direction change

TaskHandle_t xDataAcquisitionTaskHandle = NULL;
TaskHandle_t xDataPrintTaskHandle = NULL;
TaskHandle_t xMenu = NULL;
TaskHandle_t xSwitchMonitor = NULL;
TaskHandle_t xRepCounter = NULL;

/* THIS IS NOT WORKING
void LogRepCount(void *pvParameters) {
    RepCounter *repCounter = (RepCounter *)pvParameters;
    DataPoint currentData;

    while (1) {
        if (xQueueReceive(dataPointQueue1, &currentData, portMAX_DELAY) == pdPASS) {
            float primary_velocity = currentData.z_veloc;

            if ((repCounter->prev_velocity <= 0 && primary_velocity > 0) || (repCounter->prev_velocity > 0 && primary_velocity <= 0)) {
                repCounter->directionChangeCount++;

            } else if (repCounter->directionChangeCount > 0) {
                repCounter->directionChangeCount = 0;
            }

            if (repCounter->directionChangeCount >= directionChangeThreshold) {
                if (repCounter->repStartTime != 0) {
                    TickType_t currentTick = xTaskGetTickCount();
                    TickType_t repDuration = currentTick - repCounter->repStartTime;

                    repCounter->totalRepDuration += repDuration;
                    repCounter->repCount++;
                    repCounter->avgRepDuration = repCounter->totalRepDuration / repCounter->repCount;

                        xil_printf("Successufully in repCount\r\n");
                        xil_printf("Rep %d detected. Duration: %lu ticks. Average Duration: %lu ticks.\r\n", repCounter->repCount, repDuration, repCounter->avgRepDuration);
                    
                    repCounter->repStartTime = currentTick;
                } else {
                    repCounter->repStartTime = xTaskGetTickCount();
                }

                repCounter->directionChangeCount = 0;
            }

            repCounter->prev_velocity = primary_velocity;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before trying again
    }
}*/

void DataAcquisitionTask(void *pvParameters) {
    while (1) {
            if(transmitData){
                if (readPacket()) { // Check if a packet was read successfully
                    getPositionAndVelocity(&currentSensorData);
                    char floatStr[20];

                        if(currentState == STATE_DATA_GATHERING) {
                            xil_printf("\r\n");
                            // Convert position values to strings and print
                            floatToString(currentSensorData.x_pos, floatStr, 2);
                            xil_printf("%s ", floatStr);

                            floatToString(currentSensorData.y_pos, floatStr, 2);
                            xil_printf("%s ", floatStr);

                            floatToString(currentSensorData.z_pos, floatStr, 2);
                            xil_printf("%s\r\n", floatStr);

                        }

                }

                if (xQueueSend(dataPointQueue0, &currentSensorData, portMAX_DELAY) == pdPASS) {
                //xil_printf("SENT DATA TO QUEUE\r\n");
                xTaskNotifyGive(xDataPrintTaskHandle);

                } else {
                    xil_printf("FAILED TO SEND DATA TO QUEUE\r\n");
                }
            } 

            if(!transmitData) {
                if(currentState != STATE_MENU) {
                   xil_printf("DATA IS NOT TRANSMITTING, FLIP SWITCH 0\r\n"); 
                }
                
            }
        
            
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before trying again
    }
}

void DataPrintTask(void *pvParameters) {
    DataPoint receivedData;
    // Wait for the notification to start Print
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    while (1) {
        if(transmitData && xQueueReceive(dataPointQueue0, &receivedData, portMAX_DELAY) == pdPASS){

            

            if(currentState == STATE_WARM_UP) {
                //xil_printf("Entered procesing task\r\n");
                    
                char floatStr[20];
                DataPoint tempData = receivedData;

                xil_printf("\r\n");
                // Convert position values to strings and print
                floatToString(tempData.x_pos, floatStr, 2);
                xil_printf("Position Data: x = %s,", floatStr);

                floatToString(tempData.y_pos, floatStr, 2);
                xil_printf(" y = %s,", floatStr);

                floatToString(tempData.z_pos, floatStr, 2);
                xil_printf(" z = %s\r\n", floatStr);

                                xil_printf("\r\n");
                // Convert position values to strings and print
                floatToString(tempData.x_veloc, floatStr, 2);
                xil_printf("Velocity Data: x = %s,", floatStr);

                floatToString(tempData.y_veloc, floatStr, 2);
                xil_printf(" y = %s,", floatStr);

                floatToString(tempData.z_veloc, floatStr, 2);
                xil_printf(" z = %s\r\n", floatStr);

                // Update min and max values
				dataExtremes.minX = fminf(dataExtremes.minX, receivedData.x_pos);
				dataExtremes.maxX = fmaxf(dataExtremes.maxX, receivedData.x_pos);

				dataExtremes.minY = fminf(dataExtremes.minY, receivedData.y_pos);
				dataExtremes.maxY = fmaxf(dataExtremes.maxY, receivedData.y_pos);

				dataExtremes.minZ = fminf(dataExtremes.minZ, receivedData.z_pos);
				dataExtremes.maxZ = fmaxf(dataExtremes.maxZ, receivedData.z_pos);

				dataExtremes.minVelX = fminf(dataExtremes.minVelX, receivedData.x_veloc);
				dataExtremes.maxVelX = fmaxf(dataExtremes.maxVelX, receivedData.x_veloc);

				dataExtremes.minVelY = fminf(dataExtremes.minVelY, receivedData.y_veloc);
				dataExtremes.maxVelY = fmaxf(dataExtremes.maxVelY, receivedData.y_veloc);

				dataExtremes.minVelZ = fminf(dataExtremes.minVelZ, receivedData.z_veloc);
				dataExtremes.maxVelZ = fmaxf(dataExtremes.maxVelZ, receivedData.z_veloc);
            }
            if(currentState == STATE_WORKING) {
                char floatStr[20];
                DataPoint tempData = receivedData;

                xil_printf("\r\n");
                // Convert position values to strings and print
                floatToString(tempData.x_pos, floatStr, 2);
                xil_printf("%s ", floatStr);

                floatToString(tempData.y_pos, floatStr, 2);
                xil_printf("%s ", floatStr);

                floatToString(tempData.z_pos, floatStr, 2);
                xil_printf("%s\r\n", floatStr);

                // Check if current readings are outside of recorded extremes and print alerts
                if (tempData.x_pos < dataExtremes.minX || tempData.x_pos > dataExtremes.maxX) xil_printf("ALERT: X position out of range!\r\n");
                if (tempData.y_pos < dataExtremes.minY || tempData.y_pos > dataExtremes.maxY) xil_printf("ALERT: Y position out of range!\r\n");
                if (tempData.z_pos < dataExtremes.minZ || tempData.z_pos > dataExtremes.maxZ) xil_printf("ALERT: Z position out of range!\r\n");

                if (tempData.x_veloc < dataExtremes.minVelX || tempData.x_veloc > dataExtremes.maxVelX) xil_printf("ALERT: X velocity out of range!\r\n");
                if (tempData.y_veloc < dataExtremes.minVelY || tempData.y_veloc > dataExtremes.maxVelY) xil_printf("ALERT: Y velocity out of range!\r\n");
                if (tempData.z_veloc < dataExtremes.minVelZ || tempData.z_veloc > dataExtremes.maxVelZ) xil_printf("ALERT: Z velocity out of range!\r\n");
            }

        }
            
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait a bit before trying again

    }
}

void vMenuTask(void *pvParameters) {
    xil_printf("DEBUG: Entered vMenuTask()\r\n");
    while (1){
        if(currentState == STATE_MENU) {
            xil_printf("INFO:vMenuTask()\tECE 544 Final Project\r\n");
            xil_printf("INFO:vMenuTask()\tBy Ibrahim Binmahfood (ibrah5@pdx.edu)\r\n");
            xil_printf("INFO:vMenuTask()\tand Robert Wilcox     (wilcox6@pdx.edu)\r\n");

            // Prompt User to select a lift out of list
            xil_printf("Select a Lift from the list by the number it is associated with: \r\n");
            xil_printf("0 [Squat], 1 [Bench], 2 [Bicep Curl]...\r\n");
            user_input.curr_lift_type = XUartLite_RecvByte(USB_UART_BASEADDR);

            if (user_input.curr_lift_type == '0') { // SQUAT
                const char* squatCommand = "SQUAT\n";
                for (int i = 0; squatCommand[i] != '\0'; i++) {
                    sendByteToESP32(squatCommand[i]);
                }
            } else if (user_input.curr_lift_type == '1') { // BENCH_PRESS
                const char* benchCommand = "BENCH_PRESS\n";
                for (int i = 0; benchCommand[i] != '\0'; i++) {
                    sendByteToESP32(benchCommand[i]);
                }
            } else if (user_input.curr_lift_type == '2') { // BICEP_CURL
                const char* curlCommand = "BICEP_CURL\n";
                for (int i = 0; curlCommand[i] != '\0'; i++) {
                    sendByteToESP32(curlCommand[i]);
                }
            } else { // Invalid selection
                xil_printf("Invalid selection. Please select 0, 1, or 2.\r\n");
            }


            if (1) {

                // Prompt User to select Warm Up MODE for that lift
                xil_printf("Select record or coach [r/c]: \r\n");
                user_input.curr_warmup_mode = XUartLite_RecvByte(USB_UART_BASEADDR);
               // xil_printf("Received: %c\r\n", user_input.curr_warmup_mode);

                if (user_input.curr_warmup_mode == 'a') {
                    currentState = STATE_DATA_GATHERING;
                }
                else if(user_input.curr_warmup_mode == 'r') {
                    const char* workingCommand = "WARMUP\n";
                    for (int i = 0; workingCommand[i] != '\0'; i++) {
                        sendByteToESP32(workingCommand[i]);
                    }
                    xil_printf("Switched to WARMUP data mode.\n");
                    currentState = STATE_WARM_UP;
                }
                else if(user_input.curr_warmup_mode == 'c') {
                    const char* warmupCommand = "WORKING\n";
                    WarmUp_flag = true;
                    for (int i = 0; warmupCommand[i] != '\0'; i++) {
                        sendByteToESP32(warmupCommand[i]);
                    }
                    xil_printf("Switched to WORKING data mode.\n");
                    currentState = STATE_WORKING;
                }
                else {
                    xil_printf("Require to Select either a or p\r\n");
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
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

// Task to monitor switch 0 to control data transmission
void vSwitchMonitorTask(void *pvParameters) {
    uint16_t switchStatePrev = 0;
    uint16_t switchState = 0;

    while (1) {
        switchState = NX4IO_getSwitches(); // Get the current state of the switches

        // Handle START/STOP data transmission based on switch 0
        if ((switchState & 0x0001) != (switchStatePrev & 0x0001)) {
            if (switchState & 0x0001) { // If switch 0 is ON (STOP)
                const char* startCommand = "START\n";
                for (int i = 0; startCommand[i] != '\0'; i++) {
                    sendByteToESP32(startCommand[i]);
                }
                transmitData = true; // Stop data transmission
                xil_printf("Data transmission started.\r\n");
            } else { // If switch 0 is OFF (START)
                const char* startCommand = "STOP\n";
                for (int i = 0; startCommand[i] != '\0'; i++) {
                    sendByteToESP32(startCommand[i]);
                }
                transmitData = false; // Resume data transmission
                xil_printf("Data transmission stopped.\r\n");
            }
        }

        // Handle data type change based on switch 1 without affecting data transmission
        if ((switchState & 0x0002) != (switchStatePrev & 0x0002)) {
            if (switchState & 0x0002) { // If switch 1 is ON (WORKING mode)
                WarmUp_flag = false;
            } else { // If switch 1 is OFF (WARMUP mode)
                
            }
        }

        // Handle data type change based on switch 2 without affecting data transmission
        if ((switchState & 0x0004) != (switchStatePrev & 0x0004)) {
            if (switchState & 0x0004) { // If switch 2 is ON (WORKING mode)
                currentState = STATE_MENU;
            } else { 
                   // xil_printf("SWITCH 2 is off\r\n");
            }
        }

        // Handle data type change based on switch 3 without affecting data transmission
        if ((switchState & 0x0008) != (switchStatePrev & 0x0008)) {
            if (switchState & 0x0008) { // If switch 2 is ON (WORKING mode)
                char floatStr[20];

                xil_printf("Extremes:\r\n");

                // Position X extremes
                floatToString(dataExtremes.minX, floatStr, 2);
                xil_printf("Position X: min = %s, ", floatStr);
                floatToString(dataExtremes.maxX, floatStr, 2);
                xil_printf("max = %s\r\n", floatStr);

                // Position Y extremes
                floatToString(dataExtremes.minY, floatStr, 2);
                xil_printf("Position Y: min = %s, ", floatStr);
                floatToString(dataExtremes.maxY, floatStr, 2);
                xil_printf("max = %s\r\n", floatStr);

                // Position Z extremes
                floatToString(dataExtremes.minZ, floatStr, 2);
                xil_printf("Position Z: min = %s, ", floatStr);
                floatToString(dataExtremes.maxZ, floatStr, 2);
                xil_printf("max = %s\r\n", floatStr);

                // Velocity X extremes
                floatToString(dataExtremes.minVelX, floatStr, 2);
                xil_printf("Velocity X: min = %s, ", floatStr);
                floatToString(dataExtremes.maxVelX, floatStr, 2);
                xil_printf("max = %s\r\n", floatStr);

                // Velocity Y extremes
                floatToString(dataExtremes.minVelY, floatStr, 2);
                xil_printf("Velocity Y: min = %s, ", floatStr);
                floatToString(dataExtremes.maxVelY, floatStr, 2);
                xil_printf("max = %s\r\n", floatStr);

                // Velocity Z extremes
                floatToString(dataExtremes.minVelZ, floatStr, 2);
                xil_printf("Velocity Z: min = %s, ", floatStr);
                floatToString(dataExtremes.maxVelZ, floatStr, 2);
                xil_printf("max = %s\n\n", floatStr);

            } else { 
                   // xil_printf("SWITCH 3 is off\r\n");
            }
        }

        switchStatePrev = switchState; // Update previous state
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for debouncing
    }
}

int main(void) {

    init_platform();

    // Iniitialize NX4IO
    int status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    dataPointQueue0 = xQueueCreate(20, sizeof(DataPoint)); // Create a queue for 10 DataPoint items
    //dataPointQueue1 = xQueueCreate(10, sizeof(DataPoint)); // Create a queue for 10 DataPoint items

    if (dataPointQueue0 != NULL) {
        xTaskCreate(vMenuTask, "MENU TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xMenu);
        xTaskCreate(DataAcquisitionTask, "DataAcquisition", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, &xDataAcquisitionTaskHandle);
        xTaskCreate(DataPrintTask, "DataPrint", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xDataPrintTaskHandle);
        xTaskCreate(vSwitchMonitorTask, "Switch Monitor Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &xSwitchMonitor);
        //xTaskCreate(LogRepCount, "LogRepCount", configMINIMAL_STACK_SIZE, &repCounter, tskIDLE_PRIORITY + 2, &xRepCounter);
        vTaskStartScheduler();
    }

    // If we get here, there was insufficient heap memory for the queue
    for (;;) {}
    return 0;
}
