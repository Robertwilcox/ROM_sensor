/**
 *  @file main.c
 *  @brief Mainline program of BarBuddy. MicroBlaze FreeRTOS implemented on
 *  the Boolean FPGA board. The program also uses the custom IP Nexys4IO API
 *  that created by Roy Kravitz (roy.kravitz@pdx.edu).
 *
 *  This program creates four tasks {vMenuTask, vDataAcquisitionTask,
*   vDataPrintTask, vSwitchMonitorTask} where:
*
*   - vMenuTask() prompts the user with selected options via the UART console 
*   of the type of lift workouts and more.
*
*   - vDataAcquisitionTask() retrieves the newly read packets of data from
*   the ESP32 connected via PMOD A to the Boolean board and notifies
*   vDataPrintTask() with the newly recieved sensor data.
*
*   - vDataPrintTask() logs the sensor data to UART console and depending on
*   WARM UP mode/WORK mode applies checks on the sensor data.
*
*   - vSwitchMonitorTask() monitors switches and controls data transmissions to
*   the ESP32
*
 *  @author Robert Wilcox (wilcox6@pdx.edu), Ibrahim Binmahfood (ibrah5@pdx.edu)
 *  @version 1.2.1
 *  @date 3.12.2024
 */

// C Lib
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <math.h>

// FreeRTOS Lib
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Xilinx MicroBlaze Lib
#include "xintc.h"
#include "xuartlite_l.h"
#include "xuartlite.h"
#include "xstatus.h"
#include "xparameters.h"
#include "platform.h"
#include "microblaze_sleep.h"
#include "xil_printf.h"

// Custom IP Nexys4IO Lib API
#include "nexys4IO.h"

#include "esp32_BLE_uart.h"
#include "packetReader.h"
#include "updatePandV.h"

// Alias for the USB UART Lite Peripheral
#define USB_UART_BASEADDR       XPAR_AXI_UARTLITE_0_BASEADDR

// Aliases for NEXYS4IO Peripheral
#define N4IO_DEVICE_ID          XPAR_NEXYS4IO_0_DEVICE_ID
#define N4IO_BASEADDR           XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define N4IO_HIGHADDR           XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

/* Helper functions for floatToString() to stdout (UARTLITE) */
// Reverse the string
void reverseString(char* str, int length);

// Integer to String
int intToStr(int x, char str[], int d);

// Float to String
void floatToString(float n, char *res, int afterpoint);

/* Tasks */
// Menu Task:
void vMenuTask(void *pvParameters);

// Data Aquisition Task
void vDataAcquisitionTask(void *pvParameters);

// Printing the Data Aquired Task
void vDataPrintTask(void *pvParameters);

// Monitor the Switches Task
void vSwitchMonitorTask(void *pvParameters);

// System States
typedef enum {
    STATE_DATA_GATHERING,
    STATE_WARM_UP,
    STATE_MENU,
    STATE_WORKING
} SystemState;

// User Input
typedef struct {
    uint8_t curr_lift_type;
    uint8_t curr_warmup_mode;
} UserInput, *UserInputPtr;

// Min/Max Position and Velocity Data
typedef struct {
    float minX, maxX;
    float minY, maxY;
    float minZ, maxZ;
    float minVelX, maxVelX;
    float minVelY, maxVelY;
    float minVelZ, maxVelZ;
} Extremes;

// Globals
volatile SystemState currentState = STATE_MENU; // Global state of system
volatile bool transmitData        = false;  // Global flag to control data transmission
volatile bool WarmUp_flag         = true;   // Global flag to control warmup

static const int  directionChangeThreshold = 3; // Threshold for confirming a direction change

// Initialize to largest max/mins per position and velocity
volatile Extremes dataExtremes = {
    .minX = 10000, .maxX = -1000,
    .minY = 10000, .maxY = -1000,
    .minZ = 10000, .maxZ = -1000,
    .minVelX = 10000, .maxVelX = -1000,
    .minVelY = 10000, .maxVelY = -1000,
    .minVelZ = 10000, .maxVelZ = -1000
};

// Initialization
DataPoint currentSensorData;    // The most current sensor data

QueueHandle_t dataPointQueue0;   // Queue for sensor data

UserInput user_input;           // Struct for user input

// Task Handles
TaskHandle_t xDataAcquisitionTaskHandle = NULL;
TaskHandle_t xDataPrintTaskHandle       = NULL;
TaskHandle_t xMenu                      = NULL;
TaskHandle_t xSwitchMonitor             = NULL;

int main(void) {

    init_platform();

    // Iniitialize NX4IO
    int status = NX4IO_initialize(N4IO_BASEADDR);
    if (status != XST_SUCCESS) {
        return -1;
    }
    
    // Create a queue for 20 DataPoint items
    dataPointQueue0 = xQueueCreate(20, sizeof(DataPoint)); 

    // Check IF Queue was SUCCESSFUL
    if (dataPointQueue0 != NULL) {
        // Create Tasks with priorities assigned and stack sizes
        xTaskCreate(vMenuTask, "MENU TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xMenu);
        xTaskCreate(vDataAcquisitionTask, "DataAcquisition", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, &xDataAcquisitionTaskHandle);
        xTaskCreate(vDataPrintTask, "DataPrint", configMINIMAL_STACK_SIZE * 3, NULL, tskIDLE_PRIORITY + 1, &xDataPrintTaskHandle);
        xTaskCreate(vSwitchMonitorTask, "Switch Monitor Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, &xSwitchMonitor);

        vTaskStartScheduler();
    }
    // ELSE terminate system

    cleanup_platform();
    return 0;
} 

/**
 * @brief The function that reverses a string.
 * 
 * This function takes in a buffer with a length and begins reversing the
 * string buffer with reference to the index and length.
 *
 * @param str: the string buffer to be reversed
 * @param str: the buffer where the parameter x will be converted to char[] type
 * @param d:   the number of digits required of int type
 */
void reverseString(char* str, int length) {
    int start = 0;  // starting index
    int end = length - 1; // ending index

    // Traverse through till ending index
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;    // increase the starting index
        end--;      // decrease the ending index
    }
}

/**
 * @brief The function that converts a integer number to string
 * 
 * This function takes in a buffer that will hold the integer number to
 * be converted with reference to the number of digits required.
 * 
 * @param x:   the number to be converted of int type
 * @param str: the buffer where the parameter x will be converted to char[] type
 * @param d:   the number of digits required of int type
 *
 * @return Returns the index of type int where the null terminator for the string buffer.
 */
int intToStr(int x, char str[], int d) {
    int i = 0;
    bool isNegative = false;

    // Check for polarity/sign
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

/**
 * @brief The function that converts a floating point number to string
 * 
 * This function takes in a buffer that will hold the floating point number to
 * be converted with reference to the number of places after the decimal point.
 * 
 * @param n:   the number to be converted of float type
 * @param res: the buffer where the parameter n will be converted to char* type
 * @param afterpoint: the number of decimal places after the decimal point required of int type
 */
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

/**
 * @brief The task function for retrieving data.
 * 
 * This task when ready to transmit data, reads packets from the ESP 32. IF it
 * was successful then it prints out position values to display that data is
 * indeed present. It also updates the currentSensorData struct via
 * getPositionandVelocity() function. After, it sends the currentSensorData via
 * a Queue dataPointQueue0 which is then handled in vDataPrintTask() when
 * notified.
 * 
 * @param pvParameters Parameters for the task, not used.
 */
void vDataAcquisitionTask(void *pvParameters) {

    while (1) {
        // Check IF Ready to Transmit data 
        if(transmitData){
            // Check IF packet was read successfully
            if (readPacket()) {
                getPositionAndVelocity(&currentSensorData); //update the sensor data
                char floatStr[20];

                // Check IF Current System State is in Data Gathering Mode
                if(currentState == STATE_DATA_GATHERING) {
                    print("\r\n");
                    // Convert position values to strings and print
                    floatToString(currentSensorData.x_pos, floatStr, 2);
                    xil_printf("%s ", floatStr);

                    floatToString(currentSensorData.y_pos, floatStr, 2);
                    xil_printf("%s ", floatStr);

                    floatToString(currentSensorData.z_pos, floatStr, 2);
                    xil_printf("%s\r\n", floatStr);
                }
            }

            // Check IF able to enqueue the sensor data, ELSE delay for MAX_DELAY
            if (xQueueSend(dataPointQueue0, &currentSensorData, portMAX_DELAY) == pdPASS) {
                xTaskNotifyGive(xDataPrintTaskHandle); // Notify Data Print Task
            } else {
                print("FAILED TO SEND DATA TO QUEUE\r\n");
            }
        } 

        // Check IF NOT Ready to Transmit data
        if(!transmitData) {
            // Check IF Current System State NOT MENU
            if(currentState != STATE_MENU) {
                print("DATA IS NOT TRANSMITTING, FLIP SWITCH 0\r\n"); 
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Wait a bit before trying again
    }
}

/**
 * @brief The task function for printing out data.
 * 
 * This task when notified to start printing process, prints the position and
 * velocity {x, y, z} data recieved. In addition to that, updates the minimum
 * and maximum values for the dataExtremes struct. Also, depending on the state
 * of the system, in STATE_WARM_UP mode, it only prints the data recieved and
 * updates the min and max values. However, in STATE_WORKING mode, it applies
 * checks of how much the user deiviates based upon the position and velocity
 * data.
 *
 * @param pvParameters Parameters for the task, not used.
 */
void vDataPrintTask(void *pvParameters) {
    DataPoint receivedData;
    // Wait for the notification to start Print
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (1) {
        // Check IF ready to transmit data and successfully recieved sensor data
        // ELSE delay for MAX_DELAY
        if(transmitData && xQueueReceive(dataPointQueue0, &receivedData, portMAX_DELAY) == pdPASS){
            // Check IF Current System State in Warm Up mode
            if(currentState == STATE_WARM_UP) {
                char floatStr[20];
                DataPoint tempData = receivedData;

                print("\r\n");
                // Convert position values to strings and print
                floatToString(tempData.x_pos, floatStr, 2);
                xil_printf("Position Data: x = %s,", floatStr);

                floatToString(tempData.y_pos, floatStr, 2);
                xil_printf(" y = %s,", floatStr);

                floatToString(tempData.z_pos, floatStr, 2);
                xil_printf(" z = %s\r\n", floatStr);

                print("\r\n");
                // Convert position values to strings and print
                floatToString(tempData.x_veloc, floatStr, 2);
                xil_printf("Velocity Data: x = %s,", floatStr);

                floatToString(tempData.y_veloc, floatStr, 2);
                xil_printf(" y = %s,", floatStr);

                floatToString(tempData.z_veloc, floatStr, 2);
                xil_printf(" z = %s\r\n", floatStr);

                // Update min and max values to dataExtremes
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

            // Check IF Current System State in Working mode
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

                // Check IF current readings are outside of recorded extremes
                // then Alert user via UART console
                if (tempData.x_pos < dataExtremes.minX || tempData.x_pos > dataExtremes.maxX) print("ALERT: X position out of range!\r\n");
                if (tempData.y_pos < dataExtremes.minY || tempData.y_pos > dataExtremes.maxY) print("ALERT: Y position out of range!\r\n");
                if (tempData.z_pos < dataExtremes.minZ || tempData.z_pos > dataExtremes.maxZ) print("ALERT: Z position out of range!\r\n");

                if (tempData.x_veloc < dataExtremes.minVelX || tempData.x_veloc > dataExtremes.maxVelX) print("ALERT: X velocity out of range!\r\n");
                if (tempData.y_veloc < dataExtremes.minVelY || tempData.y_veloc > dataExtremes.maxVelY) print("ALERT: Y velocity out of range!\r\n");
                if (tempData.z_veloc < dataExtremes.minVelZ || tempData.z_veloc > dataExtremes.maxVelZ) print("ALERT: Z velocity out of range!\r\n");
            }
        }
            
        vTaskDelay(pdMS_TO_TICKS(200)); // Wait a bit before trying again
    }
}

/**
 * @brief The menu task function.
 * 
 * This task prompts the user for lift types, record or coach modes, and or
 * data gathering mode. 
 *
 * @param pvParameters Parameters for the task, not used.
 */
void vMenuTask(void *pvParameters) {
    print("DEBUG: Entered vMenuTask()\r\n");

    while (1){
        // Check IF Current System State in Menu mode
        if(currentState == STATE_MENU) {
            print("INFO:vMenuTask()\tECE 544 Final Project\r\n");
            print("INFO:vMenuTask()\tBy Ibrahim Binmahfood (ibrah5@pdx.edu)\r\n");
            print("INFO:vMenuTask()\tand Robert Wilcox     (wilcox6@pdx.edu)\r\n");

            // Prompt User to select a lift out of list
            print("Select a Lift from the list by the number it is associated with: \r\n");
            print("0 [Squat], 1 [Bench], 2 [Bicep Curl]...\r\n");
            user_input.curr_lift_type = XUartLite_RecvByte(USB_UART_BASEADDR);

            // Check IF user selected Squat
            if (user_input.curr_lift_type == '0') { // SQUAT
                const char* squatCommand = "SQUAT\n";
                for (int i = 0; squatCommand[i] != '\0'; i++) {
                    sendByteToESP32(squatCommand[i]);
                }
            } 
            // Check IF user selected Bench Press
            else if (user_input.curr_lift_type == '1') { // BENCH_PRESS
                const char* benchCommand = "BENCH_PRESS\n";
                for (int i = 0; benchCommand[i] != '\0'; i++) {
                    sendByteToESP32(benchCommand[i]);
                }
            } 
            // Check IF user selected Bicep Curl
            else if (user_input.curr_lift_type == '2') { // BICEP_CURL
                const char* curlCommand = "BICEP_CURL\n";
                for (int i = 0; curlCommand[i] != '\0'; i++) {
                    sendByteToESP32(curlCommand[i]);
                }
            } 
            // ELSE user selected an invalid selection
            else {
                print("Invalid selection. Please select 0, 1, or 2.\r\n");
            }

            // Prompt User to select Warm Up MODE for that lift
            print("Select record or coach [r/c]: \r\n");
            user_input.curr_warmup_mode = XUartLite_RecvByte(USB_UART_BASEADDR);

            // Check IF user selected data gathering mode
            if (user_input.curr_warmup_mode == 'a') {
                currentState = STATE_DATA_GATHERING;
            }
            // Check IF user selected record mode
            else if(user_input.curr_warmup_mode == 'r') {
                const char* workingCommand = "WARMUP\n";
                for (int i = 0; workingCommand[i] != '\0'; i++) {
                    sendByteToESP32(workingCommand[i]);
                }
                print("Switched to WARMUP data mode.\n");
                currentState = STATE_WARM_UP;
            }
            // Check IF user selected coach mode
            else if(user_input.curr_warmup_mode == 'c') {
                const char* warmupCommand = "WORKING\n";
                WarmUp_flag = true;
                for (int i = 0; warmupCommand[i] != '\0'; i++) {
                    sendByteToESP32(warmupCommand[i]);
                }
                print("Switched to WORKING data mode.\n");
                currentState = STATE_WORKING;
            }
            // ELSE user selected invalid selection
            else {
                print("Require to Select either a or r or c\r\n");
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief The task function to monitor switches and control data transmission.
 * 
 * This task controls the four switches (from left side to right side SW[3:0])
 * and depending on the switch either data transmission START/STOP commands are
 * sent to the ESP32, or ends working/warmup modes.
 *
 * @param pvParameters Parameters for the task, not used.
 */
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
                print("Data transmission started.\r\n");
            } else { // If switch 0 is OFF (START)
                const char* startCommand = "STOP\n";
                for (int i = 0; startCommand[i] != '\0'; i++) {
                    sendByteToESP32(startCommand[i]);
                }
                transmitData = false; // Resume data transmission
                print("Data transmission stopped.\r\n");
            }
        }

        // Handle data type change based on switch 1 without affecting data transmission
        if ((switchState & 0x0002) != (switchStatePrev & 0x0002)) {
            if (switchState & 0x0002) { // If switch 1 is ON (WORKING mode)
                WarmUp_flag = false;
            }
            // ELSE switch 1 is OFF (WARMUP mode)
        }

        // Handle data type change based on switch 2 without affecting data transmission
        if ((switchState & 0x0004) != (switchStatePrev & 0x0004)) {
            if (switchState & 0x0004) { // If switch 2 is ON (WORKING mode)
                currentState = STATE_MENU;
            } 
            // ELSE switch 2 is OFF 
        }

        // Handle data type change based on switch 3 without affecting data transmission
        if ((switchState & 0x0008) != (switchStatePrev & 0x0008)) {
            if (switchState & 0x0008) { // If switch 2 is ON (WORKING mode)
                char floatStr[20];

                print("Extremes:\r\n");

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

            } 
            // ELSE switch 3 is OFF
        }

        switchStatePrev = switchState; // Update previous state
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for debouncing
    }
}
