/**
 * @file packetReader.c
 * @brief This file implements the functionality for reading, processing, 
 * and validating packets received over BLE (Bluetooth Low Energy) on an 
 * ESP32 module. It defines structures and functions for packet handling 
 * including initialization, validation of checksums, and extraction of 
 * packet contents based on predefined packet types.
 *
 * @author Robert Wilcox, Ibrahim Binmahfood
 * @version 1.0
 * @date 3.21.2024
 */

#include "packetReader.h"
#include "esp32_BLE_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "xil_printf.h"

// Packet constants
#define START_MARKER 0x02
#define END_MARKER 0x03
#define MAX_PACKET_SIZE 16

// Packet identifiers
#define PACKET_TYPE_VELOCITY 'V'
#define PACKET_TYPE_POSITON 'P'


// Packet buffer and index
static uint8_t packetBuffer[MAX_PACKET_SIZE];
static uint8_t packetIndex = 0;
static bool isCollecting = false;
static uint8_t lastValidPacket[MAX_PACKET_SIZE];
static uint8_t lastValidPacketLength = 0;

/**
 * Resets the packet collection process by clearing the packet buffer index and the collecting state flag.
 */
static void resetPacketCollection() {
    packetIndex = 0;
    isCollecting = false;
}

/**
 * Processes each received byte from the ESP32 BLE interface. It 
 * checks for packet start and end markers to assemble packets and 
 * manage the collection process.
 * 
 * @param byte The received byte to be processed.
 * @return A boolean indicating if a complete packet has been received.
 */
bool processReceivedByte(uint8_t byte) {
    if (byte == START_MARKER) {                         // Check for start of packet
        packetIndex = 0;
        isCollecting = true;
        packetBuffer[packetIndex++] = byte;             // Store start marker
        return false;                                   // Continue receiving bytes
    } else if (isCollecting) {
        if (byte == END_MARKER || packetIndex >= MAX_PACKET_SIZE - 1) {
            
            // End of packet or buffer overflow
            packetBuffer[packetIndex++] = byte;         // Store end marker or last byte
            isCollecting = false;
            return true;                                // Packet assembly complete
        } else {
            
            // Collecting packet bytes
            packetBuffer[packetIndex++] = byte;
        }
    }
    return false;  // Continue receiving bytes
}

/**
 * Validates the checksum of the assembled packet to ensure data 
 * integrity. The checksum is a simple XOR of all bytes excluding 
 * the start/end markers and the checksum byte itself.
 * 
 * @return A boolean indicating if the checksum is valid.
 */
bool validateChecksum() {
    if (packetIndex < 3) return false; // Not enough data for a valid packet

    uint8_t checksum = 0;
    for (int i = 1; i < packetIndex - 2; i++) { // Exclude start marker, checksum byte, and end marker
        checksum ^= packetBuffer[i];
    }

    return (checksum == packetBuffer[packetIndex - 2]); // Compare calculated and received checksums
}

/**
 * Main function to read and process packets from the ESP32 BLE interface. 
 * It calls `processReceivedByte` for each byte received and checks the packet's validity.
 * 
 * @return A boolean indicating if a valid packet has been received and processed.
 */
bool readPacket() {
    uint8_t byte;
    while (receiveByteFromESP32(&byte)) {
        if (processReceivedByte(byte)) {
            
            // Packet received, validate checksum
            if (validateChecksum()) {
                
                // Valid packet received
                memcpy(lastValidPacket, packetBuffer, packetIndex);
                lastValidPacketLength = packetIndex;

                //xil_printf("Valid packet received\r\n");
                return true;
            } else {
                // Checksum mismatch
                xil_printf("Checksum mismatch\r\n");
                resetPacketCollection();
                return false;
            }
        }
    }
    return false; // No complete packet received yet
}

/**
 * Retrieves the last valid packet received along with its length. This function 
 * allows other components of the system to access the contents of the most recently 
 * received and validated packet.
 *
 * @param length Pointer to a variable where the length of the last valid packet will be stored.
 * @return Pointer to the start of the last valid packet data.
 */
const uint8_t* getLastValidPacket(uint8_t* length) {
    *length = lastValidPacketLength;
    return lastValidPacket;
}

char getPacketType() {
    if (lastValidPacketLength >= 2) { // Ensure there's enough data for type identification

        // The packet type is indicated by the second byte
        return lastValidPacket[1];
    }
    return '\0'; // Return a null character if unable to determine
}

/**
 * Processes the last valid packet based on its type. This function decodes the 
 * packet contents and updates the corresponding system parameters (e.g., velocity or position). 
 * It serves as a dispatcher, routing packets to the appropriate handling code based on their type.
 *
 * @param velocity Pointer to a Velocity structure where velocity data will be stored if the packet type is velocity.
 * @param position Pointer to a Position structure where position data will be stored if the packet type is position.
 */
void processPacket(Velocity * velocity, Position* position) {
    uint8_t length;
    const uint8_t* packet = getLastValidPacket(&length);
    char packetType = getPacketType();

    switch(packetType) {
        case PACKET_TYPE_VELOCITY: {
            // Deserialize velocity
            if(length >= 3 + sizeof(Velocity)) {
                memcpy(velocity, packet + 2, sizeof(Velocity));
            }
            break;
        }
        case PACKET_TYPE_POSITON: {
            // Deserialize position
            if(length >= 3 + sizeof(Position)) {
                memcpy(position, packet + 2, sizeof(Position));
            }
            break;
        }
        default: {
           // xil_printf("Unknown packet type: %c\n", packetType);
            break;
        }
    }
}


