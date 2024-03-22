/**
 * @file packetReader.h
 * @brief This header file implements the functionality for reading, processing, 
 * and validating packets received over BLE (Bluetooth Low Energy) on an 
 * ESP32 module. It defines structures and functions for packet handling 
 * including initialization, validation of checksums, and extraction of 
 * packet contents based on predefined packet types.
 *
 * @author Robert Wilcox, Ibrahim Binmahfood
 * @version 1.0
 * @date 3.21.2024
 */


#ifndef PACKET_READER_H
#define PACKET_READER_H

#include <stdbool.h>
#include <stdint.h>

// Define struct for Velocity
typedef struct {
    float xVel;
    float yVel;
    float zVel;
} Velocity;

// Define struct for Position
typedef struct {
    float xPos;
    float yPos;
    float zPos;
} Position;


/**
 * @brief Reads and processes packets from the ESP32 BLE interface.
 * This function continuously reads bytes from the ESP32, assembles packets,
 * and validates them using a checksum. It determines if a complete and valid
 * packet has been received.
 * 
 * @return true if a valid packet has been received and processed, false otherwise.
 */
bool readPacket();

/**
 * @brief Retrieves the last valid packet received and its length.
 * This function is used to access the contents of the most recently
 * received and validated packet by other components of the system.
 * 
 * @param length Pointer to a variable where the length of the last valid packet will be stored.
 * @return Pointer to the start of the last valid packet data.
 */
const uint8_t* getLastValidPacket(uint8_t* length);

/**
 * @brief Determines the type of the last valid packet received.
 * This function examines the last valid packet and returns its type
 * based on predefined packet type identifiers.
 * 
 * @return The character representing the packet type, or '\0' if unable to determine.
 */
char getPacketType();

/**
 * @brief Processes the last valid packet based on its type.
 * Depending on the type of the packet (velocity or position), this function
 * updates the corresponding system parameters by decoding the packet contents.
 * 
 * @param velocity Pointer to a Velocity structure where velocity data will be stored if the packet type is velocity.
 * @param position Pointer to a Position structure where position data will be stored if the packet type is position.
 */
void processPacket(Velocity * velocity, Position* position);

/**
 * @brief Converts a byte array to a floating-point number.
 * This utility function is used for converting raw bytes received in a packet
 * into floating-point values for further processing.
 * 
 * @param bytes Pointer to the byte array to be converted.
 * @return The floating-point number representation of the byte array.
 */
float bytesToFloat(const uint8_t *bytes);
#endif // PACKET_READER_H
