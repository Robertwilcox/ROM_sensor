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


// Function prototypes
bool readPacket();
const uint8_t* getLastValidPacket(uint8_t* length);
char getPacketType();
void processPacket(Velocity * velocity, Position* position);
float bytesToFloat(const uint8_t *bytes);

#endif // PACKET_READER_H
