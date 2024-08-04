#ifndef BRAIN_H
#define BRAIN_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_PACKET_LENGTH 32
#define EEG_POWER_BANDS 8

typedef struct {
    uint8_t signalQuality;
    uint8_t attention;
    uint8_t meditation;
    uint32_t eegPower[EEG_POWER_BANDS];
    bool hasPower;
    char latestError[50];
    bool freshPacket;
    bool inPacket;
    uint8_t packetIndex;
    uint8_t packetLength;
    uint8_t packetData[MAX_PACKET_LENGTH];
    uint8_t latestByte;
    uint8_t lastByte;
    uint8_t checksum;
    uint8_t checksumAccumulator;
    char csvBuffer[124];
} Brain;

void Brain_init(Brain *brain);
bool Brain_update(Brain *brain, uint8_t latestByte);
void Brain_clearPacket(Brain *brain);
void Brain_clearEegPower(Brain *brain);
bool Brain_parsePacket(Brain *brain);
char* Brain_readErrors(Brain *brain);
char* Brain_readCSV(Brain *brain);
void Brain_printDebug(Brain *brain);
uint8_t Brain_readSignalQuality(Brain *brain);
uint8_t Brain_readAttention(Brain *brain);
uint8_t Brain_readMeditation(Brain *brain);
uint32_t* Brain_readPowerArray(Brain *brain);
uint32_t Brain_readDelta(Brain *brain);
uint32_t Brain_readTheta(Brain *brain);
uint32_t Brain_readLowAlpha(Brain *brain);
uint32_t Brain_readHighAlpha(Brain *brain);
uint32_t Brain_readLowBeta(Brain *brain);
uint32_t Brain_readHighBeta(Brain *brain);
uint32_t Brain_readLowGamma(Brain *brain);
uint32_t Brain_readMidGamma(Brain *brain);

#endif // BRAIN_H