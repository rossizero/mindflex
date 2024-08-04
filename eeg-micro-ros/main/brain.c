#include "brain.h"
#include <stdio.h>
#include <string.h>

void Brain_init(Brain *brain) {
    brain->freshPacket = false;
    brain->inPacket = false;
    brain->packetIndex = 0;
    brain->packetLength = 0;
    brain->hasPower = false;
    brain->checksum = 0;
    brain->checksumAccumulator = 0;

    brain->signalQuality = 200;
    brain->attention = 0;
    brain->meditation = 0;

    Brain_clearEegPower(brain);
}

void Brain_clearPacket(Brain *brain) {
    for (uint8_t i = 0; i < MAX_PACKET_LENGTH; i++) {
        brain->packetData[i] = 0;
    }
}

void Brain_clearEegPower(Brain *brain) {
    for (uint8_t i = 0; i < EEG_POWER_BANDS; i++) {
        brain->eegPower[i] = 0;
    }
}

bool Brain_update(Brain *brain, uint8_t latestByte) {
    brain->latestByte = latestByte;

    if (brain->inPacket) {
        if (brain->packetIndex == 0) {
            brain->packetLength = brain->latestByte;
            if (brain->packetLength > MAX_PACKET_LENGTH) {
                sprintf(brain->latestError, "ERROR: Packet too long %i", brain->packetLength);
                brain->inPacket = false;
            }
        } else if (brain->packetIndex <= brain->packetLength) {
            brain->packetData[brain->packetIndex - 1] = brain->latestByte;
            brain->checksumAccumulator += brain->latestByte;
        } else if (brain->packetIndex > brain->packetLength) {
            brain->checksum = brain->latestByte;
            brain->checksumAccumulator = 255 - brain->checksumAccumulator;

            if (brain->checksum == brain->checksumAccumulator) {
                if (Brain_parsePacket(brain)) {
                    brain->freshPacket = true;
                } else {
                    sprintf(brain->latestError, "ERROR: Could not parse");
                }
            } else {
                sprintf(brain->latestError, "ERROR: Checksum");
            }

            brain->inPacket = false;
        }

        brain->packetIndex++;
    }

    if ((brain->latestByte == 170) && (brain->lastByte == 170) && !brain->inPacket) {
        brain->inPacket = true;
        brain->packetIndex = 0;
        brain->checksumAccumulator = 0;
    }

    brain->lastByte = brain->latestByte;

    if (brain->freshPacket) {
        brain->freshPacket = false;
        return true;
    } else {
        return false;
    }
}

bool Brain_parsePacket(Brain *brain) {
    brain->hasPower = false;
    bool parseSuccess = true;

    Brain_clearEegPower(brain);

    for (uint8_t i = 0; i < brain->packetLength; i++) {
        switch (brain->packetData[i]) {
            case 0x2:
                brain->signalQuality = brain->packetData[++i];
                break;
            case 0x4:
                brain->attention = brain->packetData[++i];
                break;
            case 0x5:
                brain->meditation = brain->packetData[++i];
                break;
            case 0x83:
                i++;
                for (int j = 0; j < EEG_POWER_BANDS; j++) {
                    uint8_t a, b, c;
                    a = brain->packetData[++i];
                    b = brain->packetData[++i];
                    c = brain->packetData[++i];
                    brain->eegPower[j] = ((uint32_t)a << 16) | ((uint32_t)b << 8) | (uint32_t)c;
                }
                brain->hasPower = true;
                break;
            case 0x80:
                i += 3;
                break;
            default:
                parseSuccess = false;
                break;
        }
    }
    return parseSuccess;
}

char* Brain_readErrors(Brain *brain) {
    return brain->latestError;
}

char* Brain_readCSV(Brain *brain) {
    if (brain->hasPower) {
        sprintf(brain->csvBuffer, "%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
                brain->signalQuality,
                brain->attention,
                brain->meditation,
                brain->eegPower[0],
                brain->eegPower[1],
                brain->eegPower[2],
                brain->eegPower[3],
                brain->eegPower[4],
                brain->eegPower[5],
                brain->eegPower[6],
                brain->eegPower[7]);
    } else {
        sprintf(brain->csvBuffer, "%d,%d,%d",
                brain->signalQuality,
                brain->attention,
                brain->meditation);
    }
    return brain->csvBuffer;
}

void Brain_printDebug(Brain *brain) {
    printf("\n--- Start Packet ---\n");
    printf("Signal Quality: %d\n", brain->signalQuality);
    printf("Attention: %d\n", brain->attention);
    printf("Meditation: %d\n", brain->meditation);

    if (brain->hasPower) {
        printf("\nEEG POWER:\n");
        printf("Delta: %lu\n", brain->eegPower[0]);
        printf("Theta: %lu\n", brain->eegPower[1]);
        printf("Low Alpha: %lu\n", brain->eegPower[2]);
        printf("High Alpha: %lu\n", brain->eegPower[3]);
        printf("Low Beta: %lu\n", brain->eegPower[4]);
        printf("High Beta: %lu\n", brain->eegPower[5]);
        printf("Low Gamma: %lu\n", brain->eegPower[6]);
        printf("Mid Gamma: %lu\n", brain->eegPower[7]);
    }

    printf("\nChecksum Calculated: %d\n", brain->checksumAccumulator);
    printf("Checksum Expected: %d\n", brain->checksum);
    printf("--- End Packet ---\n");
}

uint8_t Brain_readSignalQuality(Brain *brain) {
    return brain->signalQuality;
}

uint8_t Brain_readAttention(Brain *brain) {
    return brain->attention;
}

uint8_t Brain_readMeditation(Brain *brain) {
    return brain->meditation;
}

uint32_t* Brain_readPowerArray(Brain *brain) {
    return brain->eegPower;
}

uint32_t Brain_readDelta(Brain *brain) {
    return brain->eegPower[0];
}

uint32_t Brain_readTheta(Brain *brain) {
    return brain->eegPower[1];
}

uint32_t Brain_readLowAlpha(Brain *brain) {
    return brain->eegPower[2];
}

uint32_t Brain_readHighAlpha(Brain *brain) {
    return brain->eegPower[3];
}

uint32_t Brain_readLowBeta(Brain *brain) {
    return brain->eegPower[4];
}

uint32_t Brain_readHighBeta(Brain *brain) {
    return brain->eegPower[5];
}

uint32_t Brain_readLowGamma(Brain *brain) {
    return brain->eegPower[6];
}

uint32_t Brain_readMidGamma(Brain *brain) {
    return brain->eegPower[7];
}
