// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer linkLayer;
    memset(&linkLayer, 0, sizeof(linkLayer));

    strncpy(linkLayer.serialPort, serialPort, sizeof(linkLayer.serialPort) - 1);
    linkLayer.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    linkLayer.baudRate = baudRate;
    linkLayer.nRetransmissions = nTries;
    linkLayer.timeout = timeout;

    printf("[applicationLayer] Role: %s | Port: %s | Baud: %d\n", role, serialPort, baudRate);

    if(llopen(linkLayer) == -1) {
        perror("Failed to open link layer connection");
        exit(1);
    }
    else {
        printf("llopen() successful!\n");
    }

    if (linkLayer.role == LlTx) {

        FILE *file = fopen(filename, "rb");
        if(!file) {
            perror("Error opening file");
            exit(1);
        }

        fseek(file, 0L, SEEK_END);
        long int fileSize = ftell(file);
        fseek(file, 0L, SEEK_SET);

        unsigned int controlPacketSize;
        unsigned char* startControlPacket = encodeControlPacket(1, filename, fileSize, &controlPacketSize);
        if(llwrite(startControlPacket, controlPacketSize) == -1) {
            printf("Error in writing start packet\n");
            fclose(file);
            free(startControlPacket);
            exit(1);
        }
        free(startControlPacket);

        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        long int totalBytesSent = 0;

        while((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            unsigned int dataPacketSize;
            unsigned char* dataPacket = encodeDataPacket(buffer, bytesRead, &dataPacketSize);

            if(!dataPacket) {
                printf("Failed to encode data packet\n");
                fclose(file);
                exit(1);
            }

            if(llwrite(dataPacket, dataPacketSize) == -1) {
                printf("Failed to send data packet\n");
                free(dataPacket);
                fclose(file);
                exit(1);
            }

            free(dataPacket);

            totalBytesSent += bytesRead;

            printf("\rProgress: %.1f%% (%ld/%ld bytes)",
                100.0 * totalBytesSent / fileSize,
                totalBytesSent,
                fileSize);
            fflush(stdout);
        }

        if (ferror(file)) {
            perror("Error reading from file");
            fclose(file);
            exit(1);
        }

        unsigned char* endControlPacket = encodeControlPacket(3, filename, fileSize, &controlPacketSize);
        if(llwrite(endControlPacket, controlPacketSize) == -1) {
            printf("Error in writing end packet\n");
            fclose(file);
            free(endControlPacket);
            exit(1);
        }

        free(endControlPacket);

        fclose(file);

        if(llclose() == -1) {
            perror("Failed to close link layer connection");
            exit(1);
        }
        else {
            printf("llclose() successful!\n");
        }
        
    } 

    else if (linkLayer.role == LlRx) {
        unsigned char packet[MAX_PAYLOAD_SIZE + 10];
        int packetSize;

        do {
            packetSize = llread(packet);
        } while (packetSize < 0); 

        unsigned long int fileSize = 0;
        unsigned char* receivedFilename = decodeControlPacket(packet, packetSize, &fileSize);
        if (!receivedFilename) {
            fprintf(stderr, "Failed to decode start control packet\n");
            llclose();
            exit(1);
        }

        printf("Receiving file: %s (%lu bytes)\n", receivedFilename, fileSize);

        FILE *file = fopen((char*) receivedFilename, "wb"); // use argument filename as destination
        if (!file) {
            perror("Error creating output file");
            free(receivedFilename);
            fclose(file);
            exit(1);
        }

        long int totalBytesReceived = 0;
        int done = 0;

        while (!done) {
            do {
                packetSize = llread(packet);
            } while (packetSize < 0);

            unsigned char packetType = packet[0];

            if (packetType == 2) {
                unsigned int dataSize;
                unsigned char* data = decodeDataPacket(packet, packetSize, &dataSize);

                if (!data) {
                    fprintf(stderr, "Failed to decode data packet\n");
                    fclose(file);
                    free(receivedFilename);
                    llclose();
                    exit(1);
                }

                size_t written = fwrite(data, 1, dataSize, file);
                if (written < dataSize) {
                    perror("Error writing to output file");
                    free(data);
                    fclose(file);
                    free(receivedFilename);
                    llclose();
                    exit(1);
                }

                totalBytesReceived += dataSize;
                free(data);

                printf("\rProgress: %.1f%% (%ld/%lu bytes)",
                    100.0 * totalBytesReceived / fileSize,
                    totalBytesReceived, fileSize);
                fflush(stdout);
            }

            else if (packetType == 3) {
                done = 1;
            }

            else {
                fprintf(stderr, "Unknown packet type: %u\n", packetType);
            }
        }

        printf("\nFile reception complete!\n");

        fclose(file);
        free(receivedFilename);

        // 3️⃣ --- Close link ---
        if (llclose() == -1) {
            perror("Failed to close link layer connection");
            exit(1);
        } else {
            printf("llclose() successful!\n");
        }
    }
}

unsigned char* encodeControlPacket(const unsigned int c, const char* filename, long int filesize, unsigned int* packetsize) {

    unsigned int tmp = filesize;
    int L1 = 0;
    do {
        L1++;
        tmp >>= 8;
    } while (tmp > 0);
    const int L2 = strlen(filename);

    *packetsize = 1 + 2 + L1 + 2 + L2;
    unsigned char *packet = (unsigned char*)malloc(*packetsize);
    if (!packet) return NULL;

    unsigned int pos = 0;
    packet[pos++] = c;
    packet[pos++] = 0;
    packet[pos++] = L1;

    for(int i = L1 - 1; i >= 0; i--) {
        packet[pos++] = (filesize >> (8 * i)) & 0xFF;
    }

    packet[pos++] = 1;
    packet[pos++] = L2;
    memcpy(packet + pos, filename, L2);

    return packet;
}

unsigned char* decodeControlPacket(const unsigned char* packet, unsigned int packetsize, unsigned long int* filesize) {
    if(!packet || packetsize < 5 || !filesize) return NULL;

    unsigned int pos = 0;
    pos++;
    if(packet[pos++] != 0) return NULL;

    unsigned char L1 = packet[pos++];
    if(L1 == 0 || pos + L1 > packetsize) return NULL;

    *filesize = 0;
    for(unsigned int i = 0; i < L1; i++) {
        *filesize = (*filesize << 8) | packet[pos++];
    }

    if(packet[pos++] != 1) return NULL;

    unsigned char L2 = packet[pos++];
    if(L2 == 0 || pos + L2 > packetsize) return NULL;

    unsigned char* filename = (unsigned char*)malloc(L2+1);
    if(!filename) return NULL;
    
    memcpy(filename, packet + pos, L2);
    filename[L2] = '\0';

    return filename;
}


unsigned char* encodeDataPacket(const unsigned char* data, unsigned int datasize, unsigned int* packetsize) {
    if(!data || datasize == 0 || !packetsize) return NULL;

    unsigned char L1 = datasize % 256;
    unsigned char L2 = datasize / 256;

    *packetsize = 3 + datasize;
    unsigned char* packet = (unsigned char*)malloc(*packetsize);
    if(!packet) return NULL;

    unsigned int pos = 0;

    packet[pos++] = 2;
    packet[pos++] = L2;
    packet[pos++] = L1;

    memcpy(packet + pos, data, datasize);
    return packet;
}

unsigned char* decodeDataPacket(const unsigned char* packet, const unsigned int packetsize, unsigned int* datasize) {
    if(!packet || packetsize < 3 || !datasize) return NULL;

    unsigned int pos = 0;
    
    if(packet[pos++] != 2) return NULL;

    unsigned char L2 = packet[pos++];
    unsigned char L1 = packet[pos++];

    *datasize = (256 * L2) + L1;

    if(*datasize == 0 || (pos + datasize) > packetsize) return NULL;

    unsigned char* data = (unsigned char*)malloc(*datasize);
    if(!data) return NULL;

    memcpy(data, packet + pos, *datasize);

    return data;
}