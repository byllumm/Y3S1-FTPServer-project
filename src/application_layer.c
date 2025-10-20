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
        // Transmitter: send a single byte
        unsigned char byteToSend = 0x7D; // ASCII 'B' for testing
        if (llwrite(&byteToSend, 1) < 0) {
            printf("llwrite failed\n");
        } else {
            printf("llwrite successful!\n");
        }
    } 
    else if (linkLayer.role == LlRx) {
        // Receiver: read a single byte
        unsigned char receivedByte;
        int n = llread(&receivedByte);
        if (n > 0) {
            printf("Received byte: 0x%02X ('%c')\n", receivedByte, receivedByte);
        } else {
            printf("llread failed\n");
        }
    }

}


// unsigned int cpSize; 
// unsigned char *startPacket = getControlPacket(2, filename, fileSize, &cpSize); // 2=start 
// if (llwrite(startPacket, cpSize) == -1) { 
//     fprintf(stderr, "Failed to send start packet\n"); 
//     free(startPacket); 
//     fclose(file); 
//     llclose(); 
//     return; 
// } 
// free(startPacket); 
// // Send DATA packets 
// unsigned char buffer[MAX_DATA_SIZE]; 
// int bytesRead; 
// while ((bytesRead = fread(buffer, 1, MAX_DATA_SIZE, file)) > 0) { 
//     if (llwrite(buffer, bytesRead) == -1) { 
//         fprintf(stderr, "Failed to send data packet\n"); 
//         fclose(file); llclose(); 
//         return; 
//     } 
// } 
// // Send END control packet 
// unsigned char *endPacket = getControlPacket(3, filename, fileSize, &cpSize); // 3=end 
// if (llwrite(endPacket, cpSize) == -1) { 
//     fprintf(stderr, "Failed to send end packet\n"); 
// } 
// free(endPacket); 
// fclose(file);



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