// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG    0x7E
#define ESC     0x7D
#define A_ER    0x03
#define A_RE    0X01
#define C_SET   0x03
#define C_UA    0x07
#define C_I0    0x00
#define C_I1    0x80
#define C_RR0   0xAA
#define C_RR1   0xAB
#define C_REJ0  0x54
#define C_REJ1  0x55
#define C_DISC  0x0B

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    DATA_RCV,
    ESC_RCV,
    BCC2_OK,
    STOP_STATE,
    ERROR_STATE
} FrameState;

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
int retransmissions = 0;
int timeout = 0;
int tramaTx = 0;
int tramaRx = 0;


void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

unsigned char readControlFrame() {

    unsigned char byte;
    unsigned char controlField = 0;

    FrameState state = START;

    while(state != STOP_STATE && alarmEnabled) {

        if(readByteSerialPort(&byte) != 1) continue;

        switch(state) {
            case START:
                if(byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if(byte == A_RE) state = A_RCV;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case A_RCV:
                if(byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {
                    state = C_RCV;
                    controlField = byte;
                }
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case C_RCV:
                if(byte == (A_RE ^ controlField)) state = BCC1_OK;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case BCC1_OK:
                if(byte == FLAG) state = STOP_STATE;
                else state = START;
                break;
            default:
                break;

        }
    }
    return controlField;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    unsigned char SET[5] = {FLAG, A_ER, C_SET, A_ER ^ C_SET, FLAG};
    unsigned char UA[5] = {FLAG, A_RE, C_UA, A_RE ^ C_UA, FLAG};

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0){
        perror("openSerialPort");
        exit(-1);
    }

    retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

    unsigned char byte;
    FrameState state = START;
    int retries = connectionParameters.nRetransmissions;

    if (connectionParameters.role == LlTx) {

        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if(sigaction(SIGALRM, &act, NULL) == -1){
            perror("sigaction");
            exit(1);
        }

        while(retries > 0 && state != STOP_STATE) {
           
            writeBytesSerialPort(SET, 5);
            alarmEnabled = TRUE;
            alarm(connectionParameters.timeout);

            while(alarmEnabled && state != STOP_STATE) {

                if(readByteSerialPort(&byte) != 1) continue;

                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_RE) state = A_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case A_RCV:
                        if (byte == C_UA) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_RE ^ C_UA)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break; 
                    case BCC1_OK:
                        if(byte == FLAG) state = STOP_STATE;
                        else state = START;
                        break;
                    default:
                        break;
                }
            }

            
            if (state == STOP_STATE) alarm(0);
            else retries--;
        }

        if (state != STOP_STATE) return -1;
    }

    else if (connectionParameters.role == LlRx) {
        
        while (state != STOP_STATE) {

            if (readByteSerialPort(&byte) != 1) {
                continue;
            }

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_ER) state = A_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case A_RCV:
                    if (byte == C_SET) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (A_ER ^ C_SET)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (byte == FLAG) state = STOP_STATE;
                    else state = START;
                    break;
                default:
                    break;
            }
        }
        writeBytesSerialPort(UA, 5);
    }
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    if(bufSize > MAX_PAYLOAD_SIZE) {
        printf("[llwrite] Error: Payload too large (%d bytes > %d)\n", bufSize, MAX_PAYLOAD_SIZE);
        return -1;
    }

    int frameSize = 4 + bufSize + 2;
    unsigned char *frame = (unsigned char*)malloc(frameSize);
    
    frame[0] = FLAG;
    frame[1] = A_ER;
    frame[2] = tramaTx ? C_I1 : C_I0;
    frame[3] = frame[1] ^ frame[2];

    printf("[llwrite] Building frame: A=0x%02X, C=0x%02X, BCC1=0x%02X\n", frame[1], frame[2], frame[3]);

    unsigned char BCC2 = buf[0];
    for(int i = 1; i < bufSize; i++) {
        BCC2 ^= buf[i];
    }

    printf("[llwrite] BCC2 without stuffing: BCC2=0x%02X\n", BCC2);

    int j = 4;

    for(int i = 0; i < bufSize; i++) {
        unsigned char byte = buf[i];

        if(byte == FLAG || byte == ESC) {
            frame = realloc(frame, ++frameSize);
            frame[j++] = ESC;
            frame[j++] = byte ^ 0x20;
            printf("[llwrite] Byte stuffing: original=0x%02X -> ESC 0x%02X\n", byte, byte ^ 0x20);
        }
        else {
            frame[j++] = byte;
        }
    }

    if(BCC2 == FLAG || BCC2 == ESC) {
        frame = realloc(frame, ++frameSize);
        frame[j++] = ESC;
        frame[j++] = BCC2 ^ 0x20;
        printf("[llwrite] BCC2 stuffed: original=0x%02X -> ESC 0x%02X\n", BCC2, BCC2 ^ 0x20);
    }
    else {
        frame[j++] = BCC2;
    }

    frame[j++] = FLAG;
    printf("[llwrite] Frame complete, total size = %d bytes\n", j);

    int retries = retransmissions;
    int acknowledged = 0;

    while(retries > 0 && !acknowledged) {
        printf("[llwrite] Sending frame (try #%d)\n", retransmissions - retries + 1);
        
        writeBytesSerialPort(frame, j);
        alarmEnabled = TRUE;
        alarm(timeout);
        printf("[llwrite] Waiting for ACK/REJ (timeout = %ds)\n", timeout);

        while(alarmEnabled && !acknowledged) {
            unsigned char control = readControlFrame();
            if(control == (tramaTx ? C_RR0 : C_RR1)) {
                printf("[llwrite] Received RR%d (ACK)\n", tramaTx ? 1 : 0);
                acknowledged = 1;
                tramaTx = (tramaTx + 1) % 2;
                alarm(0);
            }
            else if (control == (tramaTx ? C_REJ1 : C_REJ0)) {
                printf("[llwrite] Received REJ%d — retransmitting\n", tramaTx ? 1 : 0);
                break;
            }
        }

        if(!acknowledged) {
            retries--;
            if (retries > 0) printf("[llwrite] Timeout or REJ — retrying (%d retries left)\n", retries);
            else printf("[llwrite] Failed after %d attempts\n", retransmissions);
        }
    }

    free(frame);

    if (acknowledged) {
        printf("[llwrite] Frame successfully acknowledged!\n");
        return bufSize;
    } 
    else {
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
