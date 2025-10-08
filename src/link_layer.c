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
#define C_RR0   0xAA
#define C_RR1   0xAB
#define C_REJ0  0x54
#define C_REJ1  0x55
#define C_DISK  0x0B

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

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
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
    // TODO: Implement this function

    return 0;
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
