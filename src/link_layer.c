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
    BCC2_OK,
    STOP_STATE
} FrameState;

volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
int tramaTx = 0;
int tramaRx = 0;

int retransmissions = 0;
int timeout = 0;
LinkLayerRole role;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d received\n", alarmCount);
}

unsigned char readControlFrame() {

    unsigned char byte;
    unsigned char controlField = 0;
    FrameState state = START;

    printf("[readControlFrame] Waiting for control frame...\n");

    while(state != STOP_STATE && alarmEnabled) {

        if(readByteSerialPort(&byte) != 1) continue;

        switch(state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    printf("[readControlFrame] FLAG received (start of frame)\n");
                }
                break;
            case FLAG_RCV:
                if (byte == A_RE) {
                    state = A_RCV;
                    printf("[readControlFrame] A=0x%02X (A_RE) received\n", byte);
                }
                else if (byte != FLAG) state = START;
                break;
            case A_RCV:
                if (byte == C_RR0 || byte == C_RR1 || byte == C_REJ0 || byte == C_REJ1) {
                    state = C_RCV;
                    controlField = byte;
                    const char *type = 
                        (byte == C_RR0) ? "RR0" :
                        (byte == C_RR1) ? "RR1" :
                        (byte == C_REJ0) ? "REJ0" :
                        (byte == C_REJ1) ? "REJ1" : "UNKNOWN";
                    printf("[readControlFrame] C=0x%02X (%s) detected\n", byte, type);
                }
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case C_RCV:
                if (byte == (A_RE ^ controlField)) {
                    state = BCC1_OK;
                    printf("[readControlFrame] BCC1 OK (0x%02X)\n", byte);
                }
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case BCC1_OK:
                if (byte == FLAG) {
                    state = STOP_STATE;
                    printf("[readControlFrame] FLAG received — frame complete!\n");
                }
                else state = START;
                break;
            default:
                break;

        }
    }

    if (state == STOP_STATE) printf("[readControlFrame] Control frame received successfully: 0x%02X\n", controlField);
    else printf("[readControlFrame] Timeout or alarm triggered — control frame incomplete\n");

    return controlField;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    unsigned char SET[5] = {FLAG, A_ER, C_SET, A_ER ^ C_SET, FLAG};
    unsigned char UA[5] = {FLAG, A_RE, C_UA, A_RE ^ C_UA, FLAG};
    
    printf("[llopen] Initializing link on port %s (baud=%d)\n", connectionParameters.serialPort, connectionParameters.baudRate);

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0){
        perror("[llopen] Error opening serial port");
        exit(-1);
    }

    retransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    role = connectionParameters.role;
    printf("[llopen] Parameters: timeout=%ds, retransmissions=%d\n", timeout, retransmissions);

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
            printf("[llopen] Sending SET frame (attempt %d of %d)\n", connectionParameters.nRetransmissions - retries + 1, connectionParameters.nRetransmissions);
           
            writeBytesSerialPort(SET, 5);
            alarmEnabled = TRUE;
            alarm(connectionParameters.timeout);

            printf("[llopen] Waiting for UA (timeout = %ds)\n", timeout);

            while(alarmEnabled && state != STOP_STATE) {

                if(readByteSerialPort(&byte) != 1) continue;

                switch (state) {
                    case START:
                        if (byte == FLAG) {
                            state = FLAG_RCV;
                            printf("[llopen] FLAG received (start of frame)\n");
                        }
                        break;
                    case FLAG_RCV:
                        if (byte == A_RE)  {
                            state = A_RCV;
                            printf("[llopen] Received A=0x%02X (expected A_RE)\n", byte);
                        }
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_UA) {
                            state = C_RCV;
                            printf("[llopen] Received C=0x%02X (UA)\n", byte);
                        }
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_RE ^ C_UA)) {
                            state = BCC1_OK;
                            printf("[llopen] BCC1 OK (0x%02X)\n", byte);
                        }
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break; 
                    case BCC1_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                            printf("[llopen] UA frame complete! Link established.\n");
                        }
                        else state = START;
                        break;
                    default:
                        break;
                }
            }

            
            if (state == STOP_STATE) {
                alarm(0);
            } 
            else {
                retries--;
                printf("[llopen] No UA received (timeout). Retries left: %d\n", retries);
            }
        }

        if (state != STOP_STATE) {
            printf("[llopen] Failed to establish connection after %d attempts.\n", connectionParameters.nRetransmissions);
            return -1;
        }
    }

    else if (connectionParameters.role == LlRx) {

        printf("[llopen] Waiting for SET frame from transmitter...\n");
        
        while (state != STOP_STATE) {

            if (readByteSerialPort(&byte) != 1) {
                continue;
            }

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_ER) {
                        state = A_RCV;
                        printf("[llopen] Received A=0x%02X (A_ER)\n", byte);
                    }
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == C_SET) {
                        state = C_RCV;
                        printf("[llopen] Received C=0x%02X (SET)\n", byte);
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (A_ER ^ C_SET)) {
                        state = BCC1_OK;
                        printf("[llopen] BCC1 OK (0x%02X)\n", byte);
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (byte == FLAG) {
                        state = STOP_STATE;
                        printf("[llopen] SET frame complete! Sending UA response.\n");
                    }
                    else state = START;
                    break;
                default:
                    break;
            }
        }
        writeBytesSerialPort(UA, 5);
        printf("[llopen] Sent UA frame. Link established.\n");
    }

    printf("[llopen] Link layer open successful!\n");
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
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
                printf("[llwrite] Received RR%d (ACK)\n", tramaTx ? 0 : 1);
                acknowledged = 1;
                tramaTx = (tramaTx + 1) % 2;
                alarm(0);
            }
            else if (control == (tramaTx ? C_REJ1 : C_REJ0)) {
                printf("[llwrite] Received REJ%d — retransmitting\n", tramaTx ? 1 : 0);
                break;
            }
            else continue;
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
    unsigned char byte;
    FrameState state = START;
    unsigned char control = 0;
    int packetIndex = 0;
    int escapeNext = 0;
    unsigned char BCC2_calc = 0, receivedBCC2 = 0;

    printf("[llread] Waiting for information frame...\n");

    while(state != STOP_STATE) {
        if(readByteSerialPort(&byte) != 1) continue;
        printf("[llread] Read byte: 0x%02X\n", byte);

        switch(state) {
            case START:
                if(byte == FLAG) {
                    state = FLAG_RCV;
                    printf("[llread] FLAG received (start of frame)\n");
                }
                break;

            case FLAG_RCV:
                if(byte == A_ER) {
                    state = A_RCV;
                    printf("[llread] Received A=0x%02X (expected A_ER)\n", byte);
                }
                else if (byte != FLAG) state = START;
                break;

            case A_RCV:
                if(byte == C_I0 || byte == C_I1) {
                    state = C_RCV;
                    control = byte;
                    printf("[llread] Received C=0x%02X\n", byte);
                }
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;

            case C_RCV:
                if(byte == (A_ER ^ control)) {
                    state = BCC1_OK;
                    printf("[llread] BCC1 OK (0x%02X)\n", byte);
                }
                else if (byte == FLAG) state = FLAG_RCV;
                else state = START;
                break;

            case BCC1_OK:
                if(byte == FLAG) {
                    state = START;
                    printf("[llread] Empty frame, ignoring FLAG\n");
                } 
                else {
                    state = DATA_RCV;
                    packetIndex = 0;
                    escapeNext = 0;
                    printf("[llread] Starting payload collection\n");
                    goto handle_data_byte;
                }
                break;

            case DATA_RCV:
            handle_data_byte:
                if(byte == FLAG) {
                    if(packetIndex < 1) { state = START; break; }
                    receivedBCC2 = packet[packetIndex - 1];
                    BCC2_calc = 0;

                    for (int i = 0; i < packetIndex - 1; i++) {
                        BCC2_calc ^= packet[i];
                    }
                
                    packetIndex--;
                    state = STOP_STATE;
                    printf("[llread] Received end FLAG\n");
                }
                else if(escapeNext) {
                    byte ^= 0x20;
                    packet[packetIndex++] = byte;
                    escapeNext = 0;
                    printf("[llread] Escaped data byte: 0x%02X\n", byte, BCC2_calc);
                }
                else if(byte == ESC) {
                    escapeNext = 1;
                }
                else {
                    packet[packetIndex++] = byte;
                    printf("[llread] Data byte: 0x%02X\n", byte, BCC2_calc);
                }
                break;


            default:
                break;
        }
    }

    if(BCC2_calc == receivedBCC2) {
        printf("[llread] BCC2 OK (0x%02X)\n", receivedBCC2);

        unsigned char response[5];
        response[0] = FLAG;
        response[1] = A_RE;
        response[2] = (tramaRx == 0) ? C_RR1 : C_RR0;
        response[3] = response[1] ^ response[2];
        response[4] = FLAG;
        writeBytesSerialPort(response, 5);
        printf("[llread] Sent RR%d\n", (tramaRx == 0) ? 1 : 0);

        tramaRx = (tramaRx + 1) % 2;
        return packetIndex;
    } 
    
    else {
        printf("[llread] BCC2 ERROR\n");

        unsigned char response[5];
        response[0] = FLAG;
        response[1] = A_RE;
        response[2] = (tramaRx == 0) ? C_REJ0 : C_REJ1;
        response[3] = response[1] ^ response[2];
        response[4] = FLAG;
        writeBytesSerialPort(response, 5);
        printf("[llread] Sent REJ%d\n", (tramaRx == 0) ? 0 : 1);

        return -1;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    unsigned char DISC_TX[5] = {FLAG, A_ER, C_DISC, A_ER ^ C_DISC, FLAG};
    unsigned char DISC_RX[5] = {FLAG, A_RE, C_DISC, A_RE ^ C_DISC, FLAG};
    unsigned char UA[5]      = {FLAG, A_ER, C_UA, A_ER ^ C_UA, FLAG};

    unsigned char byte;
    FrameState state = START;
    int retries = retransmissions;

    printf("[llclose] Initiating link termination...\n");

    if(role == LlTx) {

        while (retries > 0 && state != STOP_STATE) {
            printf("[llclose] Sending DISC (attempt %d of %d)\n", retransmissions - retries + 1, retransmissions);

            writeBytesSerialPort(DISC_TX, 5);
            alarmEnabled = TRUE;
            alarm(timeout);

            state = START;

            while (alarmEnabled && state != STOP_STATE) {
                
                if(readByteSerialPort(&byte) != 1) continue;

                switch (state) {
                    case START:
                        if(byte == FLAG) state = FLAG_RCV;
                        break;

                    case FLAG_RCV:
                        if (byte == A_RE) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    
                    case A_RCV:
                        if (byte == C_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;

                    case C_RCV:
                        if (byte == (A_RE ^ C_DISC)) state = BCC1_OK;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;

                    case BCC1_OK:
                        if (byte == FLAG) {
                            state = STOP_STATE;
                            printf("[llclose] Received DISC from receiver\n");
                        }
                        else state = START;
                        break;

                    default:
                        break;
                }
            }

            if(state == STOP_STATE) {
                alarm(0);
            }
            else {
                retries--;
                printf("[llclose] Timeout waiting for DISC, retries left: %d\n", retries);
            }
        }

        if (state != STOP_STATE) {
            printf("[llclose] Failed to receive DISC after %d attempts.\n", retransmissions);
            return -1;
        }

        writeBytesSerialPort(UA, 5);
        printf("[llclose] Sent UA — disconnection complete.\n");
    }

    else if (role == LlRx) {
        printf("[llclose] Waiting for DISC from transmitter...\n");
        
        state = START;

        while (state != STOP_STATE) {
            if (readByteSerialPort(&byte) != 1) continue;

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == A_ER) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;

                case A_RCV:
                    if (byte == C_DISC) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case C_RCV:
                    if (byte == (A_ER ^ C_DISC)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case BCC1_OK:
                    if (byte == FLAG) {
                        state = STOP_STATE;
                        printf("[llclose] Received DISC from transmitter.\n");
                    } 
                    else state = START;
                    break;

                default:
                    break;
            }
        }

        printf("[llclose] Sending DISC response...\n");
        writeBytesSerialPort(DISC_RX, 5);

        state = START;
        printf("[llclose] Waiting for UA from transmitter...\n");

        while (state != STOP_STATE) {
            if (readByteSerialPort(&byte) != 1) continue;

            switch(state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == A_ER) state = A_RCV;
                    else if (byte != FLAG) state = START;
                    break;

                case A_RCV:
                    if (byte == C_UA) state = C_RCV;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case C_RCV:
                    if (byte == (A_ER ^ C_UA)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;

                case BCC1_OK:
                    if(byte == FLAG) {
                        state = STOP_STATE;
                        printf("[llclose] Received UA - disconnection complete.\n");
                    } 
                    else state = START;
                    break;

                default:
                    break;
            }
        }
    }

    closeSerialPort();
    printf("[llclose] Serial port closed successfully.\n");
    printf("[llclose] Transmission summary:\n");
    printf("    - Total retransmissions: %d\n", alarmCount);
    printf("    - Last frame: tramaTx=%d tramaRx=%d\n", tramaTx, tramaRx);
    printf("------------------------------------------------------\n");
    printf("Link layer terminated successfully.\n");

    return 0;
}
