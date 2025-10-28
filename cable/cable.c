// Virtual cable program to test serial port.
// Creates a pair of virtual Tx / Rx serial ports using "socat".
//
// Author: Manuel Ricardo [mricardo@fe.up.pt]
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]
// Modified by: Rui Prior [rcprior@fc.up.pt]
// Updated by: ChatGPT (GPT-5) to support directional disconnection (offrx/onrx/offtx/ontx)

#include <fcntl.h>
#include <math.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define TXDEV "/dev/ttyS10"
#define RXDEV "/dev/ttyS11"
#define TX_EMULATOR "/dev/emulatorTx"
#define RX_EMULATOR "/dev/emulatorRx"

#define BAUDRATE B9600
#define DEFAULT_BAUDRATE 9600
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1
#define BUF_SIZE 2048

// Current running parameters
struct Parameters {
    int cableOn;
    int txToRxOn;
    int rxToTxOn;
    double byteER;
    struct timespec byteDelay;
    unsigned long propDelay;
    int bufSize;
    char *tx2rx;
    char *tx2rxValid;
    long tx2rxIdx;
    char *rx2tx;
    char *rx2txValid;
    long rx2txIdx;
    FILE *logfile;
};

struct Parameters par = {
    .cableOn = TRUE,
    .txToRxOn = TRUE,
    .rxToTxOn = TRUE,
    .byteER = 0.0,
    .propDelay = 0,
    .tx2rx = NULL,
    .tx2rxValid = NULL,
    .rx2tx = NULL,
    .rx2txValid = NULL,
    .logfile = NULL};

// === [Functions to open and configure serial ports] ===
int openSerialPort(const char *serialPort, struct termios *oldtio, struct termios *newtio) {
    int fd = open(serialPort, O_RDWR | O_NONBLOCK | O_NOCTTY);
    if (fd < 0) return -1;

    if (tcgetattr(fd, oldtio) == -1) return -1;

    memset(newtio, 0, sizeof(*newtio));
    newtio->c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio->c_iflag = IGNPAR;
    newtio->c_oflag = 0;
    newtio->c_lflag = 0;
    newtio->c_cc[VTIME] = 0;
    newtio->c_cc[VMIN] = 0;
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, newtio) == -1) return -1;
    return fd;
}

// === [Helper functions] ===
void addNoiseToBuffer(unsigned char *buf, size_t errorIndex) {
    buf[errorIndex] ^= 0xFF;
}

int init_ring_buffers(void) {
    long nsecPropDelay = 1000 * par.propDelay;
    long bytesInFlight = nsecPropDelay / par.byteDelay.tv_nsec;
    if (nsecPropDelay % par.byteDelay.tv_nsec > par.byteDelay.tv_nsec / 2) ++bytesInFlight;
    long actualPropDelay = bytesInFlight * par.byteDelay.tv_nsec / 1000;
    par.bufSize = bytesInFlight + 1;
    par.tx2rx = realloc(par.tx2rx, par.bufSize);
    par.tx2rxValid = realloc(par.tx2rxValid, par.bufSize);
    par.rx2tx = realloc(par.rx2tx, par.bufSize);
    par.rx2txValid = realloc(par.rx2txValid, par.bufSize);
    if (!par.tx2rx || !par.tx2rxValid || !par.rx2tx || !par.rx2txValid) return -1;
    bzero(par.tx2rxValid, par.bufSize);
    bzero(par.rx2txValid, par.bufSize);
    par.tx2rxIdx = 0;
    par.rx2txIdx = 0;
    printf("PROPAGATION DELAY SET TO %ld usec (DESIRED = %lu usec)\n", actualPropDelay, par.propDelay);
    return 0;
}

void set_baud_rate(unsigned long baud) {
    double delay = 1.0e10 / baud;
    par.byteDelay.tv_sec = 0;
    par.byteDelay.tv_nsec = (long)delay;
    printf("BAUD RATE: %lu\n", baud);
    init_ring_buffers();
}

void set_rt_priority(void) {
    struct sched_param sp = {.sched_priority = 50};
    if (sched_setscheduler(0, SCHED_FIFO, &sp) == -1) {
        perror("Could not set realtime priority");
    }
}

struct timespec timespec_diff(const struct timespec *t2, const struct timespec *t1) {
    struct timespec diff = {.tv_sec = t2->tv_sec - t1->tv_sec, .tv_nsec = t2->tv_nsec - t1->tv_nsec};
    if (diff.tv_nsec < 0) {
        diff.tv_nsec += 1000000000;
        --diff.tv_sec;
    }
    return diff;
}

struct timespec timespec_sum(const struct timespec *t1, const struct timespec *t2) {
    struct timespec sum = {.tv_sec = t1->tv_sec + t2->tv_sec, .tv_nsec = t1->tv_nsec + t2->tv_nsec};
    if (sum.tv_nsec >= 1000000000) {
        sum.tv_nsec -= 1000000000;
        ++sum.tv_sec;
    }
    return sum;
}

int timespec_is_negative(const struct timespec *t) {
    return (t->tv_sec < 0 || t->tv_nsec < 0);
}

void endlog(void) {
    if (par.logfile) {
        fclose(par.logfile);
        par.logfile = NULL;
    }
}

void startlog(const char *filename) {
    endlog();
    par.logfile = fopen(filename, "w");
    if (par.logfile)
        fprintf(par.logfile, "Tx->Rx | Rx->Tx\n"), printf("LOGGING TO FILE %s\n", filename);
    else
        printf("ERROR OPENING FILE %s, NOT LOGGING\n", filename);
}

void help() {
    printf("\n\n"
           "Transmitter must open " TXDEV "\n"
           "Receiver must open " RXDEV "\n"
           "\n"
           "Interactive commands:\n"
           "--- help         : show this help\n"
           "--- on/off       : connect/disconnect both directions\n"
           "--- onrx/offrx   : enable/disable Rx->Tx channel (ACK direction)\n"
           "--- ontx/offtx   : enable/disable Tx->Rx channel (data direction)\n"
           "--- ber <ber>    : add noise to data bits at given BER\n"
           "--- baud <rate>  : set baud rate\n"
           "--- prop <delay> : set propagation delay (usec)\n"
           "--- log <file>   : start logging\n"
           "--- endlog       : stop logging\n"
           "--- quit         : exit program\n\n");
}

// === [Main Program] ===
int main(int argc, char *argv[]) {
    printf("\n");
    system("socat -dd PTY,link=" TXDEV ",mode=777,raw,echo=0 PTY,link=" TX_EMULATOR ",mode=777,raw,echo=0 &");
    sleep(1);
    printf("\n");
    system("socat -dd PTY,link=" RXDEV ",mode=777,raw,echo=0 PTY,link=" RX_EMULATOR ",mode=777,raw,echo=0 &");
    sleep(1);

    help();

    struct termios oldtioTx, newtioTx, oldtioRx, newtioRx;
    int fdTx = openSerialPort(TX_EMULATOR, &oldtioTx, &newtioTx);
    if (fdTx < 0) { perror("Opening Tx emulator serial port"); exit(-1); }
    int fdRx = openSerialPort(RX_EMULATOR, &oldtioRx, &newtioRx);
    if (fdRx < 0) { perror("Opening Rx emulator serial port"); exit(-1); }

    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    char rxStdin[BUF_SIZE] = {0};
    int STOP = FALSE;
    set_baud_rate(DEFAULT_BAUDRATE);
    set_rt_priority();

    printf("\nCable ready\n\n");

    struct timespec currentTime, nextTxTime, timeDiff, nextWait;
    int skipWait = FALSE;
    int unreliableRate = FALSE;
    clock_gettime(CLOCK_MONOTONIC, &nextTxTime);

    while (!STOP) {
        clock_gettime(CLOCK_MONOTONIC, &currentTime);
        timeDiff = timespec_diff(&currentTime, &nextTxTime);
        nextTxTime = timespec_sum(&nextTxTime, &par.byteDelay);
        nextWait = timespec_diff(&nextTxTime, &currentTime);
        skipWait = timespec_is_negative(&nextWait) ? TRUE : FALSE;

        // Read from both ends
        int bytesFromTx = read(fdTx, par.tx2rx + par.tx2rxIdx, 1);
        par.tx2rxValid[par.tx2rxIdx] = bytesFromTx > 0;

        int bytesFromRx = read(fdRx, par.rx2tx + par.rx2txIdx, 1);
        par.rx2txValid[par.rx2txIdx] = bytesFromRx > 0;

        // Handle cable state and directional disable
        if (!par.cableOn || !par.txToRxOn)
            par.tx2rxValid[par.tx2rxIdx] = 0;
        if (!par.cableOn || !par.rxToTxOn)
            par.rx2txValid[par.rx2txIdx] = 0;

        // Forward bytes if valid
        if (par.cableOn && par.txToRxOn && par.tx2rxValid[par.tx2rxIdx])
            write(fdRx, par.tx2rx + par.tx2rxIdx, 1);
        if (par.cableOn && par.rxToTxOn && par.rx2txValid[par.rx2txIdx])
            write(fdTx, par.rx2tx + par.rx2txIdx, 1);

        par.tx2rxIdx = (par.tx2rxIdx + 1) % par.bufSize;
        par.rx2txIdx = (par.rx2txIdx + 1) % par.bufSize;

        // Handle user commands
        int fromStdin = read(STDIN_FILENO, rxStdin, BUF_SIZE);
        if (fromStdin > 0) {
            rxStdin[fromStdin - 1] = '\0';
            if (strcmp(rxStdin, "off") == 0) {
                printf("CONNECTION OFF\n"); par.cableOn = FALSE;
            } else if (strcmp(rxStdin, "on") == 0) {
                printf("CONNECTION ON\n"); par.cableOn = TRUE;
            } else if (strcmp(rxStdin, "offrx") == 0) {
                printf("RX->TX CONNECTION OFF\n"); par.rxToTxOn = FALSE;
            } else if (strcmp(rxStdin, "onrx") == 0) {
                printf("RX->TX CONNECTION ON\n"); par.rxToTxOn = TRUE;
            } else if (strcmp(rxStdin, "offtx") == 0) {
                printf("TX->RX CONNECTION OFF\n"); par.txToRxOn = FALSE;
            } else if (strcmp(rxStdin, "ontx") == 0) {
                printf("TX->RX CONNECTION ON\n"); par.txToRxOn = TRUE;
            } else if (strncmp(rxStdin, "ber ", 4) == 0) {
                double ber; sscanf(rxStdin + 4, "%lf", &ber);
                double acc = 1 - ber; acc *= acc; acc *= acc; acc *= acc;
                par.byteER = 1.0 - acc;
                if (ber >= 0.0 && ber < 1.0) printf("BER SET TO %lf\n", ber);
                else printf("BAD BER VALUE %lf\n", ber);
            } else if (strncmp(rxStdin, "baud ", 5) == 0) {
                unsigned long baud; sscanf(rxStdin + 5, "%lu", &baud);
                switch (baud) {
                    case 1200: case 1800: case 2400: case 4800:
                    case 9600: case 19200: case 38400: case 57600: case 115200:
                        set_baud_rate(baud); break;
                    default:
                        printf("UNSUPPORTED BAUD RATE\n");
                }
            } else if (strncmp(rxStdin, "prop ", 5) == 0) {
                unsigned long propDelay;
                if (sscanf(rxStdin + 5, "%lu", &propDelay) < 1 || propDelay > 1000000)
                    printf("BAD PROPAGATION DELAY\n");
                else { par.propDelay = propDelay; init_ring_buffers(); }
            } else if (strncmp(rxStdin, "log ", 4) == 0) {
                startlog(rxStdin + 4);
            } else if (strcmp(rxStdin, "endlog") == 0) {
                endlog(); printf("NOT LOGGING\n");
            } else if (strcmp(rxStdin, "quit") == 0) {
                printf("END OF PROGRAM\n"); STOP = TRUE;
            } else if (strcmp(rxStdin, "help") == 0) {
                help();
            } else {
                printf("BAD COMMAND OR MISSING PARAMETERS\n");
            }
        }
        if (!skipWait) nanosleep(&nextWait, NULL);
    }

    tcsetattr(fdRx, TCSANOW, &oldtioRx);
    tcsetattr(fdTx, TCSANOW, &oldtioTx);
    close(fdTx); close(fdRx);
    system("killall socat");
    return 0;
}
