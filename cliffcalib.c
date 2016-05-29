/* 
  Use this program (adapted from csp3demo.c) to calibrate the cliff sensors
  for a particular surface and a particular robot.

  Usage: put the robot on the surface and then:

      cliffcalib > filename

  which causes the robot to move about for a while and then creates
  which creates a file with the given name containing the ranges of
  the four cliff sensors on the surface. This file can be used in 
  combination with the program makethresh to create the file 
  cliffThesholds.dat needed by programs that constrain the create 
  robot to limited colored regions.

  This program was originally written by Rich Sutton (rich@richsutton.com) 
  in June, 2013.
*/

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define FALSE 0
#define TRUE 1

typedef unsigned char ubyte;

//------------------------------------------------------------------
// ---------------          Create codes           -----------------
//------------------------------------------------------------------

#define CREATE_START         128
#define CREATE_SAFE          131
#define CREATE_FULL          132
#define CREATE_SPOT          134
#define CREATE_DRIVE         137
#define CREATE_SONG          140
#define CREATE_PLAY          141
#define CREATE_SENSORS       142
#define CREATE_DRIVE_DIRECT  145
#define CREATE_DIGITAL_OUTS  147
#define CREATE_STREAM        148
#define CREATE_STREAM_PAUSE  150

#define SENSOR_ALL_SENSORS        6
#define SENSOR_WHEEL_DROP         7
#define SENSOR_IRBYTE             17
#define SENSOR_DISTANCE           19
#define SENSOR_ROTATION           20
#define SENSOR_BAT_VOLTAGE        22
#define SENSOR_BAT_CURRENT        23
#define SENSOR_BAT_TEMP           24
#define SENSOR_BAT_CHARGE         25
#define SENSOR_CLIFF_LEFT         28
#define SENSOR_CLIFF_FRONT_LEFT   29
#define SENSOR_CLIFF_FRONT_RIGHT  30
#define SENSOR_CLIFF_RIGHT        31

//------------------------------------------------------------------
// ---------          Global Names and variables           ---------
//------------------------------------------------------------------

unsigned int pktNum = 0;      // Number of the packet currently being constructed 
pthread_mutex_t pktNumMutex, serialMutex, lastActionMutex; // locks
struct timeval lastPktTime;   // time of last packet
int fd = 0;                   // file descriptor for serial port
#define B 18                  // number of bytes in a packet
ubyte packet[B];              // packet is constructed here

//sensory arrays:
#define M 1000
unsigned short  sCliffL[M], sCliffR[M], sCliffFL[M], sCliffFR[M]; // small pos integers
short  sDistance[M];          // wheel rotation counts (small integers, pos/neg)
double sDeltaT[M];            // in milliseconds
//------------------------------------------------------------------

void sendBytesToRobot(ubyte* bytes, int numBytes, int dotcdrain) {
  int ret;
  pthread_mutex_lock( &serialMutex );
  if ((ret=write(fd, bytes, numBytes))==-1) {
    fprintf(stderr, "Problem with write(): %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
  if (dotcdrain) {
    if ((ret=tcdrain(fd))==-1) {
      fprintf(stderr, "Problem with tcdrain(): %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    }
  }
  pthread_mutex_unlock( &serialMutex );
}

#define MAX(a,b) (a > b?a:b)
#define MIN(a,b) (a < b?a:b)

void driveWheels(int left, int right) {
  ubyte bytes[5];
  left = MIN(500, left);
  left = MAX(-500, left);
  right = MIN(500, right);
  right = MAX(-500, right);
  bytes[0] = CREATE_DRIVE_DIRECT;
  bytes[1] = (right >> 8) & 0x00FF;
  bytes[2] = right & 0x00FF;
  bytes[3] = (left >> 8) & 0x00FF;
  bytes[4] = left & 0x00FF;
  sendBytesToRobot(bytes, 5, FALSE);
}

void setupSerialPort(char serialPortName[]) {
  struct termios options;
  // open connection
  if((fd = open(serialPortName, O_RDWR | O_NOCTTY | O_NONBLOCK))==-1) {
    fprintf(stderr, "Serial port at %s could not be opened: %s\n", 
	   serialPortName, strerror(errno));
    exit(EXIT_FAILURE);
  }
  tcflush (fd, TCIOFLUSH);
  tcgetattr(fd, &options);
  options.c_iflag = IGNBRK | IGNPAR;
  options.c_lflag = 0;
  options.c_oflag = 0;
  options.c_cflag     = CREAD | CS8 | CLOCAL; // CLOCAL not needed for cu.portname
  cfsetispeed(&options, B57600);
  cfsetospeed(&options, B57600);
  tcsetattr(fd, TCSANOW, &options);
  // go to passive mode:
  ubyte byte;
  byte = CREATE_START;
  sendBytesToRobot(&byte, 1, FALSE);
  // go to full mode:
  byte = CREATE_FULL;
  sendBytesToRobot(&byte, 1, FALSE);
  // Request stream mode:
  ubyte bytes[7];
  bytes[0] = CREATE_STREAM;
  bytes[1] = 5;
  bytes[2] = SENSOR_CLIFF_LEFT;
  bytes[3] = SENSOR_CLIFF_FRONT_LEFT;
  bytes[4] = SENSOR_CLIFF_FRONT_RIGHT;
  bytes[5] = SENSOR_CLIFF_RIGHT;
  bytes[6] = SENSOR_DISTANCE;
  sendBytesToRobot(bytes, 7, FALSE);
}

// expects (19) (SENSOR_SIZE-3) (SENSOR_CLIFF_LEFT) () () ... (checksum)
int checkPacket() {
  int i, sum;
  if (packet[0]==19 &&
      packet[1]==B-3 &&
      packet[2]==SENSOR_CLIFF_LEFT &&
      packet[5]==SENSOR_CLIFF_FRONT_LEFT &&
      packet[8]==SENSOR_CLIFF_FRONT_RIGHT &&
      packet[11]==SENSOR_CLIFF_RIGHT &&
      packet[14]==SENSOR_DISTANCE) {
    sum = 0;
    for (i = 0; i < B; i++) sum += packet[i];
    if ((sum & 0xFF) == 0) return 1;
  }
  return 0;
}

void extractPacket() {
  struct timeval currentTime;
  int p = pktNum%M;
  sCliffL[p]   = packet[3]<<8 | packet[4];
  sCliffFL[p]  = packet[6]<<8 | packet[7];
  sCliffFR[p]  = packet[9]<<8 | packet[10];
  sCliffR[p]   = packet[12]<<8 | packet[13];
  sDistance[p] = packet[15]<<8 | packet[16];
  gettimeofday(&currentTime, NULL);
  sDeltaT[p] = (currentTime.tv_sec - lastPktTime.tv_sec)*1000
    + ((double) currentTime.tv_usec - lastPktTime.tv_usec)/1000;
  lastPktTime = currentTime;
}


void* csp3(void *arg) {
  int errorCode, numBytesRead, i, j;
  ubyte bytes[B];
  int numBytesPreviouslyRead = 0;
  struct timeval timeout;
  fd_set readfs;

  pthread_mutex_init(&pktNumMutex, NULL);
  pthread_mutex_init(&serialMutex, NULL);
  pthread_mutex_init(&lastActionMutex, NULL);

  gettimeofday(&lastPktTime, NULL);
  FD_SET(fd, &readfs);

  while (TRUE) {
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    errorCode = select(fd+1, &readfs, NULL, NULL, &timeout);
    if (errorCode==0) {
      fprintf(stderr, "Timed out at select()\n");
    } else if (errorCode==-1) {
      fprintf(stderr, "Problem with select(): %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    }
    numBytesRead = read(fd, &bytes, B-numBytesPreviouslyRead);
    if (numBytesRead==-1) {
      fprintf(stderr, "Problem with read(): %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    } else {
      for (i = 0; i < numBytesRead; i++) packet[numBytesPreviouslyRead+i] = bytes[i];
      numBytesPreviouslyRead += numBytesRead;
      if (numBytesPreviouslyRead==B) {  //packet complete!
	if (checkPacket()) {
	  extractPacket();
	  pthread_mutex_lock( &pktNumMutex );
	  pktNum++;
	  pthread_mutex_unlock( &pktNumMutex );
	  numBytesPreviouslyRead = 0;
	} else {
	  fprintf(stderr, "misaligned packet.\n");
	  for (i = 1; i<B; i++) packet[i-1] = packet[i];
	  numBytesPreviouslyRead--;
	}
      }
    }
  }
  return NULL;
}

#define SPEED 150

int main(int argc, char *argv[]) {
  pthread_t tid;
  int t_err;
  unsigned int myPktNum;
  int p, min, max, value;
  int i;
  ubyte bytes[2];

  if (argc != 2) {
    fprintf(stderr, "Portname argument required -- something like /dev/tty.usbserial\n");
    return 0;
  }
  setupSerialPort(argv[1]);
  t_err = pthread_create(&tid, NULL, &csp3, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }

  for (i = 1; i < 100; i++) {  // wiggle 10sec to collect data
    driveWheels(rand()%(SPEED+1)-SPEED/2, rand()%(SPEED+1)-SPEED/2);
    usleep(100000);  // 0.1 seconds with the same random action
  }
  driveWheels(0, 0); // stop
  tcdrain(fd);

  pthread_mutex_lock( &pktNumMutex );
  myPktNum = pktNum;
  pthread_mutex_unlock( &pktNumMutex );

  if (myPktNum > M) {
    fprintf(stderr, "Buffer overflow!\n");
    exit(EXIT_FAILURE);
  }

  min = 5000;
  max = 0;
  for (p = 100; p < myPktNum; p++) {
    value = sCliffL[p];
    if (value > max) max = value;
    if (value < min) min = value;
  }
  printf("%d %d\n", min, max);

  min = 5000;
  max = 0;
  for (p = 100; p < myPktNum; p++) {
    value = sCliffFL[p];
    if (value > max) max = value;
    if (value < min) min = value;
  }
  printf("%d %d\n", min, max);

  min = 5000;
  max = 0;
  for (p = 100; p < myPktNum; p++) {
    value = sCliffFR[p];
    if (value > max) max = value;
    if (value < min) min = value;
  }
  printf("%d %d\n", min, max);

  min = 5000;
  max = 0;
  for (p = 100; p < myPktNum; p++) {
    value = sCliffR[p];
    if (value > max) max = value;
    if (value < min) min = value;
  }
  printf("%d %d\n", min, max);
  // pause streaming
  bytes[0] = CREATE_STREAM_PAUSE;
  bytes[1] = 0;
  sendBytesToRobot(bytes, 2, TRUE);
}
