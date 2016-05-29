/* 
  Sarsa demo of the Create serial-port packet processor" (aka csp3).

  This is a second version, with all interaction with the serial port
  going thru the csp3 thread, without locks.

  This file is a self-contained demonstration of the use of the
  "Create serial-port packet processor", a small program for managing
  the real-time interaction of a serial-port connection to an iRobot
  Create robot. The idea is to make it easy to write control programs
  (e.g., reinforcement learning agents) for the Create without
  sacrificing performance in any way. Here we use it to write a simple
  Sarsa(lambda) agent using the four cliff sensors in a binary way to
  make 16 states.

  The recommended usage is to make a copy of this demo file, then modify
  it according to your needs. (For example, adding or subtracting from 
  the set of sensors that are streamed out from the Create robot.)

  But first, you should understand the basics of the program's
  operation.  The serial-port connection to the Create is set up and
  put in a mode where it streams out a packet of sensor data every 15
  milliseconds. Each packet contains data for a specific set of
  sensors that is specified at setup time. The 15ms cycle corresponds
  to how fast the data is generated internally by the Create, so
  getting it in this way is arguably the best possible way. However,
  this way of receiving data is not convenient (in general) for
  writing agent code that processes the sensor data and sends commands
  to the Create. The agent code may involve extensive processing
  lasting more than 15ms, or it may want to send commands for actions
  that are meant to apply for more or less than 15ms. It would be
  awkward to try to break the processing or the actions into smaller
  pieces that fit into 15ms chunks. Even worse, when using the
  bluetooth version of the serial port (via a BAM) the interval
  between packets can become very variable, from 0 to 100ms. There is
  no easy way for agent code to fit into the spaces between packets.

  Basically, at the agent level, one would like to ignore the 15ms
  streaming cycle. Ideally the agent would just obtain the latest
  sensory information at any time, use it for action selections or
  computations of any duration, then repeat. This agent cycle, or
  timestep, might last 100ms, for example. Ideally the cycle would be
  allowed to be of any duration, even less than 15ms, and even varying
  from cycle to cycle.

  The csp3 allows you to do all these things. Basically, we just have
  two threads, one for the csp3 and one for the agent code. The csp3
  thread manages the interaction with the serial port, collecting data
  as it arrives to form a complete packet roughly every 15ms, then
  processes the packets data into a form convenient for reading by the
  agent thread when it next chooses to. Several packets (or no
  packets) may be processed in between agent cycles, and all the
  historical packet data from all of them (up to some limit into the
  past) are made available to the agent thread.

  To take actions, the agent thread sets the inter-thread global
  variable "action". The csp3 thread reads this variable when it 
  finishes processing a Create packet, and then sends a corresponding 
  command to the serial port (and thus to the robot) if needed.

  As a user and modifier of this code, you should understand the
  following key aspects of the communication and coordination between
  the csp3 thread and the agent thread:

    1. After the serial-port connection to the robot is set up, the
       csp3 thread does all the reading from and writing to the serial
       port (and because of this no lock is needed).

    2. The global variable pktNum is the number of the currently being
       constructed packet. (Thus pktNum-1 is the number of the last
       completed packet, the latest packet with reliable data in it
       that can be read by the agent thread.) pktNum starts at 0 and
       is incremented forever. (Thus, if a run lasted more than
       15*2^32 ms, which is more than two years, there could be a
       problem.) All writes of pktNum (by the csp3 thread only) and
       reads of pktNum by the agent thread, must be protected by a
       mutex_lock to remove any possibilityof corruption by the two
       processes trying to work with it at the same time.

    3. Sensor readings are constructed by the csp3 thread, and made
       available to the agent thread, in many "sensory" arrays, each
       indexed by packet number. These are given natural names
       descriptive of their sensory data, except that they begin with
       's', for "sensory". For example (but see 5 below):
           sCliffL[p] is the left cliff-sensor reading in the p-th packet
           sRemote[p] is the button pressed on the remote in the p-th packet
           sBumperR[p] is 1/0 corresponding to the right bumper in the p-th packet
           sDeltaT[p] is the time elapsed between the p-th and (p-1)th packet
       The sensory arrays are many and varied and are specific to your
       use; feel free to add more and remove those that you don't
       need. These are all written exclusively by the csp3 thread and
       read by the agent thread. No locks are needed on them because
       the agent only reads data for packets after their writing has
       been completed (p<pktNum).

    4. When the agent thread starts a cycle, it typically will make
       its own copy of pktNum, called myPktNum. The value of myPktNum
       on the last agent cycle may be kept in an agent-thread variable
       called lastPktNum. Then the agent knows that it might want to
       process, at most, all the packet data from lastPktNum to
       myPktNum-1.

    5. As described so far, the sensory arrays in 3 above would need
       an entry for every packet, which would be rather big, and
       unnecessary because generally the agent needs to see only the
       packets since its last cycle, as described in 4 above. Thus, in
       reality only data for the most recent M packets is retained,
       and all the sensory arrays are only of length M. Thus, when you
       access, say, the sCliffL data for the p-th packet, you actually
       access sCliffL[p%M] (p mod M). The mod-ing is typically done at
       array-access time; pktNum, myPktNum, and lastPktNum are all
       kept in explicit form without mod-ing.

    6. The agent thread presumably wants to send commands to the
       Create. It does this by setting the global variable "action".
       The csp3 thread reads this (with a mutex lock) at the time of
       forming each packet and then sends the appropriate command to
       the robot, if needed. This involves sending bytes (according the Create
       Open Interface specification) to the serial port. The command
       sent is recorded in a sensory array or arrays. For example:
           sDrive[p] is drive command send after the p-th packet
       The drive command is one of {0,1,2,3,4}, corresponding to 
       forward, turn left, turn right, backward, and stop.

    7. The agent variable "action" is used, in combination with the
       cliff sensor data and their thresholds, to detect when the
       action might take the robot outside a prescribed region, in
       which case the robot is instead sent a command to stop its
       motion. This is an example of a "reflex", which we here define
       as an immediate response to sensor data implemented by the
       csp3. Other reflexes might set the Create's lights or cause it
       to make sound. The ability to implement reflexes running at the
       Creates fastest sensory rate is another advantage of csp3's
       two-thread approach.

-----
  To add a new sensor you need to make a number of changes in a number 
  of places. Here are the steps:

    1. Find the sensor id of the new sensor in the iRobot Create Open 
       Interface manual. In the manual, sensor id's are called packet 
       id's (because they indicate that a little "packet" of data is 
       due to that sensor). For example, the left cliff sensor has sensor
       (packet) id 28. This code contains a list of symbolic names
       of many of the sensor id's, and the one you want to add may be included.
       If not, feel free to add it.
    2. While you have the Open Interface manual open to the page for
       your sensor, give it a good read. Fully understand the data 
       produced by your sensor. How many bytes is it? How is the data
       represented? You will need to understand these things for the 
       next steps.
    3. Find the global #define for B, the number of bytes in a sensor
       packet. With your new sensor, B will have to be increased. For
       example, if your sensor data has two bytes, then B will have to 
       be increased by 3 (1 for the sensor id and 2 for the data).
    4. Add a declaration of the sensory array for your new sensor to the
       beginning of this file in the Global Names and Variables section.
       Follow the example of the existing names.
    5. Edit the function setupSerialPort: 
         a) add 1 to the value that bytes[1] is set to
	 b) add 1 to the size of the bytes array
	 c) add a new line setting the new last element of the bytes 
	    array to the sensor id of your new sensor
	 d) add 1 to the number in the sendBytesToRobot call
    6. Edit the function checkPacket by adding one line for the new
       sensor at the end of the list of &&s. Follow the pattern of
       the existing lines. To get the array index right you will have
       to know the number of bytes in the data of the previous sensor
       (or infer it from the previous value of B).
    7. Edit the function extractPacket by adding one line for the new
       sensor just before the blank line. Follow the pattern of
       the existing lines. This is where your understanding of the data
       format for your sensor will come in.

-----
  Other things you will need to know:

  To get going, you need a port name to provide as a command line
  argument. This could be either using bluetooth and a BAM or using
  a physical wire to the robot. The wire gives faster and more uniform
  timing but then...it's a wire. In both case, after configuring, the
  port name will appear in /dev with a name like /dev/tty.something, 
  where the something might be usbserial for a wire or SerialElement-Se
  for bluetooth. To configure the wire, it may be sufficient to simply
  plug in the wire, or you may need to download a software driver 
  provided by the manufacturer of your usb-to-serial cable. For bluetooth, 
  go to system preferences > bluetooth and set up the connection and get
  the name. You can also set up a better name for the connection there.

  How do you stop? You can either interrupt your program with ^C or
  press the pause button on the remote. Or you can chase the robot down
  to press the power off button.

  It seems to be a good idea to power the robot down and up again
  before running the program.

  Many times the program can't get started properly and you have to
  try starting it several times. We have not figured that out yet but
  it does not seem to be a big deal. It will work eventually. On the
  other hand, if the robot's battery gets low it just will never start.
  The battery may be good enough for the built-in demos and for slow
  actions and still too low for the program to start with faster actions.

  This 2nd version of the program was originally written by Rupam Mahmood
  (ashique@ualberta.ca) and Rich Sutton (rich@richsutton.com) in July, 2013.
 */

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/select.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <float.h>
#include <signal.h>

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
// ---------          Global Names and Variables           ---------
//------------------------------------------------------------------
#define SPEED 300
unsigned int pktNum = 0;      // Number of the packet currently being constructed by csp3
pthread_mutex_t pktNumMutex, actionMutex, rewardMusicMutex; // locks
int action = 0;           // current action selected by agent (initially forward)
struct timeval lastPktTime;   // time of last packet
int rewardMusic = 0;
int fd = 0;                   // file descriptor for serial port
#define B 20                  // number of bytes in a packet
ubyte packet[B];              // packet is constructed here

//sensory arrays:
#define M 1000
unsigned short  sCliffL[M], sCliffR[M], sCliffFL[M], sCliffFR[M]; // small pos integers
ubyte  sCliffLB[M], sCliffRB[M], sCliffFLB[M], sCliffFRB[M];      // binary 1/0
short  sDistance[M];          // wheel rotation counts (small integers, pos/neg)
double sDeltaT[M];            // in milliseconds
ubyte sIRbyte[M];             // Infrared byte e.g. remote
ubyte sDrive[M];              // Drive command in {0, 1, 2, 3, 4}

int cliffThresholds[4];       // left, front left, front right, right
int cliffHighValue;           // binary value taken if threshold exceeded
//------------------------------------------------------------------

void setupSerialPort(char serialPortName[]);
void* csp3(void *arg);void loadCliffThresholds();
void takeAction(int action);
int epsilonGreedy(double Q[16][4], int s, double epsilon);
void endProgram();
void driveWheels(int left, int right);
void sendBytesToRobot(ubyte* bytes, int numBytes);
void ensureTransmitted();
int getPktNum();

int main(int argc, char *argv[]) {
  pthread_t tid;
  int t_err;
  unsigned int prevPktNum;
  unsigned int myPktNum;
  int p, pn;
  double Q[16][4], e[16][4];
  double stepsize = 0.1, lambda = 0.9, gamma = 0.98, epsilon = 0.01;
  int a, aprime;
  int s, sprime;
  int reward;
  int i, j;
  double delta;
  ubyte bytes[2];
  int rewardReport;
  struct timeval timeStart, timeEnd, incrementBy;
  long computationTime;
  struct sigaction act;
  struct sigaction oldact;

  act.sa_handler = endProgram;
  sigemptyset(&act.sa_mask);
  act.sa_flags = 0;
  sigaction(SIGINT, &act, &oldact);
  if (argc < 2) {
    fprintf(stderr, "Portname argument required -- something like /dev/tty.usbserial\n");
    return 0;
  }
  loadCliffThresholds();
  srand(0);
  pthread_mutex_init(&pktNumMutex, NULL);
  pthread_mutex_init(&actionMutex, NULL);
  pthread_mutex_init(&rewardMusicMutex, NULL);

  setupSerialPort(argv[1]);
  usleep(20000); // wait for at least one packet to have arrived
  t_err = pthread_create(&tid, NULL, &csp3, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }
  prevPktNum = 0;

  // initialize Q
  for (i = 0; i < 16; i++)
    for (j = 0; j < 4; j++) {
      Q[i][j] = 5 + 0.001*( rand()/((double) RAND_MAX) - 0.5);
      e[i][j] = 0;
    }
  gettimeofday(&timeStart, NULL);
  myPktNum = getPktNum();
  p = (myPktNum + M - 1) % M;
  s = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
  a = epsilonGreedy(Q, s, epsilon);
  pthread_mutex_lock( &actionMutex );
  action = a; // sets up action to be taken by csp thread
  pthread_mutex_unlock( &actionMutex );  
  prevPktNum = myPktNum;
  rewardReport = 0;
  while (TRUE) { // main agent loop
    gettimeofday(&timeEnd, NULL);
    computationTime = (timeEnd.tv_sec-timeStart.tv_sec)*1000000
      + (timeEnd.tv_usec-timeStart.tv_usec);
    printf("Time for iteration (in microseconds): %ld\n", computationTime);
    if (100000 - computationTime > 0) usleep(100000 - computationTime);
    else printf("This iteration took too long!\n\n");
    incrementBy.tv_sec = 0;
    incrementBy.tv_usec = 100000;
    timeradd(&timeStart, &incrementBy, &timeStart);
    myPktNum = getPktNum();
    if (myPktNum - prevPktNum > M) {
      fprintf(stderr, "Buffer overflow!\n");
      exit(EXIT_FAILURE);
    }
    reward = 0;
    for (pn = prevPktNum; pn < myPktNum; pn++) {
      p = pn % M;
      reward += sDistance[p];
      printf("deltaT: %f cliff sensors: %u(%u) %u(%u) %u(%u) %u(%u) distance: %hd\n",
	     sDeltaT[p],
	     sCliffL[p],sCliffLB[p],sCliffFL[p],sCliffFLB[p],
	     sCliffFR[p],sCliffFRB[p],sCliffR[p],sCliffRB[p],
	     (short) sDistance[p]);
      if (sIRbyte[p]==137) endProgram(); // quit on remote pause
    }
    rewardReport += reward;
    if (rewardReport > 50) {
      pthread_mutex_lock( &rewardMusicMutex );
      rewardMusic = 1;
      pthread_mutex_unlock( &rewardMusicMutex );    
      rewardReport -= 50;
    }
    p = (myPktNum - 1) % M;
    sprime = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
    aprime = epsilonGreedy(Q, sprime, epsilon);
    pthread_mutex_lock( &actionMutex );
    action = aprime; // sets up action to be taken by csp thread
    pthread_mutex_unlock( &actionMutex );    
    delta = reward + gamma*Q[sprime][aprime] - Q[s][a];
    for (j = 0; j < 4; j++)
      e[s][j] = 0;
    e[s][a] = 1;
    printf("s a r s' a':%d %d %d %d %d\n", s, a, reward, sprime, aprime);
    for (i = 0; i < 16; i++) {
      printf("Action values for state %d: %f %f %f %f\n",i, Q[i][0], Q[i][1], Q[i][2], Q[i][3]);
      printf("Eligibility traces for state %d: %f %f %f %f\n", i, e[i][0], e[i][1], e[i][2], e[i][3]);
      for (j = 0; j < 4; j++) {
        Q[i][j] = Q[i][j] + stepsize*delta*e[i][j];
        e[i][j] = gamma*lambda*e[i][j];
      }
    }
    s = sprime;
    a = aprime;
    prevPktNum = myPktNum;
  }
  return 0;
}

int getPktNum() {
  int myPktNum;
  pthread_mutex_lock( &pktNumMutex );
  myPktNum = pktNum;
  pthread_mutex_unlock( &pktNumMutex );
  return myPktNum;  
}

void loadCliffThresholds() {
  FILE *fd;
  if ((fd = fopen("cliffThresholds.dat", "r"))==NULL) {
    fprintf(stderr, "Error opening cliffThresholds.dat file: %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
  fscanf(fd, "%d%d%d%d%d", &cliffHighValue, 
	 &cliffThresholds[0], &cliffThresholds[1], 
	 &cliffThresholds[2], &cliffThresholds[3]);
  fclose(fd);
}

int epsilonGreedy(double Q[16][4], int s, double epsilon)
{
  int max, i, p;
  int firstAction, lastAction;

  p = (getPktNum() + M - 1) % M;
  firstAction = sCliffFLB[p] || sCliffFRB[p];
  if (sCliffLB[p] || sCliffRB[p]) lastAction = 2;
  else lastAction = 3;
  if (rand()/((double)RAND_MAX+1) < epsilon) {
    printf("random action\n\n");
    return firstAction + rand()%(lastAction + 1 - firstAction);
  } else {
    max = lastAction;
    for (i = firstAction; i < lastAction; i++)
      if (Q[s][i] > Q[s][max])
        max = i;
    return max;
  }
}

void takeAction(int action) {
    switch (action) {
    case 0  : driveWheels(SPEED, SPEED); break;    // forward
    case 1  : driveWheels(-SPEED, SPEED); break;   // left
    case 2  : driveWheels(SPEED, -SPEED); break;   // right
    case 3  : driveWheels(-SPEED, -SPEED); break;  // backward
    case 4  : driveWheels(0, 0); break;            // stop
    default : printf("Bad action\n");
    }
}

void endProgram() {
  ubyte bytes[2];
  printf("Ending Program\n");
  driveWheels(0, 0);
  // pause streaming
  bytes[0] = CREATE_STREAM_PAUSE;
  bytes[1] = 0;
  sendBytesToRobot(bytes, 2);
  tcdrain(fd);
  exit(EXIT_SUCCESS);
}

void sendBytesToRobot(ubyte* bytes, int numBytes) {
  int ret;
  if ((ret=write(fd, bytes, numBytes))==-1) {
    fprintf(stderr, "Problem with write(): %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void ensureTransmitted() {
  int ret;
  if ((ret=tcdrain(fd))==-1) {
	fprintf(stderr, "Problem with tcdrain(): %s\n", strerror(errno));
	exit(EXIT_FAILURE);
  }
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
  sendBytesToRobot(bytes, 5);
}

void setupSerialPort(char serialPortName[]) {
  struct termios options;
  ubyte byte;
  ubyte bytes[8];

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
  byte = CREATE_START;
  sendBytesToRobot(&byte, 1);
  // go to full mode:
  byte = CREATE_FULL;
  sendBytesToRobot(&byte, 1);
  // Request stream mode:
  bytes[0] = CREATE_STREAM;
  bytes[1] = 6;
  bytes[2] = SENSOR_CLIFF_LEFT;
  bytes[3] = SENSOR_CLIFF_FRONT_LEFT;
  bytes[4] = SENSOR_CLIFF_FRONT_RIGHT;
  bytes[5] = SENSOR_CLIFF_RIGHT;
  bytes[6] = SENSOR_DISTANCE;
  bytes[7] = SENSOR_IRBYTE;
  sendBytesToRobot(bytes, 8);
  // Setup songs
  bytes[0] = CREATE_SONG;
  bytes[1] = 0;
  bytes[2] = 1;
  bytes[3] = 100;
  bytes[4] = 6;
  sendBytesToRobot(bytes, 5);
  ensureTransmitted();
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
      packet[14]==SENSOR_DISTANCE &&
      packet[17]==SENSOR_IRBYTE) {
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
  sCliffLB[p]  = sCliffL[p]>cliffThresholds[0] ? cliffHighValue : 1-cliffHighValue;
  sCliffFL[p]  = packet[6]<<8 | packet[7];
  sCliffFLB[p] = sCliffFL[p]>cliffThresholds[1] ? cliffHighValue : 1-cliffHighValue;
  sCliffFR[p]  = packet[9]<<8 | packet[10];
  sCliffFRB[p] = sCliffFR[p]>cliffThresholds[2] ? cliffHighValue : 1-cliffHighValue;
  sCliffR[p]   = packet[12]<<8 | packet[13];
  sCliffRB[p]  = sCliffR[p]>cliffThresholds[3] ? cliffHighValue : 1-cliffHighValue;
  sDistance[p] = packet[15]<<8 | packet[16];
  sIRbyte[p] = packet[18];

  gettimeofday(&currentTime, NULL);
  sDeltaT[p] = (currentTime.tv_sec - lastPktTime.tv_sec)*1000
    + ((double) currentTime.tv_usec - lastPktTime.tv_usec)/1000;
  lastPktTime = currentTime;
}

void reflexes();

void* csp3(void *arg) {
  int errorCode, numBytesRead, i;
  ubyte bytes[B];
  int numBytesPreviouslyRead = 0;
  struct timeval timeout;
  fd_set readfs;

  gettimeofday(&lastPktTime, NULL);
  FD_SET(fd, &readfs);

  while (TRUE) {
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    errorCode = select(fd+1, &readfs, NULL, NULL, &timeout);
    if (errorCode==0) {
      printf("Timed out at select()\n");
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
	  reflexes();
	  ensureTransmitted();
	  pthread_mutex_lock( &pktNumMutex );
	  pktNum++;
	  pthread_mutex_unlock( &pktNumMutex );
	  numBytesPreviouslyRead = 0;
	} else {
	  printf("misaligned packet.\n");
	  for (i = 1; i<B; i++) packet[i-1] = packet[i];
	  numBytesPreviouslyRead--;
	}
      }
    }
  }
  return NULL;
}

void reflexes() {
  int p = pktNum%M;
  int myRewardMusic;
  pthread_mutex_lock( &actionMutex );
  sDrive[p] = action;
  pthread_mutex_unlock( &actionMutex );
  if ((sDrive[p]==0 && (sCliffFLB[p] || sCliffFRB[p])) || // if forward over cliff
      (sDrive[p]==3 && (sCliffLB[p] || sCliffRB[p])))    // or backward over cliff
    sDrive[p] = 4;                            // then stop instead
  takeAction(sDrive[p]);

  ubyte bytes[2];
  ubyte frontbit = sCliffFLB[p] || sCliffFRB[p];
  ubyte ledbits = (sCliffLB[p] << 2) | (frontbit << 1) | sCliffRB[p];
  bytes[0] = CREATE_DIGITAL_OUTS;
  bytes[1] = ledbits;
  sendBytesToRobot(bytes, 2);

  pthread_mutex_lock( &rewardMusicMutex );
  myRewardMusic = rewardMusic;
  pthread_mutex_unlock( &rewardMusicMutex );

  if (myRewardMusic == 1) {
    bytes[0] = 141;
    bytes[1] = 0;
    sendBytesToRobot(bytes, 2);
    pthread_mutex_lock( &rewardMusicMutex );
    rewardMusic = 0;
    pthread_mutex_unlock( &rewardMusicMutex );    
  }
}



