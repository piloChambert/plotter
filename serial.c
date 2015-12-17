#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> 
#include <math.h>
#include <sys/time.h>

typedef struct {
  char commandStr[16]; // size might not be enough?
  float distance; // 
} HPGLCommand;

int openSerial(const char *port) {
  int fd = open(port, O_RDWR | O_NOCTTY);

  // get the current options
  struct termios toptions;
  tcgetattr(fd, &toptions);

  // set speed
  cfsetispeed(&toptions, B115200);
  cfsetospeed(&toptions, B115200);

  /* 8 bits, no parity, no stop bits */
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;

  /* no hardware flow control */
  toptions.c_cflag &= ~CRTSCTS;

  /* enable receiver, ignore status lines */
  toptions.c_cflag |= CREAD | CLOCAL;
  
  /* disable input/output flow control, disable restart chars */
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY);

  /* disable canonical input, disable echo,
     disable visually erase chars,
     disable terminal-generated signals */
  toptions.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  /* disable output processing */
  toptions.c_oflag &= ~OPOST;

  /* wait for 24 characters to come in before read returns */
  toptions.c_cc[VMIN] = 0;
  
  /* no minimum time to wait before read returns */
  toptions.c_cc[VTIME] = 0;

  /* commit the options */
  tcsetattr(fd, TCSANOW, &toptions);

  return fd;
}

char answerBuffer[1024];
// read serial until ;
void readAnswer(int fd, char *buffer) {
  // we just wait for an input
  char c[1];
  c[0] = 0;
  char *ptr = buffer;

  while(c[0] != ';') {
    int count = read(fd, c, 1);

    if(count > 0) {
      *ptr = c[0];
      ptr++;

      //printf("%c\n", c);
    } else {
      usleep(1000);
    }
  }

  *ptr = 0;
}

void sendCommand(char *cmd, int ttyFile) {
  write(ttyFile, cmd, strlen(cmd));

  // read answer
  readAnswer(ttyFile, answerBuffer);
  char answer[256];
  read(ttyFile, answer, 256);
  //printf("%s\n", answerBuffer);
}

int commandCount = 0; // total number of command
float totalDistance = 0.0f; // total distance of the pen
HPGLCommand *commandBuffer = NULL;

int readNumber(char *ptr, char **endPtr) {
  int v = 0;

  while(isdigit(*ptr)) {
    v = v * 10 + (*ptr - '0');
    ptr++;
  }

  if(endPtr != NULL) {
    *endPtr = ptr;
  }

  return v;
}

// process HPGL file
void processHPGLFile(const char *hpglFilename) {
  FILE *hpglFile = fopen(hpglFilename, "r");
  if(hpglFile == NULL) {
    fprintf(stderr, "Can't open file %s!!\n", hpglFilename);
    return;
  }

  // read data from the file and send command
  char buffer[256];
  char *bufferPtr = buffer;
  char c;

  int commaCount = 0; // number of comma read, used to split the PU and PD command

  int currentX = 0;
  int currentY = 0;

  int commandBufferSize = 256;
  commandBuffer = (HPGLCommand *)malloc(sizeof(HPGLCommand) * commandBufferSize);

  while((c = fgetc(hpglFile)) != EOF) {
    *bufferPtr = c;
    bufferPtr++;

    if(c == ',') {
      commaCount++;
    }

    if((c == ';') || (c == ',' && commaCount == 2)) {
      // make sure we end with ';'
      bufferPtr--; *bufferPtr = ';'; bufferPtr++;
      *bufferPtr = 0; // end the string

      // increase buffer size if needed
      if(commandCount >= commandBufferSize) {
	commandBufferSize *= 2;
	commandBuffer = (HPGLCommand *)realloc(commandBuffer, sizeof(HPGLCommand) * commandBufferSize);
      }

      // handle the command
      float dist = 0;
      if((strncmp(buffer, "PD", 2) == 0) || (strncmp(buffer, "PU", 2) == 0)) {
	char *ptr;
	int x = readNumber(&buffer[2], &ptr);
	int y = readNumber(++ptr, NULL);

	float _x = x - currentX;
	float _y = y - currentY;

	dist = sqrtf(_x * _x + _y * _y);

	totalDistance += dist;

	//printf("%d, %d %f\n", x, y, dist);

	currentX = x;
	currentY = y;
      }

      strcpy(commandBuffer[commandCount].commandStr, buffer);
      commandBuffer[commandCount].distance = dist;
      commandCount++;      

      // reset buffer
      if(c == ',') {
	// we keep the command for the next one
	bufferPtr = &buffer[2];
      } else {
	// we finished the current command
	bufferPtr = buffer;
      }

      // reset commaCount
      commaCount = 0;
    }
  }

  fclose(hpglFile);

  printf("Command count : %d\n", commandCount);
  printf("Total pen distance : %.0f\n", totalDistance);
}

void timeToStr(double time, char *str) {
  int h = (int)time / 3600;
  int m = ((int)time % 3600) / 60;
  int s = ((int)time % 3600) % 60;
  
  sprintf(str, "%dh%dm%ds", h, m, s);
}

void sendCommandToSerial(int ttyFile) {
  struct timeval  begin, end;
  double timeSpent = 0.0;
  double nextEvalTime = 20.0;
  double speed = -1; // pen speed

  float currentDistance = 0;
  int i;

  // start timer
  gettimeofday(&begin, NULL);

  for(i = 0; i < commandCount; i++) {
    char timeStr[256];
    if(speed < 0.0) {
      sprintf(timeStr, "unknown");
    } else {
      float remainingDistance = totalDistance - currentDistance;
      float remainingTime = remainingDistance * speed;

      timeToStr(remainingTime, timeStr);
    }

    char runTime[256];
    timeToStr(timeSpent, runTime);

    // print status
    printf("%.0f%% ETA %s Running Time %s (cmd %d/%d) (dist %d/%d) %s...                                                      \r",
	   currentDistance / totalDistance * 100.0f, 
	   timeStr,
	   runTime,
	   i, commandCount,
	   (int)currentDistance, (int)totalDistance,
	   commandBuffer[i].commandStr);
    
    // send command
    sendCommand(commandBuffer[i].commandStr, ttyFile);

    // update distance
    currentDistance += commandBuffer[i].distance;

    // update timer
    gettimeofday(&end, NULL);
    timeSpent =  (double) (end.tv_usec - begin.tv_usec) / 1000000 + (double) (end.tv_sec - begin.tv_sec);

    // update speed
    if(timeSpent > nextEvalTime) {
      speed = timeSpent / currentDistance;
      nextEvalTime += 60.0; // next update 
    }    
  }

  printf("\n");
}

int main(int argc, char *argv[]) {
  if(argc != 3) {
    printf("Usage %s ttyport hpglfile\n\n", argv[0]);
    exit(0);
  }

  // process file
  processHPGLFile(argv[2]);

  int fd = openSerial(argv[1]);

  /* Wait for the Arduino to reset */
  usleep(1000*1000);
  
  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);
  
  printf("Waiting Arduino...\n");  
  readAnswer(fd, answerBuffer);
  printf("Arduino : %s\n", answerBuffer);
  
  // send commands
  sendCommandToSerial(fd);

  return 0;
}
