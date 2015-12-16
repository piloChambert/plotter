#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> 

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

void sendCommand(const char *cmd, int ttyFile) {
  write(ttyFile, cmd, strlen(cmd));

  // read answer
  readAnswer(ttyFile, answerBuffer);
  char answer[256];
  read(ttyFile, answer, 256);
  printf("%s\n", answerBuffer);
}

void sendHPGLFile(const char *hpglFilename, int ttyFile) {
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

      // send the command
      printf("\nSending : %s...\n", buffer);
      sendCommand(buffer, ttyFile);

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
}

int main(int argc, char *argv[]) {
  if(argc != 3) {
    printf("Usage %s ttyport hpglfile\n\n", argv[0]);
    exit(0);
  }

  int fd = openSerial(argv[1]);
 
  /* Wait for the Arduino to reset */
  usleep(1000*1000);
  
  /* Flush anything already in the serial buffer */
  tcflush(fd, TCIFLUSH);
  
  printf("Waiting Arduino...\n");  
  readAnswer(fd, answerBuffer);
  printf("Arduino : %s\n", answerBuffer);
  
  // send commands
  sendHPGLFile(argv[2], fd);

  return 0;
}
