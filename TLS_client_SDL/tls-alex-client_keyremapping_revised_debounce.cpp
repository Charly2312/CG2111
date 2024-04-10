#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h> // for usleep 
#include <stdint.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <fcntl.h>
#include <cstring>
#include "packet.h"
#include "serial.h"
#include "serialize.h"

// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

// debounce time

// Tells us that the network is running.
static volatile int networkActive=0;

void handleError(const char *buffer)
{
	switch(buffer[1])
	{
		case RESP_OK:
			printf("Command / Status OK\n");
			break;

		case RESP_BAD_PACKET:
			printf("BAD MAGIC NUMBER FROM ARDUINO\n");
			break;

		case RESP_BAD_CHECKSUM:
			printf("BAD CHECKSUM FROM ARDUINO\n");
			break;

		case RESP_BAD_COMMAND:
			printf("PI SENT BAD COMMAND TO ARDUINO\n");
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			break;

		default:
			printf("PI IS CONFUSED!\n");
	}
}

void handleStatus(const char *buffer)
{
	int32_t data[16];
	memcpy(data, &buffer[1], sizeof(data));

	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", data[0]);
	printf("Right Forward Ticks:\t\t%d\n", data[1]);
	printf("Left Reverse Ticks:\t\t%d\n", data[2]);
	printf("Right Reverse Ticks:\t\t%d\n", data[3]);
	printf("Left Forward Ticks Turns:\t%d\n", data[4]);
	printf("Right Forward Ticks Turns:\t%d\n", data[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", data[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", data[7]);
	printf("Forward Distance:\t\t%d\n", data[8]);
	printf("Reverse Distance:\t\t%d\n", data[9]);
	printf("Red Value:\t\t%d\n", data[10]);
	printf("Green Value:\t\t%d\n", data[11]);
	printf("Blue Value:\t\t%d\n", data[12]);
	printf("\n---------------------------------------\n\n");
}

void handleMessage(const char *buffer)
{
	printf("MESSAGE FROM ALEX: %s\n", &buffer[1]);
}

void handleCommand(const char *buffer)
{
	// We don't do anything because we issue commands
	// but we don't get them. Put this here
	// for future expansion
}

void handleNetwork(const char *buffer, int len)
{
	// The first byte is the packet type
	int type = buffer[0];

	switch(type)
	{
		case NET_ERROR_PACKET:
		handleError(buffer);
		break;

		case NET_STATUS_PACKET:
		handleStatus(buffer);
		break;

		case NET_MESSAGE_PACKET:
		handleMessage(buffer);
		break;

		case NET_COMMAND_PACKET:
		handleCommand(buffer);
		break;
	}
}

void sendData(void *conn, const char *buffer, int len)
{
	int c;
	printf("\nSENDING %d BYTES DATA\n\n", len);
	if(networkActive)
	{
		c = sslWrite(conn, buffer, len);
		networkActive = (c > 0);
	}
}

void *readerThread(void *conn)
{
	char buffer[128];
	int len;

	while(networkActive)
	{
		
		len = sslRead(conn, buffer, sizeof(buffer));
        printf("read %d bytes from server.\n", len);
		

		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");
    
	stopClient();
	EXIT_THREAD(conn);

    return NULL;
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}


//To see if keyboard has been pressed without blocking the Programms execution
bool kbhit() {
	struct termios oldt, newt;
	int ch;
	int oldf;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	if (ch != EOF) {
		return true;
	}
	else {
		return false;
	}
}

// To get a single char from standard input without echoing it to the terminal
char getch() {
	char buf = 0;
	struct termios old = { 0 };
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
	if (read(0, &buf, 1) < 0)
		perror("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		perror("tcsetattr ~ICANON");
	return (buf);
}


// Not required anymore
void getParams(int32_t *params)
{
	printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	scanf("%d %d", &params[0], &params[1]);
	flushInput();
}


// Runs and sends information from client calls getch() and kbhit()
void *writerThread(void *conn)
{
	int quit=0;
	bool stopped = true;

	char lastKey = 0;
	bool keyPressed = false;
	int debounceStartTime = std::chrono::steady_clock::now();

	printf("Command (w=forward, s=reverse, a=turn left, d=turn right, e=stop, p=clear stats, c=get stats q=exit)\n");
	while(!quit)
	{
		char ch;
	//	printf("Command (w=forward, s=reverse, a=turn left, d=turn right, e=stop, p=clear stats, c=get stats q=exit)\n");
		//scanf("%c", &ch);

		// Purge extraneous characters from input stream
		//flushInput();

		char buffer[10];
		int32_t params[2];
		buffer[0] = NET_COMMAND_PACKET;
		if (kbhit()) {
			ch = getch(); // Was char ch // also redundant since we already called it? can modify kbhit to return ch instead of checking again
			switch (ch)
			{
				// TODO Param 0 should not matter? Add in Manual case to use getparams() function
			case 'w':
			case 'W':
			case 's':
			case 'S':
			case 'a':
			case 'A':
			case 'd':
			case 'D':
				int currentTime_a = std::chrono::steady_clock::now();
				int elapsed_a = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime_a - debounceStartTime).count();

				if (!keyPressed || elapsed_a > 200) { // if key is not pressed before or pressing time exceeds 0.2s
					keyPressed = true;
					params[0] = 50;
					params[1] = 50;
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					debounceStartTime = currentTime_a; // reset debounce time
				
				}
				break;
			case 'e':
			case 'E':
			case 'c':
			case 'C':
			case 'p':
			case 'P':
				//auto currentTime = std::chrono::steady_clock::now();
				//auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - debounceStartTime).count();
				/*
				if (!keyPressed || elapsed > 200) { // if key is not pressed before or pressing time exceeds 0.2s
					keyPressed = true;
					params[0] = 0;
					params[1] = 0;
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					//debounceStartTime = currentTime; // reset debounce time
					break;
				}*/
				break;
			case 'q':
			case 'Q':
				quit = 1;
				break;
			default:
				printf("BAD COMMAND\n");
				break;
			}
		} else if(keyPressed && (ch == EOF || ch == '\n' || ch == '\r')) {
			// detect EOF, means release of button
			keyPressed = false;
			ch = 'e';
			params[0] = 0;
			params[1] = 0;
			memcpy(&buffer[2], params, sizeof(params));
			buffer[1] = ch;
			sendData(conn, buffer, sizeof(buffer));
		}
	}

	printf("Exiting keyboard thread\n");

	stopClient();
	EXIT_THREAD(conn);

    return NULL;
}

#define CA_CERT_FNAME  "signing.pem"
#define CLIENT_CERT_FNAME  "laptop.crt"
#define CLIENT_KEY_FNAME  "laptop.key"
#define SERVER_NAME_ON_CERT "mine.com"

void connectToServer(const char *serverName, int portNum)
{
		createClient(serverName, portNum, 1,CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);
}

int main(int ac, char **av)
{
	if(ac != 3)
	{
		fprintf(stderr, "\n\n%s <IP address> <Port Number>\n\n", av[0]);
		exit(-1);
	}

    networkActive = 1;
    connectToServer(av[1], atoi(av[2]));

    while(client_is_running());

	printf("\nMAIN exiting\n\n");
}
