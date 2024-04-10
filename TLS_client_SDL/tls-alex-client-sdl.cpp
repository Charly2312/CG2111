/*
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>
#include <fcntl.h>
#include <cstring>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
*/

#include <iostream>
// Installed SDL2 on laptop
#include <SDL2/SDL.h>

// Routines to create a TLS client
#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

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
	bool moving = false;
	printf("Command (w=forward, s=reverse, a=turn left, d=turn right, e=stop, p=clear stats, c=get stats q=exit)\n");

	while(!quit)
	{
		char ch;
		SDL_Event event;
		bool moving = false;
		char buffer[10];
		int32_t params[2];

		buffer[0] = NET_COMMAND_PACKET;
	
		while(SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT) {
				quit = 1;
			}

			const Uint8* state = SDL_GetKeyboardState(NULL);
			if (state[SDL_SCANCODE_W]) {
				printf("Moving Forward\n");
				ch = 'w';
				moving = true; break; 
			} else if (state[SDL_SCANCODE_S]) {
				printf("Moving Backwards\n");
				ch = 's'; 
				moving = true; break;
			} else if (state[SDL_SCANCODE_D]) {
				printf("Moving Right\n");
				ch = 'd'; 
				moving = true; break;
			} else if (state[SDL_SCANCODE_A]) {
				printf("Moving Left\n");
				ch = 'a'; 
				moving = true; break;
			} else if (state[SDL_SCANCODE_E]) {
				printf("Stop\n");
				ch = 'e';
			} else if (state[SDL_SCANCODE_Q]) {
				printf("Exit\n");
				ch = 'q'; 
			} else if (state[SDL_SCANCODE_P]) {
				printf("Clear Stats\n");
				ch = 'p';
			} else if (state[SDL_SCANCODE_C]) {
				printf("Get Stats\n");
				ch = 'q';
			}  else	if (moving) {
				printf("Stop\n");
				ch = 'E';
				moving = false;
			} 
			/*	
			switch (ch)
			{
			case 'w':
			case 'W':
			case 's':
			case 'S':
				params[0] = 50; 
				params[1] = 50;
				buffer[1] = ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(conn, buffer, sizeof(buffer));
				moving = true;
				break;
			case 'a':
			case 'A':
			case 'd':
			case 'D':
				params[0] = 50;
				params[1] = 50;
				buffer[1] = ch;
				memcpy(&buffer[2], params, sizeof(params));
				sendData(conn, buffer, sizeof(buffer));
				moving = true;	
				break;
			case 'e':
			case 'E':	
			case 'c':
			case 'C':
			case 'p':
			case 'P':
				params[0] = 0;
				params[1] = 0;
				memcpy(&buffer[2], params, sizeof(params));
				buffer[1] = ch;
				sendData(conn, buffer, sizeof(buffer));
				moving = false;
				break;
			case 'q':
			case 'Q':
				quit = 1;
				break;
			default:
				printf("BAD COMMAND\n");
			}
			*/
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
	
	// Create SDL Window
	SDL_Window* window=nullptr;
	
	if(SDL_Init(SDL_INIT_VIDEO) < 0){
		std::cout << "SDL could not be initialised: " << SDL_GetError();
	} else {
		std::cout << "SDL video system is a go\n";
	}

	// Create window
	window =  SDL_CreateWindow("C++ SDL2 Window", 
		0,
		2500,
		640,
		480,
		SDL_WINDOW_SHOWN); 
	
    while(client_is_running());
	
	SDL_DestroyWindow(window);
	SDL_Delay(3000);
	SDL_Quit();

	printf("\nMAIN exiting\n\n");
	return 0;
}
