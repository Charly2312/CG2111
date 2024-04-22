// To Compile
// g++ tls-alex-client-sdl.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lcrypto -lSDL2 -o tls-alex-client

// To make c++ code work
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

// Only send new packet when receive ack from Server
static volatile bool sending_status = false;
static volatile bool waiting_for_data = false;

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
			waiting_for_data = false;
			break;

		case RESP_BAD_RESPONSE:
			printf("PI GOT BAD RESPONSE FROM ARDUINO\n");
			waiting_for_data = false;
			break;

		default:
			sending_status = false;
			waiting_for_data = false;
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
	printf("Color:\t\t\t\t%d\n", data[10]);
	printf("Red value:\t\t\t%d\n", data[11]);
	printf("Green value:\t\t\t%d\n", data[12]);
	printf("Blue value:\t\t\t%d\n", data[13]);
	printf("Distance from wall or object:\t%d\n", data[14]);
	printf("\n---------------------------------------\n\n");

}

void handleDistance(const char *buffer)
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
	printf("Distance from wall or object:\t%d\n", data[14]);
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
			sending_status = false;
			break;

		case NET_STATUS_PACKET:
			handleStatus(buffer);
			waiting_for_data = false;
			sending_status = false; // Just added
			break;

		case NET_DISTANCE_PACKET:
			handleDistance(buffer);
			waiting_for_data = false;
			sending_status = false; // Just
			break;

		case NET_MESSAGE_PACKET:
			handleMessage(buffer);
			waiting_for_data = false;
			break;

			// Not needed
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
		//  Insert SSL write here to write buffer to network
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
		// Insert SSL read here into buffer		
		len = sslRead(conn, buffer, sizeof(buffer));
		printf("read %d bytes from server.\n", len);


		networkActive = (len > 0);

		if(networkActive)
			handleNetwork(buffer, len);
	}

	printf("Exiting network listener thread\n");

	// Stop client loop and call EXIT_THREAD 
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


void *writerThread(void *conn)
{
	int quit=0;
	bool moving = false;
	printf("Command (w=forward, s=reverse, a=turn left, d=turn right, e=stop, p=clear stats, c=get stats q=exit)\n");

	char ch;
	while(!quit)
	{
		SDL_Event event;

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
				ch = 'e'; break;
			} else if (state[SDL_SCANCODE_Q]) {
				printf("Exit\n");
				ch = 'q'; break;
			} else if (state[SDL_SCANCODE_C]) {
				printf("Clear Stats\n");
				ch = 'c'; break;
			} else if (state[SDL_SCANCODE_G]) {
				printf("Get Stats\n");
				ch = 'g'; break;
			} else if (state[SDL_SCANCODE_R]) {
				printf("Get Distance\n");
				ch = 'r'; break;
			} else if (state[SDL_SCANCODE_M]) {
				printf("Get Manual L (left) or R (right) \n");
				scanf("%c", &ch); break;
			} else if(moving) {
				printf("Stop\n");
				ch = '.'; // Stop after moving case
				break;
			} /*else {
				ch = '/';
			} */
		}

		if (!sending_status && !waiting_for_data && ch != '/')  {
			printf("%c\n", ch);
			switch (ch)
			{
				case 'w':
				case 's':
					sending_status = true;
					params[0] = 10; 
					params[1] = 40;
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					moving = true;
					break;
				case 'a':
				case 'd':
					sending_status = true;
					params[0] = 10;
					params[1] = 50;
					buffer[1] = ch;
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
					moving = true;	
					break;
				case 'e':
				case 'c':
					sending_status = true;
					params[0] = 0;
					params[1] = 0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = ch;
					sendData(conn, buffer, sizeof(buffer));
					moving = false;
					break;

				case 'g':
				case 'r':
					sending_status = true; // Just
					waiting_for_data = true;
					params[0] = 0;
					params[1] = 0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = ch;
					sendData(conn, buffer, sizeof(buffer));
					moving = false;
					break;
				case 'L':
					sending_status = true;
					getParams(params);
					buffer[1] = 'a';
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
				case 'R':
					sending_status = true;
					getParams(params);
					buffer[1] = 'd';
					memcpy(&buffer[2], params, sizeof(params));
					sendData(conn, buffer, sizeof(buffer));
				case 'q':
					quit = 1;
					break;
				case '.':
					sending_status = true;
					params[0] = 0;
					params[1] = 0;
					memcpy(&buffer[2], params, sizeof(params));
					buffer[1] = 'e';
					sendData(conn, buffer, sizeof(buffer));
					moving = false;
					ch = '/';
					break;
				default:
					printf("BAD COMMAND\n");
			}
			ch = '/';
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
			200,
			200,
			SDL_WINDOW_SHOWN); 

	while(client_is_running());

	SDL_DestroyWindow(window);
	SDL_Delay(3000);
	SDL_Quit();

	printf("\nMAIN exiting\n\n");
	return 0;
}
