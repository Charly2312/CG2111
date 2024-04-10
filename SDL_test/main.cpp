#include <iostream>
#include <SDL2/SDL.h>

int main(int argc, char* arg[]) {
	

	SDL_Window* window=nullptr;

	if(SDL_Init(SDL_INIT_VIDEO) < 0){
		std::cout << "SDL could not be initialised: " << SDL_GetError();
	}  else {
		std::cout << "SDL video system is a go\n";
	}

	window = SDL_CreateWindow("C++ SDL2 Window",
			0,
			2500,
			640,
			480,
			SDL_WINDOW_SHOWN);


	char ch = '.';
	
	bool running = true;
	bool moving = false;

	while(running) {
		SDL_Event event;
		while(SDL_PollEvent(&event)) { 
		
			if(event.type == SDL_QUIT) {
				running= false;
			}
			else if (event.type=SDL_KEYDOWN) {
				//printf("A key has been pressed\n");
				if (event.key.keysym.sym == SDLK_0) {
				printf("0 was pressed\n"); break;
				}				
			} 
			else if (event.type=SDL_KEYUP) {
				//printf("A key has been pressed\n");
				if (event.key.keysym.sym == SDLK_0) {
				printf("0 was released\n");
				}				
			}
			const Uint8* state = SDL_GetKeyboardState(NULL);
			if (state[SDL_SCANCODE_1]){
				printf("1 is pressed\n");
				moving = true; break;
			} else if (state[SDL_SCANCODE_2]){
				printf("2 is pressed\n");
				moving = true; break;
			}  else if (moving) {
				printf("Sending stop\n");
				moving = false;
			}
		}
	}

	SDL_DestroyWindow(window);
	SDL_Delay(3000);
	SDL_Quit();
	
	return 0;

}
