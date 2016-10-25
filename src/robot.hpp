#ifndef ROBOT_HPP
#define ROBOT_HPP

///BASICS LIB
#include <iostream>

///SDL 2.0 LIB
#include <SDL2/SDL_system.h>
#include <SDL2/SDL.h>
#include <SDL2_ttf/SDL_ttf.h>

// Nos libs
#include "Lidar_tri.hpp"

using namespace std;

typedef struct {
	
	int r=0;
	int g=0;
	int b=0;
	
}RGB;

typedef struct {
	
	int w=0;
	int h=0;
	int x=0;
	int y=0;
	bool hasCollision;
	RGB rgb;
}Block;



class Robot {    
    public :
		Robot(int SCREEN_WIDTH,int SCREEN_HEIGHT );

		~Robot();
		bool moveForaward();
		bool pointInBlock(int point_x, int point_y, Block blc );
		void drawBlockRect(Block bp);
		void drawBlockSecurity();
		void drawRobot();
		void drawBumpers();
		void scan();
		
	public :
		Block blcRobot;
		Block blcSecu;
		Block blcBumperRight;
		Block blcBumperLeft;
		
		int speedLeft;
		int speedRight;
		SDL_Renderer* context;
		Block pointTest;
		double ** lidar;
		
};


#endif
