#ifndef ROBOT_HPP
#define ROBOT_HPP

///BASICS LIB
#include <iostream>

///SDL 2.0 LIB
#include <SDL2/SDL_system.h>
#include <SDL2/SDL.h>
#ifdef __APPLE__
#include <SDL2_ttf/SDL_ttf.h>
#else
#include <SDL2/SDL_ttf.h>
#endif

// Nos libs
#include "LidarTreatments.hpp"

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
    int nbPoints=0;
}Block;
typedef struct {
    int x=0;
    int y=0;
    
}Point;



class Robot {
    public :
    Robot(int SCREEN_WIDTH,int SCREEN_HEIGHT );
    
    ~Robot();
    bool moveForaward();
    bool pointInBlock(int point_x, int point_y, Block blc );
    void drawBlockRect(Block bp, int tx=0, int ty=0);
    void drawBlockSecurity();
    void drawRobot();
    void drawBumpers();
    void scan(int maxPointBumper,int maxPointSecu);
    
    public :
    Point robot;
    Block blcRobot;
    Block blcSecu;
    Block blcBumperRight;
    Block blcBumperLeft;
    
    int speedLeft;
    int speedRight;
    int coefV;
    SDL_Renderer* context;
    Block pointTest;
    double ** lidar;
    int SCREEN_WIDTH;
    int SCREEN_HEIGHT;
};


#endif
