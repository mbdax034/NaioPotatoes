
#include "robot.hpp"




Robot::Robot(int SCREEN_WIDTH,int SCREEN_HEIGHT ){
    
    
    robot.x=SCREEN_WIDTH/2 ;
    robot.y=SCREEN_HEIGHT/2;
    
    int widthBump= 20;
    int heightBump= 40;
    
    blcRobot.w=40;
    blcRobot.h=100;
    blcRobot.x= -(blcRobot.w / 2);
    blcRobot.y=0;
    blcRobot.rgb.r=100;
    blcRobot.rgb.g=100;
    blcRobot.rgb.b=200;
    
    
    
    blcSecu.w=blcRobot.w  ;
    blcSecu.h=heightBump;
    blcSecu.x=-blcSecu.w/2 ;
    blcSecu.y=-blcSecu.h-5;
    blcSecu.rgb.r=20;
    blcSecu.rgb.g=10;
    blcSecu.rgb.b=10;
    
    
    blcBumperRight.w=widthBump;
    blcBumperRight.h=heightBump;
    blcBumperRight.x=(blcRobot.w / 2);
    blcBumperRight.y=-heightBump;
    blcBumperRight.rgb.r=100;
    blcBumperRight.rgb.g=100;
    blcBumperRight.rgb.b=100;
    
    blcBumperLeft.w=widthBump;
    blcBumperLeft.h=heightBump;
    blcBumperLeft.x=-widthBump-(blcRobot.w / 2);
    blcBumperLeft.y=-heightBump;
    blcBumperLeft.rgb.r=100;
    blcBumperLeft.rgb.g=100;
    blcBumperLeft.rgb.b=100;
    
    speedLeft=5;
    speedRight=5;
    context=nullptr;
    coefV=2;
    
    
    pointTest.w=4;
    pointTest.h=4;
    pointTest.x=blcSecu.x-100;
    pointTest.y=blcSecu.y-pointTest.h/2 +10;
    
    pointTest.rgb.r=100;
    pointTest.rgb.g=200;
    pointTest.rgb.b=255;
    
}

Robot::~Robot( ) {
    
}

bool Robot::pointInBlock(int point_x, int point_y, Block blc) {
    if( point_x >= blc.x && point_x <= blc.x+blc.w && point_y >= blc.y && point_y <= blc.y+blc.h ) {
        return true;
    }
    
    return false ;
}
void Robot::drawBlockRect(Block bp, int tx, int ty){
    
    SDL_SetRenderDrawColor( context, bp.rgb.r, bp.rgb.g, bp.rgb.b, 100 ); // the rect color (solid red)
    SDL_Rect blocRect;
    blocRect.w = bp.w;
    blocRect.h = bp.h;
    blocRect.x = bp.x+tx;
    blocRect.y = bp.y+ty;
    
    
    SDL_RenderFillRect(context, &blocRect);
}


void Robot::drawBlockSecurity(){
    
    
    
    
    if(blcSecu.hasCollision){
        blcSecu.rgb.r=255;
        blcSecu.rgb.g=100;
        blcSecu.rgb.b=100;
    }else{
        blcSecu.rgb.r=50;
        blcSecu.rgb.g=50;
        blcSecu.rgb.b=10;
    }
    
    drawBlockRect(blcSecu,robot.x,robot.y);
    
}

void Robot::drawRobot(){
    
    
    
    drawBlockRect(blcRobot,robot.x,robot.y);
    
}

void Robot::drawBumpers(){
    
    RGB rgbNormale,rgbWarning;
    
    rgbNormale.r=240;
    rgbNormale.g=240;
    rgbNormale.b=240;
    
    rgbWarning.r=255;
    rgbWarning.g=255;
    rgbWarning.b=0;
    
    if(blcBumperLeft.hasCollision){
        blcBumperLeft.rgb=rgbWarning;
    }else{
        blcBumperLeft.rgb=rgbNormale;
    }
    
    if(blcBumperRight.hasCollision){
        blcBumperRight.rgb=rgbWarning;
    }else{
        blcBumperRight.rgb=rgbNormale;
    }
    
    drawBlockRect(blcBumperLeft,robot.x,robot.y);
    
    drawBlockRect(blcBumperRight,robot.x,robot.y);
}
void Robot::scan(){
    blcBumperLeft.hasCollision=false;
    blcBumperRight.hasCollision=false;
    blcSecu.hasCollision=false;
    
    int x,y;
    for( int i = 0 ; i < 180 ; i++ ) {
        
        x=lidar[i][0]*0.1;
        y=-lidar[i][1]*0.1;
        cout <<"X:"<<x<< " Y:"<<y<< endl;
        Block p;
        p.w=10;
        p.h=10;
        
        p.x=robot.x+(x);
        p.y=robot.y+(y);
        
        
        
        
        p.rgb.r=0;
        p.rgb.g=0;
        p.rgb.b=255;
        
        drawBlockRect(p);
        if(blcBumperLeft.hasCollision==false)
            blcBumperLeft.hasCollision=pointInBlock(x,y,blcBumperLeft);
        
        if(blcBumperRight.hasCollision==false)
            blcBumperRight.hasCollision=pointInBlock(x,y,blcBumperRight);
        
        if(blcSecu.hasCollision==false)
            blcSecu.hasCollision=pointInBlock(x,y,blcSecu);
        
    }
    
    
    
}



bool Robot::moveForaward(){
    
    bool isMove=false;
    if(blcSecu.hasCollision){
        speedLeft = 0;
        speedRight = 0;
        isMove=true;
    }
    else if (  blcBumperRight.hasCollision ) { // bumper Right
        speedLeft = 4;
        speedRight = 8*coefV;
        isMove=true;
    } else if(  blcBumperLeft.hasCollision) {// bumper Left
        speedLeft = 8*coefV;
        speedRight = 4;
        isMove = true;
    } else{ // go foraward
        speedLeft = 8;
        speedRight = 8;
        isMove=true;
    }
    
    return isMove;
    
}


