
#include "robot.hpp"


void printNull(){

	cout << "rien'" << endl;
	
}

Robot::Robot(int SCREEN_WIDTH,int SCREEN_HEIGHT ){
		blcRobot.w=42;
		blcRobot.h=80;
		blcRobot.x=SCREEN_WIDTH/2 - ( blcRobot.w / 2);
		blcRobot.y=SCREEN_HEIGHT/2 + 80 - blcRobot.h;
		blcRobot.rgb.r=100;
		blcRobot.rgb.g=100;
		blcRobot.rgb.b=100;
		
		blcSecu.w=blcRobot.w /2 ;
		blcSecu.h=20;
		blcSecu.x=blcRobot.x+ blcSecu.w/2 ;
		blcSecu.y=blcRobot.y- blcSecu.h;
		blcSecu.rgb.r=100;
		blcSecu.rgb.g=10;
		blcSecu.rgb.b=10;
		
		int widthBump= 20;
		int heightBump= 40;
		blcBumperRight.w=widthBump;
		blcBumperRight.h=heightBump;
		blcBumperRight.x=blcRobot.x+blcRobot.w;
		blcBumperRight.y=blcRobot.y-blcBumperRight.h;
		blcBumperRight.rgb.r=100;
		blcBumperRight.rgb.g=100;
		blcBumperRight.rgb.b=100;
		
		blcBumperLeft.w=widthBump;
		blcBumperLeft.h=heightBump;
		blcBumperLeft.x=blcRobot.x-blcBumperLeft.w;
		blcBumperLeft.y=blcRobot.y-blcBumperLeft.h;
		blcBumperLeft.rgb.r=100;
		blcBumperLeft.rgb.g=100;
		blcBumperLeft.rgb.b=100;
		
		speedLeft=5;
		speedRight=5;
		context=nullptr;
	
		

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
void Robot::drawBlockRect(Block bp){
	
	SDL_SetRenderDrawColor( context, bp.rgb.r, bp.rgb.g, bp.rgb.b, 255 ); // the rect color (solid red)
		SDL_Rect blocRect;
		blocRect.w = bp.w;
		blocRect.h = bp.h;
		blocRect.x = bp.x;
		blocRect.y = bp.y;
		SDL_RenderFillRect(context, &blocRect);
}


void Robot::drawBlockSecurity(){
	
	
	
	
	if(blcSecu.hasCollision){
		blcSecu.rgb.r=255;
		blcSecu.rgb.g=200;
		blcSecu.rgb.b=200;
	}else{
		blcSecu.rgb.r=150;
		blcSecu.rgb.g=10;
		blcSecu.rgb.b=10;
	}
	drawBlockRect(blcSecu);
	
}

void Robot::drawRobot(){
		cout << "robot "<<context<<"\n";
		
		
		
		drawBlockRect(blcRobot);
		
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
	
	drawBlockRect(blcBumperLeft);
	drawBlockRect(blcBumperRight);
}
void Robot::scan(){
	blcBumperLeft.hasCollision=false;
	blcBumperRight.hasCollision=false;
	blcSecu.hasCollision==false;
	
	int x,y;
	for( int i = 0 ; i < 180 ; i++ ) {
		
		x=lidar[i][0]+blcRobot.x+blcRobot.w/2;
		y=lidar[i][1]+blcRobot.y;
	
		if(blcBumperLeft.hasCollision==false)
			blcBumperLeft.hasCollision=pointInBlock(x,y,blcBumperLeft);
			
		if(blcBumperRight.hasCollision==false)
			blcBumperRight.hasCollision=pointInBlock(x,y,blcBumperRight);
			
		if(blcSecu.hasCollision==false)
			blcSecu.hasCollision=pointInBlock(x,y,blcSecu);
		cout << i << " -  RobotX:" << blcRobot.x+blcRobot.w/2<< " RobotY:" << blcRobot.y<< " X:" << lidar[i][0] << " Y:" << lidar[i][1] << endl;
	}
	
	
}


