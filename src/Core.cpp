#include "Core.hpp"
#include <fstream>
#include <sstream>
#include <cstring>
#include <ctime>

Core::Core( ) :
		stopThreadAsked_{ false },
		threadStarted_{ false },
		graphicThread_{ },
		hostAdress_{ "10.0.1.1" },
		hostPort_{ 5555 },
		socketConnected_{false},
		naioCodec_{ },
		sendPacketList_{ },
		ha_lidar_packet_ptr_{ nullptr },
		ha_odo_packet_ptr_{ nullptr },
		api_post_packet_ptr_{nullptr },
		ha_gps_packet_ptr_{ nullptr },
		controlType_{ ControlType::CONTROL_TYPE_MANUAL },
		last_motor_time_{ 0L },
		last_left_motor_{ 0 },
		last_right_motor_{ 0 },
		mouseX{ 0},
		mouseY{ 0},
		mouseWheel{ 0},
		mouseState{ 0},

        Var_min_radius{DEFAULT_VAR_min_radius},
        Var_max_radius{DEFAULT_VAR_max_radius},
        Var_packet_radius{DEFAULT_VAR_packet_radius},
        Var_packet_density{DEFAULT_VAR_packet_density}
{
}

Core::~Core( ) {
    
}

void Core::init( std::string hostAdress, uint16_t hostPort ) {
    hostAdress_ = hostAdress;
    hostPort_ = hostPort;
    
    etalonnage = new Etalonnage();
    robot = new Robot(SCREEN_WIDTH,SCREEN_HEIGHT);
    
    
    stopThreadAsked_ = false;
    threadStarted_ = false;
    
    socketConnected_ = false;
    
    serverReadthreadStarted_ = false;
    stopServerWriteThreadAsked_ = false;
    
    // ignore unused screen
    (void)screen_;
    
    for ( int i = 0 ; i < SDL_NUM_SCANCODES ; i++ )
        sdlKey_[i] = 0;
    
    std::cout << "Connecting to : " << hostAdress << ":" <<  hostPort << std::endl;
    
#if CONNECT_TO_ROBOT == 1
    struct sockaddr_in server;
    
    //Create socket
    socket_desc_ = socket( AF_INET, SOCK_STREAM, 0 );
    
    if (socket_desc_ == -1)
        std::cout << "Could not create socket" << std::endl;
    
    server.sin_addr.s_addr = inet_addr( hostAdress.c_str() );
    server.sin_family = AF_INET;
    server.sin_port = htons( hostPort );
    
    //Connect to remote server
    if ( connect( socket_desc_, ( struct sockaddr * ) &server, sizeof( server ) ) < 0 ) {
        puts( "connect error" );
    } else {
        puts( "Connected\n" );
        socketConnected_ = true;
    }
#endif
    
    //creates main thread
    //graphic_thread();
    //graphicThread_ = thread( &Core::graphic_thread, this );
    
#if CONNECT_TO_ROBOT == 1
    serverReadThread_ = thread( &Core::server_read_thread, this );
    //this->server_read_thread();
    serverWriteThread_ = thread( &Core::server_write_thread, this );
#endif
    
    graphic_thread();
}

void Core::stop( ) {
    if( threadStarted_ ) {
        stopThreadAsked_ = true;
        graphicThread_.join();
        threadStarted_ = false;
    }
}

void Core::stopServerReadThread( ) {
    if( serverReadthreadStarted_) {
        stopServerReadThreadAsked_ = true;
        serverReadThread_.join();
        serverReadthreadStarted_ = false;
    }
}

void Core::server_read_thread( ) {
    cout << "Starting server read thread !" << endl;
    
    uint8_t receiveBuffer[400000];
    
    while(!stopServerReadThreadAsked_) {
        // any time : read incoming messages.
        int readSize = (int) read( socket_desc_, receiveBuffer, 400000 );
        
        if (readSize > 0) {
            
            bool packetHeaderDetected = false;
            
            bool atLeastOnePacketReceived = naioCodec_.decode( receiveBuffer, static_cast<uint>( readSize ), packetHeaderDetected );
            
            // manage received messages
            if ( atLeastOnePacketReceived == true ) {
                for ( auto &&packetPtr : naioCodec_.currentBasePacketList ) {
                    manageReceivedPacket( packetPtr );
                }
                
                naioCodec_.currentBasePacketList.clear();
            }
        }
    }
    
    serverReadthreadStarted_ = false;
    stopServerReadThreadAsked_= false;
}

void Core::graphic_thread( ) {
    
    
    std::cout << "Starting main thread." << std::endl;
    
    // create graphics
    screen_ = initSDL( "Api Client", SCREEN_WIDTH, SCREEN_HEIGHT );
    robot->context= renderer_;
    // prepare timers for real time operations
    milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
    
    int64_t now = static_cast<int64_t>( ms.count() );
    int64_t duration = MAIN_GRAPHIC_DISPLAY_RATE_MS;
    int64_t nextTick = now + duration;
    
    threadStarted_ = true;
    
    int oldFrontLeftVal = 0;
    int oldRearLeftVal = 0;
    int oldFrontRightVal = 0;
    int oldRearRightVal = 0;
    if(ha_odo_packet_ptr_ != nullptr){
        oldFrontLeftVal = ha_odo_packet_ptr_->fl;
        oldRearLeftVal = ha_odo_packet_ptr_->rl;
        oldFrontRightVal = ha_odo_packet_ptr_->fr;
        oldRearRightVal = ha_odo_packet_ptr_->rr;
    }
    
    int newFrontLeftVal = 2;
    int newRearLeftVal = 2;
    int newFrontRightVal = 2;
    int newRearRightVal = 2;
    
    int tickFrontLeftWheel = 0;
    int tickRearLeftWheel = 0;
    int tickFrontRightWheel = 0;
    int tickRearRightWheel = 0;
    
    while( !stopThreadAsked_ ) {
        ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
        now = static_cast<int64_t>( ms.count() );
        
        // drawing part.
        SDL_SetRenderDrawColor( renderer_, 0, 0, 0, 255 ); // the rect color (solid red)
        SDL_Rect background;
        background.w = SCREEN_WIDTH;
        background.h = SCREEN_HEIGHT;
        background.y = 0;
        background.x = 0;
        
        SDL_RenderFillRect( renderer_, &background );

        robot->drawBlockRect(robot->pointTest);
        
        draw_robot();
        
        drawIMUAxis();
        draw_leaning_angle();
        
        uint16_t lidar_distance_[ 271 ];
        
        ha_lidar_packet_ptr_access_.lock();
        
        if( ha_lidar_packet_ptr_ != nullptr ) {
            for( int i = 0; i < 271 ; i++ ) {
                lidar_distance_[ i ] = ha_lidar_packet_ptr_->distance[ i ];
            }
        } else {
            for( int i = 0; i < 271 ; i++ ) {
                lidar_distance_[ i ] = 3000;
            }
        }
        
        ha_lidar_packet_ptr_access_.unlock();
        
        
        draw_lidar( lidar_distance_, 127 );
        this->lidarTreatments = new LidarTreatments();
        robot->lidar = lidarTreatments->Lidar_Tri_Get_Corrected(lidar_distance_, 200, 2000, 300, 1) ;
        lidarTreatments->getRangeeFromLidar(robot->lidar);
        draw_lidar_corrected( robot->lidar, 255 );
        robot->scan();
        robot->drawBlockSecurity();
        robot->drawBumpers();
        robot->pointTest.x=mouseX-2;
        robot->pointTest.y=mouseY-4;
        Thomas_draw_interface();
        
        char gyro_buff[ 100 ];
        
        ha_gyro_packet_ptr_access_.lock();
        HaGyroPacketPtr ha_gyro_packet_ptr = ha_gyro_packet_ptr_;
        ha_gyro_packet_ptr_access_.unlock();
        
        if( ha_gyro_packet_ptr != nullptr ) {
            snprintf( gyro_buff, sizeof( gyro_buff ), "Gyro  : %d ; %d, %d", ha_gyro_packet_ptr->x, ha_gyro_packet_ptr->y, ha_gyro_packet_ptr->z );
            
            //std::cout << gyro_buff << std::endl;
        } else {
            snprintf( gyro_buff, sizeof( gyro_buff ), "Gyro  : N/A ; N/A, N/A" );
        }
        
        ha_accel_packet_ptr_access_.lock();
        HaAcceleroPacketPtr ha_accel_packet_ptr = ha_accel_packet_ptr_;
        ha_accel_packet_ptr_access_.unlock();
        
        char accel_buff[100];
        if( ha_accel_packet_ptr != nullptr ) {
            snprintf( accel_buff, sizeof( accel_buff ), "Accel : %d ; %d, %d", ha_accel_packet_ptr->x, ha_accel_packet_ptr->y, ha_accel_packet_ptr->z );
            
            //std::cout << accel_buff << std::endl;
        } else {
            snprintf(accel_buff, sizeof(accel_buff), "Accel : N/A ; N/A, N/A" );
        }
        
        ha_odo_packet_ptr_access.lock();
        HaOdoPacketPtr ha_odo_packet_ptr = ha_odo_packet_ptr_;
        ha_odo_packet_ptr_access.unlock();
        
        char odo_buff[100];
        if( ha_odo_packet_ptr != nullptr ) {
            snprintf( odo_buff, sizeof( odo_buff ), "ODO -> RF : %d ; RR : %d ; RL : %d, FL : %d", ha_odo_packet_ptr->fr, ha_odo_packet_ptr->rr, ha_odo_packet_ptr->rl, ha_odo_packet_ptr->fl );
            //std::cout << odo_buff << std::endl;
        } else {
            snprintf( odo_buff, sizeof( odo_buff ), "ODO -> RF : N/A ; RR : N/A ; RL : N/A, FL : N/A" );
        }
        
        if(ha_odo_packet_ptr != nullptr){
            newFrontLeftVal = ha_odo_packet_ptr->fl;
            newRearLeftVal = ha_odo_packet_ptr->rl;
            newFrontRightVal = ha_odo_packet_ptr->fr;
            newRearRightVal = ha_odo_packet_ptr->rr;
        }
        
        if (newFrontLeftVal != oldFrontLeftVal){
            tickFrontLeftWheel += 1;
            oldFrontLeftVal = newFrontLeftVal;
        }
        
        if (newRearLeftVal != oldRearLeftVal){
            tickRearLeftWheel += 1;
            oldRearLeftVal = newRearLeftVal;
        }
        
        if (newFrontRightVal != oldFrontRightVal){
            tickFrontRightWheel += 1;
            oldFrontRightVal = newFrontRightVal;
        }
        
        if (newRearRightVal != oldRearRightVal){
            tickRearRightWheel += 1;
            oldRearRightVal = newRearRightVal;
        }
        
        std::cout << "FL : " << tickFrontLeftWheel <<std::endl;
        std::cout << "RL : " << tickRearLeftWheel <<std::endl;
        std::cout << "FR : " << tickFrontRightWheel <<std::endl;
        std::cout << "RR : " << tickRearRightWheel <<std::endl;
        
        ha_gps_packet_ptr_access_.lock();
        HaGpsPacketPtr ha_gps_packet_ptr = ha_gps_packet_ptr_;
        ha_gps_packet_ptr_access_.unlock();
        
        char gps1_buff[ 100 ];
        char gps2_buff[ 100 ];
        if( ha_gps_packet_ptr_ != nullptr ) {
            snprintf( gps1_buff, sizeof( gps1_buff ), "GPS -> lat : %lf ; lon : %lf ; alt : %lf", ha_gps_packet_ptr->lat, ha_gps_packet_ptr->lon, ha_gps_packet_ptr->alt ) ;
            snprintf( gps2_buff, sizeof( gps2_buff ), "GPS -> nbsat : %d ; fixlvl : %d ; speed : %lf ", ha_gps_packet_ptr->satUsed,ha_gps_packet_ptr->quality, ha_gps_packet_ptr->groundSpeed ) ;
        } else {
            snprintf( gps1_buff, sizeof( gps1_buff ), "GPS -> lat : N/A ; lon : N/A ; alt : N/A" );
            snprintf( gps2_buff, sizeof( gps2_buff ), "GPS -> lnbsat : N/A ; fixlvl : N/A ; speed : N/A" );
        }
        
        draw_text( gyro_buff, 10, 410 );
        draw_text( accel_buff, 10, 420 );
        draw_text( odo_buff, 10, 430 );
        draw_text( gps1_buff, 10, 440 );
        draw_text( gps2_buff, 10, 450 );
        
        ApiPostPacketPtr api_post_packet_ptr = nullptr;
        
        api_post_packet_ptr_access_.lock();
        api_post_packet_ptr = api_post_packet_ptr_;
        api_post_packet_ptr_access_.unlock();
        
        if( api_post_packet_ptr != nullptr ) {
            //std::cout << api_post_packet_ptr->postList.size() <<std::endl;
            for( uint i = 0 ; i < api_post_packet_ptr->postList.size() ; i++ ) {
                if( api_post_packet_ptr->postList[ i ].postType == ApiPostPacket::PostType::RED ) {
                    draw_red_post( static_cast<int>( api_post_packet_ptr->postList[ i ].x * 100.0 ), static_cast<int>( api_post_packet_ptr->postList[ i ].y * 100.0 ) );
                    cout << "x: " << api_post_packet_ptr->postList[i].x << " y: "<< api_post_packet_ptr->postList[i].y << endl;;
                }
            }
        }
        
        
        ///DRAW ONE PIXEL SAMPLE
        static int flying_pixel_x = 0;
        
        if( flying_pixel_x > SCREEN_WIDTH )
            flying_pixel_x = 0;
        
        SDL_SetRenderDrawColor( renderer_, 0, 150, 0, 255 );
        SDL_Rect flying_pixel;
        flying_pixel.w = 3;
        flying_pixel.h = 6;
        flying_pixel.y = SCREEN_HEIGHT-10;
        flying_pixel.x = flying_pixel_x;
        
        flying_pixel_x++;
        
        SDL_RenderFillRect(renderer_, &flying_pixel);
        
        SDL_RenderPresent( renderer_ );
        ///END DRAW ONE PIXEL SAMPLE
        
        // compute wait time
        milliseconds end_ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
        int64_t end_now = static_cast<int64_t>( end_ms.count() );
        int64_t wait_time = nextTick - end_now;
        
        if( wait_time <= 0 ) {
            wait_time = 10;
        }
        
        //std::cout << "display time took " << display_time << " ms so wait_time is " << wait_time << " ms " << std::endl;
        
        std::this_thread::sleep_for( std::chrono::milliseconds( wait_time ) );
        
        readSDLKeyboard();
        manageSDLKeyboard();
    }
    
    threadStarted_ = false;
    stopThreadAsked_ = false;
    
    exitSDL();
    
    std::cout << "Stopping main thread." << std::endl;
}

void Core::draw_text( char buffer[100], int x, int y ) {
    SDL_Surface* surfaceMessageAccel = TTF_RenderText_Solid( ttf_font_, buffer, sdl_color_white_ );
    SDL_Texture* messageAccel = SDL_CreateTextureFromSurface( renderer_, surfaceMessageAccel );
    
    SDL_FreeSurface( surfaceMessageAccel );
    
    SDL_Rect message_rect_accel;
    message_rect_accel.x = x;
    message_rect_accel.y = y;
    
    SDL_QueryTexture( messageAccel, NULL, NULL, &message_rect_accel.w, &message_rect_accel.h );
    SDL_RenderCopy( renderer_, messageAccel, NULL, &message_rect_accel );
    
    SDL_DestroyTexture( messageAccel );
}

void Core::drawIMUAxis(){
#if DRAW_IMU_AXIS == 1
    SDL_SetRenderDrawColor(renderer_, 255, 255, 255, 255);
    SDL_RenderDrawLine(renderer_, 150, 150, 250, 150);
    draw_text("x", 150, 150);
    SDL_RenderDrawLine(renderer_, 250, 150, 250, 50);
    draw_text("y", 255, 50);
#endif
}

void Core::draw_lidar( uint16_t lidar_distance_[ 271 ], int color ) {
    for( int i = 0; i < 271 ; i++ ) {
        double dist = static_cast<double>( lidar_distance_[ i ] ) / 10.0f;
        
        if( dist < 3.0f ) dist = 5000.0f;
        
        if( i > 45  && i < 271-45) {
            double x_cos = dist * cos(  static_cast<double>( ( i - 45 ) * M_PI / 180. ) );
            double y_sin = dist * sin(  static_cast<double>( ( i - 45 ) * M_PI / 180. ) );
            
            double x = SCREEN_WIDTH/2 - x_cos;
            double y = SCREEN_HEIGHT/2 - y_sin;
            
            SDL_SetRenderDrawColor( renderer_, color, color, color, 255 );
            SDL_Rect lidar_pixel;
            
            lidar_pixel.w = 1;
            lidar_pixel.h = 1;
            lidar_pixel.x = static_cast<int>( x );
            lidar_pixel.y = static_cast<int>( y );
            
            SDL_RenderFillRect( renderer_, &lidar_pixel );
        }
    }
}

void Core::draw_lidar_corrected( double **lidar_distance_, int color ) {
    for( int i = 0; i < 180 ; i++ ) {
        double x = SCREEN_WIDTH/2 + lidar_distance_[i][0]/10;
        double y = SCREEN_HEIGHT/2 - lidar_distance_[i][1]/10;
        
        SDL_SetRenderDrawColor( renderer_, color, color, color, 255 );
        SDL_Rect lidar_pixel;
        
        lidar_pixel.w = 1;
        lidar_pixel.h = 1;
        lidar_pixel.x = static_cast<int>( x );
        lidar_pixel.y = static_cast<int>( y );
        
        SDL_RenderFillRect( renderer_, &lidar_pixel );
        if(lidar_distance_[i][0] != 0 && lidar_distance_[i][1] != 0){
            /*stringstream ptStringStream ;
            ptStringStream << "x : " << to_string(lidar_distance_[i][0]);
            ptStringStream << " y : " << to_string(lidar_distance_[i][1]);
            char* pt = strdup(ptStringStream.str().c_str());
            draw_text(pt, x, y);*/
        }
    }
}

void Core::draw_red_post( int x, int y ) {
    SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 255);
    SDL_Rect rp;
    rp.w = 2;
    rp.h = 2;
    rp.y = SCREEN_HEIGHT/2 - x - 1;
    rp.x = SCREEN_WIDTH/2 - y - 1;
    
    SDL_RenderFillRect( renderer_, &rp );
}

void Core::draw_robot() {
    SDL_SetRenderDrawColor( renderer_, 200, 200, 200, 255 );
    SDL_Rect main;
    main.w = 42;
    main.h = 80;
    main.y = SCREEN_HEIGHT/2 + 80 - main.h;
    main.x = SCREEN_WIDTH/2 - ( main.w / 2);
    
    
    
    SDL_RenderFillRect( renderer_, &main );
    
    SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
    SDL_Rect flw;
    flw.w = 8;
    flw.h = 20;
    flw.y = SCREEN_HEIGHT/2 + 80 - 75;
    flw.x = SCREEN_WIDTH/2 - 21;
    
    SDL_RenderFillRect( renderer_, &flw );
    
    SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
    SDL_Rect frw;
    frw.w = 8;
    frw.h = 20;
    frw.y = SCREEN_HEIGHT/2 + 80 - 75;
    frw.x = SCREEN_WIDTH/2 + 21 - 8;
    
    SDL_RenderFillRect( renderer_, &frw );
    
    SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
    SDL_Rect rlw;
    rlw.w = 8;
    rlw.h = 20;
    rlw.y = SCREEN_HEIGHT/2 + 80 - 5 - 20;
    rlw.x = SCREEN_WIDTH/2 - 21;
    
    SDL_RenderFillRect( renderer_, &rlw );
    
    SDL_SetRenderDrawColor( renderer_, 100, 100, 100, 255 );
    SDL_Rect rrw;
    rrw.w = 8;
    rrw.h = 20;
    rrw.y = SCREEN_HEIGHT/2 + 80 - 5 -20;
    rrw.x = SCREEN_WIDTH/2 + 21 - 8;
    
    SDL_RenderFillRect( renderer_, &rrw );
    
    SDL_SetRenderDrawColor( renderer_, 120, 120, 120, 255 );
    SDL_Rect lidar;
    lidar.w = 8;
    lidar.h = 8;
    lidar.y = SCREEN_HEIGHT/2 + 80 - 80 - 8;
    lidar.x = SCREEN_WIDTH/2 - 4;
    
    SDL_RenderFillRect( renderer_, &lidar );
}

void Core::draw_leaning_angle(){
    if(this->ha_accel_packet_ptr_ != nullptr){
        //Calcul de l'angle d'axe X
        double angleX = this->leaning.getAbsoluteXAxisLeaning(ha_accel_packet_ptr_->x,
                                                              ha_accel_packet_ptr_->y,
                                                              ha_accel_packet_ptr_->z,
                                                              etalonnage->getAccelX());
        stringstream strAngleX ;
        strAngleX << "Inclinaison axe X :" << to_string(angleX);
        char* inclinaisonText = strdup(strAngleX.str().c_str());
        draw_text(inclinaisonText, 50, 50);
        
        //Calcul de l'angle d'axe Y
        double angleY = this->leaning.getAbsoluteYAxisLeaning(ha_accel_packet_ptr_->x,
                                                              ha_accel_packet_ptr_->y,
                                                              ha_accel_packet_ptr_->z,
                                                              etalonnage->getAccelY());
        stringstream strAngleY ;
        strAngleY << "Inclinaison axe Y :" << to_string(angleY);
        char* inclinaisonTextY = strdup(strAngleY.str().c_str());
        draw_text(inclinaisonTextY, 50, 75);
        
        
        stringstream strAngleRobot ;
        strAngleRobot << "Angle Robot Z :" << to_string(angle);
        char* inclinaisonAngleRobotZ = strdup(strAngleRobot.str().c_str());
        draw_text(inclinaisonAngleRobotZ, 50, 100);
        
    }
}

SDL_Window* Core::initSDL( const char* name, int szX, int szY ) {
    std::cout << "Init SDL";
    
    SDL_Window *screen;
    std::cout << ".";
    
    SDL_Init( SDL_INIT_EVERYTHING );
    std::cout << ".";
    
    screen = SDL_CreateWindow( name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, szX, szY, SDL_WINDOW_SHOWN );
    std::cout << ".";
    
    renderer_ =  SDL_CreateRenderer( screen, 0, SDL_RENDERER_ACCELERATED );
    std::cout << ".";
    
    TTF_Init();
    std::cout << ".";
    
    // Set render color to black ( background will be rendered in this color )
    SDL_SetRenderDrawColor( renderer_, 0, 0, 0, 255 );
    std::cout << ".";
    
    SDL_RenderClear( renderer_ );
    std::cout << ".";
    
    sdl_color_red_ = { 255, 0, 0, 0 };
    sdl_color_white_ = { 255, 255, 255, 0 };
    ttf_font_ = TTF_OpenFont("mono.ttf", 12);
    
    if (ttf_font_ == nullptr)
        std::cerr << "Failed to load SDL Font! Error: " << TTF_GetError() << '\n';
    
    std::cout << "DONE" << std::endl;
    
    return screen;
}

void Core::exitSDL() {
    SDL_Quit();
}

void Core::readSDLKeyboard(){
    SDL_Event event;
    
    while ( SDL_PollEvent(&event) ) {
        switch( event.type ) {
                // Cas d'une touche enfoncée
            case SDL_KEYDOWN:
                sdlKey_[ event.key.keysym.scancode ] = 1;
                break;
                // Cas d'une touche relâchée
            case SDL_KEYUP:
                sdlKey_[ event.key.keysym.scancode ] = 0;
                break;
            case SDL_QUIT:
                stopThreadAsked_ = true;
				break;
		}

		SDL_GetMouseState(&mouseX,&mouseY);

		if( event.button.button == SDL_BUTTON_LEFT ) {
            if( event.type == SDL_MOUSEBUTTONDOWN ) {
                mouseState = 1 ;
            } else if( event.type == SDL_MOUSEBUTTONUP ) {
                mouseState = -1 ;
            }
        }


		if( event.type == SDL_MOUSEWHEEL ) {
            if( sdlKey_[ SDL_SCANCODE_LSHIFT ] == 1 ) {
                mouseWheel = event.wheel.y*100 ;
            } else if( sdlKey_[ SDL_SCANCODE_LCTRL ] == 1 ) {
                mouseWheel = event.wheel.y*10 ;
            } else {
                mouseWheel = event.wheel.y ;
            }
		}
	}
}

bool Core::manageSDLKeyboard() {
    bool keyPressed = false;
    
    int8_t left = 0;
    int8_t right = 0;
    
    if( sdlKey_[ SDL_SCANCODE_ESCAPE ] == 1) {
        stopThreadAsked_ = true;
        
        return true;
    }
    
    if( sdlKey_[ SDL_SCANCODE_UP ] == 1 and sdlKey_[ SDL_SCANCODE_LEFT ] == 1 ) {
        left = 32;
        right = 63;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_UP ] == 1 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 ) {
        left = 63;
        right = 32;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 and sdlKey_[ SDL_SCANCODE_LEFT ] == 1 ) {
        left = -32;
        right = -63;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 and sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 ) {
        left = -63;
        right = -32;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_UP ] == 1 ) {
        left = 63;
        right = 63;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_DOWN ] == 1 ) {
        left = -63;
        right = -63;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_LEFT ] == 1 ) {
        left = -63;
        right = 63;
        keyPressed = true;
    } else if ( sdlKey_[ SDL_SCANCODE_RIGHT ] == 1 ) {
        left = 63;
        right = -63;
        keyPressed = true;
    }
    
    last_motor_access_.lock();
    last_left_motor_ = static_cast<int8_t >( left * 2 );
    last_right_motor_ = static_cast<int8_t >( right * 2 );
    last_motor_access_.unlock();
    
    return keyPressed;
}

void Core::manageReceivedPacket( BaseNaio01PacketPtr packetPtr ) {
    //std::cout << "Packet received id : " << static_cast<int>( packetPtr->getPacketId() ) << std::endl;
    
    if( std::dynamic_pointer_cast<HaLidarPacket>( packetPtr )  ) {
        HaLidarPacketPtr haLidarPacketPtr = std::dynamic_pointer_cast<HaLidarPacket>( packetPtr );
        
        ha_lidar_packet_ptr_access_.lock();
        ha_lidar_packet_ptr_ = haLidarPacketPtr;
        ha_lidar_packet_ptr_access_.unlock();
    } else if ( std::dynamic_pointer_cast<HaGyroPacket>( packetPtr )  ) {
        HaGyroPacketPtr haGyroPacketPtr = std::dynamic_pointer_cast<HaGyroPacket>( packetPtr );
        calculAngle(haGyroPacketPtr, ha_gyro_packet_ptr_);
        ha_gyro_packet_ptr_access_.lock();
        ha_gyro_packet_ptr_ = haGyroPacketPtr;
        ha_gyro_packet_ptr_access_.unlock();
        etalonnage->etalonnageGyro(ha_gyro_packet_ptr_->x, ha_gyro_packet_ptr_->y, ha_gyro_packet_ptr_->z);
        
    } else if ( std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr )  ) {
        HaAcceleroPacketPtr haAcceleroPacketPtr = std::dynamic_pointer_cast<HaAcceleroPacket>( packetPtr );
        
        ha_accel_packet_ptr_access_.lock();
        ha_accel_packet_ptr_ = haAcceleroPacketPtr;
        ha_accel_packet_ptr_access_.unlock();
        etalonnage->etalonnageAccel(ha_accel_packet_ptr_->x, ha_accel_packet_ptr_->y, ha_accel_packet_ptr_->z);
    } else if ( std::dynamic_pointer_cast<HaOdoPacket>( packetPtr )  ) {
        HaOdoPacketPtr haOdoPacketPtr = std::dynamic_pointer_cast<HaOdoPacket>( packetPtr );
        
        ha_odo_packet_ptr_access.lock();
        ha_odo_packet_ptr_ = haOdoPacketPtr;
        ha_odo_packet_ptr_access.unlock();
    } else if ( std::dynamic_pointer_cast<ApiPostPacket>( packetPtr )  ) {
        ApiPostPacketPtr apiPostPacketPtr = std::dynamic_pointer_cast<ApiPostPacket>( packetPtr );
        
        api_post_packet_ptr_access_.lock();
        api_post_packet_ptr_ = apiPostPacketPtr;
        api_post_packet_ptr_access_.unlock();
    } else if ( std::dynamic_pointer_cast<HaGpsPacket>( packetPtr )  ) {
        HaGpsPacketPtr haGpsPacketPtr = std::dynamic_pointer_cast<HaGpsPacket>( packetPtr );
        ha_gps_packet_ptr_access_.lock();
        ha_gps_packet_ptr_ = haGpsPacketPtr;
        ha_gps_packet_ptr_access_.unlock();
    }
}

void Core::joinMainThread() {
    //graphicThread_.join();
}

void Core::joinServerReadThread()
{
    serverReadThread_.join();
}

void Core::server_write_thread( ) {
    stopServerWriteThreadAsked_ = false;
    serverWriteThreadStarted_ = true;
    
    for( int i = 0 ; i < 100 ; i++ ) {
        ApiMotorsPacketPtr first_packet = std::make_shared<ApiMotorsPacket>( 0, 0 );
        cl_copy::BufferUPtr first_buffer = first_packet->encode();
        write( socket_desc_, first_buffer->data(), first_buffer->size() );
    }
    
    while( not stopServerWriteThreadAsked_ ) {
        
        last_motor_access_.lock();
        
        HaMotorsPacketPtr haMotorsPacketPtr = std::make_shared<HaMotorsPacket>( last_left_motor_, last_right_motor_ );
        
        last_motor_access_.unlock();
        
        sendPacketListAccess_.lock();
        
        sendPacketList_.push_back( haMotorsPacketPtr );
        
        for( auto&& packet : sendPacketList_ ) {
            cl_copy::BufferUPtr buffer = packet->encode();
            
            int sentSize = (int)write( socket_desc_, buffer->data(), buffer->size() );
            
            (void)sentSize;
        }
        
        sendPacketList_.clear();
        
        sendPacketListAccess_.unlock();
        
        std::this_thread::sleep_for( std::chrono::milliseconds( SERVER_SEND_COMMAND_RATE_MS ) );
    }
    
    stopServerWriteThreadAsked_ = false;
    serverWriteThreadStarted_ = false;
}

void Core::calculAngle(HaGyroPacketPtr newPacket, HaGyroPacketPtr oldPacket){
    milliseconds ms = duration_cast< milliseconds >( system_clock::now().time_since_epoch() );
    int64_t now = static_cast<int64_t>( ms.count() );
    //Si c'est le premier packet reçu
    if(oldPacket == nullptr){
        angle = 0;
    } else {
        float value = now-msReference;
        double deltaZ = newPacket->z - oldPacket->z;
        double zRad = deltaZ*(value/1000);
        //double zRad = ((0.5*(newPacket->z + oldPacket->z))*(value/1000));
        
        if (newPacket->z < this->etalonnage->getGyroZMin() && deltaZ < 0){ //tourne dans un sens
            this->angle = this->angle + zRad;
        }
        if (newPacket->z > this->etalonnage->getGyroZMax() && deltaZ > 0){ //tourne dans un sens
            this->angle = this->angle + zRad;
        }
    }
    msReference = now;
}




void Core::Thomas_draw_rect(int x, int y, int w, int h, int r, int g, int b, int a) {
    SDL_SetRenderDrawColor(renderer_, r, g, b, a); // the rect color (solid red)
    SDL_Rect rectangle;
    rectangle.x = x;
    rectangle.y = y;
    rectangle.w = w;
    rectangle.h = h;
    SDL_RenderFillRect(renderer_, &rectangle);
}

int Core::Thomas_check_wheel(int x, int y, int w, int h) {
    if( mouseX >= x && mouseX <= x+w )
        if( mouseY >= y && mouseY <= y+h )
            return mouseWheel ;
    return 0 ;
}

int Core::Thomas_check_clicked(int x, int y, int w, int h) {
    if( mouseX >= x && mouseX <= x+w ) {
        if( mouseY >= y && mouseY <= y+h ) {
            if( mouseState ) {
                return mouseState ;
            }
            
            return -10 ;
        }
    }
    return 0 ;
}

int Core::Thomas_draw_text_centered(char buffer[100], int x_centered, int y_centered) {
    SDL_Surface* surfaceMessageAccel = TTF_RenderText_Solid( Var_ttf_font_, buffer, { 0, 0, 0, 0 } );
    SDL_Texture* messageAccel = SDL_CreateTextureFromSurface( renderer_, surfaceMessageAccel );
    
    SDL_FreeSurface( surfaceMessageAccel );
    
    SDL_Rect message_rect_accel;
    SDL_QueryTexture( messageAccel, NULL, NULL, &message_rect_accel.w, &message_rect_accel.h );
    message_rect_accel.x = x_centered - message_rect_accel.w/2 ;
    message_rect_accel.y = y_centered - message_rect_accel.h/2 ;
    
    SDL_RenderCopy( renderer_, messageAccel, NULL, &message_rect_accel );
    
    SDL_DestroyTexture( messageAccel );
    
    return message_rect_accel.w ;
}

int Core::Thomas_draw_text(char buffer[100], int x, int y) {
    SDL_Surface* surfaceMessageAccel = TTF_RenderText_Solid( Var_ttf_font_, buffer, { 0, 0, 0, 0 } );
    SDL_Texture* messageAccel = SDL_CreateTextureFromSurface( renderer_, surfaceMessageAccel );
    
    SDL_FreeSurface( surfaceMessageAccel );
    
    SDL_Rect message_rect_accel;
    SDL_QueryTexture( messageAccel, NULL, NULL, &message_rect_accel.w, &message_rect_accel.h );
    message_rect_accel.x = x ;
    message_rect_accel.y = y ;
    
    SDL_RenderCopy( renderer_, messageAccel, NULL, &message_rect_accel );
    
    SDL_DestroyTexture( messageAccel );
    
    return message_rect_accel.w ;
}

int Core::Thomas_button(int x, int y, int w, int h, int r, int g, int b) {
    double alpha = Thomas_check_clicked(x, y, w, h) ;
    if( alpha == -10 )
        alpha = 0.75 ;
    else if( alpha > 0 )
        alpha = 1 ;
    else
        alpha = 0.5 ;
    
    Thomas_draw_rect(x, y, w, h, r*alpha, g*alpha, b*alpha) ;
    
    if( alpha == 1 ) return 1 ;
    return 0 ;
}

int Core::Thomas_box(int x, int y, int &Var, int var_default) {
    int is_valid = 0 ;
    char text_buff[100];
    
    Var += Thomas_check_wheel(x, y, 36+10, 18+10) ;
    if( Thomas_button(x, y, 36+10, 18+10) ) {
        Var = var_default ;
        is_valid = 1 ;
    }
    
    snprintf( text_buff, sizeof( text_buff ), "%d", Var);
    Thomas_draw_text_centered(text_buff, x+(36+10)/2, y+(18+10)/2);
    
    return is_valid ;
}

int Core::Thomas_box(int x, int y, int &Var, int var_default, char* title) {
    int is_valid = 0 ;
    int title_width = Thomas_draw_text(title, x+5, y+5);
    if( title_width < 36 ) title_width = 36 ;
    
    Var += Thomas_check_wheel(x, y, title_width+10, 18*2+10) ;
    if( Thomas_button(x, y, title_width+10, 18*2+10) ) {
        Var = var_default ;
        is_valid = 1 ;
    }
    
    Thomas_draw_rect(x+2, y+18+8, title_width+6, 18, 255, 255, 255) ;
    
    Thomas_draw_text(title, x+5, y+5);
    
    char text_buff[100];
    snprintf( text_buff, sizeof( text_buff ), "%d", Var);
    Thomas_draw_text_centered(text_buff, x+(title_width+10)/2, y+18+8+(18)/2);
    
    return is_valid ;
}

int Core::Thomas_box(int x, int y, char* title) {
    int is_valid = 0 ;
    int title_width = Thomas_draw_text(title, x+5, y+5);
    if( title_width < 36 ) title_width = 36 ;
    
    if( Thomas_button(x, y, title_width+10, 18+10) ) {
        is_valid = 1 ;
    }
    
    Thomas_draw_text(title, x+5, y+5);
    
    return is_valid ;
}

void Core::Thomas_draw_interface(){
    Thomas_box(10, 10, Var_min_radius, DEFAULT_VAR_min_radius, "Petit Cercle") ;
    Thomas_box(10, 70, Var_max_radius, DEFAULT_VAR_max_radius, "Grand cercle") ;
    if( Thomas_box(10, 130, "reset") ) {
        Var_min_radius = DEFAULT_VAR_min_radius ;
        Var_max_radius = DEFAULT_VAR_max_radius ;
        Var_packet_radius = DEFAULT_VAR_packet_radius ;
        Var_packet_density = DEFAULT_VAR_packet_density ;
    }
    if( Thomas_box(10, SCREEN_HEIGHT-18-20, "START") ) {
        Var_min_radius = DEFAULT_VAR_min_radius ;
        Var_max_radius = DEFAULT_VAR_max_radius ;
        Var_packet_radius = DEFAULT_VAR_packet_radius ;
        Var_packet_density = DEFAULT_VAR_packet_density ;
    }
}

