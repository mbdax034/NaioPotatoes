#ifndef CORE_HPP
#define CORE_HPP

///BASICS LIB
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/resource.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <zlib.h>
#include <iostream>
#include <cstdlib>
#include <thread>

///SDL 2.0 LIB
#include <SDL2/SDL_system.h>
#include <SDL2/SDL.h>

#ifdef __APPLE__
#include <SDL2_ttf/SDL_ttf.h>
#else
#include <SDL2/SDL_ttf.h>
#endif

///APICODEC LIB
#include "Naio01Codec.hpp"
#include "HaLidarPacket.hpp"
#include "HaOdoPacket.hpp"
#include "HaMotorsPacket.hpp"
#include "HaGyroPacket.hpp"
#include "HaAcceleroPacket.hpp"
#include "HaGpsPacket.hpp"
#include "ApiCommandPacket.hpp"
#include "ApiMotorsPacket.hpp"
#include "ApiStatusPacket.hpp"
#include "ApiLidarPacket.hpp"
#include "ApiPostPacket.hpp"
#include "ApiGpsPacket.hpp"
#include "ApiWatchdogPacket.hpp"

#include "Leaning.hpp"
#include "Etalonnage.hpp"
#include "robot.hpp"


#define PORT_ROBOT_MOTOR 5555
#define DEFAULT_HOST_ADDRESS "10.0.1.1"

#define CONNECT_TO_ROBOT    0
#define DRAW_IMU_AXIS       0
#define SCREEN_WIDTH        800
#define SCREEN_HEIGHT       600
#define COEFF_SDL_RANGEE_LINE 1
#define DEFAULT_VAR_min_radius      200
#define DEFAULT_VAR_max_radius      2000
#define DEFAULT_VAR_packet_radius   300
#define DEFAULT_VAR_packet_density  2
#define DEFAULT_VAR_maxPointBumper 	4
#define DEFAULT_VAR_maxPointSecu 	20
#define DEFAULT_VAR_moveAuto		0



using namespace std;
using namespace std::chrono;


class Core
{
public:
    enum ControlType : uint8_t
    {
        CONTROL_TYPE_MANUAL = 0x01,
    };
    
    const int64_t MAIN_GRAPHIC_DISPLAY_RATE_MS = 100;
    const int64_t SERVER_SEND_COMMAND_RATE_MS = 25;
public:
    
    Core( );
    ~Core( );
    
    // launch core
    void init( std::string hostAdress_, uint16_t hostPort_ );
    
    // thread management
    void stop( );
    void stopServerReadThread( );
    void joinMainThread();
    void joinServerReadThread();
    
private:
    // thread function
    void graphic_thread( );
    
    // main server 5555 thread function
    void server_read_thread( );
    void server_write_thread( );
    
    // communications
    void manageReceivedPacket( BaseNaio01PacketPtr packetPtr );
    
    // graph
    SDL_Window *initSDL(const char* name, int szX, int szY );
    
    void exitSDL();
    
    void readSDLKeyboard();
    bool manageSDLKeyboard();
    
    void draw_robot();
    void draw_lidar( uint16_t lidar_distance_[271], int color );
    void draw_lidar_corrected( double **lidar_distance_, int color ) ;
    void draw_text( char gyro_buff[100], int x, int y );
    void draw_red_post( int x, int y );
    void draw_leaning_angle();
    int64_t getTimeMsReference(){return msReference;};
    void calculAngle(HaGyroPacketPtr newPacket, HaGyroPacketPtr oldPacket);
    
    int Thomas_draw_text_centered(char gyro_buff[100], int x_centered, int y_centered);
    int Thomas_draw_text(char gyro_buff[100], int x, int y);
    void Thomas_draw_rect(int x, int y, int w, int h, int r = 255, int g = 255, int b = 255, int a = 255) ;
    int Thomas_check_clicked(int x, int y, int w, int h);
    int Thomas_check_wheel(int x, int y, int w, int h);
    int Thomas_button(int x, int y, int w, int h, int r = 255, int g = 255, int b = 255) ;
    int Thomas_box(int x, int y, int &Var, int var_default) ;
    int Thomas_box(int x, int y, int &Var, int var_default, char* title) ;
    int Thomas_box(int x, int y, char* title) ;
    void draw_interface();
    void move();
    double getXfromLidarToSDL(double x);
    double getYfromLidarToSDL(double y);
    
private:
    LidarTreatments* lidarTreatments;
    Leaning leaning;
    int64_t msReference = 0;
    double angle = 0;
    
    // thread part
    void drawIMUAxis();
    bool stopThreadAsked_;
    bool threadStarted_;
    std::thread graphicThread_;
    
    bool etalonnageLaunched = false;
    bool stopServerReadThreadAsked_;
    bool serverReadthreadStarted_;
    std::thread serverReadThread_;
    
    bool stopServerWriteThreadAsked_;
    bool serverWriteThreadStarted_;
    std::thread serverWriteThread_;
    
    // socket part
    std::string hostAdress_;
    uint16_t hostPort_;
    int socket_desc_;
    bool socketConnected_;
    
    // sdl part
    int sdlKey_[SDL_NUM_SCANCODES];
    
    // codec part
    Naio01Codec naioCodec_;
    std::mutex sendPacketListAccess_;
    std::vector< BaseNaio01PacketPtr > sendPacketList_;
    
    std::mutex ha_lidar_packet_ptr_access_;
    HaLidarPacketPtr ha_lidar_packet_ptr_;
    
    std::mutex ha_gyro_packet_ptr_access_;
    HaGyroPacketPtr ha_gyro_packet_ptr_;
    HaGyroPacketPtr old_ha_gyro_packet_ptr_;
    
    std::mutex ha_accel_packet_ptr_access_;
    HaAcceleroPacketPtr ha_accel_packet_ptr_;
    
private:
    std::mutex ha_odo_packet_ptr_access;
    HaOdoPacketPtr ha_odo_packet_ptr_;
    
    std::mutex api_post_packet_ptr_access_;
    ApiPostPacketPtr api_post_packet_ptr_;
    
    std::mutex ha_gps_packet_ptr_access_;
    HaGpsPacketPtr ha_gps_packet_ptr_;
    
    // ia part
    ControlType controlType_;
    
    Etalonnage* etalonnage;
    
    SDL_Window* screen_;
    SDL_Renderer* renderer_;
    
    SDL_Color sdl_color_red_;
    SDL_Color sdl_color_white_;
    TTF_Font* ttf_font_;
    
    uint64_t last_motor_time_;
    std::mutex last_motor_access_;
    int8_t last_left_motor_;
    int8_t last_right_motor_;
    
    uint64_t last_image_received_time_;
    
    Robot * robot;
    int mouseX;
    int mouseY;
    
    
    
    
    int mouseWheel;
    
    int mouseState; //0 - normal, 1 - pressed, 2 - still pressed, -1 - released
    
    TTF_Font* var_ttf_font_;
    int var_min_radius ;
    int var_max_radius ;
    int var_packet_radius ;
    int var_packet_density ;
    int var_maxPointBumper;
    int var_maxPointSecu;
    int var_moveAuto;
};

#endif
