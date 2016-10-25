#include "Lidar_tri.hpp"
#include <iostream>
#include <cmath>

using namespace std; 


double **Lidar_Tri_Get_Corrected(uint16_t _lidar[ 271 ], int _min_radius, int _max_radius, int _packet_radius, int _packet_density) {
    double **lidar_ori = new double*[180] ; //X & Y original coordinate for all Lidar's ray
    double **lidar_final = new double*[271] ; //X & Y final coordinate for all Lidar's ray

    //For all point, retrieve X & Y coordinates
	for( int i = 0 ; i < 180 ; i++ ) {
        lidar_final[i] = new double[2] ;
        lidar_ori[i] = new double[2] ;
        lidar_final[i][0] = 0 ;
        lidar_final[i][1] = 0 ;

		double dist = static_cast<double>( _lidar[i+45] );

		if( dist < 30.0f ) dist = 5000.0f;

        if( dist >= _min_radius && dist <= _max_radius ) { //Chech if point is in the accepted area
            lidar_ori[i][0] = -dist * cos(  static_cast<double>( i * M_PI / 180. ) );
            lidar_ori[i][1] = dist * sin(  static_cast<double>( i * M_PI / 180. ) );
        } else {
			lidar_ori[i][0] = 0 ;
			lidar_ori[i][1] = 0 ;
		}
	}

	for( int i = 0 ; i < 180 ; i++ ) {
        int density = 0 ;
        for( int j = 0 ; j < 180 ; j++ ) {
            double diff_x = lidar_ori[i][0] - lidar_ori[j][0] ;
            double diff_y = lidar_ori[i][1] - lidar_ori[j][1] ;

            if( sqrt(diff_x*diff_x + diff_y*diff_y) < _packet_radius ) {
                density++;
            }
        }

        if( density >= _packet_density ) {
            lidar_final[i][0] = lidar_ori[i][0] ;
            lidar_final[i][1] = lidar_ori[i][1] ;
            //if( lidar_final[i][0] < 1 ) 
				//lidar_final[i][0] = 0 ;
        }
	}

	//Free all tabs
	for( int i = 0 ; i < 180 ; i++ )
        delete( lidar_ori[i] ) ;
	delete[]( lidar_ori ) ;

	return lidar_final ;
}
