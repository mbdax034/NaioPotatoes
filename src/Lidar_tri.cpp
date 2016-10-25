#include "Lidar_tri.hpp"
#include <iostream>
#include <cmath>
#include <vector>

using namespace std; 


double** LidarTreatments::Lidar_Tri_Get_Corrected(uint16_t _lidar[ 271 ], int _min_radius, int _max_radius, int _packet_radius, int _packet_density) {
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

//Récupère les X les plus présents sur le lidar
std::vector<std::vector<double>> LidarTreatments::getRangeeFromLidar(double** pts){
    std::vector<std::vector<double>> xList;
    for(int i = 0; i < 180 ; i++){
        //Première valeur ajoutée
        if(xList.size() == 0) {
            vector<double> xs = *new vector<double>();
            xs.push_back(pts[i][0]);
            xList.push_back(xs);
        } else {
            bool valueHasBeenAdded = false;
            for(std::vector<double> xs : xList){
                if((pts[i][0] < xs.at(0)+2) && (pts[i][0] > xs.at(0)-2)){
                    xs.push_back(pts[i][0]);
                }
            }
            if(!valueHasBeenAdded) {
                vector<double> xs = *new vector<double>();
                xs.push_back(pts[i][0]);
                xList.push_back(xs);
            }
        }
    }
    
    //On ne récupère que les deux liste avec le plus de valeur
    std::vector<std::vector<double>> xListRetains;
    std::sort (xList.end(), xList.begin(), xList.size());
    xListRetains.push_back(xList.at(0));
    xListRetains.push_back(xList.at(1));
    return xListRetains;
}
