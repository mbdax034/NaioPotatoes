#include "LidarTreatments.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#define MARGE_ERREUR 150

using namespace std; 


double** LidarTreatments::Lidar_Tri_Get_Corrected(uint16_t _lidar[ 271 ], int _min_radius, int _max_radius, int _packet_radius, int _packet_density) {
    double **lidar_ori = new double*[180] ; //X & Y original coordinate for all Lidar's ray
    double **lidar_final = new double*[180] ; //X & Y final coordinate for all Lidar's ray

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
        }
	}

	//Free all tabs
	for( int i = 0 ; i < 180 ; i++ )
        delete[]( lidar_ori[i] ) ;
	delete[]( lidar_ori ) ;

	return lidar_final ;
}

bool less_vectors(const std::vector<double> &a,const std::vector<double> &b) {
    return a.size() < b.size();
}

//Récupère les X les plus présents sur le lidar
vector<vector<double>> LidarTreatments::getRangeeFromLidar(double** pts){
    vector<vector<double>> xList;
    
    for(int i = 0; i < 180 ; i++){
        
        //Si les coordonnées du points sont différents de 0
        if((pts[i][0] != 0 && pts[i][1] != 0)){
            
            //Première valeur ajoutée
            if(xList.size() == 0) {
                vector<double> xs = *new vector<double>();
                xs.push_back(pts[i][0]);
                xList.push_back(xs);
            } else {
                bool valueAdded = false;
                for(int xArrayIt = 0 ; xArrayIt < xList.size() ; xArrayIt ++){
                    
                    //TODO rajouter gestion de largeur de rang
                    //Pour toutes les listes de x on vérifie si on peux les classés
                    if((pts[i][0] <= xList.at(xArrayIt).at(0) + MARGE_ERREUR) &&
                       (pts[i][0] >= xList.at(xArrayIt).at(0) - MARGE_ERREUR))
                    {
                        xList.at(xArrayIt).push_back(pts[i][0]);
                        //cout << "Added To Array : x : "<< pts[i][0] << "  y : " << pts[i][1] << endl;
                        valueAdded = true;
                    }
                    //Si la valeur à été ajoutée on sort de la boucle
                    if(valueAdded){
                        break;
                    }
                }
                //On créer une liste pour le x non classé
                if(!valueAdded){
                    vector<double> xs = *new vector<double>();
                    xs.push_back(pts[i][0]);
                    xList.push_back(xs);
                    //cout << "New Array x : "<< pts[i][0] << "  y : " << pts[i][1] << endl;
                }
            }
        }
    }
    
    //On ne récupère que les deux liste avec le plus de valeur
    vector<vector<double>> xListRetains;
    std::sort (xList.begin(), xList.end(), less_vectors);
    if(xList.size() > 2){
        xListRetains.push_back(xList.at(xList.size()-1));
        xListRetains.push_back(xList.at(xList.size()-2));
        if(xListRetains.size() > 0 &&
           xListRetains.at(0).size() > 2 &&
           xListRetains.at(1).size() > 2) {
            cout << "Rangé detectée" << endl;
        }
    }
    return xListRetains;
}

