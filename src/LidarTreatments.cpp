#include "LidarTreatments.hpp"

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
            //cout << density << ">" << _packet_density << endl ;
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
    
    //for(int angle = -20 ; angle <= 20 ; angle+= 5) {
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
                        //Pour toutes les listes de x on vérifie si on peux les classés
                        if( ( pts[i][0] <= xList.at(xArrayIt).at(0) + margeErreur ) &&
                            ( pts[i][0] >= xList.at(xArrayIt).at(0) - margeErreur ) )
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
    //}
    
    //On ne récupère que les deux liste avec le plus de valeur
    vector<vector<double>> xListRetains;
    if(xList.size() > 2){
        for(int xIt = 0 ; xIt < xList.size() ; xIt++){
            for(int xIt2 = 0 ; xIt2 < xList.size() ; xIt2++){
                double moyenne = getMoyenne(xList.at(xIt));
                double moyenne2 = getMoyenne(xList.at(xIt2));
                double largeur = abs(moyenne-moyenne2);
                //cout <<"Largeur : " << largeur << endl;
                
                if( largeur <= largeurRangee + margeErreurLargeurRangee &&
                    largeur >= largeurRangee - margeErreurLargeurRangee &&
                    xList.at(xIt).size() > 2 &&
                    xList.at(xIt2).size() > 2 ){
                    
                    if( xListRetains.size() == 0 ){
                        xListRetains.push_back(xList.at(xIt2));
                        xListRetains.push_back(xList.at(xIt));
                    }
                    else {
                        double oldLargeur = abs(getMoyenne(xListRetains.at(1))-getMoyenne(xListRetains.at(0)));
                        if( xList.at(xIt2).size() > xListRetains.at(0).size() &&
                            xList.at(xIt).size() > xListRetains.at(1).size() &&
                            (oldLargeur - largeurRangee) > (largeur - largeurRangee) ){
                            
                            //A partir d'un certain nombre de point trier
                            if(oldX1.size() % 10 == 0){
                                trierXliste(oldX1);
                            }
                            
                            if(oldX2.size() % 10 == 0){
                                trierXliste(oldX2);
                            }
                            
                            //On prend en compte les anciennes positions
                            if(oldX2.size() >50 && oldX1.size() > 50){
                                double moyenneOldX1 = getMoyenne(oldX1);
                                double moyenneOldX2 = getMoyenne(oldX2);
                                if( moyenne > ( moyenneOldX1-margeErreur ) &&
                                    moyenne < ( moyenneOldX1+margeErreur ) &&
                                    moyenne2 > ( moyenneOldX2-margeErreur ) &&
                                    moyenne2 < ( moyenneOldX2+margeErreur ) ){
                                    
                                    xListRetains.clear();
                                    xListRetains.push_back(xList.at(xIt2));
                                    oldX1.push_back(moyenne);
                                    xListRetains.push_back(xList.at(xIt));
                                    oldX2.push_back(moyenne2);
                                    this->oldX1.erase(this->oldX1.begin());
                                    this->oldX2.erase(this->oldX2.begin());
                                }
                            }
                            //Pas assez de point pour prendre en compte les anciennes position
                            else {
                                xListRetains.clear();
                                xListRetains.push_back(xList.at(xIt2));
                                oldX1.push_back(moyenne);
                                xListRetains.push_back(xList.at(xIt));
                                oldX2.push_back(moyenne2);
                            }
                        }
                    }
                }
            }
        }
        if(xListRetains.size() > 2){
            cout << "Rangée detectée" << endl;
        }
    }
    //cout << "Taille :" << oldX1.size() << " : " << oldX2.size() << endl;
    return xListRetains;
}

void LidarTreatments::trierXliste(vector<double> oldx){    
    double sum = 0;
    vector<double> oldXTmp;
    int iteration = 1;
    
    for(auto xIt = oldx.begin(); xIt != oldx.end() ; xIt++){
        int index = xIt-oldx.begin();
        sum += oldx.at(index);
        double moyenne = sum/iteration;
        if(oldx.at(index) >  moyenne-margeErreur &&
            oldx.at(index) <  moyenne+margeErreur){

            oldXTmp.push_back(oldx.at(index));
        }else{
            sum -=  oldx.at(index);
            iteration -=1;
        }
        iteration++;
    }
    oldx = oldXTmp;
    cout << "" <<endl;
}

double LidarTreatments::getMoyenne(vector<double> xList){
    double moyenne = 0;
    for(double x : xList){
        moyenne += x;
    }
    return moyenne/xList.size();
}
