#include "Naio01Codec.hpp"
#include <iostream>
#include <cmath>
#include <vector>
using namespace std;


#define MARGE_ERREUR 25
#define LARGEUR_RANGEE 800
#define MARGE_ERREUR_LARGEUR_RANGEE 60


class LidarTreatments {
    private:
        vector<double> oldX1;
        vector<double> oldX2;
    public:
        void trierXliste(vector<double> oldx);
        double largeurRangee = LARGEUR_RANGEE;
        double margeErreur = MARGE_ERREUR;
        double margeErreurLargeurRangee = MARGE_ERREUR_LARGEUR_RANGEE;
    
        double **Lidar_Tri_Get_Corrected(unsigned short _lidar[ 271 ], int _min_radius, int _max_radius, int _packet_radius, int _packet_density) ;
    
        std::vector<std::vector<double>> getRangeeFromLidar(double** rangees);
    
        double getMoyenne(std::vector<double> xList);
};


