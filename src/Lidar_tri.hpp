#ifndef LIDAR_TRI_HPP
#define LIDAR_TRI_HPP

#include "Naio01Codec.hpp"
class LidarTreatments {
    private:
    public:
        LidarTreatments();
        double **Lidar_Tri_Get_Corrected(unsigned short _lidar[ 271 ], int _min_radius, int _max_radius, int _packet_radius, int _packet_density) ;
    
        std::vector<std::vector<double>> getRangeeFromLidar(double** rangees);
};

#endif
