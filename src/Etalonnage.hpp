//
//  Etalonnage.hpp
//  ApiClientMac
//
//  Created by Jeremy on 20/10/2016.
//  Copyright © 2016 Imerir. All rights reserved.
//

#ifndef Etalonnage_hpp
#define Etalonnage_hpp

#include <stdio.h>

class Etalonnage {
    
    private :
    int accelX;
    int accelY;
    int accelZ;
    int counterAccel;
    bool accelEtalonnageDone;
    
    int magnetoX;
    int magnetoY;
    int magnetoZ;
    int counterMagneto;
    bool magnetoEtalonnageDone;
    
    int gyroX;
    int gyroY;
    int gyroZ;
    int counterGyro;
    bool gyroEtalonnageDone;
    
    public:
    Etalonnage();
    void etalonnageAccel(int accelX, int accelY, int accelZ);
    void etalonnageMagneto(int gyroX, int gyroY, int gyroZ);
    void etalonnageGyro(int gyroX, int gyroY, int gyroZ);
    
    int getAccelX(){return accelX;};
    int getAccelY(){return accelY;};
    int getAccelZ(){return accelZ;};
    
    int getMagnetoX(){return magnetoX;};
    int getMagnetoY(){return magnetoY;};
    int getMagnetoZ(){return magnetoZ;};
    
    int getGyroX(){return gyroX;};
    int getGyroY(){return gyroY;};
    int getGyroZ(){return gyroZ;};
    
};


#endif /* Etalonnage_hpp */
