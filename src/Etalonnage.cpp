//
//  Etalonnage.cpp
//  ApiClientMac
//
//  Created by Jeremy on 20/10/2016.
//  Copyright © 2016 Imerir. All rights reserved.
//

#include "Etalonnage.hpp"
#include <iostream>
#include <thread>
#include <chrono>

//Etalonnage des valeur du magneto, de l'accelerometre et du gyrometre
Etalonnage::Etalonnage(){
    this->accelX = 0;
    this->accelY = 0;
    this->accelZ = 0;
    this->gyroX = 0;
    this->gyroY = 0;
    this->gyroZ = 0;
    this->magnetoX = 0;
    this->magnetoY = 0;
    this->magnetoZ = 0;
    this->counterGyro = 0;
    this->counterMagneto = 0;
    this->counterAccel = 0;
}

void Etalonnage::etalonnageAccel(int accelX, int accelY, int accelZ){
    if(!this->accelEtalonnageDone){
        if(this->counterAccel < 1000){
            this->accelX += accelX;
            this->accelY += accelY;
            this->accelZ += accelZ;
            counterAccel++;
        } else {
            this->accelEtalonnageDone = true;
            this->accelX = this->accelX/1000;
            this->accelY = this->accelY/1000;
            this->accelZ = this->accelZ/1000;
            std::cout << "Accelerometer [gx,gy,gz] : [" << this->accelX << "," << this->accelY << "," << this->accelZ << "]" << std::endl;
        }
    }
}

void Etalonnage::etalonnageMagneto(int magnetoX, int magnetoY, int magnetoZ){
    if(!this->magnetoEtalonnageDone){
        if(this->counterMagneto < 1000){
            this->magnetoX += magnetoX;
            this->magnetoY += magnetoY;
            this->magnetoZ += magnetoZ;
            counterMagneto++;
        } else {
            this->magnetoEtalonnageDone = true;
            this->magnetoX = this->magnetoX/1000;
            this->magnetoY = this->magnetoY/1000;
            this->magnetoZ = this->magnetoZ/1000;
            std::cout << "Gyrometer [x,y,z] : [" << this->magnetoX << "," << this->magnetoY << "," << this->magnetoZ << "]" << std::endl;
        }
    }
}

void Etalonnage::etalonnageGyro(int gyroX, int gyroY, int gyroZ){
    if(!this->gyroEtalonnageDone){
        if(this->counterGyro < 1000){
            this->gyroX += gyroX;
            this->gyroY += gyroY;
            this->gyroZ += gyroZ;
            counterGyro++;
        } else {
            this->gyroEtalonnageDone = true;
            this->gyroX = this->gyroX/1000;
            this->gyroY = this->gyroY/1000;
            this->gyroZ = this->gyroZ/1000;
            std::cout << "Gyrometer [x,y,z] : [" << this->gyroX << "," << this->gyroY << "," << gyroZ << "]" << std::endl;
        }
    }
}
