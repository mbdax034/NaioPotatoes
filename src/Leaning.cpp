//
//  Leaning.cpp
//  ApiClientMac
//
//  Created by Jeremy on 21/10/2016.
//  Copyright © 2016 Imerir. All rights reserved.
//

#include "Leaning.hpp"
#include <cmath>

double Leaning::getAbsoluteXAxisLeaning(double gX, double gY, double gZ, double gXinit ){
    double sum = ( gX*gX ) + ( gY*gY ) + ( gZ*gZ );
    double resultante = sqrt(sum);
    double alpha = acos( (gX-gXinit) / resultante );
    alpha = (alpha)*(180/M_PI);
    return (90-alpha);
}

double Leaning::getAbsoluteYAxisLeaning( double gX, double gY, double gZ, double gYinit ){
    double sum = ( gX*gX ) + ( gY*gY ) + ( gZ*gZ );
    double resultante = sqrt(sum);
    double alpha = acos( (gY-gYinit) / resultante );
    alpha = alpha*(180/M_PI);
    return (90-alpha);
}

