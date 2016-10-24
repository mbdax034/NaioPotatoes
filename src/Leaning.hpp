//
//  Leaning.hpp
//  ApiClientMac
//
//  Created by Jeremy on 21/10/2016.
//  Copyright © 2016 Imerir. All rights reserved.
//

#ifndef Leaning_hpp
#define Leaning_hpp

#include <stdio.h>
class Leaning {    
    public :
    double getAbsoluteXAxisLeaning(double gX, double gY, double gZ, double gXinit );
    double getAbsoluteYAxisLeaning(double gX, double gY, double gZ, double gYinit);
};
#endif /* Leaning_hpp */
