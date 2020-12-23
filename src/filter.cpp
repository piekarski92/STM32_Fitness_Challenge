#include "filter.h"


const double CanonFilter::b[3] = {  0.20657208, 0.41314417, 0.20657208  };
const double CanonFilter::a[3] = {  1.        , -0.36952738,  0.19581571 };
double y0 = 0;
double w0 = 0;
double w1 = 0;
double w2 = 0;

double CanonFilter::filterData(double x0){
    /*
    Based on Canonical Filter.
    Low pass settings set for 20Hz cutoff frequency.
    This is a 2nd ordered filter.
    w0 = x0 - a1*w1 - a2*w2
    y0 = b0*w0 + b1*w1 + b2*w2
    
    This function passes in a variable from a continuous stream
    of collected data and filters it in one for loop below.

    Unfiltered
    [x0 x1 x2 ... xN]

    Then it finds y0 from above equation and inserts into array.
    i=0
    [y0 x1 x2 ... xN]
    i=1
    [y0 y1 x2 ... xN]
    ...
    i=N-1
    [y0 y1 y2 ... yN-1 xN]
    i=N
    [y0 y1 y2 ... yN-1 yN]

    After function is complete, the current state can utilize the block
    for further processing.
    */
    w0 = x0 - a[1]*w1 - a[2]*w2;
    y0 = b[0]*w0 + b[1]*w1 + b[2]*w2;
    w2 = w1;
    w1 = w0;
    return y0;
}

void CanonFilter::resetStates(){

    /*
    To use the filter after a jump in time, it's imperitive to clear the 'memory' of past states.
    This is to ensure optimal operation of the filter.
    */
    double y0 = 0;
    double w0 = 0;
    double w1 = 0;
    double w2 = 0;
}
//};