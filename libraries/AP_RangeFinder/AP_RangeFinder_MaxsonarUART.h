// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_RANGEFINDER_MAXSONARI2CXL_H__
#define __AP_RANGEFINDER_MAXSONARI2CXL_H__

#include "RangeFinder.h"

#define AP_RANGEFINDER_MAXSONARI2CXL                4
#define AP_RANGE_FINDER_MAXSONARI2CXL_SCALER        1.0
#define AP_RANGE_FINDER_MAXSONARI2CXL_MIN_DISTANCE  20
#define AP_RANGE_FINDER_MAXSONARI2CXL_MAX_DISTANCE  765

class AP_RangeFinder_MaxsonarUART : public RangeFinder
{

public:

    // constructor
    AP_RangeFinder_MaxsonarUART(FilterInt16 *filter);

    // init - simply sets port
    void init(AP_HAL::UARTDriver* uart) { _uart = uart; }

    // take_reading - ask sensor to make a range reading
    bool            take_reading();

    // read value from sensor and return distance in cm
    int             read();

    // heath
    bool            healthy;

protected:
    AP_HAL::UARTDriver* _uart;

};
#endif  // __AP_RANGEFINDER_MAXSONARI2CXL_H__
