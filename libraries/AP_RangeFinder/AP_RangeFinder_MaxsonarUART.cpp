// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       AP_RangeFinder_MaxsonarI2CXL.cpp - Arduino Library for MaxBotix I2C XL sonar
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *       datasheet: http://www.maxbotix.com/documents/I2CXL-MaxSonar-EZ_Datasheet.pdf
 *
 *       Sensor should be connected to the I2C port
 *
 *       Variables:
 *               bool healthy : indicates whether last communication with sensor was successful
 *
 *       Methods:
 *               take_reading(): ask the sonar to take a new distance measurement
 *               read() : read last distance measured (in cm)
 *
 */

// AVR LibC Includes
#include "AP_RangeFinder_MaxsonarUART.h"
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_MaxsonarUART::AP_RangeFinder_MaxsonarUART( FilterInt16 *filter ) :
    RangeFinder(NULL, filter),
    healthy(true)
{
    min_distance = AP_RANGE_FINDER_MAXSONARI2CXL_MIN_DISTANCE;
    max_distance = AP_RANGE_FINDER_MAXSONARI2CXL_MAX_DISTANCE;
}

// Public Methods //////////////////////////////////////////////////////////////

// take_reading - ask sensor to make a range reading
bool AP_RangeFinder_MaxsonarUART::take_reading()
{
    healthy = true;
    return true;
}

// read - return last value measured by sensor
int AP_RangeFinder_MaxsonarUART::read()
{
    int16_t ret_value = 1;
    char rx;
    int result = 0;
    int got_data = 0;
    char buf [10];

    healthy = false;
	while ((rx = _uart->read())>=0 && !got_data) {
		switch (_stage) {
			case 0:
				_chars_read = 0;
				if (rx == 'R') {
					_stage = 1;
				}
				break;
			case 1:
				if ((rx<='9')&&(rx>='0')) {
					_buff[_chars_read++] = rx;
					if (_chars_read >= 3) {
						_buff[3] = 0;
						result = atoi(_buff);
						_stage = 2;
					}
				} else {
					_stage = 0;
				}
				break;
			case 2:
				if (rx == 13)
					got_data = 1;
				_stage = 0;
				break;
			default:
				break;
		}
	}
	if (got_data) {
		ret_value = result;
		healthy = true;
	}
    
    // ensure distance is within min and max
    ret_value = constrain_float(ret_value, 0, max_distance);
    
    ret_value = _mode_filter->apply(ret_value);
    
    return ret_value;
}
