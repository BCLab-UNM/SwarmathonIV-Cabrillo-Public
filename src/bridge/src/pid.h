/*
 * pid.h
 *
 *  Created on: Jun 7, 2017
 *      Author: maximus
 */

#ifndef BRIDGE_SRC_PID_H_
#define BRIDGE_SRC_PID_H_

#include <time.h>

class PID {

public:
        PID(double p, double i, double d, double db, double hi, double lo, double stick=-1);
	~PID();

	double step(double setpoint, double feeback, double now=0);

private:

	double getNow();

	// Configuration
        double _kp, _ki, _kd, _dband, _hi, _lo, _stiction;

	// State
	double _out, _sum, _lasterr, _lastsp, _lasttime;
};

#endif /* BRIDGE_SRC_PID_H_ */
