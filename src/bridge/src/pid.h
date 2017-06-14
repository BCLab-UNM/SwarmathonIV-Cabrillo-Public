/*
 * pid.h
 *
 *  Created on: Jun 7, 2017
 *      Author: maximus
 */

#ifndef BRIDGE_SRC_PID_H_
#define BRIDGE_SRC_PID_H_

#include <time.h>

#define DRIVE_MODE_STOP   0
#define DRIVE_MODE_PID    1

class PID {

public:
	PID(double p, double i, double d, double db, double hi, double lo, double stick=-1, double wu=0);
	~PID();

	double step(double setpoint, double feeback, double now=0);
	void reconfig(double p, double i, double d, double db, double st, double wu);
	void reset();

private:

	double getNow();

	// Configuration
	double _kp, _ki, _kd, _dband, _hi, _lo, _stiction, _windup;

	// State
	double _out, _sum, _lasterr, _lastsp, _lasttime;
};

#endif /* BRIDGE_SRC_PID_H_ */
