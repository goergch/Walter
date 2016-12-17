// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.21 2015/08/25 04:57:29 mikem Exp mikem $

#include "AccelStepper.h"

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
	int i;

	for (i = 0; i < l; i++)
	{
		Serial.print(p[i], HEX);
		Serial.print(" ");
	}
	Serial.println("");
}
#endif

void AccelStepper::moveTo(long absolute)
{
	if (_targetPos != absolute)
	{
		_targetPos = absolute;
		computeNewSpeed();
		// compute new n?
	}
}

void AccelStepper::move(long relative)
{
	moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
boolean AccelStepper::runSpeed()
{
	// Dont do anything unless we actually have a step interval
	if (!_stepInterval)
		return false;

	unsigned long time = micros();
	unsigned long nextStepTime = _lastStepTime + _stepInterval;
	// Gymnastics to detect wrapping of either the nextStepTime and/or the current time
	if ( ((nextStepTime >= _lastStepTime) && ((time >= nextStepTime) || (time < _lastStepTime))) ||
		 ((nextStepTime < _lastStepTime) && ((time >= nextStepTime) && (time < _lastStepTime))))
	{
		if (_direction == DIRECTION_CW)
		{
			// Clockwise
			_currentPos++;
		}
		else
		{
			// Anticlockwise
			_currentPos--;
		}
		step(_currentPos);

		_lastStepTime = time;
		return true;
	}
	else
	{
		return false;
	}
}

// 0 pin step function (ie for functional usage)
void AccelStepper::step(long step)
{
	if (_speed_fp1 > 0)
		_forward(obj);
	else
		_backward(obj);
}

inline long AccelStepper::distanceToGo()
{
	return _targetPos - _currentPos;
}

long AccelStepper::targetPosition()
{
	return _targetPos;
}

long AccelStepper::currentPosition()
{
	return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper::setCurrentPosition(long position)
{
	_targetPos = _currentPos = position;
	_n = 0;
	_stepInterval = 0;
	_speed_fp1 = 0;
}

void AccelStepper::computeNewSpeed()
{
	long distanceTo = distanceToGo(); // +ve is clockwise from curent location

	long stepsToStop = mul16s_rsh(mul16s_rsh(_speed_fp1,one_by_2_times_acceleration_fp24,13),_speed_fp1, 13); // Equation 16

	if (distanceTo == 0 && stepsToStop <= 1)
	{
		// We are at the target and its time to stop
		_stepInterval = 0;
		_speed_fp1 = 0;
		_n = 0;
		return;
	}

	if (distanceTo > 0)
	{
		// We are anticlockwise from the target
		// Need to go clockwise from here, maybe decelerate now
		if (_n > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
				_n = -stepsToStop; // Start deceleration
		}
		else if (_n < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
				_n = -_n; // Start accceleration
		}
	}
	else if (distanceTo < 0)
	{
		// We are clockwise from the target
		// Need to go anticlockwise from here, maybe decelerate
		if (_n > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
				_n = -stepsToStop; // Start deceleration
		}
		else if (_n < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
				_n = -_n; // Start accceleration
		}
	}

	// Need to accelerate or decelerate
	if (_n == 0)
	{
		// First step from stopped
		_cn_fp0 = _c0_fp0;
		_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	else
	{
		// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
		_cn_fp0 -=  (_cn_fp0<<1) / ( (_n<<2) + 1); // Equation 13
		_cn_fp0 = max(_cn_fp0, _cmin_fp0);
	}
	_n++;
	_stepInterval = _cn_fp0;
	_speed_fp1 = (1000000L << 1) / _cn_fp0;
	if (_direction == DIRECTION_CCW)
		_speed_fp1 = -_speed_fp1;

	#if 0
	Serial.println(FP2FLOAT(_speed_fp1,1));
	Serial.println(_acceleration);
	Serial.println(_cn_fp0);
	Serial.println(_c0_fp0);
	Serial.println(_n);
	Serial.println(_stepInterval);
	Serial.println(distanceTo);
	Serial.println(stepsToStop);
	Serial.println("-----");
	#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
boolean AccelStepper::run()
{
	if (runSpeed())
		computeNewSpeed();
	return _speed_fp1 != 0 || distanceToGo() != 0;
}

void AccelStepper::setup(void* pObj, void (*forward)(void* obj), void (*backward)(void* obj)) {
	_forward = forward;
	_backward = backward;	
	obj = pObj;
}

AccelStepper::AccelStepper()
{
	_currentPos = 0;
	_targetPos = 0;
	_speed_fp1 = 0;
	_maxSpeed = 1.0;
	_acceleration = 0.0;
	_sqrt_twoa = 1.0;
	_stepInterval = 0;
	_minPulseWidth = 1;
	_lastStepTime = 0;

	// NEW
	_n = 0;
	_c0_fp0 = 0;
	_cn_fp0 = 0;
	_cn_fp0 = 1;
	_direction = DIRECTION_CCW;

	int i;
	for (i = 0; i < 4; i++)
	// Some reasonable default
	setAcceleration(1);
}

void AccelStepper::setMaxSpeed(float speed)
{
	if (_maxSpeed != speed)
	{
		_maxSpeed = speed;
		_cmin_fp0 = FLOAT2FP16( 1000000.0 / speed,0);
		// Recompute _n from current speed and adjust speed if accelerating or cruising
		if (_n > 0)
		{
			_n = (long)(FP2FLOAT(mul16s_rsh(_speed_fp1, _speed_fp1,1),2) / (2.0 * _acceleration)); // Equation 16
			computeNewSpeed();
		}
	}
}

float   AccelStepper::maxSpeed()
{
	return _maxSpeed;
}

void AccelStepper::setAcceleration(float acceleration)
{
	if (acceleration == 0.0)
	return;
	if (_acceleration != acceleration)
	{
		// Recompute _n per Equation 17
		_n = _n * (_acceleration / acceleration);
		// New c0 per Equation 7, with correction per Equation 15
		_c0_fp0 = FLOAT2FP16(0.676 * sqrt(2.0 / acceleration) * 1000000.0,0); // Equation 15
		_acceleration = acceleration;
		one_by_2_times_acceleration_fp24 = FLOAT2FP16(1.0/(2.0*_acceleration),24);
			
		computeNewSpeed();
	}
}

void AccelStepper::setSpeed(float speed)
{
	if (FLOAT2FP16(speed,1) == _speed_fp1)
	return;
	speed = constrain(speed, -_maxSpeed, _maxSpeed);
	if (speed == 0.0)
	_stepInterval = 0;
	else
	{
		_stepInterval = fabs(1000000.0 / speed);
		_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	_speed_fp1 = _speed_fp1;
}

float AccelStepper::speed()
{
	return FP2FLOAT(_speed_fp1,1);
}



void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
	_minPulseWidth = minWidth;
}

// Blocks until the target position is reached and stopped
void AccelStepper::runToPosition()
{
	while (run())
	;
}

boolean AccelStepper::runSpeedToPosition()
{
	if (_targetPos == _currentPos)
	return false;
	if (_targetPos >_currentPos)
	_direction = DIRECTION_CW;
	else
	_direction = DIRECTION_CCW;
	return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
	moveTo(position);
	runToPosition();
}

void AccelStepper::stop()
{
	if (_speed_fp1 != 0)
	{
		long stepsToStop = (long)(FP2FLOAT(mul16s_rsh(_speed_fp1, _speed_fp1,1),1) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
		if (_speed_fp1 > 0)
		move(stepsToStop);
		else
		move(-stepsToStop);
	}
}

bool AccelStepper::isRunning()
{
	return !(_speed_fp1 == 0 && _targetPos == _currentPos);
}
