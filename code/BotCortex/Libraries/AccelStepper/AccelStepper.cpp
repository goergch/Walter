// AccelStepper.cpp
//
// Copyright (C) 2009 Mike McCauley
// $Id: AccelStepper.cpp,v 1.14 2012/12/22 21:41:22 mikem Exp mikem $

#include <AccelStepper.h>
#include "pins.h"
#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
    int i;

    for (i = 0; i < l; i++)
    {
	logger->print(p[i], HEX);
	logger->print(" ");
    }
    logger->println("");
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
	unsigned long _nextStepTime = _lastStepTime + _stepInterval;

    // Gymnastics to detect wrapping of either the nextStepTime and/or the current time
    // unsigned long nextStepTime = _lastStepTime + _stepInterval;
    if (   ((_nextStepTime >= _lastStepTime) && ((time >= _nextStepTime) ||
    		(time < _lastStepTime))) ||
    		((_nextStepTime < _lastStepTime) && ((time >= _nextStepTime) && (time < _lastStepTime))))
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
		step(_currentPos & 0x7); // Bottom 3 bits (same as mod 8, but works with + and - numbers)

		_lastStepTime = time;
		return true;
    }
    else
    {
    	return false;
    }
}

long AccelStepper::distanceToGo()
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
}

void AccelStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    // long stepsToStop = (long)((_speed * _speed) / (_acceleration +  _acceleration)); // Equation 16
    long stepsToStop = (long)(_speed * _speed * _one_by_2times_acc); // Equation 16
    if (distanceTo == 0 && stepsToStop <= 1)
    {
	// We are at the target and its time to stop
        _stepInterval = 0;
        _speed = 0.0;
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
		else
			if (_n < 0)
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
		else
			if (_n < 0)
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
		_cn = _c0;
		_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
		// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
		_cn -= (_cn + _cn) / (float(4*_n + 1)); // Equation 13
		_cn = max(_cn, _cmin);
	}
    _n++;
    _stepInterval = _cn;

    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
    	_speed = -_speed;

#if 0
    logger->println(_speed);
    logger->println(_acceleration);
    logger->println(_cn);
    logger->println(_c0);
    logger->println(_n);
    logger->println(_stepInterval);
    logger->println(distanceTo);
    logger->println(stepsToStop);
    logger->println("-----");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if we are still running to position
void AccelStepper::run()
{
    if (runSpeed())
	   computeNewSpeed();
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
	_speed = 0;
	_maxSpeed = 1.0;
	_acceleration = 0.0;
	_stepInterval = 0;
	_minPulseWidth = 1;
	_lastStepTime = 0;

	// NEW
	_n = 0;
	_c0 = 0;
	_cn = 0;
	_cn = 1;
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
    	_cmin = 1000000.0 / speed;
    	// Recompute _n from current speed and adjust speed if accelerating or cruising
		if (_n > 0)
		{
			// _n = (long)((_speed * _speed) / (_acceleration + _acceleration)); // Equation 16
			_n = (long)(_speed * _speed *_one_by_2times_acc ); // Equation 16
			computeNewSpeed();
		}
    }
}

void AccelStepper::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
    	return;
    if (_acceleration != acceleration)
    {
		// Recompute _n per Equation 17
		_n *= (_acceleration / acceleration);
		// New c0 per Equation 7
		_c0 = sqrt((2.0* 1000000.0 * 1000000.0) / acceleration);
		_acceleration = acceleration;
		_one_by_2times_acc = 0.5 / (_acceleration); // precomputation to speed up calculation
		computeNewSpeed();
    }
}

void AccelStepper::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
        _stepInterval = 0;
    else
    {
	    _stepInterval = fabs(1000000.0 / speed);
	    _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
}

float AccelStepper::speed()
{
    return _speed;
}


void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}


// Blocks until the target position is reached and stopped
void AccelStepper::runToPosition()
{
    while (_speed != 0 || distanceToGo() != 0)
    	run();
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

// 0 pin step function (ie for functional usage)
void AccelStepper::step(int  step)
{
	if (_speed > 0) {
		// logger->print("+");
		_forward(obj);
	}
	else {
		// logger->print("-");
		_backward(obj);
	}
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void AccelStepper::stop()
{
    if (_speed != 0.0)
    {
	long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
	if (_speed > 0)
	    move(stepsToStop);
	else
	    move(-stepsToStop);
    }
}
