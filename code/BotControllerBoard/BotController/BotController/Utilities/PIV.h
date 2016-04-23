#ifndef PIV_h
#define PIV_h

class PIV
{
	public:

	//Constants used in some of the functions below
	#define DIRECT  0
	#define REVERSE  1

	PIV();		   //   Setpoint.  Initial tuning parameters are also set here

	// * performs the PIV calculation.  it should be
	//   called every time loop() cycles. ON/OFF and
	//   calculation frequency can be set using setSampleTime respectively

	bool compute(float input, float setpoint, float &output); 
	
	//clamps the output to a specific range. 0-255 by default, but
	//it's likely the user will want to change this depending on
	//the application
	void setOutputLimits(float, float); 
	
	// set Kp,Ki,kd
	void setTunings(float, float,float);         	  
	
	// * Sets the Direction, or "Action" of the controller. DIRECT
	//   means the output will increase when error is positive. REVERSE
	//   means the opposite.  it's very unlikely that this will be needed
	//   once it is set in the constructor.
	void setControllerDirection(int);	  

	// * sets the frequency, in Milliseconds, with which
	//   the PIV calculation is performed.  default is 100
	void setSampleTime(int);              
	
	
	
	//Display functions ****************************************************************
	float getKp();						  // These functions query the PIV for interal values.
	float getKi();						  //  they were created mainly for the PIV front-end,
	float getKd();						  // where it's important to know what is actually
	int getMode();						  //  inside the PIV.
	int getDirection();					  //

	private:
	
	float dispKp;				// * we'll hold on to the tuning parameters in user-entered
	float dispKi;				//   format for display purposes
	float dispKd;				//
	
	float kp;                  // * (P)roportional Tuning Parameter
	float ki;                  // * (I)ntegral Tuning Parameter
	float kd;                  // * (D)erivative Tuning Parameter

	int controllerDirection;

	//   what these values are.  with pointers we'll just know.
	
	unsigned long lastTime;
	float ITerm, lastInput;

	unsigned long sampleTime;
	float outMin, outMax;
};
#endif

