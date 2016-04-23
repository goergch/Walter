#include "Arduino.h"

#include <PIV.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PIV::PIV()
{
	PIV::setOutputLimits(-16384,16384);			//default output limit corresponds to 
													//the arduino pwm limits
    sampleTime = 100;								//default Controller Sample Time is 0.1 seconds

    PIV::setControllerDirection(DIRECT);

    lastTime = millis()-sampleTime;				
}
 
 
/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   PIV Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/ 
bool PIV::compute(float input, float setpoint, float &output)
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=sampleTime)
   {
      /*Compute all the working error variables*/
      float error = setpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      float dInput = (input - lastInput);
 
      /*Compute PIV Output*/
      output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}


/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/ 
void PIV::setTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; dispKi = Ki; dispKd = Kd;
   
   float SampleTimeInSec = ((float)sampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
  
/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PIV::setSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)sampleTime;
      ki *= ratio;
      kd /= ratio;
      sampleTime = (unsigned long)NewSampleTime;
   }
}
 
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PIV::setOutputLimits(float Min, float Max)
{
   if(Min >= Max) return;

   outMin = Min;
   outMax = Max;
 }

 

/* SetControllerDirection(...)*************************************************
 * The PIV will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PIV::setControllerDirection(int Direction)
{
   if(Direction !=controllerDirection)
   {
	  kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }   
   controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PIV.  they're here for display 
 * purposes.  this are the functions the PIV Front-end uses for example
 ******************************************************************************/
float PIV::getKp(){ return  dispKp; }
float PIV::getKi(){ return  dispKi;}
float PIV::getKd(){ return  dispKd;}
int PIV::getDirection(){ return controllerDirection;}

