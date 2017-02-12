Now we have the trajectory in terms of a sequence of joint angles. This needs to be translated into movements of the motors. This translation should be done in a manner that no motors limits are violated, and with a speed profile that avoids vibrations. Typically, this is done with a speed profile with limited acceleration. I.e. if the required acceleration is higher than an upper limit, the speed is increased with a limited acceleration until the to-be speed is reached.

Additionally, we need to control a feedback loop to ensure that the to-be angle of the motor is actually reached.

All this is done in Walters Cortex, a board based on an 32-bit ARM microcontroller (Teensy 3.5) that receives interpolated trajectory points at 10Hz, and runs a closed loop for the stepper motors at 100Hz. Furthermore, it takes care that no bumpy movements happen by limiting angle, speed, and acceleration.

<img width="700px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image103.png"/>		

The software runs on the basis of the Arduino library. This is a legacy, since I started with an 8-bit ATmega controller before I upgraded to an Arm processor (This happened when I realized, that controlling 5 steppers and encoders eats up much more computing power than I thought).

<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image104.png"/>		

The lower part of the layout contains a separate power supply for the servos, the steppers and mC. The mC gets 5V by a standard integrated voltage regulator (7805), the two servos are driven by a 7809 voltage regulator providing 2A (two HerkuleX servos required 1A at most). Additionally, there are two relays for a proper start-up procedure switching the 240W power supply for the steppers and the power supply for the servos. This has been necessary to avoid impulses to the motors inducing annoying ticks when power is switched on.  So, after booting the Trajectory Board and switching on the Cortex Board, all steppers are disabled, then the steppers power supply is turned on, then the servos power supply is switched on. By that procedure, no ticks are happening during starting up.

On the top there is the Arm Cortex M4 controller receiving points to move to and controlling everything low level in a synchronous manner. As usual, tricky part was to provide a stable power supply for all components. I struggled a lot with pikes induced by the strong steppers, AS5048 rotary sensors that are very sensitive to voltage, and UARTs with not exact baud rates.

In the end, I made one board for the power supply providing 24V 10A for the steppers, 5V 4A for the ODroid and the ARM controller, and 9V 2A for Herkulex Servos. Servo’s and Stepper’s power supply are turned on by a relay controlled by the teensy that orchestrates the startup and teardown procedure. The second board on the right has almost no more than the Teensy controller and many sockets.

<img width="700px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image105.png"/>		

The left part is providing power for uC & Servos, the right part is the ARM M4 controller

Putting all together looks like this:

<img  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image106.jpg"/>		

This is my desk with 5 Pibot drivers, a Teensy 3.5 (on the left bottom side)  and the power supply PCB on the right.
 
## Sensors

The used sensors are AMS' magnetic encoders 5048B with 14-bit resolution and an I2C interface. They are connect with VDD of 3.3 V to two I2C buses. Walter has no electronics in its base, all is connected via a long cable of approx. 1.5m. For an I2C bus this is difficult, since the capacity of that cable is around 300pF. It was working in the end, but I had to use very low pullup resistors of 1.1K&#x2126; for both I2C lines (see also (Computation of I2C pullup resistory)[http://www.ti.com/lit/an/slva689/slva689.pdf]
Since the Cortex M4 has many I2C buses, where was no need to connect all sensors to the same line.


While controlling robot servos is easy (everything is built in, even a PID controller, they only require a serial interface and a library), stepper motors are more difficult to control. While very tempting by providing high torque without a gearbox and a proper position even without encoders, they turned out to cost me many hours until they moved that smooth as expected.

First approach was the classical feed-back control system taking the encoder angle and the to-be angle of an actuator, giving that to PID controller, computing a correcting angle and giving that to the stepper motor. In the Arduino space, the standard library to control steppers is AccelStepper  which can be used to implement that and leverage from the nice feature of smooth acceleration and deceleration.

This idea was bad. AccelStepper provides the method move(targetposition) that accelerates first and and decelerates afterwards until the motor stops exactly at the targetposition. This works fine for larger moves of several seconds. But in a closed loop, this accelerates and decelerates permanently producing vibrations and – depending on the speed – resonances. Speed was very limited not to mention a very bad sound.

The solution that finally worked was to compute the acceleration that is required in one sample of 10ms, and set this acceleration explicitly for the next sample:

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image107.png"/>		

*v<sub>encoder</sub>*, is the speed that is necessary to compensate the difference between encoder angle and to-be angle. It can be approximated by

&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image108.png"/>		
   
Unfortunately, setting the acceleration requires a square root for computing the time until the next step (as done internally in AccelStepper)

&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image109.png"/>		

This equation was the reason to decommission the previously used 8-bit Atmega and go to a 32-bit ARM processor with FPU which provides the square root computation in hardware.

The final closed-loop looks like this:
<pre>
void stepperLoop () {
	float dT = sampleTime();                         // [ms], approx. 10ms
	float currentAngle =        getEncoderAngle();   // encoder value in [°]
	// interpolate trajectory to get current and next angle
	float toBeAngle =           movement.getCurrentAngle(millis());
	float nextToBeAngle =       movement.getCurrentAngle(millis()+dT);
	
	// get speed of current sample, next sample and error compared to encoder’s angle
	float currStepsPerSample =  getMicroStepsByAngle(toBeAngle - lastToBeAngle);
	float nextStepsPerSample =  getMicroStepsByAngle(nextToBeAngle - toBeAngle);
	float stepErrorPerSample =  getMicroStepsByAngle(toBeAngle  - currentAngle);

	// implement PI controller. kP is 0.3-0.5, kI can be low (< 0.1) depending on mechanics
	float Pout = kP * stepErrorPerSample;
	integral += stepErrorPerSample * dT;
	float Iout = kI * integral;
	float PIDoutput = Pout + Iout;
	float accelerationPerSample = PIDoutput;
	float distanceToNextSample = accelerationPerSample + currStepsPerSample;

	// compute the acceleration for this sample
	float sampleAcc = ((currStepsPerSample-nextStepsPerSample+stepErrorPerSample) / dT;
	accel.setAcceleration(fabs(sampleAcc));      // do not accelerative faster than sampleAcc
	accel.move(distanceToNextSample);            // move n steps
}
</pre>

Continue reading with [Bill of Material](https://github.com/jochenalt/Walter/wiki/Bill-of-Material).