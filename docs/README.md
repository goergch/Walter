<img align="right" width="100px" src="images/image002.jpg" >

Most important matter of a robot is to look good. The actuators should have human-like proportions, movements should be smooth, and – rather a matter of personal taste – I do not like humps or bulges with motors or gearboxes. All stuff should be inside the regular housing. 

After checking lots of the stuff around on youtube, I recognized that just a few DIY robots are close to what I had in mind. There's the construction by Andreas Hölldorfer ([Printable Robot Arm](https://hackaday.io/project/3800-3d-printable-robot-arm)), which got covered recently ("Moveo"). Unfortunately without mentioning the obvious inspiration coming from Andreas. I got lots of ideas from his construction, and there are still a couple of parts directly derived from his design.

Another construction called [Thor](https://hackaday.io/project/12989-thor) is coming from Ángel Larrañaga Muro which has an interesting differential gearbox for the lower actuators.

These two have the quite the most professional design, while most of the robot arms on youtube are servo based and more or less constructed the same. 

<img align="left" width="30%" src="videos/logo-animated.gif" >

This is what I had in mind. Most of the DIY robots are using servos, mostly for convenience, since the encoder is built-in already and they are easy to control. Thing is, when it comes to higher torque, the connection of the servo with the actuator becomes difficult, and hard to make of 3D printed material. If the servo or the flange moves just a little bit within the housing, the according play will magnify to a significant amount at the end of the actuator. The required precision to avoid this is way above hobby grade components. 

And servos are boringly easy to use. No fun in construction. A motor with a belt drive and a separate angle sensor solves this, it provides low backlash and allows the electronics to compensate imprecise parts with the sensor placed separately from the motor. Additionally, the motor of an actuator can be placed in the previous joint, lowering the centre of gravity of each actuator.

When a belt drive is set, choice comes naturally to stepper motors, since an additional gearbox is not necessary anymore, torque is high and the belts should compensate the vibrations coming from the stepping nature of these motors.

In general, the information flow looks like this:

<img width="600px" src="images/image013.png"/>

The visualizer is mainly the UI-based tool to plan trajectories by defining singular points. These points are sent to the Trajectory Execution, where the trajectory is interpolated using Bézier curves. Out of each point of the interpolated curve the angles are computed and send to the controller board, where a PID controller takes care that each actuator actually follows the curve.

On the mechanical side, we have two actuators driven by a servo (mainly due to space restrictions) and four actuators driven by a stepper/rotary encoder combination.

The servos are controlled directly by the cortex controller board  via a serial interface. The steppers do not have an internal feedback loop, so we need rotary encoders detecting the absolute angle of the joint and allowing to implement feedback controllers. Depending on the actuator, the steppers provide a torque between 26Ncm (elbow) and 3,1Nm (upperarm). For space reasons, the gripper and the wrist is driven by a servo.

<img align="center" width="800px" src="images/image014.png"/>

The steppers are driven by retail stepper drivers (PiBot Stepper Driver) around the popular PWM stepper driver Toshiba 6600 (4.5A max). The stepper drivers are directly connected to the Controller Board. It receives joint angles at 20Hz, interpolates the points in between, and sends the according PWM signal to the stepper drivers and to the servos. Besides micro interpolation of the trajectory, the controller board takes care of the speed profile, i.e. it limits the acceleration and speed of each actuator. The controller board is a DIY board around an ARM Cortex M4 with 120 MHz (Teensy 3.5). I started with an ATmega 644 8-bit controller, but it turned out that the ATmega was not able to control 5 steppers with a proper sample rate, let alone reading 5 encoders on top.

The controller board is fed by the trajectory board, which is an Odroid XU4 board with a 2GHz octa core.

The trajectory controller board is encapsulated by a webserver exposing the current movement and accepting commands like new trajectories.

Planning a trajectory means defining a sequence of poses in 3D space. These defined poses are interpolated in order to result in a smooth and continuous curve. Most beautiful are cubic Bézier curves.

Bézier curves are polynoms of 3rd grade using a start and an end point and two support points defining the curvature at the start and end point. The trajectory is defined by the start and the end point, the support point is not on the trajectory but used to make it smooth only. The computation is based on a parameter *t=0..1* defining the ratio of how much the current position has already made of the full curve. Let’s assume, we have the four points *P<sub>0</sub>..P<Sub>3</sub>*, of which *P<sub>1</sub>* and *P<sub>2</sub>* are support points the curve does not touch.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image015.png"/> 

This computation is done for *x*, *y*, and *z* coordinates. Normally being beautiful, but Bézier curves have a tendency to “bounce”, if the support points *P<sub>1</sub>* and *P<sub>2</sub>* differ too much in terms of the distance to the trajectory points *P<sub>0</sub>* and *P<sub>3</sub>*. So, it is necessary to normalize support points by a small trick:

The picture illustrates a trajectory defined by *A*, *B*, *C*, and *D*. We want to model the piece between *B* and *C* with a cubic Bézier curve.

<img align="left" width="450px" src="images/image016.png"/>

The support point *B’* is computed by taking point *A*, mirroring it at *B* (*A’*), and moving along the angle bisector of *A’* *B* *C* by a 3<sup>rd</sup> of the length of *BC*, *C’* is computed in an analogous manner.

This approach is rather arbitrary, but results in a smooth and non-oscillating curve. On the left, we see a linear trajectory, on the right the same curve as bezier curve.

<img align="left" width="300px" src="videos/linear interpolated curve.gif"/>
<img width="300px" src="videos/bezier curve.gif"/>

Now the curve looks fine, but simply following that curve is not enough to make a smooth movement. We need a speed profile that avoids jerky movements. This is done by speed profiles. The classical approach is to use trapezoidal speed profiles like this:

<img align="left" width="320px" src="images/image017.png"/>
<img width="320px" src="images/image018.png"/>

This trapezoid speed profile results in a constant (maximum) acceleration, then continuing with no acceleration, and a constant deceleration until zero speed is reached. To get the position on a curve, speed is integrated over time.

Despite of the corners in the speed profile, the position profile looks smooth. Still, how is a profile like that computed? Having a constant acceleration *a*, start speed vstart, an end speed vend, the distance d (length of the Bezier curve) and the desired duration of the complete profile *t<sub>g</sub>*, we need to compute the time *t<sub>0</sub>* and *t<sub>1</sub>* which is the duration of the starting acceleration and final deceleration. The full distance is given by

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image019.png" >

The duration and speed of the plateau is given by 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="images/image020.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image021.png"/>

Rearranging these equations to get *t<sub>0</sub>* ends up in

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image023.png"/>

with

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="images/image025.png"/>
	
Finally, with the equation above we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image026.png"/>
	
With the equation above on computing *d* we get *t<sub>g</sub>*.

This holds true for a trapezoid profile only, since to model a full trajectory that consist of single movements we also need profiles that represent a ramp or stairways, depending on the constrains in terms of duration, distance, and start/end speed. This ended up in quite a lot of code treating all the different cases. 

On the right, the effect of a speed profle is illustrated compared to the same trajectory without speed profile. The effect is not spectacular, but can be detected when watching the edges: the movement stops slowly and accelerates when it continues

<img align="left" width="300px" src="videos/without speed profile.gif"/>
<img width="310px" src="videos/with speed profile.gif"/>

All this is done while planning a trajectory, so that is a task done upfront. At runtime, the trajectory is fully compiled and contains bezier curves and speed profiles already. The according UI where trajectories are planned looks like this:

<img align="center" src="images/planner-screenshot.png"/>

This UI provides forward and inverse kinematics (explained in [Kinematics](https://github.com/jochenalt/Walter/wiki/Kinematics)), allows to define a trajectory by defining support points (indicated by big green balls). After having assigned some parameters like speed or duration the trajectory is compiled, i.e. the bezier curves and speed profiles are computed (indicated by small green balls). This trajectory can be send to Walter to be executed by its Cortex.

Now we have a nice trajectory with a smooth speed profile, but do not yet know how to compute the angles of the joints. 

Kinematics is about computation of the tool-centre-point (*TCP*) out of joint angles and vice versa. First is simple, latter is more tricky, but lets see later on.
But before starting any kinematics, it is necessary to define all coordinate systems.

<img width="400px" src="images/image027.png"/>

The most important design decision is to let the three upper axis’ intersect in one point, the so-call wrist-center-point (*WCP*). This decision makes the computation of the inverse kinematic solvable without numeric approaches.

The picture shows the used coordinate systems in the default position of the bot, having all angles at 0°, starting from the base (angle0) and ending with the coordinate system of the hand (angle<sub>6</sub>). For convenience the forearm (angle<sub>1</sub>) adds +90° to the real angle in order to have the base position at 0°of the bot, although the illustrated actually is -90°. The coordinate systems have been are arranged according to the Denavit Hardenberg convention, which is:

* The angle rotates around the z-axis
* The z-axis points on the direction of the next joint
* The transformation from anglei to anglei+1 is given via 
   1. rotating around the x-axis by α
   1. translation along the x-axis by α
   1. translation along the z-axis by *d*, and
   1. rotation around the z-axis by θ

So, the Denavit Hardenberg parameters are:

| Joint      | a[°] | a[mm]            | d[mm]           |
|----------  | -----| ---------------- | --------------- |
| Hip        | *-90°*| *0*              | *d<sub>0</sub>* |
| Upperarm   | *0*   | *a<sub>1</sub>*  | *0*             |
| Forearm    | *-90°* | *0*              | *0*             |
| Elbow      | *90°*  | *0*              | *d<sub>3</sub>* |
| Wrist      | *-90°* | *0*              | *0*             |
| Hand       | *0*  | *0*              | *d<sub>5</sub>* |

The general definition of a Denavit Hardenberg (DH) transformation is

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="images/image029.png"/>

which is a homogeneous matrix with two rotations *(x,z)* and two translations *(x,z)*.

Combined with the DH parameters, the following DH matrixes define the transformation from one joint to its successor:

<img  align="left" src="images/image030.png"/>

<img  align="left" src="images/image031.png"/>

<img   src="images/image032.png"/>

<img  align="left" src="images/image033.png"/>

<img  align="left" src="images/image034.png"/>

<img src="images/image035.png"/>

## Forward Kinematics

With the DH transformation matrixes at hand, computation of the bot’s pose out of the joint angles is straight forward. The matrix representing the gripper’s pose <img align="center"  src="images/image036.png"/> is 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image037.png"/> 

By multiplying the transformation matrix with the origin (as homogeneous vector), we get the absolute coordinates of the tool centre point in world coordinate system (i.e. relative to the bot’s base).

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="images/image038.png"/>

The orientation in terms of roll/nick/yaw of the tool centre point can be derived out of <img align="center"  src="images/image036.png"/>by taking the part representing the rotation matrix (<img align="center"  src="images/image040.png"/>). ([Wikipedia Roll/Nick/Yaw](https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel#Berechnung_aus_Rotationsmatrix) )

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image041.png"/>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image042.png"/>  
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image043.png"/>  
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image044.png"/>  

For <img align="center" src="images/image045.png"/> we have a singularity (*atan2* never becomes this), but wikipedia has a solution for that as well
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="images/image046.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image047.png"/>

if <img align="center" src="images/image048.png"/> we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="images/image046.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image049.png"/>

Note: Unfortunately, the gripper’s coordinate system is not appropriate for human interaction, since the default position as illustrated in the <img align="center" src=":images/image027.png|Coordinate Systems"/> not nick/roll/yaw=(0,0,0). So, in the Trajectory Visualizer it is handy to rotate the gripper matrix such that the default position becomes . The according rotation matrix represents a rotation of -90° along *x*,*y*, and *z*, done by the rotation matrix

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image051.png"/>

In the following equations, this is not considered, since it is for convenience in the UI only.

## Inverse Kinematics 
Inverse kinematics denotes the computation of all joint angles out of the tool-centre-point’s position and orientation. In general it is hard to give non-numeric solution, in our case it is possible since the upper three joint angles intersect in the WCP.

Input of inverse kinematics is the TCP’s position and orientation in terms of roll, nick, yaw, abbreviated by *γ*, *β*,and *α*.

<img align="left" src="images/image053.png"/>

<img align="center" src="images/image054.png"/>

First, we need to compute the wrist-centre-point out the tool-centre-point. This is possible by taking the TCP and moving it back along the TCP’s orientation by the hand length. For doing so, we need the transformation matrix from the base to the last joint <img align="center" src="images/image036.png"/> 
which we can derive out of the TCP’s position and orientation.

To build the transformation matrix <img align="center" src="images/image036.png"/> we need the rotation matrix defining the orientation of the TCP. This is given by multiplying the rotation matrixes for all axis (*γ*, *β*, *α*) which gives <img align="center" src="images/image056.png"/> (see also *[computation of rotation matrix out of Euler Angles](http://kos.informatik.uni-osnabrueck.de/download/diplom/node26.html)*).

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image057.png"/>

Now we can denote the transformation matrix of the TCP by builing a homogenous matrix out of *TCP<sub>orientation</sub>* and *TCP<sub>position</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image058.png"/>

From the TCP’s perspective, WCP is just translated by *d<sub>5</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image059.png"/>

Furthermore, <img align="center" src="images/image060.png"/>, so we get the WCP by

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image061.png"/>

in world coordinates.

Having a top view on the robot shows how to compute the first angle *θ<sub>0</sub>*:

<div align="center"><img width="600px" src="images/image062.png"/></div>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image064.png"/>

Actually, this angle exists in two variants: if the bot looks backwards, another valid solution is

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image065.png"/>

Thanks to the design having a wrist-centre-point where the axes of the three upper actuators intersect, the next two angles can be computed by the triangle denoted in orange:

<div align="center"><img align="center" width="400px" src="images/image027.png"/></div>

Again, there are two solutions representing, one configuration corresponds with a natural pose of the elbow, solution II is a rather unhealthy position:

<div align="center"><img width="600px" src="images/image069.png"/></div>

*a* and *b* is given by the length of the actuators *a<sub>1</sub>* und *d<sub>3</sub>*. So, with cosine law we get the angles *α* and *γ*.

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image072.png"/>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image073.png"/>
	
Finally, we get

<img align="left" src="images/image074.png"/>
<img align="center" src="images/image075.png"/>	

and the second solution 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="images/image076.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image077.png"/>	
	

The upper angles *θ<sub>4</sub>*, *θ<sub>5</sub>*, *θ<sub>5</sub>* can be obtained by considering the chain of transformation matrixes. With

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image078.png"/>	

we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image079.png"/>	


Besides the annoying multiplication <img align="center" src="images/image080.png"/> we only need to consider the rotation part of the homogenous matrixes, translation is no more relevant.

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image081.png"/>	

<img align="center" src="images/image036.png"/> - and therefore the rotation part <img align="center" src="images/image040.png"/> - is already known resp. can be obtained out of the given angles *θ<sub>0</sub>*, *θ<sub>1</sub>*, *θ<sub>2</sub>* by
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image086.png"/>	

By equalizing the previous two equations we get the required angles. First angle that seems to be easily computable is *θ<sub>4</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image088.png"/>	

having two solutions

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image089.png"/>	

For *θ<sub>3</sub>* there is no easy matrix element, but we can combine

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image090.png"/>	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image091.png"/>		

to

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image092.png"/>		

which ends up in

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image093.png"/>		

again having two solutions depending on *θ<sub>4</sub>*. Same is done on *θ<sub>5</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image094.png"/>		


If *θ<sub>4</sub>=0*, we have an infinite number of solutions *θ<sub>3</sub>* and *θ<sub>5</sub>* (gimbal lock). In that case, we consider  <img align="center" src="images/image095.png"/> :		

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image096.png"/>.		

Since we know the trigonometric addition theorem from school

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image097.png"/>		

we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image098.png"/>		

We are free to choose *θ<sub>3</sub>* and take the bot’s current angle *θ<sub>3</sub>* to not move unnecessarily.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image101.png"/>		
 
In the end, we get eight solutions by combining the possible pose configurations of  *θ<sub>0</sub>*(forward/backward), *θ<sub>1</sub>* and *θ<sub>2</sub>*(triangle flip), and *θ<sub>4</sub>*(hand orientation turn).

The correct solution is chosen by taking the one that differs the least from the current bot’s joint angles.

Now we have the trajectory in terms of a sequence of joint angles. This needs to be translated into movements of the motors. This translation should be done in a manner that no motors limits are violated, and with a speed profile that avoids vibrations. Typically, this is done with a speed profile with limited acceleration. I.e. if the required acceleration is higher than an upper limit, the speed is increased with a limited acceleration until the to-be speed is reached.

Additionally, we need to control a feedback loop to ensure that the to-be angle of the motor is actually reached.

All this is done in Walters Cortex, a board based on an 32-bit ARM microcontroller (Teensy 3.5) that receives interpolated trajectory points at 10Hz, and runs a closed loop for the stepper motors at 100Hz. Furthermore, it takes care that no bumpy movements happen by limiting angle, speed, and acceleration.

<img width="700px" src="images/image103.png"/>		

The software runs on the basis of the Arduino library. This is a legacy, since I started with an 8-bit ATmega controller before I upgraded to an Arm processor (This happened when I realized, that controlling 5 steppers and encoders eats up much more computing power than I thought).

<img src="images/image104.png"/>		

The lower part of the layout contains a separate power supply for the servos, the steppers and mC. The mC gets 5V by a standard integrated voltage regulator (7805), the two servos are driven by a 7809 voltage regulator providing 2A (two HerkuleX servos required 1A at most). Additionally, there are two relays for a proper start-up procedure switching the 240W power supply for the steppers and the power supply for the servos. This has been necessary to avoid impulses to the motors inducing annoying ticks when power is switched on.  So, after booting the Trajectory Board and switching on the Cortex Board, all steppers are disabled, then the steppers power supply is turned on, then the servos power supply is switched on. By that procedure, no ticks are happening during starting up.

On the top there is the Arm Cortex M4 controller receiving points to move to and controlling everything low level in a synchronous manner. As usual, tricky part was to provide a stable power supply for all components. I struggled a lot with pikes induced by the strong steppers, AS5048 rotary sensors that are very sensitive to voltage, and UARTs with not exact baud rates.

In the end, I made one board for the power supply providing 24V 10A for the steppers, 5V 4A for the ODroid and the ARM controller, and 9V 2A for Herkulex Servos. Servo’s and Stepper’s power supply are turned on by a relay controlled by the teensy that orchestrates the startup and teardown procedure. The second board on the right has almost no more than the Teensy controller and many sockets.

<img width="700px" src="images/image105.png"/>		

The left part is providing power for uC & Servos, the right part is the ARM M4 controller

Putting all together looks like this:

<img  src="images/image106.jpg"/>		

This is my desk with 5 Pibot drivers, a Teensy 3.5 (on the left bottom side)  and the power supply PCB on the right.
 
## Servos and Steppers 

While controlling robot servos is easy (everything is built in, even a PID controller, they only require a serial interface and a library), stepper motors are more difficult to control. While very tempting by providing high torque without a gearbox and a proper position even without encoders, they turned out to cost me many hours until they moved that smooth as expected.

First approach was the classical feed-back control system taking the encoder angle and the to-be angle of an actuator, giving that to PID controller, computing a correcting angle and giving that to the stepper motor. In the Arduino space, the standard library to control steppers is AccelStepper  which can be used to implement that and leverage from the nice feature of smooth acceleration and deceleration.

This idea was bad. AccelStepper provides the method move(targetposition) that accelerates first and and decelerates afterwards until the motor stops exactly at the targetposition. This works fine for larger moves of several seconds. But in a closed loop, this accelerates and decelerates permanently producing vibrations and – depending on the speed – resonances. Speed was very limited not to mention a very bad sound.

The solution that finally worked was to compute the acceleration that is required in one sample of 10ms, and set this acceleration explicitly for the next sample:

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="images/image107.png"/>		

*v<sub>encoder</sub>*, is the speed that is necessary to compensate the difference between encoder angle and to-be angle. It can be approximated by

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image108.png"/>		
   
Unfortunately, setting the acceleration requires a square root for computing the time until the next step (as done internally in AccelStepper)

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image109.png"/>		

This equation was the reason to decommission the previously used 8-bit Atmega and go to a 32-bit ARM processor with FPU which provides the square root computation in hardware.

The final closed-loop looks like this:
<pre>
void stepperLoop () {
	float dT = sampleTime();                                      // [ms], approx. 10ms
	float currentAngle =                   getEncoderAngle();     // encoder value in [°]
	// interpolate trajectory to get current and next angle
	float toBeAngle =                      movement.getCurrentAngle(millis());
	float nextToBeAngle =                  movement.getCurrentAngle(millis()+dT);
	
	// get speed of current sample, next sample and error compared to encoder’s angle
	float currStepsPerSample =             getMicroStepsByAngle(toBeAngle - lastToBeAngle);
	float nextStepsPerSample =             getMicroStepsByAngle(nextToBeAngle - toBeAngle);
	float stepErrorPerSample =             getMicroStepsByAngle(toBeAngle  - currentAngle);

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

Main components:
	
|Category           |  #| What                                              | From         |
|:------------------|--:|:--------------------------------------------------|:-------------|
|3D Print           | 1| 3D printer (ABS)                                   | Zortrax M200 |
|                   |  | Petroleum, Acetone                                 | Local Dealer |
|Electronics        | 5| Stepper Driver, TB 6600, 4A current                | www.pibot.com, Pibot Stepper Driver 2.2 | 
|                   | 1| Walters Cortex: ARM M4                             | www.pjrc.com, Teensy 3.5 |
|                   | 1| Walters Cerebellum: ARM A15, 2.0Ghz , running Linux| www.hardkernel, Odroid XU4 |
|                   | 5| Magnetic Encoder AMS AS5048B                       | www.digikey.com |
|                   | 1| Power supply for steppers 24V, 10A                 | www.omc-stepperonline.com S-250-24 |
|                   | 1| Power supply for electronics and servos 12V 1A / 5V5A| Meanwell RD35A |
|Motors             | 2| Robot Servos for gripper and hand                  | www.robotshop.com HerkuleX Servo DRS-101 |
|                   | 1| Stepper motor wrist, Nema 17, 42x42x39, 0.40Nm     | www.omc-stepperonline.com  17HS-0404S|
|                   | 1| Stepper motor elbow, Nema 17, 42x42x25, 0.17Nm     | www.omc-stepperonline.com  17HS10-0704S|
|                   | 1| Stepper motor forearm, Nema 24, 60x60x57, 1.9Nm    | Nanotec ST6018M2008|
|                   | 1| Stepper motor upperarm, Nema 24, 60x60x88, 3.1Nm   | Global Biz, GB24H288-40-4A|
|                   | 1| Stepper motor hip, Nema 23, 57x57x56, 1.3Nm        | www.omc-stepperonline.com  23HM22-2804S|
|Belt Drive Forearm | 1| Timing Pulley T2.5, 6mm, 15 Teeth                  | www.cncshop.at |
|                   | 1| Toothed Belt T2.5 6mm, 250mm length                | www.cncshop.at|
|Belt Drive Elbow   | 1| Timing Pulley T2.5, 6mm, 14 Teeth                  |  www.cncshop.at|
|                   | 1| Toothed Belt T2.5 6mm, 145mm length                |  www.cncshop.at|
|                   | 1| Toothed Belt T2.5 6mm, 120mm length                |  www.cncshop.at|
|Belt Drive Upperarm| 1| Timing Pulley T5, 10mm, 14 Teeth                   |  www.cncshop.at|
|                   | 1| Timing Pulley T5, 10mm, 15 Teeth                   |  www.cncshop.at|
|                   | 1| Timing Pulley T5, 10mm, 48 Teeth                   |  www.cncshop.at|
|                   | 1| Toothed Belt T5 10mm, 375mm length                 |  www.cncshop.at|
|                   | 1| Toothed Belt T5 10mm, 430mm length                 |  www.cncshop.at|
|Belt Drive Shoulder| 1| Timing Pulley T5, 10mm, 14 Teeth                   |  www.cncshop.at|
|                   | 1| Timing Pulley T5, 10mm, 14 Teeth                   |  www.cncshop.at|
|                   | 1| Timing Pulley T5, 10mm, 48 Teeth                   |  www.cncshop.at|
|                   | 1| Toothed Belt T5 10mm, 340mm length                 |  www.cncshop.at|
|                   | 1| Toothed Belt T5 10mm, 450mm length                 |  www.cncshop.at|
|Belt Drive Hip     | 1| Timing Pulley T5 10mm, 10 Teeth                    |  www.cncshop.at|
|                   | 1| Toothed Belt T5 10mm, 510mm length                 |  www.cncshop.at|
|Gripper Bearings   |16| 3x7x3m                                             | www.kugellager-express.de |
|Wrist Bearings     | 1| DIN 625 SKF - 61807 35x47x7mm                      | www.kugellager-express.de |
|                   | 1| DIN 625 SKF - 61807 35x44x5mm                      | www.kugellager-express.de |
|                   | 1| DIN 625 SKF - 61902 15x28x7mm                      | www.kugellager-express.de |
|Forearm Bearings   | 2| 3x10x4mm                                           | www.kugellager-express.de |
|Elbow   Bearings   | 2| DIN 625 SKF - 61807 35x47x7mm                      | www.kugellager-express.de |
|                   | 2| 6x19x6                                             | www.kugellager-express.de |
|                   | 4| 3x10x4                                             | www.kugellager-express.de |
|Upperarm Bearings  | 2| DIN 625 SKF - 61807 35x47x7mm                      | www.kugellager-express.de |
|                   |16| 4x13x5                                             | www.kugellager-express.de |
|                   | 2| 8x22x7                                             | www.kugellager-express.de |
|Shoulder Bearings  | 6| 4x13x5                                             | www.kugellager-express.de |
|                   | 2| 8x22x7                                             | www.kugellager-express.de |
|                   | 2| DIN 625 SKF - SKF 61818 - 80x100x10                | www.kugellager-express.de |
|Hip Bearings       | 1| DIN 625 SKF - SKF 61818 - 90x115x13                | www.kugellager-express.de |
|                   | 6| 4x13x5                                             | www.kugellager-express.de |
|Housing Bearings   |16| 3x8x3                                              | www.kugellager-express.de |

Note: all bearings are deep groove ball bearing, dimensions are given in diameter inside/diameter outside/width in mm.

Welcome to Walters **Gallery**!

<table>
    <tr valign="top">
        <td width="25%">Gripper<br><a href="galery/gripper1.jpg"><img width="133" src="galery/gripper1.jpg"></a></td>
        <td width="25%">Gripper<br><a href="galery/gripper2.jpg"><img width="133" src="galery/gripper2.jpg"></a></td>
        <td width="25%">Gripper<br><a href="galery/gripper3.jpg"><img width="133" src="galery/gripper3.jpg"></a></td>
    </tr><tr valign="top">
        <td width="25%">Gripper/Wrist<br><a href="galery/gripperwrist1.jpg"><img width="133" src="galery/gripperwrist1.jpg">
</a></td>
        <td width="25%">Gripper/Wrist<br><a href="galery/gripperwrist2.jpg"><img width="133" src="galery/gripperwrist2.jpg">
        <td width="25%">Wrist<br><a href="galery/wrist1.jpg"><img width="133" src="galery/wrist1.jpg">
</a></td>
