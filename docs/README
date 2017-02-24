<img align="right" width="100px" src="images/image002.jpg" >
Since man built the first robot, he wanted it to look like himself. I am way above being young, so my goal was to make him look vintage. But, movements should be smooth, and – rather a matter of personal taste – I do not like visible humps or bulges with motors or gearboxes, nor horrible cables hanging everywhere. Reminds me too much of my dentist. All stuff should be inside the enclosure. 

After checking youtube, I recognized that just a few DIY robots are close to what I had in mind. There's the construction by Andreas Hölldorfer ([Printable Robot Arm](https://hackaday.io/project/3800-3d-printable-robot-arm)), which got covered recently ("Moveo"). Unfortunately without mentioning the obvious inspiration coming from Andreas. I got lots of ideas from his construction, and there are still a couple of parts directly derived from his design.

Another nice construction called [Thor](https://hackaday.io/project/12989-thor) is coming from Ángel Larrañaga Muro which has an interesting differential gearbox for the lower actuators.

<img align="left" width="30%" src="videos/logo-animated.gif" >

This is what I had in mind. Most of the DIY robots are using servos, mostly for convenience, since the encoder is built-in already and they are easy to control. Thing is, when it comes to higher torque, the connection of the servo with the actuator becomes difficult, and hard to make of 3D printed material. If the servo or the flange moves just a little bit within the housing, the according play becomes an issue. An the required precision to avoid this is way above hobby grade components. 

And servos are boringly easy to use. No fun in construction. A motor with a belt drive and a separate angle sensor solves this, it provides low backlash and allows the electronics to compensate imprecise parts with the sensor placed separately from the motor. Additionally, the motor of an actuator can be placed in the previous joint, lowering the centre of gravity of each actuator.

When a belt drive is set, choice comes naturally to stepper motors, since an additional gearbox is not necessary anymore, torque is high and the belts should compensate the vibrations coming from the stepping nature of these motors.

Software is also not easy. Most robot makers stop as soon as limps move, what makes these robots look diy. Making the full stack up to trajectory planning means four months of work when you have weekends only, of which some parts were really difficult (inverse kinematics). The data flow of the full stack looks like this:

<img width="600px" src="images/image013.png"/>

We have:
* [Trajectory Planner](https://github.com/jochenalt/Walter/wiki/Trajectory) ([source code](https://github.com/jochenalt/Walter/tree/master/code/WalterPlanner))
This is a UI for planning trajectories. All animated gifs in this wiki are made with it. Trajectories are planned by defining single support. After planning is done, the trajectory is transfered to the

* [Trajectory Execution](https://github.com/jochenalt/Walter/wiki/Webserver) ([source code](https://github.com/jochenalt/Walter/tree/master/code/WalterServer)). This component consists of a webserver that runs trajectories by interpolating Bézier curves betweeen support points, computing the inverse [Kinematics](https://github.com/jochenalt/Walter/wiki/Kinematics) per pose and sends the resulting series of angles to the 

* [Cortex](https://github.com/jochenalt/Walter/wiki/Cortex) ([source code](https://github.com/jochenalt/Walter/tree/master/code/BotCortex)). This low level component takes interpolated poses and controls the actuators accordingly by applying control algorithms ([PID controller](https://en.wikipedia.org/wiki/PID_controller)). Servos are controlled directly by the cortex controller board  via a serial interface. Steppers do not have an internal feedback loop, so we need rotary encoders detecting the absolute angle of the joint and allowing to implement feedback controllers.
<img align="center" width="800px" src="images/image014.png"/>

On the mechanical side, we have two actuators driven by a servo (mainly due to space restrictions) and four actuators driven by a stepper/rotary encoder combination. Details are shown in [Construction](https://github.com/jochenalt/Walter/wiki/Construction).

Steppers are driven by retail stepper drivers (PiBot Stepper Driver) around the popular PWM stepper driver Toshiba 6600 (4.5A max). The stepper drivers are directly connected to the Cortex. It receives joint angles at 10Hz, interpolates the points in between, and sends the according PWM signal to the stepper drivers and to the servos. Besides micro interpolation of the trajectory, the controller board takes care of the speed profile, i.e. it limits the acceleration and speed of each actuator. The controller board is a DIY board around an ARM Cortex M4 (Teensy 3.5), running the control loop with 100 Hz. I started with an ATmega 644 8-bit controller, but it turned out that the ATmega was not able to control 5 steppers with a proper sample rate, let alone reading 5 encoders on top. 

The trajectory controller board is encapsulated by a webserver exposing the current movement and accepting commands like new trajectories.


# Construction
Inverse kinematics, i.e. computation of joint angles out of the gripper’s position can be hard. If the design is too playful, nearly impossible complex. So, it is a good idea to ease the maths by having the upper three axes joining in one point. Later in the chapter Kinematics we will see that with that limitation kinematics becomes possible without having a mathematician at hand (still not easy, but feasible). 

## Dimensioning 

Before starting the design of the construction it is necessary to calculate the torque in all actuators in order to select the steppers and gear ratios properly. In principle, this is simple: The gripper should be able to manipulate 500gr, the forarm has a length of 400mm and the upperarm a length of 350mm, assuming a certain weight per actuator the law of the lever allows to compute the torque of each motor. Tricky part is, that depending on the stepper you choose, the weight of an actuator changes.  In the end I did that computation with excel, and came to these torques 
and gear ratios, which are used to select the stepper dimensions.

| Actuator | ActuatorTorque   | Gear Ratio | Min Stepper Torque | Stepper Size     | Act. Stepper Torque | 
|--------- |------------------| ---------- | ------------------ | ---------------- | ------------------- |
| Wrist    | 0.6 Nm           |  1:4       | 0.18 Nm            | NEMA 17 42x42x39 | 0.4Nm               |
| Elbow    | 0.6 Nm           |  1:7       | 0.11 Nm            | NEMA 17 42x42x25 | 0.17Nm              |
| Forearm  | 11 Nm            |  1:14      | 0.8 Nm             | NEMA 24 60x60x57 | 1.9Nm               |
| Upperarm | 34 Nm            |  1:18      | 1.8 Nm             | NEMA 24 60x60x87 | 3 Nm                |
| Hip      | 8 Nm             |  1:9       | 0.9 Nm             | NEMA 23 57x57x56 | 1.2 Nm              |

The steppers are placed in the previous actuator of the moved actuator in order to move the centre of gravity away from the biggest lever. So, the three  heavy steppers actually do not move when the arm goes up or down.

## Design Patterns

With this amount of torque, a shaft-hub joints need to be really stable. While the small steppers have the pulley connected with grub screws, the big ones need something different. Although a lot of filing is involved, I went with feather keys for the middle shaft of gearboxes and the connection between stepper motor and timing pulley.

<img align="left" width="300px" src="images/cad-shaft-hub-joint.png" >
<img width="200px" src="images/image012.png" >


## Gripper

Due to space limitations, it seems to be appropriate to use a servo for the gripper. I used a standard design principle where one lever is driven, and the other lever mirrors the movement by a gear wheel. The servo is hidden in a small box, it is a HerkuleX Robot Servo with 0.12 Nm.

<img width="500px" src="images/cad-gripper.png" >

<img align="left" width="200px" src="images/IMG_20170219_111144_cr.png" >

<img align="right" width="300px" src="images/IMG_20170219_105428.jpg" >

Assembly has to start from the top, not for mechanical reasons, but due to the cables that are all placed inside the robot going down from the gripper to the base.
The gripper has bearings in all moving parts. The left gearwheel has the servo behind, mounted with the encloded servo disk. To increase stability, the hole over the servo screw is used for another bearing to lock this lever from both sides in its position.

<img align="left" width="170px" src="images/IMG_20170219_152759.png" >
The servo's cable is going through the servo housing into the flange where the wrist will be placed.

I'm still not really happy with the gripper. Although it works fine, the bulge containing the servo is really ugly. But, the space below the gripper is already occupied by the servo turning the wrist, unfortunately. I played with other design types, but always came back to this one due to its simplicity.

## Wrist

The wrist is also designed with the same servo. A small flange connects the wrist with the two halves of the gripper housing, the hole hides the cable of the gripper servo. Worth to mention is that the bearings of the wrist have a different size, since the servo looks through the inner hole of the bigger bearing. On the other side, in the middle of the smaller bearing there is the hole for the magnet used by the magnetic encoder of the forearm. The cable of both servos (gripper and wrist) is going through the wrist underneath the servo.

<img align width="800px" src="images/cad-wrist.png" >

## Forearm

The forearm is more complex, the wrist ist driven with a belt drive and a stepper motor with an gear ratio of 1:4. The belt drive is hold tight with a spanner. At the other side of the wrist, the magnetic encoder is located. All cables are meeting in the space at the bottom of the forearm, and going down through the hole of the disk.

<img align width="800px" src="images/cad-forearm.png" >

## Elbow

The elbow consumed most time for design, it is a two stage belt-drive with a ratio of 1:7 and a stepper with 17Ncm. The flange in the middle is the connection to the forearm. It is mounted with two  bigger bearings and has a cable channel with space for a self made cable drag chain that allows to have the cables inside. This was difficult since the centre of the flange was already occupied by an magnetic encoder.

<img align width="600px" src="images/cad-elbow.png" >

## Upperarm

The upperarm contains a strong stepper with 1.9Nm and a two-staged gear with a ratio of 1:14. On the left side a magnetic encoder samples the angle of the ellbow, above the encoder the cable channel is located. The cables are going down through a hole in the middle block down to the left side of the bttom part. The ride side contains the belt to the elbow. All belts are tighened with a clamp that can be adjusted from outside.

<img align width="800px" src="images/cad-upperarm.png" >

## Shoulder

The shoulder contains the strongest stepper moving the upperarm with approx. 3 Nm. A gear ratio of 1:18 could deliver 50Nm, but this number is rather theoretical, since 3D-printed parts would not survive this. But, this allows to reduce the current and use intense micro-stepping improving the movement.

On the ## left flange, there is a segment-shaped cable channel below the location of the magnetic encoder. The right flange has a big hole to make room for the stepper's backside. This is hidden by a lid that rotates with the upperarm, which gives a nice technical touch. Inside the middle block between the flanges, there is a shaft with two drive pulleys for the two-staged gearbox, same construction as in the upperarm.

<img align width="800px" src="images/cad-shoulder.png" >

## Hip

Finally, the hip stepper is what makes the housing of the shoulder look like an iglu. It is a simple belt drive to the shoulder. The shoulder is residing on a drive pulley disk that is mounted on a big bearing.

<img align width="800px" src="images/cad-hip.png" >

## Housing

The housing of the shoulder is not only to hide the hip stepper, but also to stabilize the shoulder by having lots of small bearings on the top edge supporting the shoulder.

<img align width="800px" src="images/cad-housing.png" >


#Trajectory

Planning a trajectory means defining a sequence of poses in 3D space. These defined poses are interpolated in order to result in a smooth and continuous curve. Most beautiful are cubic Bézier curves.

Bézier curves are polynoms of 3rd grade using a start and an end point and two support points defining the curvature at the start and end point. The trajectory is defined by the start and the end point, the support point is not on the trajectory but used to make it smooth only. The computation is based on a parameter *t=0..1* defining the ratio of how much the current position has already made of the full curve. Let’s assume, we have the four points *P<sub>0</sub>..P<Sub>3</sub>*, of which *P<sub>1</sub>* and *P<sub>2</sub>* are support points the curve does not touch.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image015.png"/> 

This computation is done for *x*, *y*, and *z* coordinates. Normally being beautiful, but Bézier curves have a tendency to “bounce”, if the support points *P<sub>1</sub>* and *P<sub>2</sub>* differ too much in terms of the distance to the trajectory points *P<sub>0</sub>* and *P<sub>3</sub>*. So, it is necessary to normalize support points by a small trick:

The picture illustrates a trajectory defined by *A*, *B*, *C*, and *D*. We want to model the piece between *B* and *C* with a cubic Bézier curve.

<img align="left" width="450px" src="images/image016.png"/>

The support point *B’* is computed by taking point *A*, mirroring it at *B* (*A’*), and moving along the angle bisector of *A’* *B* *C* by a 3<sup>rd</sup> of the length of *BC*, *C’* is computed in an analogous manner.

This approach is rather arbitrary, but results in a smooth and non-oscillating curve. On the left, we see a linear trajectory, on the right the same curve as bezier curve. All this is implemented in [BezierCurve.cpp](https://github.com/jochenalt/Walter/blob/master/code/WalterKinematics/src/BezierCurve.cpp).

<img align="left" width="300px" src="videos/linear interpolated curve.gif"/>
<img width="300px" src="videos/bezier curve.gif"/>

Now the curve looks fine, but simply following that curve is not enough to make a smooth movement. We need a speed profile that avoids jerky movements. This is done by speed profiles. The classical approach is to use trapezoidal speed profiles like this:

<img align="left" width="320px" src="images/image017.png"/>
<img width="320px" src="images/image018.png"/>

This trapezoid speed profile results in a constant (maximum) acceleration, then continuing with no acceleration, and a constant deceleration until zero speed is reached. To get the position on a curve, speed is integrated over time 

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

On the right, the effect of a speed profle is illustrated compared to the same trajectory without speed profile. The effect is not spectacular, but can be detected when watching the edges: the movement stops slowly and accelerates when it continues. Speed profiles are implemented in [SpeedProfile.cpp](https://github.com/jochenalt/Walter/blob/master/code/WalterKinematics/src/SpeedProfile.cpp).


<img align="left" width="300px" src="videos/without speed profile.gif"/>
<img width="310px" src="videos/with speed profile.gif"/>

All this is done while planning a trajectory, so that is a task done upfront. At runtime, the trajectory is fully compiled and contains bezier curves and speed profiles already. The according UI where trajectories are planned looks like this (source code is [WalterPlanner](https://github.com/jochenalt/Walter/blob/master/code/WalterPlanner))

<img align="center" src="images/planner-screenshot.png"/>

This UI provides forward and inverse kinematics (explained in [Kinematics](https://github.com/jochenalt/Walter/wiki/Kinematics)), allows to define a trajectory by defining support points (indicated by big green balls). After having assigned some parameters like speed or duration the trajectory is compiled, i.e. the bezier curves and speed profiles are computed (indicated by small green balls). This trajectory can be send to Walter to be executed by its Cortex. 

Now we have a nice trajectory with a smooth speed profile, but do not yet know how to compute the angles of the joints. 

# Kinematics 
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

| Joint      | a[°]  | a[mm]            | d[mm]           |
|----------  | ------| ---------------- | --------------- |
| Hip        | *-90°*| *0*              | *d<sub>0</sub>* |
| Upperarm   | *0*   | *a<sub>1</sub>*  | *0*             |
| Forearm    | *-90°*| *0*              | *0*             |
| Elbow      | *90°* | *0*              | *d<sub>3</sub>* |
| Wrist      | *-90°*| *0*              | *0*             |
| Hand       | *0*   | *0*              | *d<sub>5</sub>* |

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

Note: Unfortunately, the gripper’s coordinate system is not appropriate for human interaction, since the default position as illustrated in the [Coordinate Systems](images/image027.png) is not nick/roll/yaw=(0,0,0). So, in the Trajectory Visualizer it is handy to rotate the gripper matrix such that the default position becomes . The according rotation matrix represents a rotation of -90° along *x*,*y*, and *z*, done by the rotation matrix

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

Again, there are two solutions (aka configurations), one configuration corresponds with a natural pose of the elbow, solution II is a rather unhealthy position:

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

To ease the annoying multiplication <img align="center" src="images/image080.png"/> we only need to consider the rotation part of the homogenous matrixes, translation is no more relevant, since the orientation alone defines the three upper angles. 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image081.png"/>	

<img align="center" src="images/image036.png"/> - and therefore the rotation part <img align="center" src="images/image040.png"/> - is already known resp. can be obtained out of the given angles *θ<sub>0</sub>*, *θ<sub>1</sub>*, *θ<sub>2</sub>* by
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image086.png"/>	

By equalizing the previous two equations we get a bunch of equations defining the upper angles, still these equations are hard to solve, due to the complex combination of trignometric functions. But, there are some equations which are simpler and can be solved for an angle. First angle that seems to be easily computable is *θ<sub>4</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="images/image088.png"/>	

gives two solutions

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

We are free to choose *θ<sub>3</sub>* and arbitrarily select the bot’s current angle *θ<sub>3</sub>*, such that this one will not move in that specific case.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image101.png"/>		
 
In the end, we get eight solutions by combining the possible pose configurations of  *θ<sub>0</sub>*(forward/backward), *θ<sub>1</sub>* and *θ<sub>2</sub>*(triangle flip), and *θ<sub>4</sub>*(hand orientation turn).

The correct solution is chosen by taking the one that differs the least from the current bot’s joint angles. 
What is with all the other equations from <img align="center" src="images/image040.png"/>? We could use them to invalidate some of the four solutions, but still there will remain a list of solutions we need to choose from. So, it does not really make sense to take this route.

The selection algorithm is quite simple:

1. consider only mechanically valid solutions, i.e. omit those that violate mechanical boundaries.

2. Compute an angle-wise "*distance*" to the current position and take the solution with the least distance, i.e. take the solution with the least movement.

The latter has the consequence that the pose will try to remain in one configuration and no sudden movements like a turn of 180° happens. All this is implemented in [Kinematics.cpp](https://github.com/jochenalt/Walter/blob/master/code/WalterKinematics/src/Kinematics.cpp).


Speaking of configurations: if you want to change the configuration of the bot, e.g. from elbow down to elbow up which includes turning the base by 180° the approach of having a linear movement from one pose to the other and interplating in between does not work anymore, since the poses are identical.

<img width="500px" src="images/different configurations.png"/>

First possibility to change configuration is to have a non-linear movement that interpolates angle-wise resulting in a weired movement that would be really dangerous in a real world:

<img width="500px" src="videos/angle-wise configuration change.gif">

So, you rather introduce an intermediate pose where you can safely turn the critical actuator (mostly the shoulder). This is not less expansing, but a more controlled way:

<img width="500px" src="videos/singularity configuration change.gif">


Thing is, that one has to go via a singularity (the upright position) which is normally avoided like hell. One reason is the way how we compute *θ<sub>4</sub>=0*, and a numerical reason that singularities are kind of a black hole, the closer you get, the more you are sucked into imprecisions of floating numbers since you approach poles of the underlying functions.

# Webserver

Trajectory Execution is done in a webserver that accepts a trajectory from the [Trajectory Planner](https://github.com/jochenalt/Walter/wiki/Trajectory). The sent trajectory is *compiled* already, i.e. all intermediate points with a sample rate of 10Hz have been computed, speed profile is applied and inverse [kinematics](https://github.com/jochenalt/Walter/wiki/Kinematics) has been precomputed.

So, the webservers receives a long list of interpolated poses including all actuator angles and all the timing information.
The webserver has the simple job of running the trajectors by sending the interpolated points with the correct timing to the [Cortex](https://github.com/jochenalt/Walter/wiki/Cortex). It is implemented with [Mongoose](https://www.cesanta.com). 

In addition to its main task, it provides a webpage for debugging purposes, where direct commands to the cortex can be entered. Since the Cortex does not unterstand poses but expects actuator angles, the webserver is able to run the kinematics as well, especially during startup and teardown, when the Walter will move to its default position.

This webpage is done with [Webix](http://webix.com), a small JS-Framework and implemented in [index.html](https://github.com/jochenalt/Walter/blob/master/code/WalterServer/web_root/index.html). The Webserver can be found [here](https://github.com/jochenalt/Walter/tree/master/code/WalterServer).


<img width="1000" align="center" src="images/website.png" >

# Cortex

Assuming that we have the trajectory defined in terms of a sequence of joint angles, we still need to translate that into movements of the motors. This translation should be done in a manner that no motors limits are violated, and with a speed profile that avoids vibrations by limiting the acceleration. 

Additionally, we need a feedback loop to ensure that the to-be angle of the motor is actually reached.

All this is done in Walters Cortex, a board based on an 32-bit ARM microcontroller (Teensy 3.5) that receives interpolated trajectory points at 10Hz, and runs a closed loop for the stepper motors at 100Hz. In that closed loop, the encoders are sampled reading the angle of each actuator with a precision of 14-bit = 0.02° generateing stepper impules that follow the trajectory. Furthermore, it takes care that no bumpy movements happen by limiting angle, speed, and acceleration. This should have happened during trajectory planning already, but you never know.

<img width="700px" src="images/image103.png"/>		

## Schematics

<img src="images/image104.png"/>		

The lower part of the layout contains a separate power supply for the servos, the steppers and mC. The mC gets 5V by a standard integrated voltage regulator (7805), the two servos are driven by a 7809 voltage regulator providing 2A (two HerkuleX servos required 1A at most). Additionally, there are two relays for a proper start-up procedure switching the 240W power supply for the steppers and the power supply for the servos. This has been necessary to avoid impulses to the motors inducing annoying ticks when power is switched on.  So, after booting the Trajectory Board and switching on the Cortex, all steppers are disabled, afterwards the steppers power supply is turned on, then the servos power supply is switched on. By that procedure, no ticks are happening during starting up.

On top there is the Arm Cortex M4 controller. As usual, tricky part was to provide a stable power supply for all components. I struggled a lot with pikes induced by the strong steppers; particularily the magentic encoders respond very sensitive to voltage.

So, I made a separate board providing 24V/10A for the steppers (kind of overpowered, 6A would have been sufficient as well), 5V 4A for the Odroid XU4 and the Teensy 3.5, and 9V 2A for Herkulex servos. Servo’s and stepper’s power supply are turned on by a relay. 

This is the PCB layout, which turned out to be rather simple. Especially the Teensy part consists more or less of sockets only:

<img width="700px" src="images/image105.png"/>		

Putting all together looks like this:

<img  src="images/image106.jpg"/>		

This is my desk with 5 Pibot drivers, a Teensy 3.5 (on the left bottom side)  and the power supply PCB on the right. The wooden board is the backside of the [control cabinet](https://github.com/jochenalt/Walter/wiki/Control Cabinet)

## Sensors

The used sensors are AMS' magnetic encoders 5048B with 14-bit resolution and an I2C interface. They are driven 3.3 V on two I2C buses. Walter has no electronics in its base, all is connected via a long cable of approx. 1.5m. In general, this is a bad idea, since the capacity of that cable is alone is around 250pF. So, I had to use very small pullup resistors of 1.1K&#x2126; for both I2C lines (see also [Computation of I2C pullup resistors](http://www.ti.com/lit/an/slva689/slva689.pdf) to get that stable. Minimum is around 966&#x2126; to not exceed 3mA in SDA/SCl lines.

## Software 
The software of the Cortex runs on the basis of the Arduino library. This is a legacy, since I started with an 8-bit ATmega controller before I upgraded to an Arm processor (This happened when I realized, that controlling 5 steppers and encoders eats up much more computing power than I thought).
 
## Steppers

While controlling robot servos is easy (everything is built in, even a PID controller, they only require a serial interface and a library), stepper motors are more difficult to control. While very tempting by providing high torque without a gearbox and a proper position even without encoders, they turned out to cost me many hours until they moved that smooth as expected.

First approach was the classical feed-back control system taking the encoder angle and the to-be angle of an actuator, giving that to PID controller, computing a correcting angle and giving that to the stepper motor. In the Arduino space, the standard library to control steppers is AccelStepper  which can be used to implement that and leverage from the nice feature of smooth acceleration and deceleration.

This idea was bad. AccelStepper provides the method move(targetposition) that accelerates first and decelerates afterwards until the motor stops exactly at the targetposition. This works fine for larger moves of several seconds. But in a closed loop, this permanent acceleration and deceleration produced vibrations ending up in resonances. Speed was very limited not to mention a nasty sound.

The solution that finally worked was to compute the acceleration that is required in one sample of 10ms, and set this acceleration explicitly for the next sample:

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="images/image107.png"/>		

*v<sub>encoder</sub>* is the speed that is necessary to compensate lost steps. It can be approximated by

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image108.png"/>		
   
Unfortunately, setting the acceleration requires a square root for computing the time until the next step (as done internally in AccelStepper)

&nbsp;&nbsp;&nbsp;&nbsp;<img src="images/image109.png"/>		

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
Source code is in [WalterCortex](https://github.com/jochenalt/Walter/tree/master/code/BotCortex), control loop above can be found in [GearedStepperDriver.cpp](https://github.com/jochenalt/Walter/blob/master/code/BotCortex/GearedStepperDrive.cpp).

#Control Cabinet

<img width="800px" src="images/IMG_20170219_125046.jpg" >

Here we see the control cabinet in its natural environment, accompanied by a japanese flower arrangement. The inners is a scaffold for the power supply, the stepper drivers, and some smaller PCBs.

<img align="left" width="300px" src="images/cad-cabinet 1.png" >
<img width="370px" src="images/cad-cabinet 2.png" >

I tried to give it a GDR-vintage-style, mostly by having these great round panel meters (I got via ebay from China) and a panel of mostly useless lamps. 

<img align="left" width="240px" src="images/IMG_20170219_125144.jpg" >
<img align="right" width="300px" src="images/WP_20170122_00_55_52_Pro.jpg" >

All sockets are on the left side of the wooden box, biggest socket is for the fat 28-pin cable containing lines for the steppers, encoders and servos. Above there are two USB ports, one for the serial interface of the Odroid XU4, which is handy if Wifi is not working, and one for the Teensy (Walter's Cortex). Power supplies are fixed on the backside with screws, the scuffold for electronics is glued to the back side. The picture on the right shows the tidy version of the inners with power supplies and stepper drivers only. When the other PCBs are added and all the cabeling is done, you better close the box. On top of the box a nice emergency stop button is placed.


# Bill of Material 
This is the list of main components. Besides that, a lot of incidentals (screws etc.) have been necessary

|Category           |  #| What                                              | From         |
|:------------------|--:|:--------------------------------------------------|:-------------|
|3D Print           | 1 | 3D printer (ABS)                                   | Zortrax M200 |
|                   |   | Petroleum, Acetone                                 | Local Dealer |
|Electronics        | 5 | Stepper Driver, TB 6600, 4A current                | Pibot Stepper Driver 2.2, www.pibot.com | 
|                   | 1 | Walters Cortex: ARM M4                             | Teensy 3.5, www.pjrc.com |
|                   | 1 | Walters Cerebellum: ARM A15, 2.0Ghz , running Linux| Odroid XU4, www.hardkernel.com |
|                   | 5 | Magnetic Encoder                                   | AMS AS5048B, www.digikey.com |
|                   | 1 | Power supply for steppers 24V, 10A                 | S-250-24, www.omc-stepperonline.com |
|                   | 1 | Power supply for electronics and servos 12V1A/5V5A | Meanwell RD35A |
|Motors             | 2 | Robot Servos for gripper and hand                  | HerkuleX Servo DRS-101, www.robotshop.com |
|                   | 1 | Stepper motor wrist, Nema 17, 42x42x39, 0.40Nm     | 17HS-0404S, www.omc-stepperonline.com|
|                   | 1 | Stepper motor elbow, Nema 17, 42x42x25, 0.17Nm     | 17HS10-0704S, www.omc-stepperonline.com  |
|                   | 1 | Stepper motor forearm, Nema 24, 60x60x57, 1.9Nm    | Nanotec ST6018M2008|
|                   | 1 | Stepper motor upperarm, Nema 24, 60x60x88, 3.1Nm   | Global Biz, GB24H288-40-4A|
|                   | 1 | Stepper motor hip, Nema 23, 57x57x56, 1.3Nm        | 23HM22-2804S, www.omc-stepperonline.com  |
|Belt Drive Forearm | 1 | Timing Pulley T2.5, 6mm, 15 Teeth                  | www.cncshop.at |
|                   | 1 | Toothed Belt T2.5 6mm, 250mm length                | www.cncshop.at|
|Belt Drive Elbow   | 1 | Timing Pulley T2.5, 6mm, 14 Teeth                  | www.cncshop.at|
|                   | 1 | Toothed Belt T2.5 6mm, 145mm length                | www.cncshop.at|
|                   | 1 | Toothed Belt T2.5 6mm, 120mm length                |  www.cncshop.at|
|Belt Drive Upperarm| 1 | Timing Pulley T5, 10mm, 14 Teeth                   |  www.cncshop.at|
|                   | 1 | Timing Pulley T5, 10mm, 15 Teeth                   |  www.cncshop.at|
|                   | 1 | Timing Pulley T5, 10mm, 48 Teeth                   |  www.cncshop.at|
|                   | 1 | Toothed Belt T5 10mm, 375mm length                 |  www.cncshop.at|
|                   | 1 | Toothed Belt T5 10mm, 430mm length                 |  www.cncshop.at|
|Belt Drive Shoulder| 1 | Timing Pulley T5, 10mm, 14 Teeth                   |  www.cncshop.at|
|                   | 1 | Timing Pulley T5, 10mm, 14 Teeth                   |  www.cncshop.at|
|                   | 1 | Timing Pulley T5, 10mm, 48 Teeth                   |  www.cncshop.at|
|                   | 1 | Toothed Belt T5 10mm, 340mm length                 |  www.cncshop.at|
|                   | 1 | Toothed Belt T5 10mm, 450mm length                 |  www.cncshop.at|
|Belt Drive Hip     | 1 | Timing Pulley T5 10mm, 10 Teeth                    |  www.cncshop.at|
|                   | 1 | Toothed Belt T5 10mm, 510mm length                 |  www.cncshop.at|
|Gripper Bearings   |16 | 3x7x3m                                             | www.kugellager-express.de |
|Wrist Bearings     | 1 | DIN 625 SKF - 61807 35x47x7mm                      | www.kugellager-express.de |
|                   | 1 | DIN 625 SKF - 61807 35x44x5mm                      | www.kugellager-express.de |
|                   | 1 | DIN 625 SKF - 61902 15x28x7mm                      | www.kugellager-express.de |
|Forearm Bearings   | 2 | 3x10x4mm                                           | www.kugellager-express.de |
|Elbow   Bearings   | 2 | DIN 625 SKF - 61807 35x47x7mm                      | www.kugellager-express.de |
|                   | 2 | 6x19x6                                             | www.kugellager-express.de |
|                   | 4 | 3x10x4                                             | www.kugellager-express.de |
|Upperarm Bearings  | 2 | DIN 625 SKF - 61807 35x47x7mm                      | www.kugellager-express.de |
|                   |16 | 4x13x5                                             | www.kugellager-express.de |
|                   | 2 | 8x22x7                                             | www.kugellager-express.de |
|Shoulder Bearings  | 6 | 4x13x5                                             | www.kugellager-express.de |
|                   | 2 | 8x22x7                                             | www.kugellager-express.de |
|                   | 2 | DIN 625 SKF - SKF 61818 - 80x100x10                | www.kugellager-express.de |
|Hip Bearings       | 1 | DIN 625 SKF - SKF 61818 - 90x115x13                | www.kugellager-express.de |
|                   | 6 | 4x13x5                                             | www.kugellager-express.de |
|Housing Bearings   |16 | 3x8x3                                              | www.kugellager-express.de | 

Note: all bearings are deep groove ball bearing, dimensions are given in diameter inside/diameter outside/width in mm.


# Gallery

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
   </tr><tr valign="top">
        <td width="25%">Forearm rotary encoder<br><a href="galery/forearm 1.png"><img width="133" src="galery/forearm 1.png">
</a></td>
        <td width="25%">Forearm inside<br><a href="galery/forearm 2.png"><img width="100" src="galery/forearm 2.png">
</a></td>
        <td width="25%">Forearm outside<br><a href="galery/forearm 3.png"><img width="133" src="galery/forearm 3.png">
</a></td>
   </tr>
   <tr valign="top">
        <td width="25%">Elbow flange with magnet for encoder<br><a href="galery/elbow1.png"><img width="133" src="galery/elbow1.png">
</a></td>
        <td width="25%">Elbow inside<br><a href="galery/elbow1.png"><img width="100" src="galery/elbow2.png">
</a></td>
        <td width="25%">Elbow gearbox<br><a href="galery/elbow3.png"><img width="133" src="galery/elbow3.png">
</a></td>
   </tr>
</table>
