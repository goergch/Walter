Kinematics is about computation of the tool-centre-point (*TCP*) out of joint angles and vice versa. First is simple, latter is more tricky, but lets see later on.
But before starting any kinematics, it is necessary to define all coordinate systems.

<img width="400px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image027.png"/>

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

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image029.png"/>

which is a homogeneous matrix with two rotations *(x,z)* and two translations *(x,z)*.

Combined with the DH parameters, the following DH matrixes define the transformation from one joint to its successor:

<img  align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image030.png"/>

<img  align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image031.png"/>

<img   src="https://github.com/jochenalt/Walter/blob/master/docs/images/image032.png"/>

<img  align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image033.png"/>

<img  align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image034.png"/>

<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image035.png"/>

## Forward Kinematics

With the DH transformation matrixes at hand, computation of the bot’s pose out of the joint angles is straight forward. The matrix representing the gripper’s pose <img align="center"  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image036.png"/> is 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image037.png"/> 

By multiplying the transformation matrix with the origin (as homogeneous vector), we get the absolute coordinates of the tool centre point in world coordinate system (i.e. relative to the bot’s base).

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image038.png"/>

The orientation in terms of roll/nick/yaw of the tool centre point can be derived out of <img align="center"  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image036.png"/>by taking the part representing the rotation matrix (<img align="center"  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image040.png"/>). ([Wikipedia Roll/Nick/Yaw](https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel#Berechnung_aus_Rotationsmatrix) )

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image041.png"/>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image042.png"/>  
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image043.png"/>  
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image044.png"/>  

For <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image045.png"/> we have a singularity (*atan2* never becomes this), but wikipedia has a solution for that as well
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image046.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image047.png"/>

if <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image048.png"/> we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image046.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image049.png"/>

Note: Unfortunately, the gripper’s coordinate system is not appropriate for human interaction, since the default position as illustrated in the <img align="center" src=":https://github.com/jochenalt/Walter/blob/master/docs/images/image027.png|Coordinate Systems"/> not nick/roll/yaw=(0,0,0). So, in the Trajectory Visualizer it is handy to rotate the gripper matrix such that the default position becomes . The according rotation matrix represents a rotation of -90° along *x*,*y*, and *z*, done by the rotation matrix

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image051.png"/>

In the following equations, this is not considered, since it is for convenience in the UI only.

## Inverse Kinematics 
Inverse kinematics denotes the computation of all joint angles out of the tool-centre-point’s position and orientation. In general it is hard to give non-numeric solution, in our case it is possible since the upper three joint angles intersect in the WCP.

Input of inverse kinematics is the TCP’s position and orientation in terms of roll, nick, yaw, abbreviated by *γ*, *β*,and *α*.

<img align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image053.png"/>

<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image054.png"/>

First, we need to compute the wrist-centre-point out the tool-centre-point. This is possible by taking the TCP and moving it back along the TCP’s orientation by the hand length. For doing so, we need the transformation matrix from the base to the last joint <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image036.png"/> 
which we can derive out of the TCP’s position and orientation.

To build the transformation matrix <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image036.png"/> we need the rotation matrix defining the orientation of the TCP. This is given by multiplying the rotation matrixes for all axis (*γ*, *β*, *α*) which gives <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image056.png"/> [[rotation matrix out of Euler Angles](http://kos.informatik.uni-osnabrueck.de/download/diplom/node26.html)].

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image057.png"/>

Now we can denote the transformation matrix of the TCP by builing a homogenous matrix out of *TCP<sub>orientation</sub>* and *TCP<sub>position</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image058.png"/>

From the TCP’s perspective, WCP is just translated by *d<sub>5</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image059.png"/>

Furthermore, <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image060.png"/>, so we get the WCP by

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image061.png"/>

in world coordinates.

Having a top view on the robot shows how to compute the first angle *θ<sub>0</sub>*:

<div align="center"><img width="600px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image062.png"/></div>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image064.png"/>

Actually, this angle exists in two variants: if the bot looks backwards, another valid solution is

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image065.png"/>

Thanks to the design having a wrist-centre-point where the axes of the three upper actuators intersect, the next two angles can be computed by the triangle denoted in orange:

<div align="center"><img align="center" width="400px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image027.png"/></div>

Again, there are two solutions representing, one configuration corresponds with a natural pose of the elbow, solution II is a rather unhealthy position:

<div align="center"><img width="600px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image069.png"/></div>

*a* and *b* is given by the length of the actuators *a<sub>1</sub>* und *d<sub>3</sub>*. So, cosine sentence yields the angles *α* and *γ*.

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image072.png"/>

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image073.png"/>
	
Finally, we get

<img align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image074.png"/>
<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image075.png"/>	

and the second solution 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image076.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image077.png"/>	
	

The upper angles *θ<sub>4</sub>*, *θ<sub>5</sub>*, *θ<sub>5</sub>* can be obtained by considering the chain of transformation matrixes. With

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image078.png"/>	

we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image079.png"/>	


Besides the annoying multiplication <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image080.png"/> we only need to consider the rotation part of the homogenous matrixes, translation is no more relevant.

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image081.png"/>	

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image036.png"/> - and therefore the rotation part <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image040.png"/> - is already known resp. can be obtained out of the given angles *θ<sub>0</sub>*, *θ<sub>1</sub>*, *θ<sub>2</sub>* by
	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image086.png"/>	

By equalizing the previous two equations we get the required angles. First angle that seems to be easily computable is *θ<sub>4</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image088.png"/>	

having two solutions

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image089.png"/>	

For *θ<sub>3</sub>* there is no easy matrix element, but we can combine

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image090.png"/>	
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image091.png"/>		

to

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image092.png"/>		

which ends up in

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image093.png"/>		

again having two solutions depending on *θ<sub>4</sub>*. Same is done on *θ<sub>5</sub>*:

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image094.png"/>		


If *θ<sub>4</sub>=0*, we have an infinite number of solutions *θ<sub>3</sub>* and *θ<sub>5</sub>* (gimbal lock). In that case, we consider  <img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image095.png"/> :		

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image096.png"/>.		

Since we know the trigonometric addition theorem from school

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image097.png"/>		

we get

<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image098.png"/>		

We are free to choose *θ<sub>3</sub>* and take the bot’s current angle *θ<sub>3</sub>* to not move unnecessarily.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image101.png"/>		
 
In the end, we get eight solutions by combining the possible pose configurations of  *θ<sub>0</sub>*(forward/backward), *θ<sub>1</sub>* and *θ<sub>2</sub>*(triangle flip), and *θ<sub>4</sub>*(hand orientation turn).

The correct solution is chosen by taking the one that differs the least from the current bot’s joint angles.

Continue reading with [Moving](https://github.com/jochenalt/Walter/wiki/Moving).
