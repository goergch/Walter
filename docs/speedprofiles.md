Purpose of speed profiles is to avoid jerky movements by having a smooth acceleration and decelaration. The classical approach is to use trapezoidal speed profiles like this:

<img align="left" width="300px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image017.png"/>
<img align="left" width="300px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image018.png"/>

This trapezoid speed profile results in a constant (maximum) acceleration, then continuing with no acceleration, and a constant deceleration until zero speed is reached. To get the position on a curve, speed is integrated over time.

Despite of the corners in the speed profile, the position profile looks smooth. Still, how is a profile like that computed? Having a constant acceleration *a*, start speed vstart, an end speed vend, the distance d (length of the Bezier curve) and the desired duration of the complete profile *t<sub>g</sub>*, we need to compute the time *t<sub>0</sub>* and *t<sub>1</sub>* which is the duration of the starting acceleration and final deceleration. The full distance is given by

&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image019.png" >

The duration and speed of the plateau is given by 

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image020.png"/>
&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image021.png"/>

Rearranging these equations to get *t<sub>0</sub>* ends up in

&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image023.png"/>

with

&nbsp;&nbsp;&nbsp;&nbsp;<img  src="https://github.com/jochenalt/Walter/blob/master/docs/images/image025.png"/>
	
Finally, with the equation<img align="center" src=":https://github.com/jochenalt/Walter/blob/master/docs/images/image020.png|Distance Computation"/> we get

&nbsp;&nbsp;&nbsp;&nbsp;<img align="center" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image026.png"/>
	
With the equation above on computing *d* we get *t<sub>g</sub>*.

This holds true for a trapezoid profile only, since to model a full trajectory that consist of single movements we also need profiles that represent a ramp or stairways, depending on the constrains in terms of duration, distance, and start/end speed. This ended up in quite a lot of code treating all the different cases

Having a nice trajectory with a smooth speed profile, we need to convert the absolute coordinates into angles of the joints. This is done by inverse kinematics described in the following chapter.


Continue reading with [Trajectory](https://github.com/jochenalt/Walter/wiki/Trajectory).