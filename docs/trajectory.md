
Planning a trajectory means defining a sequence of poses in 3D space. These defined poses are interpolated in order to result in a smooth and continuous curve. Most beautiful are cubic Bézier curves.

Bézier curves are polynoms of 3rd grade using a start and an end point and two support points defining the curvature at the start and end point. The trajectory is defined by the start and the end point, the support point is not on the trajectory but used to make it smooth only. The computation is based on a parameter t=0..1 defining the ratio of how much the current position has already made of the full curve. Let’s assume, we have the four points P0..P3, of which P1 and P2 are support points the curve does not touch.

&nbsp;&nbsp;&nbsp;&nbsp;<img src="https://github.com/jochenalt/Walter/blob/master/docs/images/image015.png"/> 

This computation is done for x, y, and z coordinates. Normally being beautiful, but Bézier curves have a tendency to “bounce”, if the support points *P<sub>1</sub>* and *P<sub>2</sub>* differ too much in terms of the distance to the trajectory points *P<sub>0</sub>* and *P<sub>3</sub>*. So, it is necessary to normalize support points by a small trick:

The picture illustrates a trajectory defined by *A*, *B, *C*, and *D*. We want to model the piece between *B* and *C* with a cubic Bézier curve.

&nbsp;&nbsp;&nbsp;&nbsp;<img align="left" width="400px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/image016.png"/>

The support point *B’* is computed by taking point *A*, mirroring it at *B* (*A’*), and moving along the angle bisector of *A’* *B* *C* by a 3<sup>rd</sup> of the length of *BC*, *C’* is computed in an analogous manner.

This approach is rather arbitrary, but results in a smooth and non-oscillating curve.

Now the curve looks fine, but simply following that curve is not enough to make a smooth movement. We need a speed profile that avoids jerky movements.
