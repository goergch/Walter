# Construction

Inverse kinematics, i.e. computation of joint angles out of the gripper’s position can be hard. If the design is too playful, nearly impossible complex. So, it is a good idea to ease the maths by having the upper three axes joining in one point. Later in the chapter Kinematics we will see that with that limitation kinematics becomes possible without having a mathematician at hand (still not easy, but feasible). 

But, lets start with the gripper. 

## Gripper

Due to space limitations, it seems to be appropriate to use a servo for the gripper. I used a standard principle where one lever is driven, in the other lever repeats the same movement by a gear wheel. The servo is hidden in a small box, it is a HerkuleX Robot Servo with 0.12 Nm.

<img align="left" src="https://github.com/jochenalt/Walter/blob/master/doc/images/wiki/cad-gripper.png" >

