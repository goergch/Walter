Inverse kinematics, i.e. computation of joint angles out of the gripper’s position can be hard. If the design is too playful, nearly impossible complex. So, it is a good idea to ease the maths by having the upper three axes joining in one point. Later in the chapter Kinematics we will see that with that limitation kinematics becomes possible without having a mathematician at hand (still not easy, but feasible). 

But, lets start with the gripper. 

## Gripper

Due to space limitations, it seems to be appropriate to use a servo for the gripper. I used a standard principle where one lever is driven, in the other lever repeats the same movement by a gear wheel. The servo is hidden in a small box, it is a HerkuleX Robot Servo with 0.12 Nm.

<img width="500px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/cad-gripper.png" >

## Wrist

The wrist is also designed with the same servo. A small flange connects the wrist with the two halves of the gripper housing, the hole hides the cable of the gripper servo. Worth to mention is that the bearings of the wrist have a diffent size, since the servo looks through the inner hole of the bigger bearing. On the other side, in the middle of the smaller bearing there is the hole for the magnet used by the magnetic encoder of the forearm. The cable of both servos (gripper and wrist) is going through the wrist underneath the servo.

<img align width="800px" src="https://github.com/jochenalt/Walter/blob/master/docs/images/cad-wrist.png" >

Continue reading with [Speed Profile](https://github.com/jochenalt/Walter/wiki/Speed-Profile).

