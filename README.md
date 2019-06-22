# Quadrotor-Control-in-the-Plane
MATLAB Programming Exercise 2

# 1 Introduction
In this exercise, you will be implementing the PD controller discussed in lecture to control the motion of the quadrotor in the Y-Z plane.

# 2 System Model
# 2.1 Coordinate Systems
The coordinate systems and free body diagram for the planar model of a quadrotor are shown in Fig. 1. The inertial frame, A, is deﬁned by the axes a2, and a3. The body frame, B, is attached to the center of mass of the quadrotor with b2 coinciding with the preferred forward direction and b3 perpendicular to the rotors pointing vertically up (see Fig. 1).
# 2.2 Dynamics 
For a quadrotor modeled in the Y −Z plane, its orientation is deﬁned by a roll angle, φ. It is assumed that its pitch and yaw angles are 0. You will need the rotation matrix for transforming components of vectors in B to components of vectors in A:

       A[R] B=[cos(φ),−sin(φ);sin(φ),cos(φ)]. (1)
       
We will denote the components of angular velocity of the robot in the body frame by ˙ φ:

     AωB = ˙ φ.
     
In the planer model of the quadrotor, we only consider the thrust force on two of the rotors. The quadrotor has two inputs: the thrust force (u1) and the moment (u2). u1 is the sum of the thrusts at each rotor

    u1 = Σ(i=1 TO 2 )  Fi ,
while u2 is proportional to the diﬀerence between the thrusts of two rotors

    u2 = L(F1 −F2) 

Here, L is the arm length of the quadrotor. Let r = [y,z]T denote the position vector of planar quadrotor inA. The forces on the system are gravity, in the −a3 direction, and the thrust force, in b3 direction. Hence, by Newton’s Equations of Motion,


    m¨ r = m[¨ y ¨ z]=[ 0 −mg]+A[R]B*[0 u1]=[ 0 −mg]+[−u1*sin(φ) u1*cos(φ)]. (2) 
The angular acceleration is determined by Euler’s equation of motion 

    Ixx ¨ φ = L(F1 −F2) = u2
As a result, the system model can be written as,

    [ ¨ y,¨ z,¨ φ ]=[ 0 −g 0 ]+[ −1/m*sin(φ),0;1/m*cos(φ),0; 0,1/Ixx][u1 u2] (3)
     

# 3 Controller
# 3.1 Linearization
The dynamic model of the quadrotor (Eq. 3) is nonlinear. However, a PD controller is designed for a linear system. To use a linear controller for this nonlinear system, we ﬁrst linearize the equation of motions about an equilibrium conﬁguration. In the case of the quadrotor, the equilibrium conﬁguration is the hover conﬁguration at any arbitrary position y0,z0, with zero roll angle. The corresponding thrust force needed to hover at this conﬁguration is exactly mg, while the moment must be zero. Explicitly, the values of the related variables at the hover conﬁguration are
     y0,z0,φ0 = 0,u1,0 = mg,u2,0 = 0
To linearize the dynamics, we replace all non-linear function of the state and control variables with their ﬁrst order Taylor approximations at the equilibrium location. In this case, the non-linear functions are sin(φ) and cos(φ). Near φ = 0, sin(φ) ≈ φ and    cos(φ) ≈ 1

       ¨ y =−gφ
       
       ¨ z =−g + u1/m
       
       ¨ φ = u2 Ixx
       
Let r denote a state variable, either y, z or φ. We can ﬁnd the commanded acceleration of that state, ¨ rc, corresponding to a (PD) controller as follows. Deﬁne the position and velocity errors as 

      ep =rT(t)−r
   
     ev =˙ rT(t)− ˙ r 
   
We want error to satisfy the following diﬀerential equation, which will result in convergence of the error for some value of kp and kd.

    (¨ rT(t)−¨ rc) + kpep + kvev = 0 (4)
    
From this, we can see that

    ¨ rc = ¨ rT(t) + kpep + kvev, (5)
    
where kp and kv are proportional and derivative gains respectively. As a result, the inputs u1,u2, can be derived as:

    u1 =mg + m¨ zc = m{g + ¨ zT(t) + kv,z(˙ zT(t)− ˙ z) + kp,z(zT(t)−z)} (6)
    u2 =Ixx ¨ φT(t) = Ixx(¨ φc + kv,φ( ˙ φc − ˙ φ) + kp,φ(φc −φ)) (7)
    φc =− ¨ yc g = − 1 g (¨ yT(t) + kv,y(˙ yT(t)− ˙ y) + kp,y(yT(t)−y)) (8)
    
# 3.2 Hover Controller
Hovering is the special case of which the desired position is constant and the desired roll is zero. From
rT(t) = r0 = [y0,z0]T,φT(t) = φ0,r˙T(t) = ¨rT(t) = 0 we get:

    u1 =m[g−kv,z ˙ z + kp,z(z0 −z)]
    u2 =Ixx(¨ φc + kv,φ( ˙ φc − ˙ φ) + kp,φ(φc −φ)) φc =− 1/g (kv,y(−˙ y) + kp,y(y0 −y)
    
# 3.3 Trajectory Controller
For trajectory following, given the desired trajectories for each state and their derivatives, rT(t), ˙ (r)T(t), ¨ (r)T(t), the inputs u1,u2 can be calculated using Equations 6-8.

# Tasks
You will need to complete the implementation of a PD controller in the ﬁle controller.m so that the simulated quadrotor can follow a desired trajectory. Before implementing your own functions, you should ﬁrst try to run runsim.m in your MATLAB environment. If you see a quadrotor falling from position (0,0), then the simulator works on your computer and you may continue with other tasks. Again, the quadrotor falls because the default outputs of the controller.m function are all zeros, and no thrust is applied. To test diﬀerent trajectories, you will need to modify the variable trajhandle inside the runsim.m to point to the appropriate function. Examples are provided inside the runsim.m ﬁle.

# OUTPUT

![OUTPUT](https://user-images.githubusercontent.com/36922299/59966094-c8ba4780-9534-11e9-91b3-63f615ce8b5a.png)
