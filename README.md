# Quadrotor Control in the Plane

Requirements: MATLAB

This is an exercise to implement a Proportional Derivative (PD) controller in the plane for a Quadrotor. This assigment was part of the Aerial robotics course given by the University of Pennsylvania through Coursera.

In the following description, the italicized text indicates that it is textually taken from the assignment writeup provided during the course, as well as the figures.

[//]: # (Image References)

[image1]: ./images/fig1.png
[image2]: ./images/fig2.png

## System model

The orientation of the quadrotor is defined by the angle *phi*, as shown in the following figure:

![alt text][image1]

where axes **a**<sub>2</sub> and **a**<sub>3</sub> define the inertial frame, and **b**<sub>2</sub> and **b**<sub>3</sub> the body frame, which is attached to the quadrotor center of mass.  *u*<sub>1</sub> = *F*<sub>1</sub> + *F*<sub>2</sub> is the sum of each motor thrust, and *u*<sub>2</sub> = *L*(*F*<sub>1</sub> - *F*<sub>2</sub>), and *L* is the arm length of the quadrotor.

Gravity acts in the -**a**<sub>3</sub> direction. If **r** = [*y*  *z*]<sup>*T*</sup>be the position vector of the quadrotor, then,

*m*(d<sup>2</sup>**r**/d*t*<sup>2</sup>) = [0  -*mg*]<sup>*T*</sup> + [-*u*<sub>1</sub>sin(*phi*)  *u*<sub>1</sub>cos(*phi*)]<sup>*T*</sup>

and by Euler's equation of motion,

*I<sub>xx</sub>*·d<sup>2</sup>*phi*/d*t*<sup>2</sup> = *u*<sub>2</sub>

## Controller

Since, d<sup>2</sup>*y*/d*t*<sup>2</sup> = -*u*<sub>1</sub>sin(*phi*)/*m* (Eq. 1) and d<sup>2</sup>*z*/d*t*<sup>2</sup> = *u*<sub>1</sub>cos(*phi*)/*m* - *g* (Eq. 2), the dynamic model of the quadrotor is nonlinear. However, a PD controller is designed for a linear system. To use a linear controller for this nonlinear system, the equations 1 and 2 can be linearized about a hover configuration at any position *y*<sub>0</sub>, *z*<sub>0</sub> with zero roll angle. Therefore, the required inputs at this hover configuration are *u*<sub>1,0</sub> = *mg*, *u*<sub>2,0</sub> = 0.

Using first order Taylor approximations at *phi* = 0, sin(*phi*) is approximately equal to *phi*, and cos(*phi*) is approximately equal to 1. Hence, near *phi* = 0,

d<sup>2</sup>*y*/d*t*<sup>2</sup> = -*g·phi*

d<sup>2</sup>*z*/d*t*<sup>2</sup> = -*g* + *u*<sub>1</sub>/*m*

d<sup>2</sup>*phi*/d*t*<sup>2</sup> = *u*<sub>2</sub>/*I*<sub>*xx*</sub>

The objective is to find out the required inputs *u*<sub>1</sub> and *u*<sub>2</sub>. A PD controller (Position Controller) can be used to calculate the required d<sup>2</sup>*z*/d*t*<sup>2</sup>, denoted as d<sup>2</sup>*z<sub>c</sub>*/d*t*<sup>2</sup> for commanded acceleration, to achieve a desired trajectory *z<sub>T</sub>*. With this commanded acceleration, *u*<sub>1</sub> can be obtained. Since this PD controller depends on the position vector and its first and second derivatives, let it be denoted as PD 

But *u*<sub>2</sub> cannot be obtained using the same Position Controller, because *u*<sub>2</sub> depends on d<sup>2</sup>*phi*/d*t*<sup>2</sup>, which is not obtained using the Position Controller. To calculate the required angular acceleration d<sup>2</sup>*phi*/d*t*<sup>2</sup>, a second PD controller (Attitude Controller) is required, which depends on the commanded roll angle *phi*, denoted as *phi<sub>c</sub>*, the commanded angular velocity d*phi<sub>c</sub>*/d*t*, the commanded angular acceleration d<sup>2</sup>*phi<sub>c</sub>*/d*t*<sup>2</sup>, and the current roll angle *phi* and the current angular velocity d*phi*/d*t*.

Therefore, the Attitude Controller requires as input the output of the Position Controller, as in the following figure.

![alt text][image2]

With this design decision taken, the rest is straightforward. The commanded acceleration vector **r**<sub>*c*</sub>, can be obtained with,

d<sup>2</sup>**r**<sub>*c*</sub>/d*t*<sup>2</sup> = d<sup>2</sup>**r**<sub>*T*</sub>/d*t*<sup>2</sup> + *k*<sub>*p*</sub>**e**<sub>*p*</sub> + *k*<sub>*v*</sub>**e**<sub>*v*</sub>,

where **r**<sub>*c*</sub> = [*y<sub>c</sub>*, *z<sub>c</sub>*]<sup>*T*</sup>, and *k<sub>p</sub>* and *k<sub>v</sub>* are the proportional and derivative gains, respectively, **e**<sub>*p*</sub> = **r**<sub>*T*</sub> - **r**, **e**<sub>*v*</sub> = d**r**<sub>*T*</sub>/d*t* - d**r**/d*t*.

And the required angular acceleration, d<sup>2</sup>*phi*/d*t*<sup>2</sup>, can be obtained in the Position Controller, as follows,

d<sup>2</sup>*phi*/d*t*<sup>2</sup> = d<sup>2</sup>*phi*<sub>*c*</sub>/d*t*<sup>2</sup> + *k*<sub>*v,phi*</sub>(d*phi*<sub>*c*</sub>/d*t* - d*phi*/d*t*) + *k*<sub>*p,phi*</sub>(*phi*<sub>*c*</sub> - *phi*)

where *k*<sub>*p,phi*</sub> and *k*<sub>*v,phi*</sub> are the proportional and derivative gains, respectively.

Finally, *u*<sub>1</sub> and *u*<sub>2</sub> can be derived as:

*u*<sub>1</sub> = *mg* + *m*d<sup>2</sup>*z*<sub>*c*</sub>/d*t*<sup>2</sup>

*u*<sub>2</sub> = *I*<sub>*xx*</sub>d<sup>2</sup>*phi*/d*t*<sup>2</sup>

*phi*<sub>*c*</sub> = -(d<sup>2</sup>*y*<sub>*c*</sub>/d*t*<sup>2</sup>)/*g*

