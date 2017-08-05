
# Quadrotor Control in the Plane

This is an exercise to implement a Proportional Derivative (PD) controller in the plane for a Quadrotor. This assigment was part of the Aerial robotics course given by the University of Pennsylvania through Coursera.

The following description is based on the lectures and assignment writeup provided during the course. The figures were taken from the assignment writeup.

[//]: # (Image References)

[image1]: ./images/fig1.png
[image2]: ./images/fig2.png

## Installation and Execution

Requirements: MATLAB

To install, download the files. To run it, execute runsim.

The controller.m file contains the implementation of the System model, which is explained in the following section.

## System model

The orientation of the quadrotor is defined by the angle $`\phi`$, as shown in the following figure:

![alt text][image1]

where axes **a**<sub>2</sub> and **a**<sub>3</sub> define the inertial frame, and **b**<sub>2</sub> and **b**<sub>3</sub> the body frame, which is attached to the quadrotor center of mass.  *u*<sub>1</sub> = *F*<sub>1</sub> + *F*<sub>2</sub> is the sum of each motor thrust, and *u*<sub>2</sub> = *L*(*F*<sub>1</sub> - *F*<sub>2</sub>), and *L* is the arm length of the quadrotor.

Gravity acts in the -**a**<sub>3</sub> direction. Let $`\mathbf{r}=\begin{bmatrix}
y & z
\end{bmatrix}^{T}`$ be the position vector of the quadrotor, then,
```math
m\mathbf{\ddot{r}}=\begin{bmatrix}0\\-mg\end{bmatrix}+\begin{bmatrix}-u_{1}\sin\phi\\u_{1}\cos\phi\end{bmatrix}
```

and by Euler's equation of motion,
```math
I_{xx}\ddot{\phi}=u_{2}
```
where *I*<sub>*xx*</sub> is the inertia due to the roll motion. Since there is only motion in the YZ plane, there are no pitch or yaw motions, therefore *I*<sub>*yy*</sub> = *I*<sub>*zz*</sub> = 0.

## Controller

Since $`m\ddot{y}=u_{1}\sin\phi`$ and $`m\ddot{z}=u_{1}\cos\phi-mg`$, the dynamic model of the quadrotor is nonlinear. However, a PD controller is designed for a linear system. To use a linear controller for this nonlinear system, the dynamics of the quadrotor can be linearized about an equilibrium configuration $`y=y_{0}`$, $`z=z_{0}`$, $`\phi=0`$. Using first-order Taylor approximations at this equilibrium configuration, the non-linear functions $`\sin\phi`$ and $`\cos\phi`$ can be approximated to $`\phi`$ and 1, respectively. Hence,
```math
\ddot{y} = -g\phi
```
```math
\ddot{z} = -g + \frac{u_1}{m}
```
```math
\ddot{\phi} = \frac{u_2}{I_{xx}}
```

The objective is to find out the required inputs *u*<sub>1</sub> and *u*<sub>2</sub>. A PD controller (Position Controller) can be used to calculate the required $`\ddot{z}`$, denoted as $`\ddot{z}_c`$ for commanded acceleration, to achieve a desired trajectory $`z_{des}`$. With this commanded acceleration, *u*<sub>1</sub> can be obtained.  The desired trajectory $`\mathbf{r}_{des}=\begin{bmatrix}y_{des} & z_{des}\end{bmatrix}^{T}`$, together with its first and second derivatives, can be provided by the user or system that commands the quadrotor. The current position $`\mathbf{r}`$ and velocity $`\dot{\mathbf{r}}`$ can be obtained from the sensors in the quadrotor. Hence, the Position Controller has all the parameters needed to calculate *u<sub>1</sub>*.

But *u*<sub>2</sub> cannot be obtained using the same Position Controller, because *u*<sub>2</sub> depends on $`\ddot{\phi}`$, which is not obtained using the Position Controller. To calculate the required angular acceleration $`\ddot{\phi}`$, a second PD controller (Attitude Controller) is required, which depends on the commanded roll angle $`\phi_{c}`$, the commanded angular velocity $`\dot{\phi}_{c}`$, the commanded angular acceleration $`\ddot{\phi}_{c}`$, the current roll angle $`\phi`$ and the current angular velocity $`\dot{\phi}`$. The first three dependencies can be obtained from the Position Controller, and the last two from the sensors in the quadrotor.

Therefore, the Attitude Controller requires as input the output of the Position Controller, as in the following figure.

![alt text][image2]

With this design decision taken, the rest is straightforward. The commanded acceleration vector $`\ddot{\mathbf{r}}_{c}`$, can be obtained with,
```math
\ddot{\mathbf{r}}_{c}=\ddot{\mathbf{r}}_{des}+k_{p}\mathbf{e}_{p}+k_{v}\mathbf{e}_{v}
```


where $`\mathbf{r}_c=\begin{bmatrix} y_c & z_c \end{bmatrix}^{T}`$, *k<sub>p</sub>* and *k<sub>v</sub>* are the proportional and derivative gains, respectively, $`\mathbf{e}_p=\mathbf{r}_{des} - \mathbf{r}`$, and $`\mathbf{e}_v=\dot{\mathbf{r}}_{des} - \dot{\mathbf{r}}`$.

The required angular acceleration, $`\ddot{\phi}`$, can be obtained in the Position Controller, as follows,
```math
\ddot{\phi}=\ddot{\phi}_{c} + k_{v,\phi}(\dot{\phi}_c-\dot{\phi}) + k_{p,\phi}(\phi_c-\phi)
```


where $`k_{p,\phi}`$ and $`k_{v,\phi}`$ are the proportional and derivative gains, respectively.

Finally, *u*<sub>1</sub> and *u*<sub>2</sub> can be derived as:
```math
u_1 = mg + m\ddot{z}_c
```
```math
u_2 = I_{xx}\ddot{\phi}
```
```math
\phi_c = -\frac{\ddot{y}_c}{g}
```
