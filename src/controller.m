function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% FILL IN YOUR CODE HERE
ep = des_state.pos - state.pos;
ev = des_state.vel - state.vel;

Kp = 1000;
Kd = 90;

% using the same K's for y_dot and z_ddot
r_acc = des_state.acc + Kp*ep + Kd*ev;

% separate K's for y_ddot and z_ddot
Kpy = 100;
Kdy = 10;
Kpz = 100;
Kdz = 10;
Kpphi = 1000;
Kdphi = 90;
y_ddot = des_state.acc(1) + Kpy*ep(1) + Kdy*ev(1);
z_ddot = des_state.acc(2) + Kpz*ep(2) + Kdz*ev(2);
%phi_ddot = y
u1 = params.mass*(params.gravity + z_ddot);
phi_c = -y_ddot / params.gravity;
phi_dot_c = -(Kdy*(des_state.acc(1) - y_ddot) + Kpy*ev(1))/params.gravity;
phi_ddot_c = -Kpy*(des_state.acc(1) - y_ddot)/params.gravity;

u2 = params.Ixx*(phi_ddot_c + Kdphi*(phi_dot_c - state.omega) + Kpphi*(phi_c - state.rot));

end

