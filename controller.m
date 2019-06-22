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

kvz = 10;
kpz = 80;
kvy = 5;
kpy = 20;
kvphi = 20;
kpphi = 1400;

u1 = params.mass*(params.gravity+des_state.acc(2)+kvz*(des_state.vel(2)-state.vel(2))+kpz*(des_state.pos(2)-state.pos(2)));
phi_c = (-1/params.gravity)*(des_state.acc(1)+kvy*(des_state.vel(1)-state.vel(1))+kpy*(des_state.pos(1)-state.pos(1)));
u2 = params.Ixx*(0+kvphi*(0-state.omega)+kpphi*(phi_c-state.rot));

% FILL IN YOUR CODE HERE

end

