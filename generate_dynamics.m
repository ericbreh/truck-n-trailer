clear;
clc;

syms x y theta_t theta_l real
syms v phi real
syms L d real

% State vector, (x,y) are position of rear axle of truck, theta t is orientation of truck, theta l is orientation of trailer
q = [x; y; theta_t; theta_l];

% Control vector
u = [v; phi];

%% Kinematic model
q_dot = [
	v * cos(theta_t);
	v * sin(theta_t);
	(v / L) * tan(phi);
	(v / d) * sin(theta_l - theta_t) - (v/L)*tan(phi) - (v*cos(theta_l-theta_t)*(d*cos(theta_l-theta_t) - L*tan(phi)))/d
	];

if ~exist('./gen', 'dir')
	mkdir('./gen')
end
addpath('./gen')
matlabFunction(q_dot, 'File', 'gen/truck_trailer_dynamics', 'Vars', {q, u, L, d});
