clear;
clc;
close all;

addpath('./gen');

params.L = 2.8;         % Length of the truck
params.d = 5.5;         % Length of the trailer
params.target = [10; 2; 0; 0]; % Target state [x, y, theta_t, theta_l]
params.N = 50;          % Number of time steps
params.dt = 0.1;        % Time step duration

% Initial state
q0 = [0; 0; 0; 0];
u0 = zeros(2 * params.N, 1);

[u_opt, fval, exitflag, output] = fmincon(@(u) objective_function(u, q0, params), ...
	u0, [], [], [], [], [], [], ...
	@(u) constraint_function(u, q0, params));

simulate_and_plot(u_opt, q0, params);
