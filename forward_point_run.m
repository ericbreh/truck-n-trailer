clear;
clc;
close all;

addpath('./gen');

params.L = 2;         % Length of the truck
params.d = 5;         % Length of the trailer
params.target = [11; 9; 0; 0]; % Target state [x, y, theta_t, theta_l]
params.N = 50;          % Number of time steps
params.dt = 0.1;        % Time step duration

% Initial state
q0 = [0; 0; 0; 0];
u0 = zeros(2 * params.N, 1);

% Optimization options
options = optimoptions('fmincon', 'Display', 'iter', 'MaxFunctionEvaluations', 50000);

% Run fmincon
[u_opt, fval, exitflag, output] = fmincon(@(u) forward_point_objective(u, q0, params), ...
	u0, [], [], [], [], [], [], ...
	@(u) forward_point_constraints(u, q0, params), options);

simulate(u_opt, q0, params);