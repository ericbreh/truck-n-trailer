clear; clc; close all;

function xdot = dynamics_fun(x, u)
    % SYSTEM DYNAMICS HERE
    % xdot = f(x, u)
    
    xdot = zeros(size(x));  % <-- replace
end

function J = objective_fun(u, x0, x_ref, u_ref, N, dt)
    % u is a vector of length N*nu
    % reshape it:
    nu = length(u_ref);
    u = reshape(u, nu, N);

    % Choose your cost matrices:
    Q = eye(length(x0));
    R = 0.1 * eye(nu);
    P = Q;   % terminal cost (optional)

    x = x0;
    J = 0;

    for k = 1:N
        % Running cost
        J = J + (x - x_ref)'*Q*(x - x_ref) + (u(:,k) - u_ref)'*R*(u(:,k) - u_ref);

        % Propagate
        xdot = dynamics_fun(x, u(:,k));
        x = x + dt * xdot;
    end

    % Terminal cost
    J = J + (x - x_ref)'*P*(x - x_ref);
end

function [c, ceq] = mpc_constraints(u, x0, N, dt)
    % u is flattened vector of inputs over horizon
    % CONSTRAINTS here: input bounds, rate limits, state constraints, obstacle constraints, terminal region, actuator nonlinearities

    c = [];     % inequality constraints (c <= 0)
    ceq = [];   % equality constraints
end


%% PARAMETERS
dt = 0.1;            % sample time
N  = 600;             % horizon length

nx = 4;              % number of states
nu = 2;              % number of inputs

x0 = [0;0;0;0];      % initial state
x_ref = [0;0;0;0];   % target state
u_ref = [0;0];       % target input (optional)

% fmincon settings
opts = optimoptions('fmincon', ...
    'Display','none', ...
    'MaxIterations',200, ...
    'OptimalityTolerance',1e-4, ...
    'ConstraintTolerance',1e-4);

%% INITIAL INPUT GUESS (warm start)
u0 = repmat(u_ref, N, 1);    % vector of length N*nu

%% SIMULATION
Tf = 10;                         % total simulation time
steps = round(Tf/dt);
x_log = zeros(nx, steps+1);
u_log = zeros(nu, steps);
x_log(:,1) = x0;

for k = 1:steps
    
    % 3. Build objective function 
    obj = @(u) objective_fun(u, x_log(:,k), x_ref, u_ref, N, dt);
    
    % 4. Insert Constraints 
    nonlcon = @(u) mpc_constraints(u, x_log(:,k), N, dt);
    
    % 5. Insert bounds for u
    LB = [];    
    UB = [];    
    
    % 6. Solve optimization (receding horizon)
    [u_opt, ~, exitflag] = fmincon(obj, u0, [], [], [], [], LB, UB, nonlcon, opts);
    
    if exitflag <= 0
        warning('Optimization failed — using previous guess');
        u_opt = u0;
    end
    
    % 7. Take first control action
    u_applied = u_opt(1:nu);
    u_log(:,k) = u_applied;

    % 8. Simulate one step forward (implement dynamics_fun)
    x_log(:,k+1) = x_log(:,k) + dt * dynamics_fun(x_log(:,k), u_applied);
    
    % 9. Warm start next iteration (shift input sequence)
    u_shift = reshape(u_opt, nu, N);
    u_shift = [u_shift(:,2:end), u_shift(:,end)];
    u0 = u_shift(:);
end


figure;
subplot(2,1,1); plot(x_log'); title('States'); grid on;
subplot(2,1,2); plot(u_log'); title('Inputs'); grid on;
