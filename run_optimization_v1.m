clear;
clc;
close all;

addpath('./gen');

%% ========== PARAMETERS ==========
params.L = 4;         % Length of the truck (L1)
params.d = 1;         % Length of the trailer (L2)
params.dt = 0.1;      % Time step duration
params.N = 300;       % Number of time steps

% Initial state [x, y, theta_t, theta_l]
q0_initial = [0; 0; 0; 0];

% Parking slot center position
parking_slot_center = [30; 20];  % Position: x=30, y=20

% Mission 1: Target point to pass through (before parking)
mission1_target = [15; 10];  % First target: pass through this point (x=15m, y=10m)
params.mission1_target = mission1_target;  % Store for visualization

% Mission 2: Parking slot (after passing through target)
params.parking_target = parking_slot_center;  % Will be adjusted for trailer rear axle

% Cost matrices
params.Q_terminal = 1000;  % High penalty for terminal state error
params.R = diag([0.1, 0.1]); % Control effort penalty
params.Q_rate = [0.5, 0.5];  % Rate penalty weights [velocity, steering]
params.alpha_rate = 0.1;     % Rate penalty scaling

% Input constraints
params.v_max = 1.0;    % Maximum velocity (m/s)
params.v_min = -1.0;   % Minimum velocity (reverse)
params.phi_max = pi/2;  % Maximum steering angle (rad) = 90 degrees from forward
params.phi_min = -pi/2; % Minimum steering angle (rad) = -90 degrees from forward

% Optional: State constraints for orientation angles (uncomment if needed)
params.theta_t_max = pi/2;  % Maximum truck orientation angle (rad) = 90 degrees
params.theta_t_min = -pi/2; % Minimum truck orientation angle (rad) = -90 degrees
params.theta_l_max = pi/2;  % Maximum trailer orientation angle (rad) = 90 degrees
params.theta_l_min = -pi/2; % Minimum trailer orientation angle (rad) = -90 degrees

% Rate limits (for smooth control)
params.dv_max = 1;      % Maximum velocity change per step
params.dphi_max = 0.2;  % Maximum steering rate (rad/step)

% Parking slot definition
slot_width = 3;   % Width of parking slot
slot_length = 6;  % Length of parking slot
slot_center_x = parking_slot_center(1);
slot_center_y = parking_slot_center(2);

params.parking_slot.x_min = slot_center_x - slot_length/2;
params.parking_slot.x_max = slot_center_x + slot_length/2;
params.parking_slot.y_min = slot_center_y - slot_width/2;
params.parking_slot.y_max = slot_center_y + slot_width/2;
params.parking_slot.center = parking_slot_center;

params.parking_activation_radius = 5;  % meters

% For reverse parking, target the trailer rear axle position inside the slot
params.parking_target_trailer_x = slot_center_x + slot_length/4;  % Slightly right of center
params.parking_target_trailer_y = slot_center_y;

params.use_parking_slot_constraints = false;  % Set to true to enforce slot constraints

% Initialize success flags
phase1_success = false;
phase2_success = false;

%% ========== PHASE 1: PASS THROUGH TARGET (FORWARD) ==========
fprintf('\n========== PHASE 1: Pass Through Target ==========\n');

params.parking_phase = 1;  % Forward phase - pass through target
params.initial_state = q0_initial;

% Phase 1 target: Pass through the target point
params.target = [mission1_target(1); mission1_target(2); 0; 0];  % Full state target [x, y, theta_t, theta_l]
params.target_radius = 2;  % Phase 1 completes when truck is within 2m of target

% Debug: Print target to verify
fprintf('Phase 1 Target Position: [%.2f, %.2f]\n', params.target(1), params.target(2));
fprintf('Mission 1 Target: [%.2f, %.2f]\n', mission1_target(1), mission1_target(2));

% Initial guess for Phase 1: point towards target (least squares approach)
% Minimize (target - current_position) to go towards (0,0) error
dx = params.target(1) - q0_initial(1);
dy = params.target(2) - q0_initial(2);
target_angle = atan2(dy, dx);

% Initial steering: point towards target (relative to current truck orientation)
% Steering angle is relative to truck's forward direction
initial_phi = target_angle - q0_initial(3);  % Relative to current theta_t
% Clamp to ±90 degrees from forward direction
initial_phi = max(min(initial_phi, params.phi_max), params.phi_min);

u0_phase1 = zeros(2 * params.N, 1);
u0_phase1(1:2:end) = 1.0;  % Constant forward velocity
u0_phase1(2:2:end) = initial_phi;  % Constant steering towards target

% Input bounds
lb = [params.v_min * ones(params.N, 1); params.phi_min * ones(params.N, 1)];
ub = [params.v_max * ones(params.N, 1); params.phi_max * ones(params.N, 1)];

% Option to enable/disable real-time visualization
enable_visualization = true;  % Set to false to disable and speed up optimization

% Create output function for Phase 1 visualization
if enable_visualization
    outputFcn_phase1 = @(x, optimValues, state) optimization_visualization_callback(x, optimValues, state, q0_initial, params, 1);
else
    outputFcn_phase1 = [];
end

% Optimization options
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 100000, ...
    'MaxIterations', 2000, ...
    'OptimalityTolerance', 1e-3, ...
    'ConstraintTolerance', 1e-4, ...
    'Algorithm', 'sqp');

% Add output function only if visualization is enabled
if enable_visualization && ~isempty(outputFcn_phase1)
    options.OutputFcn = outputFcn_phase1;
end

% Run optimization for Phase 1
fprintf('Solving Phase 1 optimization...\n');
fprintf('This may take several minutes. Progress will be shown in the visualization window.\n');
fprintf('You can disable visualization by setting enable_visualization = false in the code.\n\n');

tic;  % Start timer
[u_opt_phase1, fval1, exitflag1, output1] = fmincon(...
    @(u) objective_function(u, q0_initial, params), ...
    u0_phase1, [], [], [], [], lb, ub, ...
    @(u) constraint_function(u, q0_initial, params), options);
time_phase1 = toc;
fprintf('\n========================================\n');
fprintf('Phase 1 optimization completed in %.1f seconds.\n', time_phase1);
fprintf('========================================\n');

% Process Phase 1 results
if exitflag1 > 0 || exitflag1 == -1
    fprintf('Phase 1 optimization successful!\n');
    fprintf('Final cost: %.4f\n', fval1);
    
    % Simulate Phase 1 to get final state
    u1 = reshape(u_opt_phase1, 2, params.N);
    q_phase1 = zeros(4, params.N + 1);
    q_phase1(:, 1) = q0_initial;
    
    % Check when we reach within target radius
    % Three points: Point1 (nose/truck front), Point2 (center/truck rear/hitch), Point3 (tail/trailer rear)
    target_reached = false;
    target_reach_index = params.N + 1;  % Default to end if not reached
    
    for k = 1:params.N
        q_dot = truck_trailer_dynamics(q_phase1(:, k), u1(:, k), params.L, params.d);
        q_phase1(:, k+1) = q_phase1(:, k) + params.dt * q_dot;
        
        % Check if Point 1 (nose/truck front) is within 2m of target
        if ~target_reached
            % Point 2: Center (truck rear axle / hitch point)
            point2 = [q_phase1(1, k+1); q_phase1(2, k+1)];
            
            % Point 1: Nose (truck front)
            point1 = point2 + params.L * [cos(q_phase1(3, k+1)); sin(q_phase1(3, k+1))];
            
            % Point 3: Tail (trailer rear)
            point3 = point2 - params.d * [cos(q_phase1(4, k+1)); sin(q_phase1(4, k+1))];
            
            % Distance from Point 1 (nose) to target
            dist_point1 = norm(point1 - mission1_target);
            
            % Stop if Point 1 (nose) is within 2m of target
            if dist_point1 <= params.target_radius
                target_reached = true;
                target_reach_index = k + 1;
                fprintf('\n========================================\n');
                fprintf('✓ Phase 1 FINISHED!\n');
                fprintf('Point 1 (nose) reached target at step %d (within %.2f m)\n', k+1, params.target_radius);
                fprintf('Point 1 position: [%.2f, %.2f]\n', point1(1), point1(2));
                fprintf('Point 2 position: [%.2f, %.2f]\n', point2(1), point2(2));
                fprintf('Point 3 position: [%.2f, %.2f]\n', point3(1), point3(2));
                fprintf('Target position: [%.2f, %.2f]\n', mission1_target(1), mission1_target(2));
                fprintf('Distance: %.3f m\n', dist_point1);
                fprintf('========================================\n');
                pause(2);  % Stop for a moment
                break;
            end
        end
    end
    
    % Use the state when target was reached (or final state if not reached)
    q0_phase2 = q_phase1(:, target_reach_index);
    
    if target_reached
        q_phase1 = q_phase1(:, 1:target_reach_index);
        u_opt_phase1 = u_opt_phase1(1:2*(target_reach_index-1));
        fprintf('  Phase 1 completed in %d steps (%.1f seconds)\n', target_reach_index-1, (target_reach_index-1)*params.dt);
        phase1_success = true;
        fprintf('  ✓ Phase 1 SUCCESS - Target reached! Moving to Phase 2...\n');
    else
        fprintf('\n⚠ Phase 1: Did not reach target within radius.\n');
        fprintf('  Using final state: [%.2f, %.2f, %.2f, %.2f]\n', q0_phase2);
        phase1_success = false;
        fprintf('  ⚠ Phase 1 did not reach target, but proceeding to Phase 2 anyway.\n');
    end
    
    fprintf('\n>>> Phase 1 Complete! Phase 2 will be implemented later.\n');
else
    warning('Phase 1 optimization failed (exitflag < 0).');
    phase1_success = false;
    q_phase1 = [];
    u_opt_phase1 = [];
    fprintf('\n⚠ Phase 1 failed.\n');
end

%% ========== PHASE 2: MOVE TO PARKING SLOT (FORWARD) - COMMENTED OUT FOR NOW ==========
% Phase 2 will be implemented later - focusing on Phase 1 first
if true  % Set to true when ready to run Phase 2
fprintf('\n========================================\n');
fprintf('PHASE 2: Move to Parking Slot\n');
fprintf('========================================\n');
fprintf('Starting from Phase 1 final position: [%.2f, %.2f, %.2f, %.2f]\n', q0_phase2);
fprintf('Target: Parking slot at [%.1f, %.1f]\n', parking_slot_center(1), parking_slot_center(2));
fprintf('\n');

params.parking_phase = 0;  % Forward to target

% For parking, target truck position to place trailer rear axle at slot center-right
target_trailer_rear = [params.parking_target_trailer_x; params.parking_target_trailer_y];
params.target = [target_trailer_rear(1) + params.d; target_trailer_rear(2); 0; 0];  % [x, y, theta_t, theta_l]

% Straight-line initial guess for Phase 2
% Compute direction from current position to target
dx2 = params.target(1) - q0_phase2(1);
dy2 = params.target(2) - q0_phase2(2);
target_angle2 = atan2(dy2, dx2);

% For a straight line, constant steering angle pointing directly at target
initial_phi2 = target_angle2 - q0_phase2(3);  % Forward direction
initial_phi2 = max(min(initial_phi2, params.phi_max), params.phi_min);  % Clamp to limits

% Create straight-line initial guess: constant velocity and constant steering
u0_phase2 = zeros(2 * params.N, 1);
u0_phase2(1:2:end) = 1.0;  % Constant forward velocity for straight line
u0_phase2(2:2:end) = initial_phi2;  % Constant steering angle pointing at target

% Create output function for Phase 2 visualization
if enable_visualization
    outputFcn_phase2 = @(x, optimValues, state) optimization_visualization_callback(x, optimValues, state, q0_phase2, params, 2);
    options.OutputFcn = outputFcn_phase2;
else
    options.OutputFcn = [];
end

% Run optimization for Phase 2
fprintf('========================================\n');
fprintf('Solving Phase 2 optimization...\n');
fprintf('========================================\n');
fprintf('Initial state for Phase 2: [%.2f, %.2f, %.2f, %.2f]\n', q0_phase2);
fprintf('Target state for Phase 2: [%.2f, %.2f, %.2f, %.2f]\n', params.target);
fprintf('This may take several minutes. Progress will be shown in the visualization window.\n\n');

tic;  % Start timer
try
    [u_opt_phase2, fval2, exitflag2, output2] = fmincon(...
        @(u) objective_function(u, q0_phase2, params), ...
        u0_phase2, [], [], [], [], lb, ub, ...
        @(u) constraint_function(u, q0_phase2, params), options);
    time_phase2 = toc;
    fprintf('\n========================================\n');
    fprintf('Phase 2 optimization completed in %.1f seconds.\n', time_phase2);
    fprintf('Exit flag: %d\n', exitflag2);
    fprintf('========================================\n');
catch ME
    time_phase2 = toc;
    fprintf('\nERROR in Phase 2 optimization:\n');
    fprintf('%s\n', ME.message);
    exitflag2 = -1;
    fval2 = inf;
    u_opt_phase2 = [];
end

if exitflag2 > 0 || exitflag2 == 0
    fprintf('Phase 2 optimization completed.\n');
    fprintf('Final cost: %.4f\n', fval2);
    
    % Simulate Phase 2
    u2 = reshape(u_opt_phase2, 2, params.N);
    q_phase2 = zeros(4, params.N + 1);
    q_phase2(:, 1) = q0_phase2;
    for k = 1:params.N
        q_dot = truck_trailer_dynamics(q_phase2(:, k), u2(:, k), params.L, params.d);
        q_phase2(:, k+1) = q_phase2(:, k) + params.dt * q_dot;
    end
    fprintf('Phase 2 final state: [%.2f, %.2f, %.2f, %.2f]\n', q_phase2(:, end));
    phase2_success = true;
else
    warning('Phase 2 optimization failed (exitflag < 0).');
    phase2_success = false;
    q_phase2 = [];
    u_opt_phase2 = [];
end
end  % End of if false block for Phase 2

%% ========== VISUALIZE PHASE 1 ==========
fprintf('\n========== Visualizing Phase 1 ==========\n');

% Store parking slot info for visualization
params.parking_slot_center = parking_slot_center;
params.target_radius = 2;  % Store target radius for visualization

% Visualize Phase 1
if phase1_success
    fprintf('Visualizing Phase 1 trajectory.\n');
    visualize_parking_maneuver(q_phase1, u_opt_phase1, q0_initial, params);
else
    warning('Phase 1 failed. Cannot visualize.');
end

fprintf('\n========== Phase 1 Complete ==========\n');

