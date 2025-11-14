function J = objective_function(u, q0, params)
u = reshape(u, 2, params.N);
v = u(1, :);
phi = u(2, :);

% Simulate the system forward in time
q = zeros(4, params.N + 1);
q(:, 1) = q0;
for k = 1:params.N
	q_dot = truck_trailer_dynamics(q(:, k), u(:, k), params.L, params.d);
	q(:, k+1) = q(:, k) + params.dt * q_dot;
end

final_state_error = q(:, end) - params.target;
cost = final_state_error' * final_state_error;

control_effort = sum(sum(u.^2));

J = cost + 0.01 * control_effort;

end
