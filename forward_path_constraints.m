function [c, ceq] = forward_path_constraints(u, q0, params)
u = reshape(u, 2, params.N);

% Simulate the system forward in time
q = zeros(4, params.N + 1);
q(:, 1) = q0;
for k = 1:params.N
	q_dot = truck_trailer_dynamics(q(:, k), u(:, k), params.L, params.d);
	q(:, k+1) = q(:, k) + params.dt * q_dot;
end

ceq = [];
c = [];

end
