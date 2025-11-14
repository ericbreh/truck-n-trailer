function simulate_and_plot(u_opt, q0, params)

u = reshape(u_opt, 2, params.N);

% Simulate the system
q = zeros(4, params.N + 1);
q(:, 1) = q0;
for k = 1:params.N
	q_dot = truck_trailer_dynamics(q(:, k), u(:, k), params.L, params.d);
	q(:, k+1) = q(:, k) + params.dt * q_dot;
end

% Plot the trajectory
figure;
hold on;
plot(q(1, :), q(2, :), 'b-', 'DisplayName', 'Trajectory');
plot(params.target(1), params.target(2), 'rx', 'MarkerSize', 10, 'DisplayName', 'Target');
xlabel('x (m)');
ylabel('y (m)');
title('Truck and Trailer Trajectory');
legend;
axis equal;
grid on;

% Animation
figure;
for k = 1:size(q, 2)
	clf;
	hold on;
	draw_truck_trailer(q(:, k), params);
	plot(params.target(1), params.target(2), 'rx', 'MarkerSize', 10);
	axis equal;
	grid on;
	xlim([-2, 12]);
	ylim([-2, 12]);
	title(sprintf('Animation (t = %.1fs)', (k-1)*params.dt));
	drawnow;
	pause(params.dt);
end

end

function draw_truck_trailer(q, params)
x = q(1);
y = q(2);
theta_t = q(3);
theta_l = q(4);

% Truck body
truck_rear_axle = [x; y];
truck_front_axle = [x + params.L * cos(theta_t); y + params.L * sin(theta_t)];
plot([truck_rear_axle(1), truck_front_axle(1)], [truck_rear_axle(2), truck_front_axle(2)], 'b-');
plot(truck_rear_axle(1), truck_rear_axle(2), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
plot(truck_front_axle(1), truck_front_axle(2), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');

% Trailer body
trailer_hitch_point = [x; y];
trailer_axle = [x - params.d * cos(theta_l); y - params.d * sin(theta_l)];
plot([trailer_hitch_point(1), trailer_axle(1)], [trailer_hitch_point(2), trailer_axle(2)], 'r-');
plot(trailer_hitch_point(1), trailer_hitch_point(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(trailer_axle(1), trailer_axle(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
end
