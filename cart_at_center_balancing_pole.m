clc;
clear;
close all;

%% =========================================================
% SINGLE INVERTED PENDULUM WITH PID CONTROL + CENTERING
%
% Teaching-oriented model:
%   Outer loop: keeps cart near x = 0
%   Inner loop: PID balances pendulum
%
% States:
% x(1) = cart position [m]
% x(2) = cart velocity [m/s]
% x(3) = pendulum angle [rad]
% x(4) = pendulum angular velocity [rad/s]
%
% Angle convention:
% theta = 0   -> pendulum upright
% theta > 0   -> pendulum tilts to the right
% theta < 0   -> pendulum tilts to the left
%% =========================================================

%% System parameters
g = 9.81;      % gravity [m/s^2]
M = 1.0;       % cart mass [kg]
m = 0.2;       % pendulum mass [kg]
l = 0.5;       % pendulum length [m]

%% Simulation settings
dt = 0.005;
T  = 12;
t  = 0:dt:T;
N  = length(t);

%% Initial condition
% [cart position; cart velocity; pendulum angle; pendulum angular velocity]
x = zeros(4, N);
x(:,1) = [0; 0; deg2rad(4); 0];

%% =========================================================
% OUTER LOOP: cart centering
% desired pendulum angle = theta_ref
%
% If cart goes to the right, make theta_ref slightly left
% so that the inner loop brings the cart back.
%% =========================================================
Kx  = 0.20;   % position-to-angle gain
Kxd = 0.47;   % velocity-to-angle gain

%% =========================================================
% INNER LOOP: PID on pendulum angle tracking
% error = theta - theta_ref
%% =========================================================
Kp = 57;
Ki = 1.2;
Kd = 20;

integral_error = 0;

%% Control saturation
u_max = 30;
u_hist = zeros(1, N);
theta_ref_hist = zeros(1, N);

%% =========================================================
% Create figure for animation
%% =========================================================
figure('Color','w');
axis equal;
axis([-2.5 2.5 -0.2 1.6]);
grid on;
hold on;
box on;
xlabel('Horizontal Position [m]');
ylabel('Vertical Position [m]');
title('Single Inverted Pendulum: PID Balance + Cart Centering');

% Ground
plot([-3 3], [0 0], 'k', 'LineWidth', 1.5);

% Center line
plot([0 0], [-0.2 1.6], '--k', 'LineWidth', 1.0);

% Cart geometry
cart_width  = 0.30;
cart_height = 0.18;
cart_y      = 0.05;
wheel_r     = 0.05;
circle_pts  = linspace(0, 2*pi, 100);

% Initial geometry
cart_pos0 = x(1,1);
theta0    = x(3,1);
pivot_y   = cart_y + cart_height;

% Draw cart
cart = rectangle('Position', ...
    [cart_pos0 - cart_width/2, cart_y, cart_width, cart_height], ...
    'FaceColor', [0.2 0.6 0.85], ...
    'EdgeColor', 'k', ...
    'LineWidth', 1.5);

% Wheels
wheel1_center = [cart_pos0 - cart_width/4, cart_y];
wheel2_center = [cart_pos0 + cart_width/4, cart_y];

wheel1 = plot(wheel1_center(1) + wheel_r*cos(circle_pts), ...
              wheel1_center(2) + wheel_r*sin(circle_pts), ...
              'k', 'LineWidth', 1.5);

wheel2 = plot(wheel2_center(1) + wheel_r*cos(circle_pts), ...
              wheel2_center(2) + wheel_r*sin(circle_pts), ...
              'k', 'LineWidth', 1.5);

% Pendulum
pend_x0 = cart_pos0 + l*sin(theta0);
pend_y0 = pivot_y + l*cos(theta0);

rod = plot([cart_pos0 pend_x0], [pivot_y pend_y0], ...
           'LineWidth', 3, 'Color', [0 0.2 0.8]);

bob = plot(pend_x0, pend_y0, 'o', ...
           'MarkerSize', 12, ...
           'MarkerFaceColor', 'r', ...
           'MarkerEdgeColor', 'k');

% Text
info_text = text(-2.35, 1.40, '', ...
    'FontSize', 11, 'FontWeight', 'bold');

%% =========================================================
% Simulation loop
%% =========================================================
for k = 1:N-1

    % Current states
    cart_pos  = x(1,k);
    cart_vel  = x(2,k);
    theta     = x(3,k);
    theta_dot = x(4,k);

    %% -----------------------------------------------------
    % OUTER LOOP: compute desired angle from cart position
    %
    % If cart drifts right (+x), ask pendulum to lean slightly left (-)
    %% -----------------------------------------------------
    theta_ref = -Kx*cart_pos - Kxd*cart_vel;

    % Limit desired angle to a small range for stability
    theta_ref_max = deg2rad(10);
    theta_ref = max(min(theta_ref, theta_ref_max), -theta_ref_max);

    theta_ref_hist(k) = theta_ref;

    %% -----------------------------------------------------
    % INNER LOOP: PID on pendulum angle
    %% -----------------------------------------------------
    error     = theta - theta_ref;
    error_dot = theta_dot;   % theta_ref_dot assumed small / ignored here

    integral_error = integral_error + error*dt;

    % Anti-windup clamp on integral term
    integral_max = 0.5;
    integral_error = max(min(integral_error, integral_max), -integral_max);

    % Correct sign for this model:
    % Positive u moves cart right and helps reduce positive theta
    u = Kp*error + Ki*integral_error + Kd*error_dot;

    % Saturation
    u = max(min(u, u_max), -u_max);
    u_hist(k) = u;

    %% -----------------------------------------------------
    % Teaching dynamics model
    %
    % Cart:
    %   x_ddot = u / (M + m)
    %
    % Pendulum:
    %   theta_ddot = (g/l)*theta - (1/l)*x_ddot
    %
    % This says:
    %   - gravity makes upright unstable
    %   - moving the cart under the pole helps recover it
    %% -----------------------------------------------------
    x_ddot     = u / (M + m);
    theta_ddot = (g/l)*theta - (1/l)*x_ddot;

    %% Integrate
    x(1,k+1) = cart_pos  + cart_vel*dt;
    x(2,k+1) = cart_vel  + x_ddot*dt;
    x(3,k+1) = theta     + theta_dot*dt;
    x(4,k+1) = theta_dot + theta_ddot*dt;

    %% -----------------------------------------------------
    % Animation update
    %% -----------------------------------------------------
    new_cart_pos = x(1,k+1);
    new_theta    = x(3,k+1);

    % Update cart
    set(cart, 'Position', ...
        [new_cart_pos - cart_width/2, cart_y, cart_width, cart_height]);

    % Update wheels
    wheel1_center = [new_cart_pos - cart_width/4, cart_y];
    wheel2_center = [new_cart_pos + cart_width/4, cart_y];

    set(wheel1, 'XData', wheel1_center(1) + wheel_r*cos(circle_pts), ...
                'YData', wheel1_center(2) + wheel_r*sin(circle_pts));

    set(wheel2, 'XData', wheel2_center(1) + wheel_r*cos(circle_pts), ...
                'YData', wheel2_center(2) + wheel_r*sin(circle_pts));

    % Update pendulum
    pend_x = new_cart_pos + l*sin(new_theta);
    pend_y = pivot_y + l*cos(new_theta);

    set(rod, 'XData', [new_cart_pos pend_x], ...
             'YData', [pivot_y pend_y]);

    set(bob, 'XData', pend_x, 'YData', pend_y);

    % Update text
    set(info_text, 'String', sprintf( ...
        ['Time = %.2f s\n' ...
         'Cart Position = %.3f m\n' ...
         'Pole Angle = %.2f deg\n' ...
         'Desired Angle = %.2f deg\n' ...
         'Force = %.2f N'], ...
         t(k+1), new_cart_pos, rad2deg(new_theta), rad2deg(theta_ref), u));

    drawnow;
    pause(0.005);
end

u_hist(end) = u_hist(end-1);
theta_ref_hist(end) = theta_ref_hist(end-1);

%% =========================================================
% Plots after animation
%% =========================================================
figure('Color','w');

subplot(4,1,1);
plot(t, x(1,:), 'LineWidth', 1.8);
yline(0,'--k');
grid on;
xlabel('Time [s]');
ylabel('x [m]');
title('Cart Position');

subplot(4,1,2);
plot(t, rad2deg(x(3,:)), 'LineWidth', 1.8);
hold on;
plot(t, rad2deg(theta_ref_hist), '--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('\theta [deg]');
title('Pendulum Angle and Desired Angle');
legend('Actual Angle','Desired Angle','Location','best');

subplot(4,1,3);
plot(t, x(2,:), 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('x dot [m/s]');
title('Cart Velocity');

subplot(4,1,4);
plot(t, u_hist, 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('u [N]');
title('Control Force');