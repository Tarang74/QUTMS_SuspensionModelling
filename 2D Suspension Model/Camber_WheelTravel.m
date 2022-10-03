%% Four-bar linkage
% Reference:
%
% Tang, C.P., 2010. Lagrangian dynamic formulation of a four-bar mechanism
% with minimal coordinates.
%
% Credit:
%
% MATLAB FileExchange: 104040-four-bar-linkage
%
clear; close all;
%% Parameters
% Should plot be animated?
animate_plot = false;
% Should animation be saved?
save_animation = false;

% Lengths in mm
% Wheel radius
wheel_r = 203.2;

% Length of bar connected to floating bar (bar 3)
wheel_link_L = 80;

% Kingpin inclination
kp = deg2rad(7);

% % Bar lengths
L1 = 247.4005416323901; % (fixed bar)
L2 = 262.6387511392788; % (upper bar)
L3 = 196.6920255119664; % (floating bar)
L4 = 427.7683752920499; % (lower bar)

% Chassis angle w.r.t. horizontal plane
chassis_angle = pi / 2 - deg2rad(34.2);

% Location of point D
origin = [L1 * cos(chassis_angle); L1 * sin(chassis_angle)];

%% Constants
% Video
tF = 10; % Final time [s]
fR = 60; % Frame rate [fps]
time = linspace(0, tF, tF * fR); % Time [s]

%% Analysis
% Rotatation matrix
origin_rotate = [cos(pi + chassis_angle) -sin(pi + chassis_angle)
            sin(pi + chassis_angle) cos(pi + chassis_angle)];

% Transformation function
transform = @(v) origin_rotate * v + origin;

% Freudenstein Equation
P1 = @(theta) -2 * L2 * L4 * sin(theta);
P2 = @(theta) 2 * L4 * (L1 - L2 * cos(theta));
P3 = @(theta) L1^2 + L2^2 - L3^2 + L4^2 - 2 * L1 * L2 * cos(theta);

% Bar 2 rotation
% Determine range of valid theta2 values
% figure
% fplot(@(x) P1(x).^2 + P2(x).^2 - P3(x).^2, [0 2 * pi])
theta2Range = [0 0];

x = linspace(0, pi, 50000);
idx = find((P1(x).^2 + P2(x).^2 - P3(x).^2) > 0, 1, 'first');
theta2Range(1) = x(idx);

x = linspace(pi, 2 * pi, 50000);
idx = find((P1(x).^2 + P2(x).^2 - P3(x).^2) > 0, 1, 'last');
theta2Range(2) = x(idx);

% Create theta2 vector
theta2Initial = theta2Range(1) + 10 * eps;
theta2Final = theta2Range(2) - 10 * eps;
% We don't need to analyse the entire domain
theta2Initial = 2/6 * pi;
theta2Final = 4/6 * pi;

if (theta2Final <= theta2Initial)
    error("theta2Final must be greater than theta2Initial");
end
if (theta2Initial <= theta2Range(1))
    error("theta2Initial must be greater than " + num2str(theta2Range(1)));
end
if (theta2Final >= theta2Range(2))
    error("theta2Final must be less than " + num2str(theta2Range(2)));
end

w = (theta2Final - theta2Initial) / time(end); % Angular velocity [rad/s]
theta2 = w * time + theta2Initial;

% Angles
theta4 = 2 * atan2(-P1(theta2) - sqrt(P1(theta2).^2 + P2(theta2).^2 - P3(theta2).^2), ...
                   P3(theta2) - P2(theta2));
theta3 = atan2(-L2 * sin(theta2) + L4 * sin(theta4), ...
               L1 - L2 * cos(theta2) + L4 * cos(theta4));

% Determine point B/C trajectory
trajectory_B = transform([L2 * cos(theta2)
                          L2 * sin(theta2)]);

trajectory_C = transform([L1 + L4 * cos(theta4)
                          L4 * sin(theta4)]);

%% Plot initialisation
color = cool(8);

figure
set(gcf, 'Position', [50 50 1280 720])

hold on; grid on; box on; axis equal
set(gca, 'FontName', 'Verdana', 'FontSize', 18)

title('Four-bar linkage')
xlim([-400 1000])
ylim([-400 600])

if (save_animation == true)
    % Create and open video writer object
    v = VideoWriter('four_bar_linkage.mp4', 'MPEG-4');
    v.Quality = 100;
    v.FrameRate = fR;
    open(v);
end

%% Plot handlers
% Trajectories
p_trajectory_B = plot(trajectory_B(1, 1), trajectory_B(2, 1), ...
    'Color', color(1, :), 'LineWidth', 3);
p_trajectory_C = plot(trajectory_C(1, 1), trajectory_C(2, 1), ...
    'Color', color(8, :), 'LineWidth', 3);

% Fixed bar
B2 = transform([0 L2 * cos(theta2(1)); 0 L2 * sin(theta2(1))]);
B4 = transform([L1 + L4 * cos(theta4(1)) L1; L4 * sin(theta4(1)) 0]);
% p_B1 = plot([B4(1, 2) B2(1, 1)], [B4(2, 2) B2(2, 1)], 'k', 'LineWidth', 7);

% Bars attached to fixed bar
p_B2 = plot([0 0], [0 0], 'Color', color(3, :), 'LineWidth', 7); % Upper
p_B4 = plot([0 0], [0 0], 'Color', color(3, :), 'LineWidth', 7); % Lower

% Floating bar
p_B3 = plot([0 0], [0 0], 'Color', color(3, :), 'LineWidth', 7);

% Floating points
p_point_B = plot(0, 0, 'ko', 'MarkerFaceColor', color(1, :), 'MarkerSize', 10);
p_point_C = plot(0, 0, 'ko', 'MarkerFaceColor', color(8, :), 'MarkerSize', 10);

% Fixed points
p_point_A = plot(B4(1, 2), B4(2, 2), 'k>', 'MarkerFaceColor', 'k', 'MarkerSize', 10);
p_point_D = plot(B2(1, 1), B2(2, 1), 'k>', 'MarkerFaceColor', 'k', 'MarkerSize', 10);

% Wheel link
p_wheel_link = plot([0 0], [0 0], 'Color', color(3, :), 'LineWidth', 7);

% Midpoint of BC
p_midpoint_BC = plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10);

% Wheel
p_wheel = plot([0 0], [0 0], 'k--', 'LineWidth', 3);

% Re-determine ground if parameters have been changed
% % Base of wheel
% wheel_y = zeros(1, length(time));
% ground = min(wheel_y)
ground = -2.527206387621802e+02;
p_ground = plot([-400 1000], [ground ground], 'k-');

legend([p_ground p_wheel], 'Ground', 'Wheel')
%% Animation
% Data to record
% Wheel travel
wheel_travel = zeros(0, length(time));

% Camber
camber = zeros(0, length(time));

for i = 1:length(time)
    % Bar 2 (upper)
    B2 = transform([0 L2 * cos(theta2(i))
                0 L2 * sin(theta2(i))]);

    % Bar 3 (floating)
    B3 = transform([L2 * cos(theta2(i)) L2 * cos(theta2(i)) + L3 * cos(theta3(i))
                L2 * sin(theta2(i)) L2 * sin(theta2(i)) + L3 * sin(theta3(i))]);

    % Bar 4 (lower)
    B4 = transform([L1 + L4 * cos(theta4(i)) L1
                L4 * sin(theta4(i)) 0]);

    % Midpoint of BC
    midpoint_BC = (B3(:, 1) + B3(:, 2)) / 2;

    % Wheel link (perpendicular to midpoint of BC)
    B3_slope = (B3(2, 2) - B3(2, 1)) / (B3(1, 2) - B3(1, 1));
    wheel_link_slope = -1 / B3_slope - tan(kp);

    wheel_link_vertical_shift = midpoint_BC(2) - wheel_link_slope * midpoint_BC(1);

    wheel_link = [midpoint_BC(1), wheel_link_L * cos(atan(wheel_link_slope)) + midpoint_BC(1);
                  midpoint_BC(2), wheel_link_slope * ...
                  (wheel_link_L * cos(atan(wheel_link_slope)) + midpoint_BC(1)) + wheel_link_vertical_shift];

    % Wheel
    wheel_slope = -1 / wheel_link_slope;

    wheel_vertical_shift = wheel_link(2, 2) - wheel_slope * wheel_link(1, 2);

    wheel = [wheel_link(1, 2) - wheel_r * cos(atan(wheel_slope)), ...
             wheel_link(1, 2) + wheel_r * cos(atan(wheel_slope));
             wheel_slope * (wheel_link(1, 2) - wheel_r * cos(atan(wheel_slope))) + wheel_vertical_shift, ...
             wheel_slope * (wheel_link(1, 2) + wheel_r * cos(atan(wheel_slope))) + wheel_vertical_shift];

    % Determine where ground is
%     % Base of wheel
%     wheel_y(i) = wheel(2, 2);

    % Data
    % Wheel travel
    wheel_travel(i) = wheel(2, 2) - ground;

    % Camber
    camber(i) = -90 -atan(wheel_slope) * 180 / pi;

    % Update plot data
    % Trajectories
    p_trajectory_B.XData = trajectory_B(1, 1:i);
    p_trajectory_B.YData = trajectory_B(2, 1:i);

    p_trajectory_C.XData = trajectory_C(1, 1:i);
    p_trajectory_C.YData = trajectory_C(2, 1:i);

    % Bars attached to fixed bar
    p_B2.XData = B2(1, :);
    p_B2.YData = B2(2, :);

    p_B4.XData = B4(1, :);
    p_B4.YData = B4(2, :);

    % Floating bar
    p_B3.XData = B3(1, :);
    p_B3.YData = B3(2, :);

    % Points
    p_point_B.XData = B3(1, 1);
    p_point_B.YData = B3(2, 1);

    p_point_C.XData = B4(1, 1);
    p_point_C.YData = B4(2, 1);

    % Midpoint of BC
    p_midpoint_BC.XData = midpoint_BC(1);
    p_midpoint_BC.YData = midpoint_BC(2);

    % Wheel link
    p_wheel_link.XData = wheel_link(1, :);
    p_wheel_link.YData = wheel_link(2, :);

    % Wheel
    p_wheel.XData = wheel(1, :);
    p_wheel.YData = wheel(2, :);

    if (animate_plot == true)
        drawnow
    end

    if (save_animation == true)
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
end

if (save_animation == true)
    close(v);
end

%% Plot data
figure
subplot(2, 1, 1)
plot(time, wheel_travel)
title('Wheel travel')
xlabel('Time (s)')
ylabel('Wheel travel (mm)')

subplot(2, 1, 2)
plot(time, camber)
title('Camber')
xlabel('Time (s)')
ylabel('Camber (deg)')

figure
plot(camber, wheel_travel)
title('Camber vs. wheel travel')
xlabel('Camber (deg)')
ylabel('Wheel travel (mm)')
