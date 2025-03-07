clc; clear; close all;

% Robot parameters
L = [0.5, 0.5, 0.4, 0.3, 0.2, 0.1]; % Link lengths for 6-DOF arm

% Start position
start_point = [0, 0, 0];

% Target position
goal = [0.9, 0.4, 0.3];

% Obstacles as blocks around the room (position and size [x, y, z, width, depth, height])
blocks = [];

% Table position and size
table_z = -0.2;

% Initial joint angles
theta = 0.5 * ones(1, 6);

dt = 0.05; % Time step
max_iter = 500;

% PD Controller gains
Kp = 7.0; % Increased Proportional gain for better response
Kd = 3.0; % Increased Derivative gain to reduce oscillations

theta_dot_prev = zeros(1, 6);

figure; hold on; axis equal;
xlim([-2 2]); ylim([-2 2]); zlim([-0.5 2]);
view(3); grid on;

for iter = 1:max_iter
    % Forward kinematics to compute full arm positions
    joint_positions = zeros(7, 3);
    x = 0; y = 0; z = 0;
    J = zeros(3, 6);
    
    for i = 1:6
        x = x + L(i) * cos(sum(theta(1:i)));
        y = y + L(i) * sin(sum(theta(1:i)));
        z = z + 0.1 * sin(theta(i));
        joint_positions(i+1, :) = [x, y, z];
        
        J(1, i) = -L(i) * sin(sum(theta(1:i)));
        J(2, i) = L(i) * cos(sum(theta(1:i)));
        J(3, i) = 0.1 * cos(theta(i));
    end
    end_effector = joint_positions(7, :);
    
    % Compute attractive force
    F_att = goal - end_effector;
    F_att = 3 * F_att / max(norm(F_att), 0.01); % Stronger attraction force
    
    % Compute repulsive forces
    F_rep = [0, 0, 0];
    eta = 0.7; % Increased repulsion coefficient to avoid obstacles more strongly
    for i = 1:size(blocks, 1)
        block_center = blocks(i, 1:3) + [0, 0, blocks(i, 6) / 2];
        block_size = blocks(i, 4:6);
        
        d = norm(end_effector - block_center);
        if d < sum(block_size)/2
            F_rep = F_rep + eta * (end_effector - block_center) / (d^3 + 0.01);
        end
    end
    
    % Total force
    F_total = F_att + F_rep;
    
    % Convert force to joint space using Jacobian transpose
    theta_dot = J' * F_total';
    
    % PD Control
    theta_ddot = Kp * theta_dot' + Kd * (theta_dot' - theta_dot_prev) / dt;
    theta = theta + dt * theta_dot' + dt^2 * theta_ddot';
    
    theta_dot_prev = theta_dot';
    
    % Plot environment
    clf; hold on; axis equal;
    xlim([-2 2]); ylim([-2 2]); zlim([-0.5 2]);
    view(3); grid on;
    
    % Plot table
    fill3([-2, 2, 2, -2], [-2, -2, 2, 2], [table_z, table_z, table_z, table_z], [0.7 0.4 0.2], 'FaceAlpha', 0.5);
    
    % Plot start point
    plot3(start_point(1), start_point(2), start_point(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Plot goal
    plot3(goal(1), goal(2), goal(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    
    % Plot blocks
    for i = 1:size(blocks, 1)
        x_b = [blocks(i,1)-blocks(i,4)/2, blocks(i,1)+blocks(i,4)/2];
        y_b = [blocks(i,2)-blocks(i,5)/2, blocks(i,2)+blocks(i,5)/2];
        z_b = [table_z, table_z+blocks(i,6)];
        [X, Y, Z] = ndgrid(x_b, y_b, z_b);
        scatter3(X(:), Y(:), Z(:), 100, 'r', 'filled');
    end
    
    % Plot robot arm
    plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), '-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
    scatter3(end_effector(1), end_effector(2), end_effector(3), 50, 'r', 'filled');
    
    pause(0.05);
    
    % Check if goal is reached
    if norm(end_effector - goal) < 0.05
        disp('Goal reached!');
        break;
    end
end
