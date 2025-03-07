clc; clear; close all;

% Robot parameters
L = [0.5, 0.5, 0.4, 0.3, 0.2, 0.1]; % Link lengths for 6-DOF arm

% Start position
start_point = [0, 0, 0];

% Target position
goal = [1.0, 0.8, 0.5];

% Obstacles as blocks around the room (position and size [x, y, z, width, depth, height])
blocks = [0.5, 0.5, 0, 0.4, 0.4, 0.3;  % Block 1
          -0.3, 0.7, 0, 0.3, 0.3, 0.25; % Block 2
          0.8, -0.5, 0, 0.5, 0.5, 0.35; % Block 3
          -1.0, -1.0, 0, 0.4, 0.4, 0.5; % Block 4
          1.2, 1.2, 0, 0.6, 0.6, 0.4]; % Block 5

% Table position and size
table_z = -0.2; % Table height

% Initial joint angles (randomized starting pose)
theta = pi/6 * ones(1, 6);

% Step size and iteration limit
step_size = 0.05;
max_iter = 300;

figure; hold on; axis equal;
xlim([-2 2]); ylim([-2 2]); zlim([-0.5 2]);
view(3); grid on;

for iter = 1:max_iter
    % Forward kinematics to compute full arm positions
    joint_positions = zeros(7, 3); % Stores positions of joints (base + 6 links)
    x = 0; y = 0; z = 0;
    J = zeros(3, 6); % Jacobian initialization
    
    for i = 1:6
        x = x + L(i) * cos(sum(theta(1:i)));
        y = y + L(i) * sin(sum(theta(1:i)));
        z = z + 0.1 * sin(theta(i)); % Simulated height variation
        joint_positions(i+1, :) = [x, y, z];
        
        % Compute Jacobian (simplified approximation)
        J(1, i) = -L(i) * sin(sum(theta(1:i))); % ∂x/∂theta_i
        J(2, i) = L(i) * cos(sum(theta(1:i)));  % ∂y/∂theta_i
        J(3, i) = 0.1 * cos(theta(i));          % ∂z/∂theta_i
    end
    end_effector = joint_positions(7, :);
    
    % Compute attractive force
    F_att = goal - end_effector;
    F_att = F_att / norm(F_att);
    
    % Compute repulsive forces
    F_rep = [0, 0, 0];
    for i = 1:size(blocks, 1)
        block_center = blocks(i, 1:3) + [0, 0, blocks(i, 6) / 2]; % Center of block
        block_size = blocks(i, 4:6);
        
        if abs(end_effector(1) - block_center(1)) < block_size(1)/2 + 0.1 && ...
           abs(end_effector(2) - block_center(2)) < block_size(2)/2 + 0.1 && ...
           abs(end_effector(3) - block_center(3)) < block_size(3)/2 + 0.1
            
            F_rep = F_rep + (end_effector - block_center) ./ block_size.^2;
        end
    end
    
    % Total force
    F_total = F_att + F_rep;
    
    % Convert force to joint space using Jacobian transpose
    theta_update = step_size * (J' * F_total');
    theta = theta + theta_update';
    
    % Plot the environment
    clf; hold on; axis equal;
    xlim([-2 2]); ylim([-2 2]); zlim([-0.5 2]);
    view(3); grid on;
    
    % Plot table
    fill3([-2, 2, 2, -2], [-2, -2, 2, 2], [table_z, table_z, table_z, table_z], [0.7 0.4 0.2], 'FaceAlpha', 0.5);
    
    % Plot start point
    plot3(start_point(1), start_point(2), start_point(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    % Plot goal
    plot3(goal(1), goal(2), goal(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    
    % Plot blocks as full 3D obstacles
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
    
    pause(0.1);
    
    % Check if goal is reached
    if norm(end_effector - goal) < 0.05
        disp('Goal reached!');
        break;
    end
end
