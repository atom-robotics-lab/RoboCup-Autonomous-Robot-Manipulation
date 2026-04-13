function config = moveSmoothTraj(robot, ik, weights, startConfig, targetPos, targetRot, totalTime, steps, visualize)
% moveSmoothTraj - Trajectory follower using MATLAB's trapveltraj
%   Uses quaternion SLERP for smooth orientation interpolation

% Set defaults
if nargin < 7
    totalTime = 2;
end
if nargin < 8
    steps = 50;
end
if nargin < 9
    visualize = true;
end

% Suppress warnings
warning('off', 'robotics:manip:invalidConfiguration');

% Get current pose
T_current = getTransform(robot, startConfig, 'tool0');
startPos = T_current(1:3, 4)';
startRot = T_current(1:3, 1:3);

% If no target rotation provided, keep current orientation
if nargin < 6 || isempty(targetRot)
    targetRot = startRot;
end

% Convert rotation matrices to quaternions for smooth interpolation
startQuat = rotm2quat(startRot);
targetQuat = rotm2quat(targetRot);

% Create position waypoints (3 x 2 matrix)
posWaypoints = [startPos; targetPos]';

% Generate trapezoidal velocity trajectory for position
[posTraj, ~, ~, t] = trapveltraj(posWaypoints, steps, 'EndTime', totalTime);

% Generate trajectory for quaternion interpolation parameter
% This creates a smooth progression from 0 to 1
alphaTraj = linspace(0, 1, steps);

fprintf('\n=== trapveltraj Motion ===\n');
fprintf('Moving from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)\n', ...
    startPos(1), startPos(2), startPos(3), ...
    targetPos(1), targetPos(2), targetPos(3));
fprintf('Total time: %.2f seconds\n', totalTime);
fprintf('Steps: %d\n', steps);

% Follow the trajectory
config = startConfig;

for i = 1:steps
    % Position at this step
    pos = posTraj(:, i)';
    
    % Orientation using quaternion SLERP (Spherical Linear Interpolation)
    alpha = alphaTraj(i);
    interpQuat = slerpQuat(startQuat, targetQuat, alpha);
    interpRot = quat2rotm(interpQuat);
    
    % Build target transform
    T_target = eye(4);
    T_target(1:3, 4) = pos;
    T_target(1:3, 1:3) = interpRot;
    
    % Solve IK
    [q_sol, info] = ik('tool0', T_target, weights, config);
    
    % Accept solution if successful or best available
    if strcmp(info.Status, 'success') || strcmp(info.Status, 'best available')
        config = q_sol;
        
        % Visualize if requested
        if visualize
            show(robot, config);
            drawnow;
            pause(totalTime / steps);
        end
    else
        fprintf('Warning: IK failed at step %d/%d\n', i, steps);
    end
end

fprintf('Motion complete!\n');

end

function q = slerpQuat(q1, q2, t)
% Spherical Linear Interpolation for quaternions
% Ensures smooth rotation between two orientations

% Normalize quaternions
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

% Compute dot product
dot = sum(q1 .* q2);

% If dot is negative, flip one quaternion to take shortest path
if dot < 0
    q2 = -q2;
    dot = -dot;
end

% For very small angles, use linear interpolation
if dot > 0.9995
    q = q1 + t * (q2 - q1);
    q = q / norm(q);
else
    theta_0 = acos(dot);
    theta = theta_0 * t;
    sin_theta = sin(theta);
    sin_theta_0 = sin(theta_0);
    
    s1 = cos(theta) - dot * sin_theta / sin_theta_0;
    s2 = sin_theta / sin_theta_0;
    
    q = s1 * q1 + s2 * q2;
    q = q / norm(q);
end

end