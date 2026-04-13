function config = moveSmooth(robot, ik, weights, config, targetPos, targetRot, steps, visualize)
% moveSmooth - Move arm smoothly to target position with orientation control

warning('off', 'robotics:manip:invalidConfiguration');

if nargin < 7
    steps = 10;
end
if nargin < 8
    visualize = false;
end

% Get current pose
T_current = getTransform(robot, config, 'tool0');
currentPos = T_current(1:3, 4)';
currentRot = T_current(1:3, 1:3);

if nargin < 6 || isempty(targetRot)
    targetRot = currentRot;
end

% Convert rotation matrices to quaternions for smooth interpolation
currentQuat = rotm2quat(currentRot);
targetQuat = rotm2quat(targetRot);

for step = 1:steps
    alpha = step / steps;
    
    % Interpolate position
    pos = (1-alpha)*currentPos + alpha*targetPos;
    
    % Interpolate orientation using quaternion SLERP
    interpQuat = slerp(currentQuat, targetQuat, alpha);
    interpRot = quat2rotm(interpQuat);
    
    % Build target transform
    T_target = eye(4);
    T_target(1:3, 4) = pos;
    T_target(1:3, 1:3) = interpRot;
    
    [q_sol, info] = ik('tool0', T_target, weights, config);
    
    if strcmp(info.Status, 'success') || strcmp(info.Status, 'best available')
        config = q_sol;
        
        if visualize
            show(robot, config);
            drawnow;
            pause(0.02);
        end
    end
end

end

function q = slerp(q1, q2, t)
% Spherical linear interpolation for quaternions
q1 = q1 / norm(q1);
q2 = q2 / norm(q2);

dot = sum(q1 .* q2);
if dot < 0
    q2 = -q2;
    dot = -dot;
end

if dot > 0.9995
    % Linear interpolation for small angles
    q = q1 + t*(q2 - q1);
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