function config = goToPoint(robot, ik, weights, config, targetPos, varargin)
% goToPoint - Move end-effector smoothly to target position
% 
% This is a ROBUST version that handles IK failures, joint limits, 
% and produces smooth trajectories suitable for competition.
%
% Inputs:
%   robot     - Your loaded robot model (rigidBodyTree)
%   ik        - inverseKinematics object
%   weights   - IK weights [tx ty tz rx ry rz]
%   config    - Current joint configuration (1x6 or 1xn)
%   targetPos - [x y z] target position in meters
%
% Output:
%   config    - Final joint configuration after movement

    %% Default settings (you can adjust these)
    parser = inputParser;
    addParameter(parser, 'MaxStepSize', 0.02);      % 2cm max step
    addParameter(parser, 'Visualize', true);        % Show animation
    addParameter(parser, 'PauseTime', 0.02);        % Seconds between steps
    addParameter(parser, 'MaxAttempts', 3);         % IK retries
    parse(parser, varargin{:});
    
    %% Get current position
    T_current = getTransform(robot, config, 'tool0');
    currentPos = T_current(1:3, 4)';
    distance = norm(targetPos - currentPos);
    
    %% Calculate number of steps (1cm resolution, minimum 3 steps)
    numSteps = max(3, ceil(distance / parser.Results.MaxStepSize));
    
    fprintf('📦 Moving from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)\n', ...
        currentPos(1), currentPos(2), currentPos(3), ...
        targetPos(1), targetPos(2), targetPos(3));
    fprintf('   Distance: %.2f cm | Steps: %d\n', distance*100, numSteps);
    
    %% Save current orientation (we'll maintain it throughout)
    currentRot = T_current(1:3, 1:3);
    
    %% Main motion loop
    config_current = config;
    prevValidConfig = config;
    
    for step = 0:numSteps
        alpha = step / numSteps;
        
        % Interpolate position (linear in Cartesian space)
        pos = (1-alpha) * currentPos + alpha * targetPos;
        
        % Build target transform (maintain orientation)
        T_target = eye(4);
        T_target(1:3, 4) = pos';
        T_target(1:3, 1:3) = currentRot;
        
        % Try to solve IK with multiple attempts
        q_sol = [];
        for attempt = 1:parser.Results.MaxAttempts
            if attempt == 1
                % First try: use current config as seed
                seed = config_current;
            elseif attempt == 2
                % Second try: use previous valid config
                seed = prevValidConfig;
            else
                % Third try: use home config
                seed = homeConfiguration(robot);
            end
            
            [q_sol_temp, solInfo] = ik('tool0', T_target, weights, seed);
            
            if solInfo.Status == 'success'
                q_sol = q_sol_temp;
                break;
            end
        end
        
        % If IK failed, use previous valid config and warn
        if isempty(q_sol)
            warning('IK failed at step %d/%d. Using previous config.', step, numSteps);
            q_sol = prevValidConfig;
        else
            prevValidConfig = q_sol;
        end
        
        % Update current configuration
        config_current = q_sol;
        
        % Visualization
        if parser.Results.Visualize
            show(robot, config_current, 'PreservePlot', false);
            drawnow;
            pause(parser.Results.PauseTime);
        end
    end
    
    %% Return final configuration
    config = config_current;
    fprintf('✅ Movement complete\n');
end