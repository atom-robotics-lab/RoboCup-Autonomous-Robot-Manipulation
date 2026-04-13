%% SETUP_ROBOT.M - Load and configure UR5e with gripper
clear; clc; close all;

%% Load robot model
load('ur5e_gripper.mat');
fprintf('✅ Robot loaded successfully\n');
fprintf('   Bodies: %d\n', robot.NumBodies);
fprintf('   End effector: %s\n', robot.BodyNames{end});

%% Setup Inverse Kinematics
ik = inverseKinematics('RigidBodyTree', robot);
ik.SolverParameters.MaxIterations = 1000;
ik.SolverParameters.AllowRandomRestarts = true;

%% Set weights
weights = [0.2, 0.2, 0.2, 0.8, 0.8, 0.8];

%% Define home configuration
homeConfig = homeConfiguration(robot);

%% Get home orientation
T_home = getTransform(robot, homeConfig, 'tool0');
homeRot = T_home(1:3, 1:3);

%% Save workspace
save('robot_workspace.mat', 'robot', 'ik', 'weights', 'homeConfig', 'homeRot');

fprintf('✅ Setup complete! Variables saved to robot_workspace.mat\n');
fprintf('   To reload later use: load(''robot_workspace.mat'')\n');
fprintf('\n   To visualize, use: show(robot, config)\n');