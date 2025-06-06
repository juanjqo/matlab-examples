% This example shows how to use basic commands in CoppeliaSim.
%
% Note:
%
% Open the DQ_Robotics_lab.ttt scene (https://github.com/dqrobotics/coppeliasim-scenes) 
% in CoppeliaSim before running this script.
% 
% (C) Copyright 2011-2025 DQ Robotics Developers
% 
% This file is part of DQ Robotics.
% 
%     DQ Robotics is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     DQ Robotics is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
%
% DQ Robotics website: https://dqrobotics.github.io/
%
% Contributors to this file:
%    1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)





clear;
close all;
clc;

include_namespace_dq;

% Instantiate the class
cs = DQ_CoppeliaSimInterfaceZMQ();


try
    % Establish the connection with CoppeliaSim. 
    cs.connect();

    % Set the stepping mode. 
    % Check more in https://manual.coppeliarobotics.com/en/simulation.htm
    cs.set_stepping_mode(true);

    % Starts the simulation. 
    cs.start_simulation();

    % Define some parameters to compute varying-time trajectories
    a = 1;
    freq = 0.1;
    time_simulation_step = 0.05;

    for i=0:300
        t = i*time_simulation_step;

        % Define a varying-time position based on the Lemniscate of Bernoulli
        p = (a*cos(t)/(1 + (sin(t)^2)))*i_ + (a*sin(t)*cos(t)/(1 + (sin(t)^2)))*j_;

        % Define a varying-time orientation
        r = cos(2*pi*freq*t) + k_*sin(2*pi*freq*t);

        % Built a varying-time unit dual quaternion
        x = r + 0.5*E_*p*r;

        % Set the object pose in CoppeliaSim
        cs.set_object_pose("/coffee_drone", x);

        % Read the pose of an object in CoppeliaSim to set the pose of
        % another object with a constant offset.
        xread = cs.get_object_pose("/coffee_drone");
        xoffset = 1 + 0.5*E_*(0.5*i_);
        xnew = xread*xoffset;
        cs.set_object_pose("/Frame_x", xnew);


        % Set the target position of the first joint of the UMIRobot arm
        target_position = sin(2*pi*freq*t);
        cs.set_joint_target_positions({'UMIRobot/UMIRobot_joint_1'}, target_position);

        % Set the target position of the third joint of the UR5 robot
        cs.set_joint_target_positions({'UR5/link/joint/link/joint'}, target_position);

        % Set the target velocity of the first joint of the Franka Emika Panda
        target_velocity = 2*pi*freq*cos(2*pi*freq*t);
        cs.set_joint_target_velocities({'Franka/joint'}, target_velocity);

        % Set the torque of the Dummy Robot
        torque = sin(2*pi*freq*t);
        cs.set_joint_target_forces({'Revolute_joint'}, torque);
        

        % Set the target velocities of the Pioneer wheels
        cs.set_joint_target_velocities({'PioneerP3DX/rightMotor'}, 0.1);
        cs.set_joint_target_velocities({'PioneerP3DX/leftMotor'}, 0.2);


        % Trigger a simulation step in CoppeliaSim
        cs.trigger_next_simulation_step();    

        % Read the joint force after perform a the simulation step
        torque_read = cs.get_joint_forces({'Revolute_joint'});

        % Save data to compare the target and read forces
        torque_log(i+1) = torque;
        torque_read_log(i+1) = torque_read;
    end

    cs.stop_simulation(); % Stop the simulation


    h1 = figure;
    set(h1, 'DefaultTextFontSize', 15);
    set(h1, 'DefaultAxesFontSize', 15);
    plot(torque_log, 'b', 'LineWidth', 2);
    hold on
    plot(torque_read_log, ':r', 'LineWidth', 2)
    legend('target force', 'measured force');
    fig = gcf;
    fig.Color = [1 1 1];
    box('off');

catch ME

    cs.stop_simulation(); % Stop the simulation
    rethrow(ME)

end

