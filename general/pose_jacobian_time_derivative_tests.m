% This example tests the pose jacobian derivative method 
% using numerical differentiation.
%
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



function J_dot = numerical_differentiation(J, T)
    % This function computes the numerical differentiation based on the four-point central 
    % finite differences (Numerical Methods for Engineers and Scientists, 3rd Edition by Amos Gilat, P318).
    %
    % Usage: J_dot = numerical_differentiation(J, T)
    %
    %      J: A cell containing the elements to compute its derivatives.
    %      T: The time step.
    %      J_dot: A cell containing the numerical derivatives. 
    %
    s = length(J);
    J_dot = cell(s,1);
    for i=3:s-2
        J_dot{i} = ((J{i-2}-8*J{i-1}+8*J{i+1}-J{i+2})/(12*T));
    end
end

function [error_norm, J_dot, numerical_J_dot] = check_pose_jacobian_derivative(robot, iterations, T,threshold, msg)
    % Given a robot, this function compares the pose_jacobian_derivative method with
    % respect to the numerical derivatives based on the four-point central
    % finite differences.
    %
    % Usage: [error_norm, J_dot, numerical_J_dot] = check_pose_jacobian_derivative(robot, ...
    %                                               iterations, T, threshold, msg);
    %
    %      robot: A DQ_Kinematics robot.
    %      iterations: The number of iterations. This number defines the
    %                  size of the trajectories.
    %      T: The time step.
    %      threshold: The threshold at which it is acceptable to maintain the error.
    %                 If the error is above this value, an exception is
    %                 raised.
    %      msg: The message to be displayed when an exception is thrown.
    %
    error_norm = zeros(iterations, 1);
    njoints    = robot.get_dim_configuration_space();   
    q = zeros(njoints,1);
    q_dot = zeros(njoints,1);

    J = cell(iterations,1);
    J_dot = cell(iterations,1);

    w = 2*pi;

    TestCase = matlab.unittest.TestCase.forInteractiveUse;

    for i=1:iterations
         t = i*T;
         theta = sin(w*t);
         theta_dot = w*cos(w*t);

         for j=1:njoints
             q(j) = theta;
             q_dot(j) = theta_dot;
         end

         J{i} = robot.pose_jacobian(q);
         J_dot{i} = robot.pose_jacobian_derivative(q, q_dot);
    end
    numerical_J_dot = numerical_differentiation(J, T);

    for i=3:iterations-2
        error_norm(i) = norm(J_dot{i} - numerical_J_dot{i}, 'fro');
        TestCase.assertEqual(J_dot{i} ,numerical_J_dot{i}, "AbsTol", threshold, msg);   
    end
end


%---------------------------------------
T = 1e-6;
iterations = 500;

w = 2*pi;
M = cell(iterations,1);
M_dot = cell(iterations,1);
for i=1:iterations
    t = i*T;
    theta = sin(w*t);
    theta_dot = w*cos(w*t);
    M{i} = theta;
    M_dot{i} = theta_dot;

end

numerical_M_dot = numerical_differentiation(M, T);
error_norm = zeros(iterations, 1);
threshold = 1e-12;
for i=3:iterations-2
    error_norm(i) = norm(M_dot{i} - numerical_M_dot{i}, 'fro');
    if (max(abs(error_norm)) > threshold)
        error("The numerical differentiation is inaccurated!")
    end
end






robots = {DQ_HolonomicBase(),...
    FrankaEmikaPandaRobot.kinematics(),...
    KukaLwr4Robot.kinematics()};
msgs = {"Error in DQ_HolonomicBase.pose_jacobian_derivative", ...
        "Error in DQ_SerialManipulatorMDH.pose_jacobian_derivative",...
        "Error in DQ_SerialManipulatorDH.pose_jacobian_derivative"};
thresholds = {1e-10, 1e-8, 1e-8};


for i=1:length(robots)
    check_pose_jacobian_derivative(robots{i}, iterations, T, thresholds{i}, msgs{i});
end

