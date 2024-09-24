close all; clear; clc;

% Test Inverse Kinematics

rb = 90;            % radius of the base of the robot [mm]
rp = 60;            % radius of the platform of the robot [mm]
hi = 125;           % the initial height of the robot [mm]
alpha_b = 20;       % half the angle between the base attachment points [deg]
alpha_p = 20;       % half the angle between the platform attachment points [deg]

trans = [0 0 0];    % the desired position vector [1x3] of the platform [mm]
orient = [20 0 0];   % the desired orientation vector [1x3] of the platform [deg]

figure
ax = axes;          % axes for plotting the robot

l = inverse_kinematics_v2(ax, rb, rp, hi, alpha_b, alpha_p, trans, orient);
disp('Required leg lengths:');
disp(l);