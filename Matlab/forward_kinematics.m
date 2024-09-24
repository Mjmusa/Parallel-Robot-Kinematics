function [A] = forward_kinematics(r_B, r_P, initial_height, leg_lengths, alpha_B, alpha_P)
% Project: MRI-Compatible Parallel Robot
% Description: This function computes the Forward Kinematics of a UPS stewart platform 
%              using newton raphson method for error control, screw based kinematic
%              jacobian and inverse kinematics 
% Mishek Musa - Graduate Research Assistant
% Medical Robotics Lab University of Arkansas
% last edited: January 2021
% 
% Inputs:
%   r_B             - radius of the base of the robot [mm]
%   r_P             - radius of the platform of the robot [mm]
%   initial_height  - the initial length of the linear actuators [mm]
%   leg_lengths     - the vector [6x1] of the leg lengths [mm]
%   alpha_B         - half the angle between the base attachment points [deg]
%   alpha_P         - half the angle between the platform attachment points [deg]
%
% Outputs:
%   A(pose)         - the matrix pose of the moving platform

%% Define Platform Geometry
% Initial Platform Pose
orient = [0 0 0]*pi/180;
trans = [0 0 initial_height];

BaseJointOffset = 20;         % mm
PlatformJointOffset = 26;     % mm
    
robotGeometry;
                  
legs = [Plat_Pts; Base_Pts];

%% Transformation Matrix
Rz = rotZ(orient(3));
Ry = rotY(orient(2));
Rx = rotX(orient(1));

Rot = Rz*Ry*Rx;

Initial_Pose = [Rot, [trans(1); trans(2); trans(3)]; zeros(1,3), 1];

%%
Initial_Leg_Length = zeros(6,1);
Initial_Jacobian = zeros(6,6);

for i = 1:6

    PlatPt = Initial_Pose*[legs(1:3,i);1];          
    BasePt = legs(4:6,i);

    r = PlatPt(1:3) - Initial_Pose(1:3,4);
    
    L = PlatPt(1) - BasePt(1);  
    M = PlatPt(2) - BasePt(2); 
    N = PlatPt(3) - BasePt(3);
    Initial_Leg_Length(i) = sqrt(L^2+M^2+N^2);
    Initial_Jacobian(i,:) = [L,M,N,cross(r,[L;M;N])']/Initial_Leg_Length(i);
end

if rank(Initial_Jacobian)<6 ; disp('Intial configuration of platform was singular'); return; end

% clearvars -except legs Initial_Leg_Length Initial_Pose

%%
GuessPlatPose = Initial_Pose;

    Actual_Leg_Length = leg_lengths;

    for i = 1:100                       
        ThLegLn = zeros(6,1);
        jac = zeros(6,6);               % with screws stacked horizontally   
        for ji = 1:6
            PlatPt = GuessPlatPose*[legs(1:3,ji);1];          
            r = PlatPt(1:3) - GuessPlatPose(1:3,4);
            BasePt = legs(4:6,ji);
            L = PlatPt(1) - BasePt(1);  
            M = PlatPt(2) - BasePt(2);  
            N = PlatPt(3) - BasePt(3);
            ThLegLn(ji) = sqrt(L^2+M^2+N^2);
            jac(ji,:) = [L,M,N,cross(r,[L;M;N])']/ThLegLn(ji);
        end

        error = Actual_Leg_Length - ThLegLn;
        
        if norm(error) < 1e-8
            break
            
            % breaking the loop in case the error in leg lengths is extremely
            % small
        end

        if rank(jac) < 6
            disp('Jacobian turned out to be singular....  Stopping')
            %GuessPlatPose = [0 0 0 0; 0 0 0 0; 0 0 0 170.025; 0 0 0 0];
            break
        end
        
        dlta = jac\error;
        GuessPlatPose = pose_update(dlta, GuessPlatPose);

    end
    disp(GuessPlatPose)
    disp(ThLegLn)
    disp(jac)
    A = GuessPlatPose;
end