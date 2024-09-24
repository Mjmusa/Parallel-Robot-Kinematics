function [length] = inverse_kinematics(ax, r_B, r_P, initial_height, alpha_B, alpha_P, trans, orient)
% Project: Ultrasound Parallel Robot
% Description: This function computes the Inverse Kinematics of a UPS stewart platform 
% Mishek Musa - Graduate Research Assistant
% University of Arkansas
% last edited: December 2021
% 
% Inputs:
%   ax              - specifies the axis to plot the robot on
%   r_B             - radius of the base of the robot [mm]
%   r_P             - radius of the platform of the robot [mm]
%   intitial_height - the initial height of the robot [mm]
%   alpha_B         - half the angle between the base attachment points [deg]
%   alpha_P         - half the angle between the platform attachment points [deg]
%   trans           - the desired position vector [1x3] of the platform [mm]
%   orient          - the desired orientation vector [1x3] of the platform [deg]
%
% Outputs:
%   length          - the vector [6x1] of the required leg lengths [mm]
%
% Two versions of the inverse kinematics are presented. The straightforward
% kinematics and a modified broken down version that makes the Jacobian
% easier to compute.

%% Desired Robot Pose

trans = [trans(1); trans(2); trans(3)+initial_height];
orient = orient(:)*pi/180;

%% Define Robot Geometry

%robotGeometry;                     
JointOffset = 46.911;           % mm

alpha_B = alpha_B*pi/180;
alpha_P = alpha_P*pi/180;

theta_B = [alpha_B,... 
          alpha_B,...
          pi/3+alpha_B,... 
          pi/3-alpha_B,... 
          pi/3-alpha_B,... 
          pi/3+alpha_B];
      
theta_P = [pi/3-alpha_P,... 
          pi/3-alpha_P,...
          pi/3+alpha_P,... 
          alpha_P,... 
          alpha_P,... 
          pi/3+alpha_P];
      
Base_Pts = r_B* [ [cos(theta_B(1)), -sin(theta_B(1)), 0]',...
                     [cos(theta_B(2)), sin(theta_B(2)), 0]',...
                     [-cos(theta_B(3)), sin(theta_B(3)), 0]',...
                     [-cos(theta_B(4)), sin(theta_B(4)), 0]',...
                     [-cos(theta_B(5)), -sin(theta_B(5)), 0]',...
                     [-cos(theta_B(6)), -sin(theta_B(6)), 0]'];
                 
Plat_Pts = r_P*[ [cos(theta_P(1)), -sin(theta_P(1)), 0]',...
                      [cos(theta_P(2)), sin(theta_P(2)), 0]',...
                      [cos(theta_P(3)), sin(theta_P(3)), 0]',...
                      [-cos(theta_P(4)), sin(theta_P(4)), 0]',...
                      [-cos(theta_P(5)), -sin(theta_P(5)), 0]',...
                      [cos(theta_P(6)), -sin(theta_P(6)), 0]'];

%% Simple Inverse Kinematics

% h = sqrt((Plat_Pts(1,:) - Base_Pts(1,:)).^2 - (Plat_Pts(2,:) - Base_Pts(2,:)).^2) - Plat_Pts(3,:);
% home_pos= [0, 0, 0]';
% R_BP = rotZ(orient(3))*rotY(orient(2))*rotX(orient(1));
% LegLengths = zeros(6,1);
% for i=1:6
%     leg(:,i)= trans + home_pos + R_BP*Plat_Pts(:,i) - Base_Pts(:,i);
%     Legs(i)= norm(leg(:,i));
%     th(i) = rad2deg(acos(leg(:,i)'*R_BP*[0;0;1]/Legs(i)));
% end
% length = Legs';
% assignin('base','th',th);

%% Modified Inverse Kinematics
legs = [Plat_Pts; Base_Pts];

Rz = rotZ(orient(3));
Ry = rotY(orient(2));
Rx = rotX(orient(1));

Rot = Rz*Ry*Rx;

DesiredPose = [Rot, [trans(1); trans(2); trans(3)]; zeros(1,3), 1];

LegLengths = zeros(6,1);
Jacobian = zeros(6,6);
PlatCoords = zeros(3,6);

for i = 1:6
    % PlatPt is coordinate of leg end attached to platform, represented
    % in base frame
    PlatPt = DesiredPose*[legs(1:3,i);1];          
    BasePt = legs(4:6,i);
    PlatCoords(:,i) = PlatPt(1:3);
    % r is the vector from the base of the platform to leg attachment point on
    % the platform represented in base frame
    r = PlatPt(1:3) - trans(1:3);
    
    L = PlatPt(1) - BasePt(1);  
    M = PlatPt(2) - BasePt(2); 
    N = PlatPt(3) - BasePt(3);
    LegLengths(i) = sqrt(L^2+M^2+N^2);
    Jacobian(i,:) = [L,M,N,cross(r,[L;M;N])']/LegLengths(i);

end
assignin('base','Jac',Jacobian);
length = LegLengths;

%disp(Jacobian)

% %% Plot the Robot
% cla(ax); 
% fill3(ax, Base_Pts(1,:),Base_Pts(2,:),Base_Pts(3,:),[0.8 0.8 0.8]);
% hold(ax, 'on')
% fill3(ax, PlatCoords(1,:),PlatCoords(2,:),PlatCoords(3,:),[0.8 0.8 0.8]);
% rotate3d(ax, 'on');
% hold(ax, 'on')
% axis(ax,'equal');
% C = {'r','c','y','b','k','g'};
% 
% for i=1:6
%     line(ax, [Base_Pts(1,i) PlatCoords(1,i)],... 
%          [Base_Pts(2,i) PlatCoords(2,i)],...
%          [Base_Pts(3,i) PlatCoords(3,i)],...
%          'Color',[0.502 0.502 0.502],'LineWidth',2);
% end
% 
% % plot3(ax,trans(1),trans(2),trans(3),'r*');
% % px = 10*cos(orient(1));
% % py = 10*sin(orient(1));
% % pz = trans(3)-140;
% % plot3(ax,trans(1),trans(2),pz,'r*');
% 
% NeedleAx = [trans(1) trans(2) trans(3);trans(1) trans(2)+180*sin(orient(1)) trans(3)-180*cos(orient(1))]
% plot3(ax,NeedleAx(:,1),NeedleAx(:,2),NeedleAx(:,3),'b-*','LineWidth',2);
% 
% r = 1.25*25.4/2;
% th = 0:pi/50:2*pi;
% xunit = r*cos(th);
% yunit = r*sin(th);
% plot(ax,xunit,yunit,'r-','LineWidth',2)

%% Create Figure
plotRobot;
% hFig = figure('color','w','position',[100 50 700 500]) 
% fill3(Base_Pts(1,:),Base_Pts(2,:),Base_Pts(3,:),'w','LineWidth',1.5);
% hold('on')
% scatter3(Base_Pts(1,:),Base_Pts(2,:),Base_Pts(3,:),100,'k','LineWidth',1.5);
% hold('on')
% fill3(PlatCoords(1,:),PlatCoords(2,:),PlatCoords(3,:),'w','LineWidth',1.5);
% hold('on')
% scatter3(PlatCoords(1,:),PlatCoords(2,:),PlatCoords(3,:),100,'k','LineWidth',1.5);
% alpha(0.5);
% 
% rotate3d('on');
% hold('on')
% axis('equal');
% view(90,30)
% for i=1:6
% 
%     line([Base_Pts(1,i) PlatCoords(1,i)],... 
%          [Base_Pts(2,i) PlatCoords(2,i)],...
%          [Base_Pts(3,i) PlatCoords(3,i)],...
%          'Color','k','LineWidth',1.5);
% end
% color = get(hFig,'Color');
% set(gca,'XColor',color,'YColor',color,'ZColor',color,'TickDir','out')
% export_fig StewartPlatform.png -r600 -CMYK
end