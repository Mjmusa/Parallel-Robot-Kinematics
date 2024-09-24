%% Plot the Robot
fill3(ax, Base_Pts(1,:),Base_Pts(2,:),Base_Pts(3,:),[0.8 0.8 0.8]);
hold(ax, 'on')
fill3(ax, PlatCoords(1,:),PlatCoords(2,:),PlatCoords(3,:),[0.8 0.8 0.8]);
rotate3d(ax, 'on');
hold(ax, 'on')
axis(ax,'equal');
C = {'r','c','y','b','k','g'};

for i=1:6
    line(ax, [Base_Pts(1,i) PlatCoords(1,i)],... 
         [Base_Pts(2,i) PlatCoords(2,i)],...
         [Base_Pts(3,i) PlatCoords(3,i)],...
         'Color',[0.502 0.502 0.502],'LineWidth',2);
end