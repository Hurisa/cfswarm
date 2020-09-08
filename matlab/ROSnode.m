close all;clear;clc;
load('2Cluster4x3map.mat')
rosshutdown;
rosinit;

figure(1)
set(gca,'Color','k');
%set(gca,'Ydir','reverse')
%set(gca,'Xdir','reverse')
camroll(90)
hold on
box on
%title('Virtual Reward Map','FontWeight','bold')
    xlabel('X [m]','FontWeight','bold')
    ylabel('Y [m]','FontWeight','bold')
    set(gca,'YAxisLocation','right')
axis([-2.05,2.05,-1.55,1.55]);
    for pp=1:size(x,2)
   
        scatter(x(:,pp),y(:,pp),20,[255,140,0]./255,'filled');

    
    end
sub1 = rossubscriber('/cf1/cfpose');
sub2 = rossubscriber('/cf2/cfpose');
sub3 = rossubscriber('/cf3/cfpose');
%sub4 = rossubscriber('/cf4/cfpose');
subpf = rossubscriber('/pointsFound',@updatepf);
%subpl = rossubscriber('/pointsLeft',@updatepl);
%param = rosparam("get",'points');
points=[];

global pf deltapf;
% for ii=1:size(param,2)
%     points(size(points,1)+1,:)=[param{ii}{:}];
% end

pf.X=[];    pf.Y=[];
%pl.X=[];    pl.Y=[];


pose1=receive(sub1,10);
pose2=receive(sub2,10);
pose3=receive(sub3,10);
%pose4=receive(sub4,10);
sr=0.1;
%p=scatter(points(:,1),points(:,2),2,[26,255,0]./255,'filled');
   pfPlot=scatter(pf.X,pf.Y,20,[0,255,255]./255,'filled');
 %  plPlot=scatter(pl.X,pl.Y,5,[26,255,0]./255,'filled');
while(1)
    
   h1=scatter(pose1.Position.X,pose1.Position.Y,20,'MarkerEdgeColor',[217, 200, 150]./255,'MarkerFaceColor',[217, 68, 150]./255);
   f1=rectangle('Position',[pose1.Position.X-sr/2,pose1.Position.Y-sr/2,sr,sr],'EdgeColor',[217, 200, 150]./255);
   
   h2=scatter(pose2.Position.X,pose2.Position.Y,20,'MarkerEdgeColor',[217, 68, 150]./255,'MarkerFaceColor',[217, 68, 150]./255);
   f2=rectangle('Position',[pose2.Position.X-sr/2,pose2.Position.Y-sr/2,sr,sr],'EdgeColor',[217, 68, 150]./255);
   
   h3=scatter(pose3.Position.X,pose3.Position.Y,20,'MarkerEdgeColor',[217, 68, 150]./255,'MarkerFaceColor',[217, 68, 150]./255);
   f3=rectangle('Position',[pose3.Position.X-sr/2,pose3.Position.Y-sr/2,sr,sr],'EdgeColor',[217, 68, 150]./255);
%    
%    h4=scatter(pose4.Position.X,pose4.Position.Y,20,'MarkerEdgeColor',[217, 68, 150]./255,'MarkerFaceColor',[217, 68, 150]./255);
%    f4=rectangle('Position',[pose4.Position.X-sr/2,pose4.Position.Y-sr/2,sr,sr],'EdgeColor',[217, 68, 150]./255);

   pose1=receive(sub1,10);pose2=receive(sub2,10);pose3=receive(sub3,10);%pose4=receive(sub4,10);
   
   if (deltapf==1)
   %delete(p)
   set(pfPlot,'XData',pf.X,'YData',pf.Y);
   %set(plPlot,'XData',pl.X,'YData',pl.Y);
   deltapf=0;
   end
   drawnow
   delete(h1);delete(h2);delete(h3);%delete(h4);
   delete(f1);delete(f2);delete(f3);%delete(f4);
end

function updatepf(src,msg)
global pf deltapf
    pf.X=msg.X; pf.Y=msg.Y;
    deltapf=1;
end

% function updatepl(src,msg)
% global pl
%     pl.X=msg.X; pl.Y=msg.Y;
% end