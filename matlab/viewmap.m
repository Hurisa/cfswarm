clear all; %close all; clc;
[file, path]=uigetfile('.mat','select map');
load(strcat(path,file));
figure('Renderer', 'painters', 'Position', [500 200 400 400])

plotLevy=false;
plotDetail=false;
sz=4;
fontsize=12;
hold;
axis(limits);
box on;
set(gca,'Color','k');
for p=1:size(x,2)
   
    scatter(x(:,p),y(:,p),sz,[26,255,0]./255,'filled');

    
end

xlabel('X [m]','FontWeight','bold')
ylabel('Y [m]','FontWeight','bold')
title('Validation Environment','FontWeight','bold')
set(gca,'FontSize',fontsize)
set(gca,'YAxisLocation','right')
camroll(270)
if plotDetail
    
    rectangle('Position',[120,525,100,100],'EdgeColor','w','LineWidth',2)
    t=text(150,480,'B','Color','w');    
    t.FontSize=12;
    t.FontWeight='bold';
    axes('position',[.45 .3 .3 .3]) 
    hold on
    axis([410 465 770 810])

    scatter(x(:,1),y(:,1),sz,[26,255,0]./255,'filled');
        ts=text(420,805,'Detail B');
    ts.FontSize=12;
    ts.FontWeight='bold';
    set(gca,'XTick',[], 'YTick', [])
    
    box on
end






if plotLevy
    it=1000;    
    xpos=zeros(1,it);xpos(1)=500;
    ypos=zeros(1,it);ypos(1)=500;
    for i=2:it
        [jump(i), theta, x, y]=levy(1,2,10);
        xpos(i)=x+xpos(i-1);
        ypos(i)=y+ypos(i-1);
    end   
    
    plot(xpos,ypos,'color',[217, 68, 150]./255);
    xlabel('X [m]','FontWeight','bold')
    ylabel('Y [m]','FontWeight','bold')   
end


function [jump, theta, x, y]=levy(size,mu,scale)


%mu=2;
%sigma=10.0;
muOne=mu-1;
muTwo=2-mu;
    
    for i=1:size

        U1=rand*2-1;
        U2=rand*2-1;
        U3=rand;

        U1 = U1*(pi/2);
        U2 = (U2+1)/2;
        phi = U3 * pi;
        r = (sin(muOne * U1) / (cos(U1)^(1/muOne))) * ((cos(muTwo * U1) / U2)^(muTwo / muOne));
        x(i) = r * cos(phi);
        y(i) = r * sin(phi);
        theta(i)=atan2(y(i),x(i))+pi;

        jump(i)=scale*sqrt((x(i)^2)+(y(i)^2));


    end


end