close all;
clc;
clear;
load('4x3map.mat')
type="hormone";
saveFlag=true;
printHeatmaps=true;
disp('Opening Files...')
Folder = uigetdir(pwd, strcat("Select a folder with ", type, " .bag files"));
Files = dir(fullfile(strcat(Folder,'/'), '*.bag'));

Agents=input('number of agents: ');
%Folder = uigetdir(pwd, 'Select a folder with hormone .bag files');
N=size(Files,1);
%% get messages
 disp('Reading messages...') 
for n=1:N

   bag=rosbag(strcat(Files(n).folder,'/',Files(n).name));
   msg=select(bag,'Topic',strcat('/SwarmPose'));
   PoseMsgs{n}=readMessages(msg,'DataFormat','struct');
   
   msg=select(bag,'Topic',strcat('/SwarmBrightness'));
   BrightnessMsgs{n}=readMessages(msg,'DataFormat','struct');
   
   msg=select(bag,'Topic',strcat('/pointsFound'));
   PointsMsgs{n}=readMessages(msg,'DataFormat','struct');   
    
end

[nPoints, nPosesMsg, Brightness]=prep(PointsMsgs,PoseMsgs,BrightnessMsgs,Agents,N);

% printMetrics(nPoints,nClusters,Distance,Times,0.50)
% printMetrics(nPoints,nClusters,Distance,Times,1.00)
% if printHeatmaps
%     %heatmap(nPosesMsg,PoseMsgs,0.25,N,x,y)
%     heatmap(nPosesMsg,PoseMsgs,0.5,N,x,y)
%     %heatmap(nPosesMsg,PoseMsgs,0.75,N,x,y)
%     heatmap(nPosesMsg,PoseMsgs,1,N,x,y)
% end
if saveFlag
    save(strcat(Folder,"/swarm_",type,".mat")],'nPoints', 'nPosesMsg', 'Brightness')
end
%% heatmap
function heatmap(nPosesMsg,PoseMsgs,p,N,xp,yp)
    s=0.001;
    radius=0.1;
    x=-2:s:2;
    y=-1.5:s:1.5;
    
    heatmap=zeros(size(x'*y,1),size(x'*y,2),N);
    for nf=1:N  
        for tt=1:round(nPosesMsg*p)
            
            xpos=find(abs(x-round(PoseMsgs{nf}{tt}.Position.X,3))<1e-10);
            xmin=max(1,xpos-(radius/s)/2);
            xmax=min(length(x),xpos+(radius/s)/2);
            
            ypos=find(abs(y-round(PoseMsgs{nf}{tt}.Position.Y,3))<1e-10);
            ymin=max(1,ypos-(radius/s)/2);
            ymax=min(length(y),ypos+(radius/s)/2);
            
            heatmap(xmin:xmax,ymin:ymax,nf)=ones(length(xmin:xmax),length(ymin:ymax));
        end
    end
    heatvalue=mean(heatmap,3);
    figure('Renderer', 'painters', 'Position', [500 200 500 550])
    hold on
    axis([min(x), max(x),min(y), max(y)])
    %set(gca,'YAxisLocation','right')
    colormap(bone)
    set(gca,'YDir','normal')
    caxis([0.5,1])
    cb=colorbar('Location','southoutside');
    
    imagesc([min(x), max(x)],[min(y), max(y)],heatvalue);
    xlabel('X [m]','FontWeight','bold')
    ylabel('Y [m]','FontWeight','bold')
    cb.Position = cb.Position - [0 .101 0 0];
    camroll(270)
    for pp=1:size(xp,2)
   
        scatter(xp(:,pp),yp(:,pp),20,[255,140,0]./255,'filled');

    
    end
    hold off
    drawnow
end
%% preprocess
function [nPoints, nPosesMsg, Brightness]=prep(PointsMsgs,PoseMsgs,BrightnessMsgs,Agents,N)
disp('pre-processing...')
pause(0.01)

minMsg=1e10;
maxMsg=0;
for n=1:N
    l=length(PointsMsgs{n});
    if l<minMsg
        minMsg=l;
    end
    if l>maxMsg
        maxMsg=l;
    end
end

nPoints=zeros(maxMsg,N);
Brightness=zeros(length(BrightnessMsgs{1}),Agents,N);
interval=zeros(N,1);
nPoses=zeros(N,1);
for nf=1:N
    %--- Points
    for tt=1:length(PointsMsgs{nf})
        nPoints(tt,nf)=size(PointsMsgs{nf}{tt}.X,1);       
    end
    if minMsg<maxMsg
        nPoints(tt+1:maxMsg,nf)=max(nPoints(tt,nf),1);    
    end
    [~,interval(nf,1)]=min(nPoints(:,nf));
    if interval(nf,1)>1
        nPoints(1:interval(nf,1)-1,nf)=0;    
    end    

    nPoses(nf)=size(PoseMsgs{nf},1);
    nPosesMsg=min(nPoses);
    
    % Brightness --
    for tt=1:length(BrightnessMsgs{nf})
        Brightness(tt,:,nf)=double([BrightnessMsgs{nf}{tt}.Values.Data]);
    end
    
    
end

disp('done pre-processing...')
pause(0.01)
end



function printMetrics(nPoints,nClusters,Distances,Times,pT)
disp('-----')
fprintf('Time percentage: %0.2f \n',pT)
finish=pT*size(nPoints,1);
disp('mean effeciency and standard deviation')
effData=nPoints(finish,:)./Distances(finish,:);
fprintf('%0.3f,%0.3f ',mean(effData),std(effData))
fprintf('\n \n')
disp('mean points and standard deviation')
ptsData=nPoints(finish,:);
fprintf('%0.3f,%0.3f ',mean(ptsData),std(ptsData))
fprintf('\n \n')
disp('mean clusters and standard deviation')
clusData=nClusters(finish,:);
fprintf('%0.3f,%0.3f ',mean(clusData),std(clusData))
fprintf('\n \n')
disp('mean time efficiency and standard deviation')
TeffData=Times(finish,:);
fprintf('%0.3f,%0.3f ',mean(TeffData),std(TeffData))
fprintf('\n \n')
end


function Times=TofArrival(nPoints)


N=size(nPoints,2);
T=size(nPoints,1);

Times=zeros(T,N);
x=0:0.1:size(nPoints,1)/10;
x(length(x))=[];
for nn=1:N
    
    Times(:,nn)=nPoints(:,nn)./x';
    
end
Times(isnan(Times))=0;
     

end








