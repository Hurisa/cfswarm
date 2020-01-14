%% process a single bag file and replay.
bag=rosbag('test.bag');
bagselect = select(bag, 'Topic', '/SwarmPose');
msgs = readMessages(bagselect,'DataFormat','struct');

%
PosMatrix=zeros(size(msgs{1}.Poses,2),2,size(msgs,1));

for t=1:size(msgs,1)
    for p=1:size(msgs{1}.Poses,2)
    
        PosMatrix(p,:,t)=[msgs{t}.Poses(p).Position.X,msgs{t}.Poses(p).Position.Y];
    
    end
end

figure(1)
hold on
box on
axis([20 45 0 23])
for t=1:size(msgs,1)

         h=scatter(PosMatrix(:,1,t), PosMatrix(:,2,t),'MarkerEdgeColor',[217, 68, 150]./255,'MarkerFaceColor',[217, 68, 150]./255);
         pause(0.001)
         cla
    
end

