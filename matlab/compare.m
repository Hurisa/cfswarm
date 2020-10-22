
close all;
clc;
clear
hormoneFile = uigetfile(pwd, 'Select the hormone.mat file');
yuragiFile  = uigetfile(pwd, 'Select the yuragi.mat file');
hormone=load(hormoneFile);
yuragi=load(yuragiFile);

fontsize=12;
Timetotal=180;
xo(1)=Timetotal/size(yuragi.nPoints,1);
xo(2)=Timetotal/size(hormone.nPoints,1);

xo1Vector=0:xo(1):Timetotal;
xo2Vector=0:xo(2):Timetotal;

[val,pos]=min([size(xo1Vector,2),size(xo2Vector,2)]);

x=0:xo(pos):Timetotal;

real=false;

pT=1;

% %% Fit Jumps
% %[fitobject,gof,output]=fit(xdata',ydata','power1')
% %N=length(yuragi.nJumps);
% figure('Renderer', 'painters', 'Position', [1000 1200 800 300])
% hold on
% box on
% grid on
% axis([0,7,0,40])
% %yJumps=[yuragi.nJumps{:}];
% yJumps=[];
% for n=1:length(yuragi.nJumps)
%     yJumps=[yJumps,yuragi.nJumps{n}(1:round(pT*length(yuragi.nJumps{n})))];
% end
% [yN,yedges,ybin]=histcounts(yJumps(1:pT*length(yJumps)));
% yxdata=yedges(2:length(yedges));
% yydata=yN/length(yuragi.nJumps);
% [yfitobject,ygof,youtput]=fit(yxdata',yydata','power1');
% bar(yxdata,yydata,'LineStyle','none','FaceColor',[71,0,179]./255,'HandleVisibility','off')
% p=plot(yfitobject,'k-');
% set(p,'LineWidth',2);
% set(p,'color',[255, 102, 51]./255)
% 
% xlabel('Walk Length (W) [m]','FontWeight','bold')
% ylabel('#W','FontWeight','bold')
% legend('Power Law Fit Curve')
% txt1 = ['y = ' num2str(round(yfitobject.a,2)) 'x^{' num2str(round(yfitobject.b,2)) '}'];
% text(3, 15, txt1,'FontSize',fontsize);
% xa = [.45 .35];
% ya = [.4 .2];
% annotation('arrow',xa,ya)
% set(gca,'FontSize',fontsize)
% %-------------
% figure('Renderer', 'painters', 'Position', [1000 1200 800 300])
% hold on
% box on
% grid on
% axis([0,7,0,40])
% %hJumps=[hormone.nJumps{:}];
% hJumps=[];
% for n=1:length(hormone.nJumps)
%     hJumps=[hJumps,hormone.nJumps{n}(1:round(pT*length(hormone.nJumps{n})))];
% end
% [hN,hedges,hbin]=histcounts(hJumps(1:pT*length(hJumps)));
% hxdata=hedges(2:length(hedges));
% hydata=hN/length(hormone.nJumps);
% [hfitobject,hgof,houtput]=fit(hxdata',hydata','power1');
% 
% bar(hxdata,hydata,'LineStyle','none','FaceColor',[0,153,0]./255,'HandleVisibility','off')
% p=plot(hfitobject);
% set(p,'LineWidth',2)
% set(p,'color',[255, 102, 51]./255)
% 
% xlabel('Walk Length (W) [m]','FontWeight','bold')
% ylabel('#W','FontWeight','bold')
% legend('Power Law Fit Curve')
% txt1 = ['y = ' num2str(round(hfitobject.a,2)) 'x^{' num2str(round(hfitobject.b,2)) '}'];
% text(2.5, 18, txt1,'FontSize',fontsize);
% xa = [.4 .28];
% ya = [.43 .28];
% annotation('arrow',xa,ya)
% xticks(0:2.5:15)
% set(gca,'FontSize',fontsize)
% -------------------------------------------------------------------------------------------------------
% -------------------------------------------------------------------------------------------------------
 %% p-value
 hPts=zeros(1,length(x));pPts=zeros(1,length(x));
% hCls=zeros(1,length(x));pCls=zeros(1,length(x));
% hEff=zeros(1,length(x));pEff=zeros(1,length(x));
% if real
% hEnEff=zeros(1,length(x));pEnEff=zeros(1,length(x));
% end
% %hTEff=zeros(1,length(x));pTEff=zeros(1,length(x));
for tt=1:length(x)-1
    [hPts(tt), pPts(tt)] = kstest2(yuragi.nPoints(tt,:),hormone.nPoints(tt,:));
    %[hCls(tt), pCls(tt)] = kstest2(yuragi.nClusters(tt,:),hormone.nClusters(tt,:));
    %[hTEff(tt), pTEff(tt)] = kstest2(yuragi.Times(tt,:),hormone.Times(tt,:));
    
%     x1=yuragi.nPoints(tt,:)./yuragi.Distances(tt,:);    x1(isnan(x1))=0;
%     x2=hormone.nPoints(tt,:)./hormone.Distances(tt,:);  x2(isnan(x2))=0;
%     [hEff(tt), pEff(tt)] = kstest2(x1,x2);
%     if real
%       [hEnEff(tt), pEnEff(tt)] = kstest2(yuragi.nBattery(tt,:),hormone.nBattery(tt,:));  
    %end
end
% if real
% [pPts(tt),pCls(tt),pEff(tt),pEnEff(tt)]
% else
% [Pts(tt),pCls(tt),pEff(tt)]
% end
% pPts(length(x))=pPts(length(x)-1);
% pCls(length(x))=pCls(length(x)-1);
% pEff(length(x))=pEff(length(x)-1);
% if real
% pEnEff(length(x))=pEnEff(length(x)-1);
% end
% % %pTEff(length(x))=pTEff(length(x)-1);
% % figure('Renderer', 'painters', 'Position', [1000 1200 800 400])
% % hold on
% % box on
% % grid on
% % axis([0,Timetotal,-0.1,1.1])
% % plot(x,pCls,'Linewidth',2)
% % plot(x,pEff,'Linewidth',2)
% % %plot(x,pTEff,'c','Linewidth',2)
% % plot(x,pPts,'k-.','Linewidth',2)
% % 
% % legend({'Patches Found','Efficiency','Rewards Found'},'Location','east')
% % xlabel('Time [s]','FontWeight','bold')
% % ylabel('p-value','FontWeight','bold')
% % set(gca,'FontSize',fontsize)
% % title('p-value over time','FontWeight','bold')
% -------------------------------------------------------------------------------------------------------
% -------------------------------------------------------------------------------------------------------
%% Rewards
figure('Renderer', 'painters', 'Position', [1000 1200 800 400])
hold on
box on
grid on
axis([0,Timetotal,0,900])
%-----> Means
hmeanPts=mean(hormone.nPoints,2);
hlast=hmeanPts(size(hmeanPts,1),1); 
hmeanPts(size(hmeanPts,1)+1,1)=hlast;


ymeanPts=mean(yuragi.nPoints,2);
ylast=ymeanPts(size(ymeanPts,1),1);
ymeanPts(size(ymeanPts,1)+1,1)=ylast;

%----> Standard Deviations
hstdPts=std(hormone.nPoints,0,2);
hlast=hstdPts(size(hstdPts,1),1);
hstdPts(size(hstdPts,1)+1,1)=hlast;
hmaxPts=hmeanPts+hstdPts;
hminPts=hmeanPts-hstdPts;

ystdPts=std(yuragi.nPoints,0,2);
ylast=ystdPts(size(ystdPts,1),1);
ystdPts(size(ystdPts,1)+1,1)=hlast;
ymaxPts=ymeanPts+ystdPts;
yminPts=ymeanPts-ystdPts;

%----> Plots

x2 = [x, fliplr(x)];inBetween = [hmaxPts(1:length(x))', fliplr(hminPts(1:length(x))')];
fill(x2, inBetween, [0, 230, 0]./255,'FaceAlpha',0.2,'LineStyle','none','HandleVisibility','off');
plot(x',hmeanPts(1:length(x)),'color',[0,153,0]./255,'Linewidth',2.0)

inBetween = [ymaxPts(1:length(x))', fliplr(yminPts(1:length(x))')];
fill(x2, inBetween, [224, 204, 255]./255,'FaceAlpha',0.5,'LineStyle','none','HandleVisibility','off');
plot(x',ymeanPts(1:length(x)),'color',[71,0,179]./255,'Linewidth',2.0)
legend({'ELW','Yuragi'},'Location','Northwest')
xlabel('Time [s]','FontWeight','bold')
ylabel('Rewards Found','FontWeight','bold')
set(gca,'FontSize',fontsize)
title('Evolution of Average Number of Rewards Found','FontWeight','bold')
% -------------------------------------------------------------------------------------------------------
% -------------------------------------------------------------------------------------------------------
% %% Clusters
% figure('Renderer', 'painters', 'Position', [1000 1200 800 400])
% hold on
% box on
% grid on
% axis([0,Timetotal,0,3])
% %-----> Means
% hmeanCls=mean(hormone.nClusters,2);
% hlast=hmeanCls(size(hmeanCls,1),1);
% hmeanCls(size(hmeanCls,1)+1,1)=hlast;
% 
% ymeanCls=mean(yuragi.nClusters,2);
% ylast=ymeanCls(size(ymeanCls,1),1);
% ymeanCls(size(ymeanCls,1)+1,1)=ylast;
% 
% %----> Standard Deviations
% hstdCls=std(hormone.nClusters,0,2);
% hlast=hstdCls(size(hstdCls,1),1);
% hstdCls(size(hstdCls,1)+1,1)=hlast;
% hmaxCls=hmeanCls+hstdCls;
% hminCls=hmeanCls-hstdCls;
% 
% ystdCls=std(yuragi.nClusters,0,2);
% ylast=ystdCls(size(ystdCls,1),1);
% ystdCls(size(ystdCls,1)+1,1)=hlast;
% ymaxCls=ymeanCls+ystdCls;
% yminCls=ymeanCls-ystdCls;
% 
% 
% %--> Plots
% inBetween = [hmaxCls(1:length(x))', fliplr(hminCls(1:length(x))')];
% fill(x2, inBetween, [0, 230, 0]./255,'FaceAlpha',0.2,'LineStyle','none','HandleVisibility','off');
% 
% 
% inBetween = [ymaxCls(1:length(x))', fliplr(yminCls(1:length(x))')];
% fill(x2, inBetween, [224, 204, 255]./255,'FaceAlpha',0.5,'LineStyle','none','HandleVisibility','off');
% plot(x',hmeanCls(1:length(x)),'color',[0,153,0]./255,'Linewidth',2.0)
% plot(x',ymeanCls(1:length(x)),'color',[71,0,179]./255,'Linewidth',2.0)
% 
% legend({'ELW','Yuragi'},'Location','SouthEast')
% xlabel('Time [s]','FontWeight','bold')
% ylabel('Rewards Found','FontWeight','bold')
% 
% set(gca,'FontSize',fontsize)
% title('Evolution of Average Number of Clusters Found','FontWeight','bold')
% 
% %% Effciency
% figure('Renderer', 'painters', 'Position', [1000 1200 800 400])
% hold on
% box on
% grid on
% axis([0,Timetotal,0,15])
% %---> Means
% hmeanEff=mean(hormone.nPoints,2)./mean(hormone.Distances,2);
% hmeanEff(isnan(hmeanEff))=0;
% hlast=hmeanEff(size(hmeanEff,1),1);
% hmeanEff(size(hmeanEff,1)+1,1)=hlast;
% 
% 
% ymeanEff=mean(yuragi.nPoints,2)./mean(yuragi.Distances,2);
% ymeanEff(isnan(ymeanEff))=0;
% ylast=ymeanEff(size(ymeanEff,1),1);
% ymeanEff(size(ymeanEff,1)+1,1)=ylast;
% 
% %---> Standard Deviations
% hstdEff=std(hormone.nPoints./hormone.Distances,0,2);
% hlast=hstdEff(size(hstdEff,1),1);
% hstdEff(size(hstdEff,1)+1,1)=hlast;
% hmaxEff=hmeanEff+hstdEff;hmaxEff(isnan(hmaxEff))=0;
% hminEff=hmeanEff-hstdEff;hminEff(isnan(hminEff))=0;
% 
% ystdEff=std(yuragi.nPoints./yuragi.Distances,0,2);
% ylast=ystdEff(size(ystdEff,1),1);
% ystdEff(size(ystdEff,1)+1,1)=ylast;
% ymaxEff=ymeanEff+ystdEff;ymaxEff(isnan(ymaxEff))=0;
% yminEff=ymeanEff-ystdEff;yminEff(isnan(yminEff))=0;
% 
% % ---> Plots
% inBetween = [hmaxEff(1:length(x))', fliplr(hminEff(1:length(x))')];
% fill(x2, inBetween, [0, 230, 0]./255,'FaceAlpha',0.2,'LineStyle','none','HandleVisibility','off');
% 
% inBetween = [ymaxEff(1:length(x))', fliplr(yminEff(1:length(x))')];
% fill(x2, inBetween, [224, 204, 255]./255,'FaceAlpha',0.5,'LineStyle','none','HandleVisibility','off');
% 
% plot(x',hmeanEff(1:length(x)),'color',[0,153,0]./255,'Linewidth',2.0)
% plot(x',ymeanEff(1:length(x)),'color',[71,0,179]./255,'Linewidth',2.0)
% legend({'ELW','Yuragi'},'Location','NorthEast')
% xlabel('Time [s]','FontWeight','bold')
% ylabel('Search Efficiency','FontWeight','bold')
% set(gca,'FontSize',fontsize)
% title('Evolution of Average Search Efficiency','FontWeight','bold')
% %%t
% 
% %% Battery Effciency
% figure('Renderer', 'painters', 'Position', [1000 1200 800 400])
% hold on
% box on
% grid on
% axis([0,Timetotal,0,600])
% %---> Means
% hmeanEff=mean(hormone.nPoints,2)./mean(hormone.nBattery(1,:)-hormone.nBattery,2);
% hmeanEff(isnan(hmeanEff))=0;
% hmeanEff(isinf(hmeanEff))=0;
% hlast=hmeanEff(size(hmeanEff,1),1);
% hmeanEff(size(hmeanEff,1)+1,1)=hlast;
% 
% 
% ymeanEff=mean(yuragi.nPoints,2)./mean(yuragi.nBattery(1,:)-yuragi.nBattery,2);
% ymeanEff(isnan(ymeanEff))=0;
% ymeanEff(isinf(ymeanEff))=0;
% ylast=ymeanEff(size(ymeanEff,1),1);
% ymeanEff(size(ymeanEff,1)+1,1)=ylast;
% 
% %---> Standard Deviations
% hstdEff=std(hormone.nPoints./(hormone.nBattery(1,:)-hormone.nBattery),0,2);
% hlast=hstdEff(size(hstdEff,1),1);
% hstdEff(size(hstdEff,1)+1,1)=hlast;
% hmaxEff=hmeanEff+hstdEff;hmaxEff(isnan(hmaxEff))=0;hmaxEff(isinf(hmaxEff))=0;
% hminEff=hmeanEff-hstdEff;hminEff(isnan(hminEff))=0;hminEff(isinf(hminEff))=0;
% 
% ystdEff=std(yuragi.nPoints./(yuragi.nBattery(1,:)-yuragi.nBattery),0,2);
% ylast=ystdEff(size(ystdEff,1),1);
% ystdEff(size(ystdEff,1)+1,1)=ylast;
% ymaxEff=ymeanEff+ystdEff;ymaxEff(isnan(ymaxEff))=0;ymaxEff(isinf(ymaxEff))=0;
% yminEff=ymeanEff-ystdEff;yminEff(isnan(yminEff))=0;yminEff(isinf(yminEff))=0;
% 
% % ---> Plots
% inBetween = [hmaxEff(1:length(x))', fliplr(hminEff(1:length(x))')];
% fill(x2, inBetween, [0, 230, 0]./255,'FaceAlpha',0.2,'LineStyle','none','HandleVisibility','off');
% 
% inBetween = [ymaxEff(1:length(x))', fliplr(yminEff(1:length(x))')];
% fill(x2, inBetween, [224, 204, 255]./255,'FaceAlpha',0.5,'LineStyle','none','HandleVisibility','off');
% 
% plot(x',hmeanEff(1:length(x)),'color',[0,153,0]./255,'Linewidth',2.0)
% plot(x',ymeanEff(1:length(x)),'color',[71,0,179]./255,'Linewidth',2.0)
% legend({'ELW','Yuragi'},'Location','NorthEast')
% xlabel('Time [s]','FontWeight','bold')
% ylabel('Energy Efficiency','FontWeight','bold')
% set(gca,'FontSize',fontsize)
% title('Evolution of Average Energy Efficiency','FontWeight','bold')
% %%t
% 
% %% Time Efficiency
% % figure('Renderer', 'painters', 'Position', [1000 1200 800 400])
% % hold on
% % box on
% % grid on
% % axis([0,Timetotal,0,4])
% % % -----> Means
% % hmeanTeff=mean(hormone.Times,2);hmeanTeff(isinf(hmeanTeff))=0;
% % hlast=hmeanTeff(size(hmeanTeff,1),1);
% % hmeanTeff(size(hmeanTeff,1)+1,1)=hlast;
% % 
% % 
% % ymeanTeff=mean(yuragi.Times,2);
% % ylast=ymeanTeff(size(ymeanTeff,1),1);
% % ymeanTeff(size(ymeanTeff,1)+1,1)=ylast;
% % 
% % %----> Standard Deviations
% % hstdTeff=std(hormone.Times,0,2);hstdTeff(isnan(hstdTeff))=0;
% % hlast=hstdTeff(size(hstdTeff,1),1);
% % hstdTeff(size(hstdTeff,1)+1,1)=hlast;
% % hmaxTeff=hmeanTeff+hstdTeff;
% % hminTeff=hmeanTeff-hstdTeff;
% % 
% % ystdTeff=std(yuragi.Times,0,2);
% % ylast=ystdTeff(size(ystdTeff,1),1);
% % ystdTeff(size(ystdTeff,1)+1,1)=hlast;
% % ymaxTeff=ymeanTeff+ystdTeff;
% % yminTeff=ymeanTeff-ystdTeff;
% % 
% % % ----> Plots
% % 
% % x2 = [x, fliplr(x)];inBetween = [hmaxTeff', fliplr(hminTeff')];
% % fill(x2, inBetween, [0, 230, 0]./255,'FaceAlpha',0.2,'LineStyle','none','HandleVisibility','off');
% % plot(x',hmeanTeff,'color',[0,153,0]./255,'Linewidth',2.0)
% % 
% % inBetween = [ymaxTeff', fliplr(yminTeff')];
% % fill(x2, inBetween, [224, 204, 255]./255,'FaceAlpha',0.5,'LineStyle','none','HandleVisibility','off');
% % plot(x',ymeanTeff,'color',[71,0,179]./255,'Linewidth',2.0)
% % legend({'ELW','Yuragi'},'Location','Northwest')
% % xlabel('Time [s]','FontWeight','bold')
% % ylabel('Rewards Found','FontWeight','bold')
% % set(gca,'FontSize',fontsize)
% % title('Evolution of Average Time Efficiency','FontWeight','bold')
% 
% 
% 
% 
% 











