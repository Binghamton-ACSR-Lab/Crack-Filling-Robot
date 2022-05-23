%% SCC planning with known target information
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

% function [res] = myCrack1_knownTarget(img_n,Gaussb)%% Initial Setup 

clc
close all
clearvars -except k 

ress=[];
for sa=1%:10
clearvars -except sa ress 

global reebEdge reebCell crackEdge allNode vertical spdist spdist2 s a total_length
inpxMap = @(x) fix(((x)*25.4)/2);
pxinMap = @(x) round((x)*2/25.4,1);
mmpxMap = @(x) fix(((x)/2));
pxmmMap = @(x) round((x)*2,1);

warning off

GCC=0;

worki=true; 
sim =false;
plt=false;

botD = 48;                              % 48" Diameter
footD= 7;                               % 7" Diameter
sensD= 4.5*12;                          % 6*12" Diameter

r1=inpxMap(botD/2);                     % Robot Radius 
a=inpxMap(footD/2);                     % Footprint Radius
s=inpxMap(sensD/2);                     % Sensor Range Radius

vertical = @(P) all(P(1,1)==P(:,1));
spdist = @(P,Ps) sqrt((P(1,1)-Ps(:,1)).^2 + (P(1,2)-Ps(:,2)).^2);
spdist2 = @(Ps1,Ps2) sqrt((Ps1(:,1)-Ps2(:,1)).^2 + (Ps1(:,2)-Ps2(:,2)).^2);
total_length = @(Ps) sum(sqrt(sum(diff(Ps).*diff(Ps),2)));


%% Importing Map

% Uniform Distributipn
den = [35,45,50,65,80,90,95,100];Gau=0;k=8;
img_n=['myCrack' num2str(k) '_' num2str(den(k)) '_1'] ;
[node,edgeList,ttt]=Image_planning_func_SCC(img_n);
rowBW = 2896; colBW = 3048; 
crackGen = zeros(rowBW,colBW);

% % Gaussian Distributipn
% den = [35,45,50,65,80,90,95,100];sig = [5,10,20];Gau=1;b=3;%k=4;
% Gaussb ='6'; sig_n = num2str(sig(b)); den_n = num2str(den(k));
% img_n=['myCrackGauss_s' sig_n '_' den_n];
% load(['Crack Maps/Gaussian' Gaussb '/' img_n '.mat'])
% crackGen(end,end)=0;
% [node,edgeList,ttt]=Image_planning6_func(crackGen);

%%%

% close all
smflag=0;
ppn=0;iMor=0;

BW3=crackGen;
[rowBW, colBW]= size(BW3);
tol =0.00001;
colors={'y','m','c','r','g','b'};
if worki 
    fig1=figure; imshow(~crackGen);
    pos=get(gcf, 'Position');hold on;set(gcf, 'Position',[-1130,315,1185,1049+50])
    set(gca,'FontSize',20,'FontWeight','bold');
    xlabel('x (m)');ylabel('y (m)')
    axis([0-150 colBW+150 0-150 rowBW+150])
    axis on
    pbaspect([1 1 1])
    xlab=get(gca,'xtickLabel');ylab=get(gca,'ytickLabel');
    xlab={};for ppx=round(pxmmMap(linspace(0,3050,7)/1000));xlab=[xlab,num2str(ppx)];end
    ylab={};for ppx=round(pxmmMap(linspace(0,2898,7)/1000));ylab=[ylab,num2str(ppx)];end
    xticks(mmpxMap(round(pxmmMap(linspace(0,3050,7)/1000))*1000));set(gca,'xtickLabel',xlab);yticks(mmpxMap(round(pxmmMap(linspace(0,3050,7)/1000))*1000));set(gca,'ytickLabel',ylab)
    xlim([0,3050]);ylim([0,2898])
    ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);
end

if plt 
    pos=get(gcf, 'Position');hold on
    xlabel('x(ft)');ylabel('y(ft)')
    axis on
    pbaspect([1 1 1])
    xlab{1}='0';
    xlab{2}='5';
    xlab{3}='10';
    xlab{4}='15';
    xlab{5}='20';  
%     xlab(end)=[]
    ylab{1}=num2str(round(pxinMap(0/12)));
    ylab{2}=num2str(round(pxinMap(483/12)));
    ylab{3}=num2str(round(pxinMap(966/12)));
    ylab{4}=num2str(round(pxinMap(1449/12)));
    ylab{5}=num2str(round(pxinMap(1932/12)));
    ylab{6}=num2str(round(pxinMap(2415/12)));
    ylab{7}=num2str(round(pxinMap(2898/12)));
    xticks(linspace(0,3050,5));set(gca,'xtickLabel',xlab);yticks(linspace(0,2898,7));set(gca,'ytickLabel',ylab)
    ylim([0,2898])
    xlim([0,3050])
end

crackEdge=edgeList;

if worki 
    load('mycrack8_100_1_c.mat');
    kk=cellfun(@(x) plot(x(:,2),x(:,1),':k','LineWidth',3),crackRaw_c);
    ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);

    CG=plot([node(crackEdge(:,1),2)';node(crackEdge(:,2),2)'],[node(crackEdge(:,1),1)';node(crackEdge(:,2),1)'],'g--','LineWidth',3);
    CG=[CG;cellfun(@(x) plot(x([1,end],2),x([1,end],1),'pr','MarkerSize',20,'MarkerFaceColor','r'),crackRaw_c)'];
    ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);

end

if plt
    load('mycrack8_100_1_c.mat');
    k=1;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=2;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=3;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=4;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=5;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=6;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=7;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=8;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=9;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=10;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=11;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=12;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=13;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=14;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
    k=15;kk=plot(crackRaw_c{k}(:,2),crackRaw_c{k}(:,1),':g',crackRaw_c{k}([1,end],2),crackRaw_c{k}([1,end],1),'pr','MarkerSize',15,'MarkerFaceColor','r');kk(1).LineWidth=3;
%     objCrackA=union(cellfun(@(x) polybuffer(fliplr([node(x(:,1),:);node(x(:,2),:)]),'line',a),mat2cell(crackEdge,ones(size(crackEdge,1),1))));objCrackA.plot
end

if ~exist('obj_den','var')
    objCrack=union(cellfun(@(x) polybuffer(fliplr([node(x(:,1),:);node(x(:,2),:)]),'line',s),mat2cell(crackEdge,ones(size(crackEdge,1),1))));
end

Y = [0 colBW colBW 0]; X = [0 0 rowBW rowBW];
extBound = polyshape(Y, X);final=extBound;      % External Boundary
final_ws = final;
final = subtract(final,objCrack);           	% Subracting object from frame
final_ws = subtract(final_ws,objCrack);      	% Subracting object from frame                                     
final_ws=polyclean(final_ws);
final_work=final_ws;
critP=0;WSarea=572.635; 
PathEdge=[];

if worki 
    objplt=plot(objCrack,'FaceColor','y','FaceAlpha',0.1,'LineStyle','none'); 
    reg=plot(final_ws,'FaceAlpha',0.1,'LineStyle','none');
    ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);
end


%% Main
tmain=tic;

if s~=1524
%%% MCD
working = final_ws;iMor=iMor+1;
[subcritP,splitReg_work,splitReg,splitEdge]=MCD(working,final_work,final_ws,critP,smflag);

subcritP=fliplr(subcritP);

%%% Reeb Graph
splitReg_work = polyclean(splitReg_work);

[reebEdge,~,~,~,~]=Reeb(splitReg_work,subcritP,splitEdge);

rMiss=find(~ismember((1:length(subcritP))',unique(reebEdge),'rows'));r_rmin=[];indj=[];%rAreaN=[];
if ~isempty(rMiss)
    for i=1:length(rMiss)
        rdis=spdist(subcritP(rMiss(i),:),subcritP);rdis(~rdis)=inf;[~,rmin(i)]=min(rdis);

        if any(rdis<s)
            if numel(find(ismember(reebEdge,reebEdge(sum(ismember(reebEdge,rmin(i)),2)>0,:),'rows')))<3
                rAreaN=find(ismember(reebEdge,reebEdge(sum(ismember(reebEdge,rmin(i)),2)>0,:),'rows'))';%rMiss(i);rAreaN;
                if ~isempty(rAreaN)
                    rNodeN=reebEdge(rAreaN,:);
                    if size(rNodeN,1)>1
                        reebEdge(rAreaN,:)=[];r_rmin=[r_rmin;rmin(i)];%(i,:)
                        rArea=union(splitReg_work(rAreaN));splitReg_work(rAreaN)=[];%(i,:)
                        splitReg_work=[splitReg_work;rArea];reebEdge=[reebEdge;rNodeN(~ismember(rNodeN,rmin(i)))'];
                    end
                end
            end
        else
            indj=[indj,i];
        end
    end
    
    rMiss(indj)=[];
    subcritP(unique([find(~ismember(1:length(subcritP),unique(reebEdge))');rMiss;r_rmin]),:)=[];
end

[reebEdge,reebCell,reeb,reebwall,remreg]=Reeb(splitReg_work,subcritP,splitEdge);
if ~isempty(remreg);splitReg_work(remreg)=[];end

if plt
    reg=plot(splitReg_work,'FaceAlpha',0.1,'LineWidth',2,'LineStyle','-');
    for rr=1:length(reg); reg(rr).EdgeColor=reg(rr).FaceColor;end
    plreeb=plot(reeb(1:2:length(reeb(:,1)),:)',reeb((1:2:length(reeb(:,1)))+1,:)','LineWidth',2);
%     nuum1=int2str((1:length(reeb(:,1))/2)');for rr=1:length(nuum1);nuum2(rr,:)=regexprep(nuum1(rr,:),'.','_$0');end;
%     reebt=text((reeb(1:2:length(reeb(:,1)),ceil(end/2))+10)',(reeb((1:2:length(reeb(:,1)))+1,ceil(end/2))+10)', [repmat('E',length(reeb(:,1))/2,1), nuum2],'FontSize',12);clear nuum1 nuum2;
    plmcdcrt=plot(subcritP(:,2),subcritP(:,1),'r.','MarkerSize',20);
%     nuum1=int2str([1:length(subcritP)]');for rr=1:length(nuum1);nuum2(rr,:)=regexprep(nuum1(rr,:),'.','_$0');end
%     text(subcritP(:,2)+10,subcritP(:,1)+10,[repmat('C',length(subcritP),1) nuum2],'FontSize',12);clear nuum1 nuum2;
end

if worki
    if iMor~=1;if exist('pllplot','var'); delete(pllplot);end;pllplot=plot(intersect(extBound,polybuffer(PathEdge(2:end,:),'line',s)),'FaceColor','y','FaceAlpha',0.1,'LineStyle','none');end
    delete(reg);reg=plot(splitReg_work,'FaceAlpha',0.1,'LineStyle','-','LineWidth',2);for r=1:length(reg);reg(r).EdgeColor=reg(r).FaceColor;end
    plreeb=plot(reeb(1:2:length(reeb(:,1)),:)',reeb((1:2:length(reeb(:,1)))+1,:)','LineWidth',2);
    nuum1=int2str((1:length(reeb(:,1))/2)');for rr=1:length(nuum1);nuum2(rr,:)=regexprep(nuum1(rr,:),'.','_$0');end
    reebt=text((reeb(1:2:length(reeb(:,1)),ceil(end/2))+20)',(reeb((1:2:length(reeb(:,1)))+1,ceil(end/2))+70)', [repmat('E',length(reeb(:,1))/2,1), nuum2],'FontSize',20);clear nuum1 nuum2;
    
    nuum1=int2str([1:length(subcritP)]');for rr=1:length(nuum1);nuum2(rr,:)=regexprep(nuum1(rr,:),'.','_$0');end
    plmcdcrtt=text(subcritP(:,2)+20,subcritP(:,1)+70,[repmat('C',length(subcritP),1) nuum2],'FontSize',20);clear nuum1 nuum2;
    plcurPP=[];
    if exist('plmcdcrt','var');delete(plmcdcrt);end;plmcdcrt=plot(subcritP(:,2),subcritP(:,1),'r.','MarkerSize',50);
    xlim([0,3050]);ylim([0,2898])
    snapnow
end


if worki;ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn)]);end

%%% Combining the all egdes and Generatign the Path, LP Problem

allNode = [node; subcritP];No_node = length(allNode);

reebEdge=max(crackEdge(:))+reebEdge;          % Keep in mind

reebNodes=unique(reebEdge);cell=regions(final_ws);
remEdge=[];
if length(cell)>1
    for i = 1:length(cell)
        cc=refinePoly(cell(i),1);
        [~,dd]=dsearchn(cc.Vertices,fliplr(allNode(reebNodes,:)));
        if sum(dd<a)>0
            remEdge=[remEdge;combnk(reebNodes(dd<a),2)];
        end
    end
    if ~isempty(remEdge)
        remEdge=[remEdge;remEdge(:,2),remEdge(:,1)];
    end
end
else
    allNode = node;No_node = length(allNode);
    GCC=1;
end
if GCC
    allEdge = [crackEdge];
else
    allEdge = [crackEdge;reebEdge];
end
No_edge = length(allEdge);

allEdge=[allEdge;allEdge(:,2),allEdge(:,1)];
adj = full(sparse(allEdge(:, 1), allEdge(:, 2), 1, No_node, No_node));
Dist =squareform(pdist(allNode));
AdjMax=adj.*Dist;

remEdge=[]; 

[Path, weight, add,st]=ChinesePostman(adj,AdjMax,Dist,[],remEdge);
Pathe=[Path(1:end-1)' Path(2:end)'];adds=Pathe(ismember(Pathe, [add; fliplr(add)], 'rows'),:);

if worki
    [~,mm]=max(spdist2(allNode(adds(:,1),:),allNode(adds(:,2),:)));addd=adds(mm,:);
    adds(mm,:)=[];
    plpath=plot([allNode(adds(:,1),2) allNode(adds(:,2),2)]',[allNode(adds(:,1),1) allNode(adds(:,2),1)]','--b','LineWidth',3);plpathA=[];
    for dA=1:length(adds(:,1))
        plpathA=[plpathA;drawArrowHead(fliplr(allNode(adds(dA,1),:)),fliplr(allNode(adds(dA,2),:)),'b')];
    end
    xlim([0,3050]);ylim([0,2898])
end

if plt
    [~,mm]=max(spdist2(allNode(add(:,1),:),allNode(add(:,2),:)));addd=add(mm,:);
    add(mm,:)=[];
    plot([allNode(add(:,1),2) allNode(add(:,2),2)]',[allNode(add(:,1),1) allNode(add(:,2),1)]','--','LineWidth',2)
    plot (node(:,2),node(:,1),'pr','MarkerSize',10,'MarkerFaceColor','r')
    nuum1=int2str([1:length(node)]');for rr=1:length(nuum1);nuum2(rr,:)=regexprep(nuum1(rr,:),'.','_$0');end
    text(node(:,2)+5,node(:,1)+5,[repmat('N',length(node),1) nuum2],'FontSize',12); clear nuum1 nuum2 
%     nuum1=int2str([1:length(allNode)]');for rr=1:length(nuum1);nuum2(rr,:)=regexprep(nuum1(rr,:),'.','_$0');end
%     text(allNode(:,2)+5,allNode(:,1)+5,[repmat('N',length(allNode),1) nuum2],'FontSize',12); clear nuum1 nuum2 
    plot (allNode(addd(1),2),allNode(addd(1),1),'v','Color','#0072BD','MarkerSize',15,'MarkerFaceColor','#0072BD')
    plot (allNode(addd(2),2),allNode(addd(2),1),'^','Color','#77AC30','MarkerSize',15,'MarkerFaceColor','#77AC30')
end

if worki;ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn)]);end

Path = [Path(1:end-1)',Path(2:end)'];

if GCC
    ppPath=Path;
    ppPath(~(ismember(Path,add,'rows')|ismember(fliplr(Path),add,'rows')),:)=1;
    [~,rrem]=max(spdist2(allNode(ppPath(:,1),:),allNode(ppPath(:,2),:)));
    Path(rrem,:)=[];

    PathEdge=fliplr(allNode([Path(:,1);Path(end,2)],:));subXY=PathEdge;
else
    %%SCC
    %%%%%
    wall_fol= zeros(size(Path,1),1);
    %%%%%
    subXY = Boustrophedon_CellCon(Path,splitReg_work,[],[],critP,wall_fol,true,sim);

    %%% New Start End Optimization 
    subXY(end,:)=subXY(1,:); % Making/Verify Euler
    [~,m]=max(spdist2(subXY(1:end-1,:),subXY(2:end,:)));subXY(end,:)=[];
    subXY=circshift(subXY,size(subXY,1)-m);
    %%%

    PathEdge = [PathEdge;subXY];
end


ttt=[ttt,toc(tmain)];disp(ttt);ttt=sum(ttt);

if worki
    delete(plpath);delete(plpathA);delete(plmcdcrtt);delete(reebt);delete(plmcdcrt);delete(plreeb);delete(reg);delete(objplt);delete(kk);delete(CG)
    reg=plot(splitReg_work,'FaceColor','w','FaceAlpha',0.1,'LineStyle','-','LineWidth',2,'EdgeColor','#D95319');
    plot(PathEdge(:,1),PathEdge(:,2),'k--','LineWidth',2);%,'Color','#D95319')
    plot(PathEdge(end,1),PathEdge(end,2),'v','Color','#77AC30','MarkerSize',15,'MarkerFaceColor','#77AC30')
    plot(PathEdge(1,1),PathEdge(1,2),'^','Color','#0072BD','MarkerSize',15,'MarkerFaceColor','#0072BD')
    for dA=15:30:length(PathEdge(:,1))
        plpathA=[plpathA;drawArrowHead(PathEdge(dA-1,:),PathEdge(dA,:),'k')];
    end
    xlim([0,3050]);ylim([0,2898])
    ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn)]);
end

if worki
   for ren=1:length(PathEdge(:,1))
        curPt=PathEdge(ren,:);
        if exist('pctp','var');delete(pctp);delete(pctps);end
        pctps=plot(polybuffer(curPt,'points',s),'FaceColor','y','FaceAlpha',0.1,'LineStyle','-','LineWidth',1);%pctps2=plot(polybuffer(curPt,'points',s*sqrt(2)),'FaceColor','r','FaceAlpha',0.1,'LineStyle','--','LineWidth',1);
        pctp=plot(polybuffer(curPt,'points',a*sqrt(2)),'FaceColor','r','FaceAlpha',0.3,'LineStyle','-','LineWidth',1);  
        ppn=ppn+1;saveas(gcf,['Results/GIF/' img_n '_Known_' num2str(ppn) '.png']);
   end
end


if plt
    plot(PathEdge(:,1),PathEdge(:,2),'k--');%,'Color','#D95319')
    plot(PathEdge(end,1),PathEdge(end,2),'^','Color','#77AC30','MarkerSize',15,'MarkerFaceColor','#77AC30')
    plot(PathEdge(1,1),PathEdge(1,2),'v','Color','#0072BD','MarkerSize',15,'MarkerFaceColor','#0072BD')
end

%% Calculations

pathLength=total_length(PathEdge);scclen=pxinMap(pathLength)/12;
areaCover = 2*pxinMap(s/sqrt(2))/12*scclen;
Ovp_Area =areaCover-WSarea;
coverPercent=areaCover/WSarea;
ovlapPercent=Ovp_Area/WSarea;

areaCover1=(area(polybuffer(PathEdge,'lines',s/sqrt(2)))/(2898*3050))*100;

res = [areaCover1,den(k),ttt,scclen,areaCover];disp(res)

ress=[ress;res];
end

% figure, imshow(~crackGen)
% pos=get(gcf, 'Position');hold on
% xlabel('x(ft)');ylabel('y(ft)')
% [rowBW, colBW]= size(BW3);
% axis([0-150 colBW+150 0-150 rowBW+150])
% axis on
% pbaspect([1 1 1])
% set(gca,'Ydir','reverse')
% xlab=get(gca,'xtickLabel');ylab=get(gca,'ytickLabel');
% xlab=cellfun(@(x) fix(str2num(x)/(15*12)),xlab,'un',0);
% ylab=cellfun(@(x) fix(str2num(x)/(15*12)),ylab,'un',0);
% xlab=cellfun(@(x) fix(pxinMap(str2num(x))/12),xlab,'un',0);
% ylab=cellfun(@(x) fix(pxinMap(str2num(x))/12),ylab,'un',0);
% set(gca,'xtickLabel',xlab)
% set(gca,'ytickLabel',ylab)
% 
% plot(subXY(:,1),subXY(:,2),'*--');
% plot(subXY(1,1),subXY(1,2),'b^','MarkerSize',10,'MarkerFaceColor','b')
% plot(subXY(end,1),subXY(end,2),'g^','MarkerSize',10,'MarkerFaceColor','g')
% text(subXY([1,end],1)+30,subXY([1,end],2)+30,{'Start','End'})

% if GCC
%     if ~Gau
%         % Uniform Distribution
%         saveas(gcf,['Results/GCC/Uniform2/' img_n '_Known.png'])
%         save(['Results/GCC/Uniform2/' img_n '_Known.mat'],'PathEdge','res')
%     else
%         % Gaussian Distribution
%         saveas(gcf,['Results/SCC/Gaussian2/Gaussian_Results' Gaussb '/' img_n '_Known.png'])
%         save(['Results/SCC/Gaussian2/Gaussian_Results' Gaussb '/' img_n '_Known.mat'],'PathEdge','res')
%     end
% else
%     if ~Gau
%         % Uniform Distribution
%         saveas(gcf,['Results/SCC/Uniform4/' img_n '_Known.png'])
%         save(['Results/SCC/Uniform4/' img_n '_Known.mat'],'PathEdge','res')
%     else
%         % Gaussian Distribution
%         saveas(gcf,['Results/SCC/Gaussian4/Gaussian_Results' Gaussb '/' img_n '_Known.png'])
%         save(['Results/SCC/Gaussian4/Gaussian_Results' Gaussb '/' img_n '_Known.mat'],'PathEdge','res')
%     end
% end
% close all
% end


%% Functions

function polyout=polyclean(polyin)
    global s
    polyout=polyshape();
    if isscalar(polyin)
        poly = regions(polyin);
        %poly=poly(~((arrayfun(@(x) max(x.Vertices(:,1))-min(x.Vertices(:,1)),poly))<s-s/sqrt(2))); % 01/20/21 New to remove thin polygons
        poly=poly(fix(poly.area*1e-03)>10);%poly.area>100
        polyout = regJoin(poly);
    else
        poly = polyin;
        %poly=poly(~((arrayfun(@(x) max(x.Vertices(:,1))-min(x.Vertices(:,1)),poly))<s-s/sqrt(2)));
        poly=poly(fix(poly.area*1e-03)>10);%poly.area>100
        polyout = poly;
    end
end

function polyout = regJoin(polyin)
    polyout=polyshape();
    for i=1:length(polyin)
        polyout=addboundary(polyout,polyin(i).Vertices);
    end
end