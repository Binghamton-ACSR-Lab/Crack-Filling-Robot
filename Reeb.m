%% Reeb Graph 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [reebEdge,reebCell,reeb,reebwall,remreg]=Reeb(polyinreg,critP,splitEdge)                             % Generating Reeb Graph
% drawArrow(p0,p1)
%
% Draws a simple arrow in 2D, from p0 to p1.
%
% INPUTS:
%   p0 = [x0; y0] = position of the tail
%   p1 = [x1; y1] = position of the tip
%   color = arrow color. Optional: default is black 
%       --> can be 'r','g','b','c','m','y','w', 'k' or a 1x3 color vector
%
% OUTPUTS:
%   hArrow = handle to the patch object representing the arrow
%
% Defaults:

    global a spdist spdist2
    TFin = [];reebCon = zeros(length(polyinreg));reebEdge=[];reeb=[];reebCell=[];remreg=[];reebwall=[];
    if exist('plreeb','var')
        delete(plreeb);delete(reebt)
    end
    for i = 1:length(polyinreg)
        buf=5; tt=polyinreg(i);
        if ~isempty(splitEdge)
            tt=polybuffer(tt,1);
            for j=1:length(splitEdge)
                tt=[tt,splitEdge(j)];
%                 tt=addboundary(tt,splitEdge(j).Vertices);
            end
            tt=regCombine(tt);
            tt=regions(tt);tt=tt(tt.area==max(tt.area));
        end
        
        temp = isinterior(tt,fliplr(critP));                % 60 Important
        while sum(temp)<2
            temp = isinterior(polybuffer(tt,buf),fliplr(critP));
            buf=buf+5;
            if buf==50
                break
            end
        end
%         if sum(temp)==0; remreg=i;continue;else;remreg=[];end
        if sum(temp)==0; remreg=[remreg,i];continue;end
        
        if sum(temp)>2
            t=find(temp);%t=squareform(pdist(critP(temp,:)));t(~t)=inf;
            if any(ismember(t,unique(reebEdge)))
                j=t(ismember(t,unique(reebEdge)));j=j(1);
                t(t==j)=[];
                [~,ddd]=max(spdist(critP(j,:),critP(t,:)));
                temp(t(1:end~=ddd))=0;
            else
                j=find(temp);
                combEdge = combnk(j,2);
                t=spdist2(critP(combEdge(:,1),:),critP(combEdge(:,2),:));
                combEdge=combEdge(max(t)==t,:);combEdge=combEdge(1,:)';
                temp(j(~ismember(j,combEdge)))=0;
            end
        end
        
        TFin = [TFin , temp];ind = find(temp);%[yyy,xxx]=polyinreg(i).centroid;
%         yyy=(min(polyinreg(i).Vertices(:,1))+max(polyinreg(i).Vertices(:,1)))/2;
%         xxx=(min(polyinreg(i).Vertices(:,2))+max(polyinreg(i).Vertices(:,2)))/2;
yyy=(min(polyinreg(i).Vertices(:,1))+max(polyinreg(i).Vertices(:,1)))/2;
jy=[min(polyinreg(i).Vertices(:,2));max(polyinreg(i).Vertices(:,2))];
in_p=intersect(polyinreg(i),polybuffer([[yyy;yyy],jy],'line',a));
xxx=(min(in_p.Vertices(:,2))+max(in_p.Vertices(:,2)))/2;
        bb=[critP(ind(1),:);[xxx,yyy];critP(ind(2),:)];bb_wall=[critP(ind(1),:);[xxx,yyy-50];critP(ind(2),:)];
        reebCon(ind(1),ind(2))=1;reebEdge=[reebEdge;[ind(1),ind(2)]];
        cs = csapi(bb(:,2)',bb(:,1)');cs_wall = csapi(bb_wall(:,2)',bb_wall(:,1)');reebCell= [reebCell;i];
        reeb=[reeb;fnplt(cs,2)];reebwall=[reebwall;fnplt(cs_wall,2)];%plreeb=plot(reeb(1,:),reeb(2,:),'LineWidth',2);
        %reebt=text(yyy+10,xxx+10,['E' int2str(i)]);
    end
end

function polyout = regCombine(polyin)
    polyout = polybuffer(union(polybuffer(polyin,1)),-1);
end

