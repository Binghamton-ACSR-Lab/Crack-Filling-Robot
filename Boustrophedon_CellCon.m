%% Cell Connection function for Boustropehdon 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function PathEdge = Boustrophedon_CellCon(Path,splitReg,see,seP,init,wall_fol,known,sim)%,allNode
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

    global reebEdge reebCell crackEdge crackRaw s allNode spdist a total_length spdist2
    o=0;fl_see=false;
    if init~=0;PathEdge=init;else;PathEdge=[];end;sensor=s/sqrt(2);                                          % Square Inscribed in a circle
    
    %%% Cell Connection 
    reebT=reebEdge;ccNode=[];ccW=[];
    for i = 1:length(Path(:,1))
        EE = Path(i,1:2);
        if ismember(EE,[reebT;reebT(:,2),reebT(:,1)],'rows') 
            ind=find(ismember(reebT,[EE;fliplr(EE)],'rows'));
            if ~isempty(ind);cell=splitReg(reebCell(ind(1)));reebT(ind(1),:)=[0,0];end
            orgcell=cell;
            
            if ~wall_fol(i)
               
                [subXY,~] = BoustrophedonPath(cell,orgcell,EE,sensor,0,[],wall_fol(i),known,fl_see);%,allNode
                if ~isempty(subXY);ccNode=[ccNode;subXY(1,:);subXY(end,:)];ccW=[ccW;total_length(rmmissing(subXY))];clear subXY;end%;else;irt=ismember(reebEdge,EE,'rows')|ismember(fliplr(reebEdge),EE,'rows');reebEdge(irt,:)=[];reebT(irt,:)=[]
                
                [subXY,~] = BoustrophedonPath(cell,orgcell,EE,sensor,1,[],wall_fol(i),known,fl_see);%,allNode
                if ~isempty(subXY);ccNode=[ccNode;subXY(1,:);subXY(end,:)];ccW=[ccW;total_length(rmmissing(subXY))];clear subXY;end
                
            else
                [subXY,~] = BoustrophedonPath(cell,orgcell,EE,sensor,0,fliplr(allNode(EE(1),:)),wall_fol(i),known,fl_see);%,allNode
                if ~isempty(subXY);ccNode=[ccNode;subXY(1,:);subXY(end,:)];ccW=[ccW;total_length(rmmissing(subXY))];clear subXY;end
                
                [subXY,~] = BoustrophedonPath(cell,orgcell,EE,sensor,1,fliplr(allNode(EE(1),:)),wall_fol(i),known,fl_see);%,allNode
                if ~isempty(subXY);ccNode=[ccNode;subXY(1,:);subXY(end,:)];ccW=[ccW;total_length(rmmissing(subXY))];clear subXY;end
            end
            
        else%if known
            subXY = [fliplr(allNode(EE(1),:));fliplr(allNode(EE(2),:))];
            if ~isempty(subXY);ccNode=[ccNode;subXY(1,:);subXY(end,:);subXY(1,:);subXY(end,:)];ccW=[ccW;total_length(rmmissing(subXY));total_length(rmmissing(subXY))];clear subXY;end
        end
    end
    i=1:2:length(ccNode(:,1));ccEdge=[i',(i+1)']; 
    sDir=repmat([0,1],1,length(i)); 
    i=2:4:length(ccNode(:,1))-3;ccEdge = [ccEdge; i',(i+3)';i',(i+5)';(i+2)',(i+3)';(i+2)',(i+5)'];
    
    ccWeight = spdist2(ccNode(ccEdge(:,1),:),ccNode(ccEdge(:,2),:));ccWeight(1:length(ccW))=ccW;%%%%%%%%%
    ccG=digraph(ccEdge(:,1),ccEdge(:,2),ccWeight);
    ccDE = [ccEdge; length(ccNode(:,1)),1;];
    ccDG = sparse(ccEdge(:,1),ccEdge(:,2),ccWeight);
    [path1(1,:),d1(1)] = shortestpath(ccG,1,length(ccNode(:,1)));if ~known;d1(1)=d1(1)+spdist2(init,ccNode(1,:));end%%%%%%%%%
    [path1(2,:),d1(2)] = shortestpath(ccG,1,length(ccNode(:,1))-2);if ~known;d1(2)=d1(2)+spdist2(init,ccNode(1,:));end%%%%%%%%%
    [path1(3,:),d1(3)] = shortestpath(ccG,3,length(ccNode(:,1)));if ~known;d1(3)=d1(3)+spdist2(init,ccNode(3,:));end%%%%%%%%%
    [path1(4,:),d1(4)] = shortestpath(ccG,3,length(ccNode(:,1))-2);if ~known;d1(4)=d1(4)+spdist2(init,ccNode(3,:));end%%%%%%%%%
    [~,l]=min(d1);
    
    i=1:2:length(ccNode(:,1));%se=[];
    ccPath = path1(l,1:2:length(path1(l,:)));clear path1 d1
    se=sDir(ismember(i,ccPath)); se=se(sum(reshape(ismember([Path(:,1:2);fliplr(Path(:,1:2))],reebEdge,'rows'),[],2),2)>0);%Path(sum(reshape(ismember([Path;fliplr(Path)],reebEdge,'rows'),[],2),2)>0,:)
    seP=ccNode(ccPath,:);seP=seP(sum(reshape(ismember([Path(:,1:2);fliplr(Path(:,1:2))],reebEdge,'rows'),[],2),2)>0,:);
    %%%
    

    for i = 1:length(Path(:,1))%length(Path)-1
        EE = Path(i,1:2);%Path(i:i+1);
        if ismember(EE,[reebEdge;reebEdge(:,2),reebEdge(:,1)],'rows')
            o=o+1;%fflg=true;
            ind=find(ismember(reebEdge,[EE;fliplr(EE)],'rows'));
            if ~isempty(ind);cell=splitReg(reebCell(ind(1)));reebEdge(ind(1),:)=[0,0];end
            
            orgcell=cell;

            [subXY,~] = BoustrophedonPath(cell,orgcell,EE,sensor,se(o),init,wall_fol(i),known,fl_see);%,allNode

            if ~sim
                subXY(subXY(:,1)<sensor,1)=sensor;                  %%% Add the sensor gap
                subXY(subXY(:,1)>3048-sensor,1)=3048-sensor;
                subXY(subXY(:,2)<sensor,2)=sensor;                  %%% Add the sensor gap
                subXY(subXY(:,2)>2898-sensor,2)=2898-sensor;
            end

        else
            if isempty(PathEdge); PathEdge=fliplr(allNode(EE(1),:));end
            if (i~=size(Path,1)) && ismember(Path(i+1,1:2),[reebEdge;reebEdge(:,2),reebEdge(:,1)],'rows') && ~isempty(seP)
                % [subXY] = [PathEdge(end,:);seP(o+1,:)];
                [marker_x1,marker_y1] = addPtsLin([PathEdge(end,1),seP(o+1,1)],[PathEdge(end,2),seP(o+1,2)],s);
                subXY =[PathEdge(end,:);[marker_x1;marker_y1]';seP(o+1,:)];
            else
                if known;[subXY] = [PathEdge(end,:);fliplr(allNode(EE(2),:))];else;[subXY] = PathEdge(end,:);end%;fliplr(allNode(EE(2),:))];fflg=false;
            end
                
            if ~sim
                subXY(subXY(:,1)<sensor,1)=sensor;                  %%% Add the sensor gap
                subXY(subXY(:,1)>3048-sensor,1)=3048-sensor;
                subXY(subXY(:,2)<sensor,2)=sensor;                  %%% Add the sensor gap
                subXY(subXY(:,2)>2898-sensor,2)=2898-sensor;
            end
        end
        
        if ~isempty(subXY)
            if ~isempty(PathEdge) && ~known
                [marker_x1,marker_y1] = addPtsLin([PathEdge(end,1),subXY(1,1)],[PathEdge(end,2),subXY(1,2)],s);
                PathEdge=[PathEdge;[marker_x1;marker_y1]';subXY];init=subXY(end,:);
            else
                PathEdge=[PathEdge;subXY];init=subXY(end,:);
            end
        end
    end
end

function [marker_x,marker_y] = addPtsLin(x,y,marker_dist)
    % Adding equidistant points along Path
    dist_from_start = cumsum( [0, sqrt((x(2:end)-x(1:end-1)).^2 + (y(2:end)-y(1:end-1)).^2)] );marker_x=[];marker_y=[];
    marker_locs = marker_dist : marker_dist : dist_from_start(end);   %replace with specific distances if desired
    if ~isempty(marker_locs) && ~(length(marker_locs)==1 && marker_locs==marker_dist)
        marker_indices = interp1( dist_from_start, 1 : length(dist_from_start), marker_locs);
        marker_base_pos = floor(marker_indices);
        weight_second = marker_indices - marker_base_pos;
        marker_x = [marker_x, x(marker_base_pos) .* (1-weight_second) + x(marker_base_pos+1) .* weight_second];
        marker_y = [marker_y, y(marker_base_pos) .* (1-weight_second) + y(marker_base_pos+1) .* weight_second];
    end
end

function polyout=polyclean(polyin)
    %global s a
    polyout=polyshape();
    if isscalar(polyin)
        poly = regions(polyin);% pp=polybuffer(poly,-s/4);
        poly=poly(fix(poly.area*1e-03)>5);%poly.area>100 10
        % poly=poly(fix(pp.area)>2*a*a*pi);
        polyout = regJoin(poly);
    else
        poly = polyin;% pp=polybuffer(poly,-s/4);
        poly=poly(fix(poly.area*1e-03)>5);%poly.area>100 10
        % poly=poly(fix(pp.area)>2*a*a*pi);
        polyout = poly;
    end
end

function polyout = regJoin(polyin)
    polyout=polyshape();
    for i=1:length(polyin)
        polyout=addboundary(polyout,polyin(i).Vertices);
    end
end

function polyout = regCombine(polyin)
    polyout = polybuffer(union(polybuffer(polyin,1)),-1);
end