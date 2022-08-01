%% Boustrophedon Path Planner Function
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [subXY,flag] = BoustrophedonPath(cell,orgcell,reebEdge,bp_gap,dir,init,wall_fol,known,fl_see)
%
% BoustrophedonPath is a simple path generation function that analyses the topology of individual 
% cells in the workspace and generates a zigzag path within the cell. 
%
% INPUTS:
%   cell = Single cell from the decomposed workspace with a negative footprint Minkowski sum, in polygon. 
%   orgcell = Single cell from the decomposed workspace, in polygon. 
%   reebEdge = Reeb edge of the cell from Reeb Graph.  
%   bp_gap = Spacing between the vertical lines in the zigzag path, sensor range.
%   dir = Variable containing path direction for each cell.
%   init = Current position of the Robot. 
%   wall_fol =  Flag for wall follow.
%   known = Flag for SCC. 
%   fl_see = Flag for simulation.
%
% OUTPUTS:
%   subXY = Final path generated containing coordinate points in a sequential order.
%   flag = Flag for simulation.

    global spdist s allNode a total_length
    subXY=[];rowBW=max(cell.Vertices(:,2));flag=~wall_fol;cell_loj=false;%in=zeros(2);
    if wall_fol
        subXY = [subXY;wall_follow(cell,orgcell,bp_gap,fliplr(allNode(reebEdge(1),:)))];%init
%         in_p=intersect(orgcell,polybuffer([[subXY(1,1);subXY(1,1)],[0;2898]],'line',s));
%         [[subXY,min(in_p.Vertices(:,2))];subXY;[,min(in_p.Vertices(:,2))]]
        if ~isempty(subXY)
            [~,lo]=min(abs(subXY(1,1)-fliplr(allNode([reebEdge(1),reebEdge(2)],2))));
            if lo==1
                if ~isempty(subXY)
                    obj = polybuffer([init;subXY],'line',bp_gap*sqrt(2));     % Keep in mind
                    cell = subtract(cell,obj);%orgcell = subtract(orgcell,obj);orgcell = polyclean(orgcell);%cell = polyclean(cell);
                    % if cell.area==0;cell=orgcell;end
                    % cell = polyclean(cell);
                    aa=regions(cell);
                    if length(aa)>1
                        cell=aa(aa.area==max(aa.area));
                    end
                    init=subXY(end,:);
                    dir=[];  %remember to check 
                end
                Start = reebEdge(2); End=reebEdge(1);
            else
                subXY=[];flag=wall_fol;
                Start = reebEdge(2); End=reebEdge(1);
            end
        else
            Start = reebEdge(1); End=reebEdge(2);
        end
    else
        Start = reebEdge(1); End=reebEdge(2);
    end
    if ~cell.area==0 && cell.area > area(polybuffer([0,0],'points',a)) % Cell area to footprint area is new
        if allNode(Start,2)>allNode(End,2) 
           if max(cell.Vertices(:,1))-min(cell.Vertices(:,1))<=2*bp_gap
               j = cell.centroid;
           else
               j=max(cell.Vertices(:,1))-bp_gap:-2*bp_gap:min(cell.Vertices(:,1))+bp_gap;
               if isempty(j);j=mean([max(cell.Vertices(:,1)),min(cell.Vertices(:,1))]);
               else
                if (j(end)- min(cell.Vertices(:,1))>bp_gap*sqrt(2))
                    j = [j,min(cell.Vertices(:,1))+bp_gap];
                end 
               end
           end
        else
           if max(cell.Vertices(:,1))-(min(cell.Vertices(:,1)))<=2*bp_gap
                j = cell.centroid;
           else
               j=min(cell.Vertices(:,1))+bp_gap:+2*bp_gap:max(cell.Vertices(:,1))-bp_gap;
               if isempty(j);j=mean([max(cell.Vertices(:,1)),min(cell.Vertices(:,1))]);
               else
                if (max(cell.Vertices(:,1)) - j(end)>bp_gap*sqrt(2))
                    j = [j,max(cell.Vertices(:,1))-bp_gap];
                end 
               end
           end
        end
        
        jx=j;
        jy=[min(cell.Vertices(:,2));max(cell.Vertices(:,2))];

        ins=[];dis=[];
        for j=jx
% %             in=intersect(cell,[[j;j],jy]);
% %             ins=[ins,in([1,end],2)];%in(:,2)
            %%New
            in_p=intersect(cell,polybuffer([[j;j],jy],'line',s));
            in=[j,min(in_p.Vertices(:,2));j,max(in_p.Vertices(:,2))];
            ins=[ins,in(:,2)];%
            dis=[dis,total_length(in)];
        end

        ins(:,dis>=2*bp_gap)=ins(:,dis>=2*bp_gap)+[+bp_gap ;-bp_gap];
        ins(:,~(dis>=2*bp_gap))=[mean(ins(:,~(dis>=2*bp_gap)));mean(ins(:,~(dis>=2*bp_gap)))];
        
        if isempty(dir)
            if init==0
                dir=0;dir=dir==2;
            else
                [~,dir] = min(spdist(init,[[jx(1);jx(1)],ins(:,1)]));dir=dir==2;
            end
        end

        if dir 
            ins(:,mod(1:length(jx),2)~=0)=flipud(ins(:,mod(1:length(jx),2)~=0));
        else 
            ins(:,mod(1:length(jx),2)==0)=flipud(ins(:,mod(1:length(jx),2)==0));
        end

        for k=1:length(jx)
            if ins(1,k)<ins(2,k) && dis(k)>bp_gap %&& ~known
                in_space=ins(1,k):bp_gap:ins(2,k);if ~(in_space(end)==ins(2,k));in_space=[in_space,ins(2,k)];end
            elseif ins(1,k)>ins(2,k) && dis(k)>bp_gap %&& ~known
                in_space=ins(1,k):-bp_gap:ins(2,k);if ~(in_space(end)==ins(2,k));in_space=[in_space,ins(2,k)];end
            else
                in_space=ins(:,k)';
            end 
            subXY=[subXY;[repmat(jx(k),length(in_space),1),in_space']];
        end
    else
        [subXY(1),subXY(2)] = cell.centroid;  %Added New 
    end
end

function subXY = wall_follow(polyin,orgpolyin,sensor,start)                                 % Wall Follow
    global vertical spdist s a
%     [startid,~,~] = nearestvertex(polyin,start);
    if isequal(orgpolyin.Vertices,polyin.Vertices)
        working = polybuffer(polyin,-sensor);ss=sensor;
        if ~area(working)==0
            while 1
                if area(polyclean(working))==0 || size(regions(working),1)>1
                    ss=ss-10;
                    working = polybuffer(polyin,-ss);
                else
                    break
                end
            end
        end
    else
        working = polyin;%working = polybuffer(polyin,-sensor);
    end
%     working=orgpolyin;
    subXY=[];%polyin.Vertices(startid,:)
    if working.area~=0
        [corPtx,idx] = polycorner(working);%plot(working.Vertices(idx,1),working.Vertices(idx,2),'r*')
    %     [vertexid,~,~] = nearestvertex(working,start);%fliplr()
        [~,vertexid]=min(spdist(start,corPtx));vertexid=idx(vertexid);
        %temp=working.Vertices(vertexid,1)==working.Vertices(:,1);
        temp=abs(working.Vertices(:,1)-working.Vertices(vertexid,1))<2;
        temp=working.Vertices(temp,:);
        [~,t(1)]=min(temp(:,2));
        [~,t(2)]=max(temp(:,2));

        [~,t(3)]=min(spdist(working.Vertices(vertexid,:),temp(t,:)));
%         [~,vertexid]=ismember(temp(t(t(3)),:),corPtx,'rows');vertexid=idx(vertexid);
        [~,vertexid]=min(spdist(temp(t(t(3)),:),corPtx));vertexid=idx(vertexid);  %New
        % plot(working.Vertices(vertexid,1),working.Vertices(vertexid,2),'y*')
        ind = [vertexid,mod(vertexid,length(working.Vertices))+1];
        if ~vertical(working.Vertices(ind,:))
            subXY=[subXY;working.Vertices(ind(1),:)];
            while true
                vid=ind(2);
                subXY = [subXY;working.Vertices(vid,:)];%plot(subXY(end,1),subXY(end,2),'r*')
                ind2 = [ind,mod(ind(2),length(working.Vertices))+1];
                ind = [ind(2),mod(ind(2),length(working.Vertices))+1];
    %             v1=working.Vertices(ind2(2),:)-working.Vertices(ind2(1),:);
    %             v2=working.Vertices(ind2(2),:)-working.Vertices(ind2(3),:);
    %             if vertical(working.Vertices(ind,:)) || ab2v(v1,v2)<80 || ab2v(v1,v2)>280
    %             if sum(ind(1)==idx)>0 && (working.Vertices(ind(1),1)-min(working.Vertices(:,1))<s || max(working.Vertices(:,1))-working.Vertices(ind(1),1)<s)
                if sum(ind(1)==idx)>0 && (fix(working.Vertices(ind(1),1))==fix(min(working.Vertices(:,1))) || fix(max(working.Vertices(:,1)))==fix(working.Vertices(ind(1),1)))
                    if size(subXY,1)==2
                       if fix(subXY(1,1))==fix(subXY(2,1));subXY=[];%end 
                       elseif spdist(subXY(1,:),subXY(2,:))<a;subXY=[];end 
                    end
                    break
                end
            end
        else
            ind = [vertexid,mod(vertexid-2,length(working.Vertices))+1];
            if ~vertical(working.Vertices(ind,:))
                subXY=[subXY;working.Vertices(ind(1),:)];
                while true
                    vid=ind(2);
                    subXY = [subXY;working.Vertices(vid,:)];
                    ind2 = [ind,mod(ind(2)-2,length(working.Vertices))+1];
                    ind = [ind(2),mod(ind(2)-2,length(working.Vertices))+1];
    %                 v1=working.Vertices(ind2(2),:)-working.Vertices(ind2(1),:);
    %                 v2=working.Vertices(ind2(2),:)-working.Vertices(ind2(3),:);
    %                 if vertical(working.Vertices(ind,:)) || ab2v(v1,v2)<80 || ab2v(v1,v2)>280
    %                 if sum(ind(1)==idx)>0 && (working.Vertices(ind(1),1)-min(working.Vertices(:,1))<s || max(working.Vertices(:,1))-working.Vertices(ind(1),1)<s)
                    if sum(ind(1)==idx)>0 && (fix(working.Vertices(ind(1),1))==fix(min(working.Vertices(:,1))) || fix(max(working.Vertices(:,1)))==fix(working.Vertices(ind(1),1)))
                        if size(subXY,1)==2
                           if fix(subXY(1,1))==fix(subXY(2,1));subXY=[];%end 
                           elseif spdist(subXY(1,:),subXY(2,:))<a;subXY=[];end 
                        end
                        break
                    end
                end
            end
        end
    end
    if ~isempty(subXY)
        if fix(subXY(end,1))==fix(subXY(end-1,1))%while
            subXY(end,:)=[];
        end
    end
    
%     %%% New 
%     aa=true;
%     if ~isempty(subXY)
%         % End Node
%         j=subXY(end,1);cell=orgpolyin;%cell=polybuffer(polyin,sensor,'JointType','miter');%cell=polyin;         %Fixing Vertical 
%         k1=[min(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2))];
%         k2=[max(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2))];
% %         [d1,d2]=min(spdist(subXY(end,:),[j,k1;j,k2]));
%         if subXY(end,2)<subXY(end-1,2)
%             d1=spdist(subXY(end,:),[j,k1]);d2=1;
%         elseif subXY(end,2)>subXY(end-1,2)
%             d1=spdist(subXY(end,:),[j,k2]);d2=2;
%         else d2=0;d1=0;
%         end
%         if d2==1 && d1>s
%             subXY=[subXY;[j,k1+sensor]];%aa=false;           % Wall follow
%         elseif d2==2 && d1>s
%             subXY=[subXY;[j,k2-sensor]];%aa=false;           % Wall follow
%         end
%         % Star Node
% %         j=subXY(1,1);%cell=polybuffer(polyin,sensor,'JointType','miter');%cell=polyin;         %Fixing Vertical 
% %         k1=min(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2));
% %         k2=max(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2));
% %         [d1,d2]=min(spdist(subXY(1,:),[j,k1;j,k2]));
% %         if d2==1 && d1>s
% %             subXY=[[j,k1+s];subXY];aa=false;           % Wall follow
% %         elseif d2==2 && d1>s
% %             subXY=[[j,k2-s];subXY];aa=false;           % Wall follow
% %         end
%         
%         % End Node
%         j=subXY(end,2);                                                                                     %Fixing horizontal
%         [~,k1]=min(cell.Vertices(:,1));[~,k2]=max(cell.Vertices(:,1));
%         % [d1,d2]=min(spdist(subXY(1,:),[cell.Vertices(k1,:);cell.Vertices(k2,:)]));
% %         [~,d2]=min(abs([cell.Vertices(k1,1);cell.Vertices(k2,1)]-subXY(1,1)));
%         if subXY(1,1)>subXY(end,1);d2=1;else;d2=2;end
%         if d2==2
%             d1=spdist(subXY(end,:),[cell.Vertices(k2,1),j]);
%             if d1>s;subXY=[subXY;[cell.Vertices(k2,1)-sensor,j]];end
%         elseif d2==1 
%             d1=spdist(subXY(end,:),[cell.Vertices(k1,1),j]);
%             if d1>s;subXY=[subXY;[cell.Vertices(k1,1)+sensor,j]];end
%         end
%         
%     end
%     %%%
    
    marker_x2=[];marker_y2=[];
    for h=1:size(subXY,1)-1
        [m,n] = addPtsLin(subXY([h h+1],1)',subXY([h h+1],2)',s);
        marker_x2=[marker_x2,[subXY(h,1) m]];marker_y2=[marker_y2,[subXY(h,2) n]];
    end
    if ~isempty(subXY)
        subXY=[[marker_x2;marker_y2]';subXY(end,:)];
    end
%     subXY(:,2)=subXY(:,2)+s;
    
    % Reduces number of points in the wall follow
%     if ~isempty(subXY)
%         [bb(:,1),bb(:,2)]=boundary(polybuffer(subXY,'line',a));
%         [waypoints,~]=pathfinder_v(subXY(1,:),subXY(end,:),bb);
%         subXY=waypoints;
%     end
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

function [corPtx,idx] = polycorner(polyin)
    corPtx=[];idx=[];global vertical
%     for i = 1: length(polyin.Vertices)
%         ind = [mod(i-2,length(polyin.Vertices))+1,i,mod(i,length(polyin.Vertices))+1];
%         %plot(polyin.Vertices(i,1),polyin.Vertices(i,2),'r*')
%         v1=polyin.Vertices(ind(2),:)-polyin.Vertices(ind(1),:);
%         v2=polyin.Vertices(ind(2),:)-polyin.Vertices(ind(3),:);%ab2v(v1,v2)
%         if vertical(polyin.Vertices(ind(2:3),:)) || ab2v(v1,v2)<=95 || ab2v(v1,v2)>=265
%             corPtx=[corPtx;polyin.Vertices(i,:)];
%             idx=[idx;i];
%         end
%     end
    i=convhull(rmmissing(polyin.Vertices));
    corPtx=[corPtx;polyin.Vertices(i,:)];
	idx=[idx;i];
end

function [marker_x,marker_y] = addPtsLin(x,y,marker_dist)
    % Adding equidistant points along Path
    dist_from_start = cumsum( [0, sqrt((x(2:end)-x(1:end-1)).^2 + (y(2:end)-y(1:end-1)).^2)] );marker_x=[];marker_y=[];
    marker_locs = marker_dist : marker_dist : dist_from_start(end);   %replace with specific distances if desired
    if ~isempty(marker_locs) && (length(rmmissing(x))>1)
        marker_indices = interp1( dist_from_start, 1 : length(dist_from_start), marker_locs);
        marker_base_pos = floor(marker_indices);
        weight_second = marker_indices - marker_base_pos;
        marker_x = [marker_x, x(marker_base_pos) .* (1-weight_second) + x(marker_base_pos+1) .* weight_second];
        marker_y = [marker_y, y(marker_base_pos) .* (1-weight_second) + y(marker_base_pos+1) .* weight_second];
    end
end