%% Boustrophedon Function
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function PathEdge = Boustrophedon(Path,splitReg,see,seP,init,wall_fol,known,sim)%,allNode
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

    global reebEdge reebCell crackEdge crackRaw s allNode spdist a total_length
    o=0;fl_see=false;
    if init~=0;PathEdge=init;else;PathEdge=[];end;sensor=s/sqrt(2);                                          % Square Inscribed in a circle
    for i = 1:length(Path(:,1))%length(Path)-1
        EE = Path(i,1:2);%Path(i:i+1);
        if ismember(EE,[reebEdge;reebEdge(:,2),reebEdge(:,1)],'rows')
            o=o+1;%fflg=true;
            ind=find(ismember(reebEdge,[EE;fliplr(EE)],'rows'));
            if ~isempty(ind);cell=splitReg(reebCell(ind(1)));reebEdge(ind(1),:)=[0,0];end
            
            orgcell=cell;
%             Start = EE(1); End=EE(2);
            
%             if max(orgcell.Vertices(:,1))-min(orgcell.Vertices(:,1))<a
%                 [subXY] = [fliplr(allNode(EE(1),:));fliplr(allNode(EE(2),:))];
%                 j = cell.centroid;
%                 rowBW=max(cell.Vertices(:,2));
%                 [in,~] = intersect(cell,[j(1),0;j(1),rowBW]);
%                 if isempty(in) 
%                    [in,~] = intersect(cell,[j(1)+1,0;j(1)+1,rowBW]);
%                    if isempty(in);[in,~] = intersect(cell,[j(1)-1,0;j(1)-1,rowBW]);end
%                 end
%                 if spdist(in(1,:),in(2,:))>2*s; in(:,2)=in(:,2)+[sensor -sensor]';end   %Remember!!
%                 [~,l]=min(spdist(init,in));
%                 
%                 if l==1;subXY=in;else;subXY=flipud(in);end
%                 if ~sim
%                     subXY(subXY(:,1)<sensor,1)=sensor;                  %%% Add the sensor gap
%                     subXY(subXY(:,1)>3048-sensor,1)=3048-sensor;
%                     subXY(subXY(:,2)<sensor,2)=sensor;                  %%% Add the sensor gap
%                     subXY(subXY(:,2)>2898-sensor,2)=2898-sensor;
%                 end
%             else
%                 if allNode(Start,2)>allNode(End,2) 
%                     if max(cell.Vertices(:,1))-min(cell.Vertices(:,1))<=2*s && isequal(cell,orgcell)   
%                        j = cell.centroid;
%                     else
%                        j=max(cell.Vertices(:,1)):-2*s:min(cell.Vertices(:,1));
%                        if length(j)==1; j=[max(cell.Vertices(:,1)),min(cell.Vertices(:,1))];end
%                        if isempty(j);j=mean([max(cell.Vertices(:,1)),min(cell.Vertices(:,1))]);end
%                     end
%                 else
%                    if max(cell.Vertices(:,1))-(min(cell.Vertices(:,1)))<=2*s && isequal(cell,orgcell)   
%                        j = cell.centroid;
%                    else
%                        j=min(cell.Vertices(:,1)):+2*s:max(cell.Vertices(:,1));
%                        if length(j)==1; j=[min(cell.Vertices(:,1)),max(cell.Vertices(:,1))];end
%                        if isempty(j);j=mean([max(cell.Vertices(:,1)),min(cell.Vertices(:,1))]);end
%                    end
%                 end
%                 rowBW=max(cell.Vertices(:,2));
%                 in=zeros(2);
%                 if ~isempty(see)
%                     fl_see=true;
%                     if see(o)=='t'
%                        [in,~] = intersect(cell,[j(1),0;j(1),rowBW]);
%                        if isempty(in) 
%                            [in,~] = intersect(cell,[j(1)+1,0;j(1)+1,rowBW]);
%                            if isempty(in);[in,~] = intersect(cell,[j(1)-1,0;j(1)-1,rowBW]);end
%                        end
%                     elseif see(o)=='b'
%                        [in,~] = intersect(cell,[j(1),rowBW;j(1),0]);
%                        if isempty(in) 
%                            [in,~] = intersect(cell,[j(1)+1,rowBW;j(1)+1,0]);
%                            if isempty(in);[in,~] = intersect(cell,[j(1)-1,rowBW;j(1)-1,0]);end
%                        end
%                     end
%                 else
%                     if length(init)>1
%                         [in,~] = intersect(cell,[j(1),0;j(1),rowBW]);
%                         if isempty(in) 
%                            [in,~] = intersect(cell,[j(1)+1,0;j(1)+1,rowBW]);
%                            if isempty(in);[in,~] = intersect(cell,[j(1)-1,0;j(1)-1,rowBW]);end
%                         end
%                         [~,l]=min(spdist(init,in));
%                         if l==1
%                             [in,~] = intersect(cell,[j(1),rowBW;j(1),0]);
%                             if isempty(in) 
%                                [in,~] = intersect(cell,[j(1)+1,rowBW;j(1)+1,0]);
%                                if isempty(in);[in,~] = intersect(cell,[j(1)-1,rowBW;j(1)-1,0]);end
%                             end
%                         end
%                     end
%                 end

                %%%
                % cell.plot
%                 [subXY,flag] = BoustrophedonPath2(cell,orgcell,EE,sensor,in,init,wall_fol(i),fl_see);%,allNode
                [subXY,flag] = BoustrophedonPath(cell,orgcell,EE,sensor,[],init,wall_fol(i),fl_see);%,allNode
                
                if ~sim
                    subXY(subXY(:,1)<sensor,1)=sensor;                  %%% Add the sensor gap
                    subXY(subXY(:,1)>3048-sensor,1)=3048-sensor;
                    subXY(subXY(:,2)<sensor,2)=sensor;                  %%% Add the sensor gap
                    subXY(subXY(:,2)>2898-sensor,2)=2898-sensor;
                end

                %%% New
%                 if ~isempty(subXY) && flag %&& 0
%                     % End Node
%                     aa=false;
%                     j=subXY(end,1);cell=orgcell;%cell=polybuffer(polyin,sensor,'JointType','miter');%cell=polyin;         %Fixing Vertical 
%                     k1=[min(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2))];if k1<10; k1=0;end
%                     k2=[max(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2))];
%     %                 [d1,d2]=min(spdist(subXY(end,:),[j,k1;j,k2]));
%                     if ~isempty(k1)&&~isempty(k2)
%                     if subXY(end,2)<subXY(end-1,2)
%                         d1=spdist(subXY(end,:),[j,k1]);d2=1;
%                     elseif subXY(end,2)>subXY(end-1,2)
%                         d1=spdist(subXY(end,:),[j,k2]);d2=2;
%                     else d2=0;d1=0;
%                     end
% 
%                     if d2==1 && d1>s
%                         subXY=[subXY;[j,k1+sensor]];aa=true;           % Wall follow
%                     elseif d2==2 && d1>s
%                         subXY=[subXY;[j,k2-sensor]];aa=true;           % Wall follow
%                     end
%                     if aa && cell.area>pi*s*s;cell=subtract(cell,polybuffer(subXY(end-1:end,:),'line',s));end
%                     end
%                     % Star Node
%                     j=subXY(1,1);%cell=polybuffer(polyin,sensor,'JointType','miter');%cell=polyin;         %Fixing Vertical 
%                     k1=min(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2));
%                     k2=max(cell.Vertices((cell.Vertices(:,1)>fix(j-s) & cell.Vertices(:,1)<fix(j+s)),2));
%     %                 [d1,d2]=min(spdist(subXY(1,:),[j,k1;j,k2]));
%                     if ~isempty(k1)&&~isempty(k2)
%                     if subXY(1,2)<subXY(2,2)
%                         d1=spdist(subXY(1,:),[j,k1]);d2=1;
%                     elseif subXY(1,2)>subXY(2,2)
%                         d1=spdist(subXY(1,:),[j,k2]);d2=2;
%                     else d2=0;d1=0;
%                     end
%                     if d2==1 && d1>s
%                         subXY=[[j,k1+sensor];subXY];%aa=false;           % Wall follow
%                     elseif d2==2 && d1>s
%                         subXY=[[j,k2-sensor];subXY];%aa=false;           % Wall follow
%                     end
%                     end
%                     % End Node
%                     j=subXY(end,2);                                                                                     %Fixing horizontal
%                     [~,k1]=min(cell.Vertices(:,1));[~,k2]=max(cell.Vertices(:,1));
%                     % [d1,d2]=min(spdist(subXY(1,:),[cell.Vertices(k1,:);cell.Vertices(k2,:)]));
%                     if ~isempty(k1)&&~isempty(k2)
%                     [~,d2]=min(abs([cell.Vertices(k1,1);cell.Vertices(k2,1)]-subXY(1,1)));
%                     if d2==1
%                         d1=spdist(subXY(end,:),[cell.Vertices(k2,1),j]);
%                         if cell.Vertices(k1,2)<s;j=sensor;elseif cell.Vertices(k1,2)>2898-s;j=2898-sensor;else; j=cell.Vertices(k1,2);end % New 
%                         [mm,~]=dsearchn(subXY,[cell.Vertices(k2,1)-sensor,j]);
%                         % if d1>s;subXY=[[cell.Vertices(k2,1)-sensor,j];subXY];end
%                         if d1>s
%                             if mm==1;subXY=[[cell.Vertices(k2,1)-sensor,j];subXY];elseif mm==size(subXY,1);subXY=[subXY;[cell.Vertices(k2,1)-sensor,j]];
%                             else subXY=[subXY(1:mm,:);[cell.Vertices(k2,1)-sensor,j];subXY(mm+1:end,:)];end
%                         end
%                     elseif d2==2 
%                         d1=spdist(subXY(end,:),[cell.Vertices(k1,1),j]);
%                         if cell.Vertices(k1,2)<s;j=sensor;elseif cell.Vertices(k1,2)>2898-s;j=2898-sensor;else; j=cell.Vertices(k1,2);end % New
%                         [mm,~]=dsearchn(subXY,[cell.Vertices(k1,1)+sensor,j]);
%                         % if d1>s;subXY=[[cell.Vertices(k1,1)+sensor,j];subXY];end
%                         if d1>s
%                             if mm==1;subXY=[[cell.Vertices(k1,1)+sensor,j];subXY];elseif mm==size(subXY,1);subXY=[subXY;[cell.Vertices(k1,1)+sensor,j]];
%                             else subXY=[subXY(1:mm,:);[cell.Vertices(k1,1)+sensor,j];subXY(mm+1:end,:)];end
%                         end
%                     end
%                     end
%                     % Star Node
%                     j=subXY(1,2);                                                                                     %Fixing horizontal
%                     [~,k1]=min(cell.Vertices(:,1));[~,k2]=max(cell.Vertices(:,1));
%                     % [d1,d2]=min(spdist(subXY(1,:),[cell.Vertices(k1,:);cell.Vertices(k2,:)]));
%                     if ~isempty(k1)&&~isempty(k2)
%                     [~,d2]=min(abs([cell.Vertices(k1,1);cell.Vertices(k2,1)]-subXY(1,1)));
%                     if d2==1
%                         d1=spdist(subXY(1,:),[cell.Vertices(k1,1),j]); 
%                         if cell.Vertices(k1,2)<s;j=sensor;elseif cell.Vertices(k1,2)>2898-s;j=2898-sensor;else; j=cell.Vertices(k1,2);end % New
%                         [mm,~]=dsearchn(subXY,[cell.Vertices(k1,1)+sensor,j]);
%                         % if d1>s;subXY=[[cell.Vertices(k1,1)+sensor,j];subXY];end
%                         if d1>s
%                             if mm==1;subXY=[[cell.Vertices(k1,1)+sensor,j];subXY];elseif mm==size(subXY,1);subXY=[subXY;[cell.Vertices(k1,1)+sensor,j]];
%                             else subXY=[subXY(1:mm,:);[cell.Vertices(k1,1)+sensor,j];subXY(mm+1:end,:)];end
%                         end
%                     elseif d2==2 
%                         d1=spdist(subXY(1,:),[cell.Vertices(k2,1),j]);
%                         if cell.Vertices(k1,2)<s;j=sensor;elseif cell.Vertices(k1,2)>2898-s;j=2898-sensor;else; j=cell.Vertices(k1,2);end % New
%                         [mm,~]=dsearchn(subXY,[cell.Vertices(k2,1)-sensor,j]);
%                         % if d1>s;subXY=[[cell.Vertices(k2,1)-sensor,j];subXY];end
%                         if d1>s
%                             if mm==1;subXY=[[cell.Vertices(k2,1)-sensor,j];subXY];elseif mm==size(subXY,1);subXY=[subXY;[cell.Vertices(k2,1)-sensor,j]];
%                             else subXY=[subXY(1:mm,:);[cell.Vertices(k2,1)-sensor,j];subXY(mm+1:end,:)];end
%                         end
%                     end
%                     end
%                 end
%             end
            %%%

            
        else 
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
            if ~isempty(PathEdge)
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