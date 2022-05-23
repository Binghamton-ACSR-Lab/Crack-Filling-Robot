%% Crack Map Extraction Funcion 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [crackRaw,line,pointX,pointY] = compCrack(I,endP,dir_map,colors)
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

    line=[];
    pointX=[];
    pointY=[];
    i=1;
    while ~isempty(endP)
        %     i=list(1);
        start=endP(1,:);    % Selecting the first row of elements
        row=start(1);
        col=start(2);
        endP(1,:)=[];       % Removing the first row of elements
        I(row,col)=0;
        tempX=[];
        tempY=[];
        %     plot(col,row,'s','color',colors{mod(i,7)+1})
        %     imshow(I);
%         plot(endP(:,2),endP(:,1),'rs')
        %                 for j=row:width
        %                     for k=col:height
        %     display('start crack');
        %     start
        crack=[];
        while (1)
            skp=true;
            tempX=[tempX row];
            tempY=[tempY col];
            b = [I(row-1,col-1:col+1) ...
                I(row, col-1) I(row, col+1)...
                I(row+1,col-1:col+1)];             % b captures the 8 surounding nodes of the end point
            number=sum(b);                         % adds all the elements in b
            if number>=2 % intersection point
                %             InterPoint=[InterPoint;row,col];
                deleteInd=[];
                [nextall]=find(b);                 % Finds which elements in b has value 1
                for j=1:length(nextall)

                    rowb=row+dir_map(nextall(j),1);
                    colb=col+dir_map(nextall(j),2);
                    I(rowb,colb)=0;
                end
                count=length(nextall);
    %             tempnext=nextall;
                for j=1:count %check false intersection point
                    dir=nextall(j);
                    rowb=row+dir_map(dir,1);
                    colb=col+dir_map(dir,2);
                    bb = [I(rowb-1,colb-1:colb+1) ...
                        I(rowb, colb-1) I(rowb, colb+1)...
                        I(rowb+1,colb-1:colb+1)];
                    [indb]=find(bb);
                    if isempty(indb)
                        deleteInd=[deleteInd j];
                        continue;
                    end

                end
                nextall(deleteInd)=[];
                if length(nextall)==1


                    row=row+dir_map(nextall,1);
                    col=col+dir_map(nextall,2);
                    crack=[crack; row col];
%                     plot(col,row,'*','color',colors{mod(i,6)+1})
                    continue;
                end
                EndPoint=[row col];
                %             nextall=find(b);
                %             sP=size(endP,1);
                for j=1:length(nextall)
                    sP=size(endP,1);
                    nextrow=row+dir_map(nextall(j),1);
                    nextcol=col+dir_map(nextall(j),2);
                    [~,ind]=ismember([nextrow nextcol],endP,'rows');
                    if ind
                        continue;
                    end
                    endP(sP+1,:)=[nextrow nextcol];
                    % endP=[nextrow nextcol;endP];
                    %                 endP(sP+j,:)=[row+dir_map(dir_can(dir,nextall(j+1)),1) col+dir_map(dir_can(dir,nextall(j+1)),2)];
                    % plot(endP(sP+1,2),endP(sP+1,1),'sr')
                    % plot(endP(1,2),endP(1,1),'sr')
                    %             if endP(end,:)==[0 0]
                    %                 display('!');
                    %             end
                end


                if all(EndPoint==start)||abs(sum(EndPoint-start))<2% ||(EndPoint(1) - start(1))^2+ (EndPoint(2)  - start(2))^2<a^2/4
                    skp=false;
                    break;
                end

                % line=[line;start EndPoint];
% % %                 if abs(start(1)-EndPoint(1))<abs(start(2)-EndPoint(2)) %Y based
% % %                     YI = linspace(start(2),EndPoint(2),12);%6
% % %                     %             XI = linspace(min(tempX),max(tempX),6);
% % %                     % obtain vector of 1-D look-up table "y" points
% % %                     %                 XI = lsq_lut_piecewise( tempY', tempX', YI );
% % %                 %     slm = slmengine(tempY,tempX);%,'degree',1);
% % %                 %     XI = slmeval(YI,slm,0);
% % %                     [tempYY, index] = unique(tempY);
% % %                     XI = interp1(tempYY,tempX(index),YI,'spline');
% % %                 else
% % %                     XI = linspace(start(1),EndPoint(1),12);%6
% % %                     %             XI = linspace(min(tempX),max(tempX),6);
% % %                     % obtain vector of 1-D look-up table "y" points
% % %                     %               YI = lsq_lut_piecewise( tempX', tempY', XI );
% % %                %      slm = slmengine(tempX,tempY);%'degree',1);
% % %                %      YI = slmeval(XI,slm,0);
% % %                     [tempXX, index] = unique(tempX);
% % %                     YI = interp1(tempXX,tempY(index),XI,'spline');
% % %                 end
                YI=tempY(round(linspace(1,length(tempY),24)));
                XI=tempX(round(linspace(1,length(tempY),24)));
                if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
                if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end

                pointX(i,:)=XI;
                pointY(i,:)=YI;
                i=i+1;
                %             temp=find(b);

                %             ind=ismember(dir_can(dir,:),temp);
                %             nextall=find(ind);
                %             if isempty(nextall)
                %                 a=1;
                %             end
                %             next=nextall(1);

                %             row=row+dir_map(dir_can(dir,next),1);
                %             col=col+dir_map(dir_can(dir,next),2);

                %             plot(col,row,'o','color',colors{mod(i,7)+1})
                break;

            end
            if number==1 % on going pt
                %             ind=find(row==endP_row);

                [ind]=find(b);
                dir=ind;
                row=row+dir_map(ind,1);
                col=col+dir_map(ind,2);
                crack=[crack; row col];
%                 plot(col,row,'*','color',colors{mod(i,6)+1})
                I(row,col)=0;
            end
            if number==0 % end point
                EndPoint=[row col];
                if all(EndPoint==start)||abs(sum(EndPoint-start))<2
                    skp=false;
                    break;
                end
                [~,ind]=ismember(EndPoint,endP,'rows');
                if ind
                    %             if col==endP_col(ind);
                    %                 list(ind)=[];
                    endP(ind,:)=[];
                end
%                 line=[line;start EndPoint];
% % %                 if abs(start(1)-EndPoint(1))<abs(start(2)-EndPoint(2)) %Y based
% % %                     YI = linspace(start(2),EndPoint(2),12);%6
% % %                     %             XI = linspace(min(tempX),max(tempX),6);
% % %                     % obtain vector of 1-D look-up table "y" points
% % %                     %               XI = lsq_lut_piecewise( tempY', tempX', YI );
% % % %                     slm = slmengine(tempY,tempX);%,'degree',1);
% % % %                     XI = slmeval(YI,slm,0);
% % %                     [tempYY, index] = unique(tempY);
% % %                     XI = interp1(tempYY,tempX(index),YI,'spline');
% % %                 else
% % %                     XI = linspace(start(1),EndPoint(1),12);%6
% % %                     %             XI = linspace(min(tempX),max(tempX),6);
% % %                     % obtain vector of 1-D look-up table "y" points
% % %                     %                 YI = lsq_lut_piecewise( tempX', tempY', XI );
% % % %                     slm = slmengine(tempX,tempY);%,'degree',1);
% % % %                     YI = slmeval(XI,slm,0);
% % %                     [tempXX, index] = unique(tempX);
% % %                     YI = interp1(tempXX,tempY(index),XI,'spline');
% % %                 end
                YI=tempY(round(linspace(1,length(tempY),24)));
                XI=tempX(round(linspace(1,length(tempY),24)));
                if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
                if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end

                pointX(i,:)=XI;
                pointY(i,:)=YI;
                i=i+1;


                %             plot(col,row,'*','color',colors{mod(i,7)+1})
                break;
            end

        end
        if ~isempty(crack)&&skp
%             if i==20
%                 disp(i-1)
%             end
            crackRaw{i-1}=[start;crack;EndPoint];
%             plot(crack(:,2),crack(:,1))
%             imshow(I)
        end
        
    end
    
    for o = 1:length(crackRaw)
        line=[line;[crackRaw{o}(1,:) crackRaw{o}(end,:)]];
    end
    
end
