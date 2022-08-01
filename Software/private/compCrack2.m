%% Crack Map Extraction Funcion 2
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [crackRaw,line,pointX,pointY] = compCrack2(I,endP,dir_map,colors)
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
    %imshow(I)
    i=1;spdist = @(P,Ps) sqrt((P(1,1)-Ps(:,1)).^2 + (P(1,2)-Ps(:,2)).^2);
    endcheck=0;circc=false;
    tempXX1=[];
    tempYY1=[];
    
%     [xx(1),xx(2)]=find(bwmorph(I,'branchpoints'));

% %     %%% New TEST funtion to increase the efficiency of the image processing
% %     I_te=I;
% %     while true
% %         intPoints = bwmorph(I, 'branchpoints');
% %         I_te(intPoints)=0;
% %         I_te=bwmorph(I_te,'clean');
% %         if ~isequal(I,I_te) 
% %            I=I_te;
% %         else
% %            break;
% %         end            
% %     end
% %     endPoints = bwmorph(I, 'endpoints');  % Finds end points of skeleton
% %     [endP_row, endP_col]=find(endPoints);
% %     endP=[endP_row endP_col];
% %     %%%

    while ~isempty(endP)
        %     i=list(1);
        start=endP(1,:);    % Selecting the first row of elements
        row=start(1);
        col=start(2);
        b = [I(row-1,col-1:col+1) ...
                I(row, col-1) I(row, col+1)...
                I(row+1,col-1:col+1)];
        number=sum(b);                         % adds all the elements in b
        if ~(number>=2)    
            endP(1,:)=[];       % Removing the first row of elements
            I(row,col)=0;
            
%         elseif number==2        % Temporary fix for loop
%             endP(1,:)=[];       % Removing the first row of elements
%             I(row,col)=0;
            
        else
            endP=circshift(endP,-1);
            %%%%%%
%             endcheck = endcheck+1;          %Test 
%             if endcheck>length(endP(:,1))   
%                 start=endP(1,:);    % Selecting the first row of elements
%                 row=start(1);
%                 col=start(2);
%                 b = [I(row-1,col-1:col+1) ...
%                     I(row, col-1) I(row, col+1)...
%                     I(row+1,col-1:col+1)];
%                 while(sum(b)>1)
%                     b1=find(b);b1=b1(1);
%                     I(row+dir_map(b1,1),col+dir_map(b1,2))=0;
%                     tempXX1=[tempXX1 row+dir_map(b1,1)];
%                     tempYY1=[tempYY1 col+dir_map(b1,2)];
%                     b = [I(row-1,col-1:col+1) ...
%                         I(row, col-1) I(row, col+1)...
%                         I(row+1,col-1:col+1)];
%                 end
%                 endP(1,:)=[];       % Removing the first row of elements
%                 I(row,col)=0;
%                 tempXX1=[tempXX1 row];
%                 tempYY1=[tempYY1 col];
%                 circc=true;
%             end
            %%%%%%
        end
        
        tempX=[];
        tempY=[];
        %     plot(col,row,'s','color',colors{mod(i,7)+1})
        %    imshow(I);
%         plot(endP(:,2),endP(:,1),'rs')
        %                 for j=row:width
        %                     for k=col:height
        %     display('start crack');
        %     start
        crack=[];
        while (1)

            tempX=[tempX row];
            tempY=[tempY col];
            b = [I(row-1,col-1:col+1) ...
                I(row, col-1) I(row, col+1)...
                I(row+1,col-1:col+1)];             % b captures the 8 surounding nodes of the end point
            number=sum(b);                         % adds all the elements in b
            if number>=2 % intersection point
                %             InterPoint=[InterPoint;row,col];
%                 deleteInd=[];
%                 [nextall]=find(b);                 % Finds which elements in b has value 1
%                 for j=1:length(nextall)
% 
%                     rowb=row+dir_map(nextall(j),1);
%                     colb=col+dir_map(nextall(j),2);
%                     I(rowb,colb)=0;
%                 end
%                 count=length(nextall);
%     %             tempnext=nextall;
%                 for j=1:count %check false intersection point
%                     dir=nextall(j);
%                     rowb=row+dir_map(dir,1);
%                     colb=col+dir_map(dir,2);
%                     bb = [I(rowb-1,colb-1:colb+1) ...
%                         I(rowb, colb-1) I(rowb, colb+1)...
%                         I(rowb+1,colb-1:colb+1)];
%                     [indb]=find(bb);
%                     if isempty(indb)
%                         deleteInd=[deleteInd j];
%                         continue;
%                     end
% 
%                 end
%                 nextall(deleteInd)=[];
%                 if length(nextall)==1
% 
% 
%                     row=row+dir_map(nextall,1);
%                     col=col+dir_map(nextall,2);
%                     crack=[crack; row col];
%                     plot(col,row,'*','color',colors{mod(i,6)+1})
%                     continue;
%                 end
%                 EndPoint=[row col];
%                 %             nextall=find(b);
%                 %             sP=size(endP,1);
%                 for j=1:length(nextall)
%                     sP=size(endP,1);
%                     nextrow=row+dir_map(nextall(j),1);
%                     nextcol=col+dir_map(nextall(j),2);
%                     [~,ind]=ismember([nextrow nextcol],endP,'rows');
%                     if ind
%                         continue;
%                     end
%                     endP(sP+1,:)=[nextrow nextcol];
%                     % endP=[nextrow nextcol;endP];
%                     %                 endP(sP+j,:)=[row+dir_map(dir_can(dir,nextall(j+1)),1) col+dir_map(dir_can(dir,nextall(j+1)),2)];
% %                     plot(endP(sP+1,2),endP(sP+1,1),'sr')
%                     % plot(endP(1,2),endP(1,1),'sr')
%                     %             if endP(end,:)==[0 0]
%                     %                 display('!');
%                     %             end
%                 end
% 
% 
%                 if all(EndPoint==start)||abs(sum(EndPoint-start))<2% ||(EndPoint(1) - start(1))^2+ (EndPoint(2)  - start(2))^2<a^2/4
%                     break;
%                 end

                %%% Intersection Point Check 
                I_c=I;
                deleteInd=[];
                [nextall]=find(b);                 % Finds which elements in b has value 1
                for j=1:length(nextall)

                    rowb=row+dir_map(nextall(j),1);
                    colb=col+dir_map(nextall(j),2);
                    I_c(rowb,colb)=0;
                end
                count=length(nextall);
                % tempnext=nextall;
                for j=1:count %check false intersection point
                    dir=nextall(j);
                    rowb=row+dir_map(dir,1);
                    colb=col+dir_map(dir,2);
                    bb = [I_c(rowb-1,colb-1:colb+1) ...
                        I_c(rowb, colb-1) I_c(rowb, colb+1)...
                        I_c(rowb+1,colb-1:colb+1)];
                    [indb]=find(bb);
                    if isempty(indb)
                        deleteInd=[deleteInd j];
                        continue;
                    end

                end
                nextall(deleteInd)=[];
                if length(nextall)==1
                    I=I_c;
                    row=row+dir_map(nextall,1);
                    col=col+dir_map(nextall,2);
                    crack=[crack; row col];
                    % plot(col,row,'*','color',colors{mod(i,6)+1})
                    continue;
                end
                %%%
                
                if length(tempX)==1
                    i=i+1;
                    break;
                end
                if circc
                    if any(spdist([tempX(end) tempY(end)],[tempXX1' tempYY1'])<2)
                       ttk = spdist([tempX(end) tempY(end)],[tempXX1' tempYY1']);
                       ttt=find(ttk<2 & min(ttk)==ttk);ttt=[ttt ttt+1];
                       tempX=[tempX tempXX1(ttt)];row = tempX(end);
                       tempY=[tempY tempYY1(ttt)];col = tempY(end);
                       circc=false;endcheck=0;
                    end
                end

%                 EndPoint=[row col];
                if ~isempty(endP)
                    [f,d]=dsearchn(endP,[row col]);
                    if d<10
                        EndPoint = endP(f,:);
                    else
                        EndPoint=[row col];
                    end
                else
                    EndPoint=[row col];
                end
                I(EndPoint(1),EndPoint(2))=1;
                if all(EndPoint==start)||spdist(EndPoint,start)<2%abs(sum(EndPoint-start))<2
                    i=i+1;
                    break;
                end
%                 [~,ind]=ismember(EndPoint,endP,'rows');
%                 if ind
%                     %             if col==endP_col(ind);
%                     %                 list(ind)=[];
%                     endP(ind,:)=[];
%                 end
                b = [I(row-1,col-1:col+1) ...
                        I(row, col-1) I(row, col+1)...
                        I(row+1,col-1:col+1)];
                number=sum(b);                         % adds all the elements in b
                if ~(number>=2)    
                    endP(1,:)=[];       % Removing the first row of elements
                end

%                 line=[line;start EndPoint];
%                 if abs(start(1)-EndPoint(1))<abs(start(2)-EndPoint(2)) %Y based
%                     YI = linspace(start(2),EndPoint(2),24);%6
%                     %             XI = linspace(min(tempX),max(tempX),6);
%                     % obtain vector of 1-D look-up table "y" points
%                     %                 XI = lsq_lut_piecewise( tempY', tempX', YI );
% %                     slm = slmengine(tempY,tempX);%,'degree',1);
% %                     XI = slmeval(YI,slm,0);
%                     [tempYY, index] = unique(tempY);
%                     XI = interp1(tempYY,tempX(index),YI,'spline');
%                 else
%                     XI = linspace(start(1),EndPoint(1),24);%6
%                     %             XI = linspace(min(tempX),max(tempX),6);
%                     % obtain vector of 1-D look-up table "y" points
%                     %               YI = lsq_lut_piecewise( tempX', tempY', XI );
% %                     slm = slmengine(tempX,tempY);%'degree',1);
% %                     YI = slmeval(XI,slm,0);
%                     [tempXX, index] = unique(tempX);
%                     YI = interp1(tempXX,tempY(index),XI,'spline');
%                 end

                % Moved to the end of the function 
% % %                 YI=tempY(round(linspace(1,length(tempY),24)));
% % %                 XI=tempX(round(linspace(1,length(tempY),24)));
% % %                 if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
% % %                 if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end
% % %                 pointX(i,:)=XI;
% % %                 pointY(i,:)=YI;
                
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
                if ~isempty(endP)&&~isequal(start,[row col])
%                     f=ismember(endP,[row col],'rows');
%                     [~,f]=setdiff(endP,[row col],'rows');
                    f=sum(endP==[row col],2)==2;
                    if any(f) %length(f)~=size(endP,1)
                        endP(f,:)=[];
                        %endP(setdiff(1:size(endP,1),f),:)=[];
                        %endP(~sum(cell2mat(arrayfun(@(x) 1:size(endP,1)==x,f,'UniformOutput',false)')),:)=[];
                    end
                end
                
% % % % % %                 %             ind=find(row==endP_row);
% % % % % % %                 if (row==1532 && col ==1190)
% % % % % % %                    disp('1') 
% % % % % % %                 end
% % % % % %                 [d1,~]=ismember([row col],endP,'rows');
% % % % % %                 b = [I(row-1,col-1:col+1) ...
% % % % % %                         I(row, col-1) I(row, col+1)...
% % % % % %                         I(row+1,col-1:col+1)];
% % % % % %                 if d1
% % % % % %                    if ~isempty(endP)
% % % % % %                         [f,d]=dsearchn(endP,[row col]);
% % % % % %                         if d<10
% % % % % %                             EndPoint = endP(f,:);
% % % % % %                         else
% % % % % %                             EndPoint=[row col];
% % % % % %                         end
% % % % % %                     else
% % % % % %                         EndPoint=[row col];
% % % % % %                    end
% % % % % %                     I(EndPoint(1),EndPoint(2))=1;
% % % % % %                     if all(EndPoint==start)||spdist(EndPoint,start)<2%abs(sum(EndPoint-start))<2
% % % % % %                         i=i+1;%pointX(i,:)=zeros(1,24);pointY(i,:)=zeros(1,24);
% % % % % %                         break;
% % % % % %                     end
% % % % % %                     %[~,ind]=ismember(EndPoint,endP,'rows');
% % % % % %                     %if ind
% % % % % %                         %endP(ind,:)=[];
% % % % % %                     %end
% % % % % % 
% % % % % % %                     if abs(start(1)-EndPoint(1))<abs(start(2)-EndPoint(2)) %Y based
% % % % % % %                         YI = linspace(start(2),EndPoint(2),24);%6
% % % % % % %                         [tempYY, index] = unique(tempY);
% % % % % % %                         XI = interp1(tempYY,tempX(index),YI,'spline');
% % % % % % %                     else
% % % % % % %                         XI = linspace(start(1),EndPoint(1),24);%6
% % % % % % %                         [tempXX, index] = unique(tempX);
% % % % % % %                         YI = interp1(tempXX,tempY(index),XI,'spline');
% % % % % % %                     end
% % % % % % 
% % % % % %                     % Moved to the end of the function 
% % % % % % % % %                     YI=tempY(round(linspace(1,length(tempY),24)));
% % % % % % % % %                     XI=tempX(round(linspace(1,length(tempY),24)));
% % % % % % % % %                     if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
% % % % % % % % %                     if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end
% % % % % % % % %                     pointX(i,:)=XI;
% % % % % % % % %                     pointY(i,:)=YI;
% % % % % %                     
% % % % % %                     i=i+1;
% % % % % %                     break;
% % % % % % 
% % % % % %                 else
                    [ind]=find(b);
                    dir=ind;
                    row=row+dir_map(ind,1);
                    col=col+dir_map(ind,2);
                    crack=[crack; row col];
    %                 plot(col,row,'*','color',colors{mod(i,6)+1})
                    I(row,col)=0;
%                     if circc
%                         if size(crack,1)>1
%                             if any(spdist(crack(end,:),[tempXX1' tempYY1'])<2)
%                                ttk = spdist(crack(end,:),[tempXX1' tempYY1']);
%                                ttt=find(ttk<2 & min(ttk)==ttk);%ttt=[ttt ttt+1];
%                                crack=[crack; tempXX1(ttt) tempYY1(ttt)];
%                             end
%                         end
%                     end
% % % % % %                 end

            end
            if number==0 % end point
                if circc
                    if any(spdist([tempX(end) tempY(end)],[tempXX1' tempYY1'])<2)
                       ttk=spdist([tempX(end) tempY(end)],[tempXX1' tempYY1']);
                       ttt=find(ttk<2 & min(ttk)==ttk);ttt=[ttt ttt+1];
                       tempX=[tempX tempXX1(ttt)];row = tempX(end);
                       tempY=[tempY tempYY1(ttt)];col = tempY(end);
                       circc=false;endcheck=0;
                    end
                end
                
%                 EndPoint=[row col];
                if ~isempty(endP)
                    [f,d]=dsearchn(endP,[row col]);
                    if d<10
                        EndPoint = endP(f,:);
                    else
                        EndPoint=[row col];
                    end
                else
                    EndPoint=[row col];
                end
                
                if all(EndPoint==start)||spdist(EndPoint,start)<2%abs(sum(EndPoint-start))<2
                    i=i+1;%pointX(i,:)=zeros(1,24);pointY(i,:)=zeros(1,24);
                    break;
                end
                rowb=EndPoint(1);colb=EndPoint(2);
                b = [I(rowb-1,colb-1:colb+1) ...
                        I(rowb, colb-1) I(rowb, colb+1)...
                        I(rowb+1,colb-1:colb+1)];
%                 [~,ind]=ismember(EndPoint,endP,'rows');
%                 [~,ind]=setdiff(EndPoint,endP,'rows');if isempty(ind);ind=true;else;ind=~(ind);end
                ind=sum(endP==EndPoint,2)==2;
                if any(ind) && sum(b)==0
                    %             if col==endP_col(ind);
                    %                 list(ind)=[];
                    endP(ind,:)=[];
                end
%                 line=[line;start EndPoint];
%                 if abs(start(1)-EndPoint(1))<abs(start(2)-EndPoint(2)) %Y based
%                     YI = linspace(start(2),EndPoint(2),24);%6
%                     %             XI = linspace(min(tempX),max(tempX),6);
%                     % obtain vector of 1-D look-up table "y" points
%                     %               XI = lsq_lut_piecewise( tempY', tempX', YI );
% %                     slm = slmengine(tempY,tempX);%,'degree',1);
% %                     XI = slmeval(YI,slm,0);                
%                     [tempYY, index] = unique(tempY);
%                     XI = interp1(tempYY,tempX(index),YI,'spline');
%                     if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
%                     if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end
%                 else
%                     XI = linspace(start(1),EndPoint(1),24);%6
%                     %             XI = linspace(min(tempX),max(tempX),6);
%                     % obtain vector of 1-D look-up table "y" points
%                     %                 YI = lsq_lut_piecewise( tempX', tempY', XI );
% %                     slm = slmengine(tempX,tempY);%,'degree',1);
% %                     YI = slmeval(XI,slm,0);
%                     [tempXX, index] = unique(tempX);
%                     YI = interp1(tempXX,tempY(index),XI,'spline');
%                     if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
%                     if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end
%                 end
                
                % Moved to the end of the function 
% % %                 YI=tempY(round(linspace(1,length(tempY),24)));
% % %                 XI=tempX(round(linspace(1,length(tempY),24)));
% % %                 if ~isequal([XI(1),YI(1)],start);XI(1)=start(1);YI(1)=start(2);end
% % %                 if ~isequal([XI(end),YI(end)],EndPoint);XI(end)=EndPoint(1);YI(end)=EndPoint(2);end
% % %                 pointX(i,:)=XI;
% % %                 pointY(i,:)=YI;
                
                i=i+1;


                %             plot(col,row,'*','color',colors{mod(i,7)+1})
                break;
            end

        end
        if ~isempty(crack)
            crackRaw{i-1}=[start;crack;EndPoint];
        else
            i=i-1;
        end
        
    end
    
    if exist('crackRaw','var')
        for o = 1:length(crackRaw)
            line=[line;[crackRaw{o}(1,:) crackRaw{o}(end,:)]];
            tempX=crackRaw{o}(:,1);tempY=crackRaw{o}(:,2);
            YI=tempY(round(linspace(1,length(tempY),24)));
            XI=tempX(round(linspace(1,length(tempY),24)));
            if ~isequal([XI(1),YI(1)],[tempX(1),tempY(1)]);XI(1)=tempX(1);YI(1)=tempY(1);end
            if ~isequal([XI(end),YI(end)],[tempX(end),tempY(end)]);XI(end)=tempX(end);YI(end)=tempY(end);end
            pointX(o,:)=XI;
            pointY(o,:)=YI;
        end
    else
        [crackRaw,line,pointX,pointY]=deal([]);
    end
    
end
