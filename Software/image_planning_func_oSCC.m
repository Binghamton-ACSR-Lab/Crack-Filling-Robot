%% Image Planning Function for onlineSCC
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [waypoint_coords,flag,crackR,ttt] = image_planning_func_oSCC(BW3,a,s,cp,acp,endP,realEndP,contEndP,ppath,preCrack)
%
% Performs Image processing, extacts cracks and metadata, and computes the waypoints for the crackGraph using Visibility Graph.
%
% INPUTS:
%   BW3 = Binary image of the sensor range. 
%   a = Footprint Range. 
%   s = Sensor Range.
%   cp = Simulated current point.
%   acp = Actual currnt point. 
%   endP = [x0; y0] = All enpoints of the extracted cracks.
%   realEndP = [x0; y0] = Real enpoints in endP. 
%   contEndP = [x0; y0] = Continuing enpoints in endP.
%   ppath = [x0; y0] = Path of the robot from start to current iteration. 
%   preCrack = [x0; y0] = Scanned craks from previous iteration. 
%
% OUTPUTS:
%   waypoint_coords = [x0; y0] = Waypoint coordinates of the crackGraph. 
%   flag = high if robot is curently filling, low if robot is just trasversing.
%   crackR = [x0; y0] = Scanned craks from current iteration.
%   ttt = Computation time.

    % Version2 Changed Connecting the Crack Graph Section

    if sum(sum(BW3))>a/4
        % clc 
        % 
        % close all
        % clear all
        spdist = @(P,Ps) sqrt((P(1,1)-Ps(:,1)).^2 + (P(1,2)-Ps(:,2)).^2);
        midP= @(P1,P2) (P1(:)+ P2(:)).'/2;
        total_length = @(Ps) sum(sqrt(sum(diff(Ps).*diff(Ps),2)));
        aP=polybuffer([0,0],'points',a);
        % footR=7;                                        % 7" Diameter
        % sensR=72;                                       % 6*12" Diameter
        % 
        % a=fix(((footR/2)*25.4)/2);                      
        % s=fix(((sensR/2)*25.4)/2);                      
        % 
        % % %%  Import Crack Image and extract the crack information
        % 
        % BW = imread('myCrack1.png');      % Importing the map w/ crack information 
        % 
        % BW = imbinarize(BW);            % Converting the image into a binary image
        % BW = BW(:,:,1);
        % % figure, imshow(BW)
        % 
        % BW = bwareaopen(BW, 50);        % Removes all objects that have fewer than 50 pixels from the binary image BW
%         [endP,~,~]=endP_ident(BW3,[],[]);
        
%         BW3 = padarray(BW3,[1 1],0);        % Adds a boundary around the map
        BW=BW3;
        BW3=padarray(BW3,[0,1],'post');
        BW3=padarray(BW3,[1,0],'post');
%         BW3 = bwmorph(BW3, 'spur', a);
        % BW = ~BW;                       % Reverses the binary
        % figure, imshow(BW)

        % IM2 = imcomplement(BW);         % Computes the complement of the image BW
        % % figure, imshow(IM2);
        % rawBW = BW;
        % 
        % BW2 = bwmorph(BW,'thin', inf);  % bwskel(BW);% Thins objects to lines, making up the image skeleton
%         BW3 = bwmorph(BW3,'thin', inf);
        % % figure, imshow(BW2);
        % 
        % BW3 = bwmorph(BW2, 'spur', a); % Removes spur pixels
        % % montage({BW,BW3},'BackgroundColor','blue','BorderSize',5)
        % figure, imshow(~BW3); hold on
        % [rowBW, colBW]= size(BW3);
        % pos=get(gcf, 'Position');
        % close(gcf)

        % skelBW = BW3;
        
        worki=false;
        
        %%% Replaced with endP_ident()
        %{
        endPoints = bwmorph(BW3, 'endpoints');  % Finds end points of skeleton
        [endP_row, endP_col]=find(endPoints);
        % plot(endP_col,endP_row,'b*'); 

        intPoints = bwmorph(BW3, 'branchpoints');
        % [intP_row, intP_col]=find(intPoints);
        % plot(intP_col,intP_row,'r*'); 

        

        endP=[endP_row endP_col];
        
        %%%%%%%%
        clear endP
        endPoints(intPoints)=1;
        [endP(:,1),endP(:,2)]=find(endPoints);
        n=1;
        while n<=length(endP)
            if sum(spdist(endP(n,:),endP(1:end ~= n,:))<5)>0
                endP(n,:)=[];
            end
            n=n+1;
        end
        endPoints = endP;%[endP_row,endP_col];
        realEndP = endPoints(~any(ismember(endPoints,[2 colBW-1 rowBW-1])')',:); plot(realEndP(:,2),realEndP(:,1),'pr')
        contEndP = endPoints(any(ismember(endPoints,[2 colBW-1 rowBW-1])')',:); plot(contEndP(:,2),contEndP(:,1),'pg')
        % plot(endP(:,2),endP(:,1),'pg');%hold off
        %%%%%%%%
        %}
        
        I=BW3;I_te=I;
        
        dir_map=[-1 -1;-1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0 ; 1 1];
        dir_can=[1 2 4 3 6;2 1 3 4 5; 3 2 5 1 8; 4 1 6 2 7 ; 5 3 8 2 7; 6 4 7 1 8; 7 6 8 4 5; 8 5 7 3 6];
        colors={'y','m','c','r','g','b'};
        
        [crackRaw,line,pointX,pointY] = compCrack2(I,endP,dir_map,colors);     % Extract the crack information from the image
        
        tic
        %%%% Alternative BWMorph Spur
        c_chq=cellfun(total_length,crackRaw);c_chq=c_chq<a;
        line(c_chq,:)=[];
        pointX(c_chq,:)=[];pointY(c_chq,:)=[];
        crackRaw(c_chq)=[];
        %%%%

%         for k=1:length(crackRaw);plot(crackRaw{k}(:,2),crackRaw{k}(:,1),'LineWidth',2);end

        % ee=reshape([line(:,[1,3]),line(:,[2,4])],length(line(:,1))*2,2);
        % plot(ee(:,2),ee(:,1),'pr')
        
        %%% Saving Crack Information 
        % save crackRaw.mat crackRaw line pointX pointY endPoints intPoints
        % save crackBW.mat rawBW skelBW
        
        %% End point check 
        
%         endlogi = reshape(ismember([line(:,1) line(:,2);line(:,3) line(:,4)],realEndP,'rows'),[],2);
        if ~isempty(realEndP)
            endlogi = reshape(sum(cell2mat(cellfun(@(s) spdist(s,[line(:,1) line(:,2);line(:,3) line(:,4)]),num2cell(realEndP, 2),'un',0)')<5,2)>0,[],2);
            endlogi_a = reshape(spdist(cp,[line(:,1) line(:,2);line(:,3) line(:,4)])<a,[],2);
        else
            endlogi = zeros(size(line,1),2);endlogi_a = zeros(size(line,1),2);
        end
%         [~,d]=dsearchn(realEndP,[line(:,1) line(:,2);line(:,3) line(:,4)]);d=d<6;endlogi=endlogi|reshape(d,[],2);
        line(~(endlogi(:,1)|endlogi(:,2)),:)=[];crackRaw(~(endlogi(:,1)|endlogi(:,2)))=[];
        pointX(~(endlogi(:,1)|endlogi(:,2)),:)=[];pointY(~(endlogi(:,1)|endlogi(:,2)),:)=[];
        endlogi_a(~logical(sum(endlogi,2)),:)=[];endlogi(~logical(sum(endlogi,2)),:)=[];
        link=struct('x',cell(1,size(line,1)),'y',cell(1,size(line,1)));

        %% To solve the cracks with sharp acute angle
        %%%% 
        fP=repmat(a,[size(line,1),1]);
        [~,yy]=size(BW3);det=[];
        for l=1:length(crackRaw)
            if total_length(crackRaw{l})>2*a*sqrt(2)
                aRan=polybuffer(crackRaw{l}([1,end],:),'points',a);%plot(polybuffer(fliplr(crackRaw{l}([1,end],:)),'points',a))
                crackW=crackRaw{l}(~ismember(crackRaw{l},rmmissing(intersect(aRan,rmmissing(crackRaw{l}))),'rows'),:);
                if total_length(crackW)>a % New ~isempty(crackW)
                    mov=crackW-crackW(1,:);
                else
                    mov=crackRaw{l}-crackRaw{l}(1,:);
                end
            else
                mov=crackRaw{l}-crackRaw{l}(1,:);%plot(mov(:,2),mov(:,1));plot(mov(end,2),mov(end,1),'r*')
            end
            v1=[0,yy];
            v2=mov(end,:);
            ang=ab2v(v1,v2);
            rotv=[cos(deg2rad(ang)) sin(deg2rad(ang)); -sin(deg2rad(ang)) cos(deg2rad(ang))];
            rv2=(rotv*[mov]')';%plot(rv2(:,2),rv2(:,1))
            [~,d]=min(rv2(:,1)); peakPts_idx=d;
            [~,d]=max(rv2(:,1)); peakPts_idx=[peakPts_idx;d];
            [~,d]=max(abs(rv2(peakPts_idx,1)));peakPts_idx=peakPts_idx(d);%plot(rv2(peakPts_idx,2),rv2(peakPts_idx,1),'r*')
            v1=-rv2(peakPts_idx,:);v2=rv2(end,:)-rv2(peakPts_idx,:);%plot([v1(2),0],[v1(1),0]);plot([v2(2),0],[v2(1),0])
            ang=ab2v(v1,v2)/2; %if angle <45 or >360-45 change 'a' and skip edge 
            if ang<45 || ang >360-45
%                disp(ang)
%                at=fP(l);
%                while(1)
%                   ht=at/tan(deg2rad(ang)); 
%                   if ht>a;at=at-.25;else; break;end
%                end
%                fP(l)=at;det=[det;l];
                fP(l)=tan(deg2rad(ang))*a;det=[det;l];
            end
        end
        
        % Hold variable to skip adding to edge connection function 
        det=det(sum(endlogi(det,:))>0);h_line=[]; % Check atleat one node is a real endpoint
        if ~isempty(det)
            h_line = [h_line;line(det,:)];
            h_pointX=pointX(det,:);h_pointY=pointY(det,:);
            h_crackRaw=crackRaw(det);h_fP=fP(det);

            line(det,:)=[];link(det)=[];
            pointX(det,:)=[];pointY(det,:)=[];
            crackRaw(det)=[];fP(det)=[]; endlogi(det,:)=[]; endlogi_a(det,:)=[];  
        end
       %%%%

        %% Connecting the Crack Graph

        % figure
        % imshow(BW3);
%         nnn=size(line,1);
        % hold on

%         detind=[];
%         templine=line;
%         for i=1:nnn
%             crackCom=crackRaw{i};
%              link(i).x=pointX(i,:);
%              link(i).y=pointY(i,:);
% 
%              if sum(templine(i,:))==0
%                 continue;
%              end
%              templine(i,:)=zeros(1,4);
%              while(1)
%                 tempdist= spdist([link(i).x(end),link(i).y(end)],templine(:,1:2));
%                 inst=find(strlength(string(round(tempdist(:))))<=2);
%                 if ~isempty(inst)
%                     v1=crackCom(bound(length(crackCom)-4,1,length(crackCom)),:)-crackCom(end,:);
%                     for l=1:length(inst)
%                         v2=crackRaw{inst(l)}(4,:)-crackRaw{inst(l)}(1,:);
%                         ang(l)=ab2v(v1,v2);
%                     end
%                     ind=inst(max(ang)==ang);clear ang
%                     mmin=tempdist(ind);
%                     if mmin<=a
%                         line(i,3:4)=line(ind,3:4);
%                         link(i).x=[link(i).x,pointX(ind,:)];
%                         link(i).y=[link(i).y,pointY(ind,:)];
%                         crackCom = [crackCom;crackRaw{ind}];
%                         line(ind,:)=zeros(1,4);
%                         templine(ind,:)=zeros(1,4);
%                         detind=[detind, ind];
%                     else
%                         tempdist= spdist([link(i).x(end),link(i).y(end)],templine(:,3:4));
%                         inst=find(strlength(string(round(tempdist(:))))<=2);
%                         if ~isempty(inst)
%                             v1=crackCom(bound(length(crackCom)-4,1,length(crackCom)),:)-crackCom(end,:);
%                             for l=1:length(inst)
%                                 v2=crackRaw{inst(l)}(bound(length(crackRaw{inst(l)})-4,1,length(crackRaw{inst(l)})),:)-crackRaw{inst(l)}(end,:);
%                                 ang(l)=ab2v(v1,v2);
%                             end
%                             ind=inst(max(ang)==ang);clear ang
%                             mmin=tempdist(ind);
%                             if mmin<=a
%                                 line(i,3:4)=line(ind,1:2);
%                                 link(i).x=[link(i).x,pointX(ind,end:-1:1)];
%                                 link(i).y=[link(i).y,pointY(ind,end:-1:1)];
%                                 crackCom = [crackCom;flipud(crackRaw{ind})];
%                                 line(ind,:)=zeros(1,4);
%                                 templine(ind,:)=zeros(1,4);
%                                 detind=[detind, ind];
%                             end
%                         else
%                             break
%                         end
%                     end
%                 else
%                     tempdist= spdist([link(i).x(end),link(i).y(end)],templine(:,3:4));
%                     inst=find(strlength(string(round(tempdist(:))))<=2);
%                     if ~isempty(inst)
%                         v1=crackCom(bound(length(crackCom)-4,1,length(crackCom)),:)-crackCom(end,:);
%                         for l=1:length(inst)
%                             v2=crackRaw{inst(l)}(bound(length(crackRaw{inst(l)})-4,1,length(crackRaw{inst(l)})),:)-crackRaw{inst(l)}(end,:);
%                             ang(l)=ab2v(v1,v2);
%                         end
%                         ind=inst(max(ang)==ang);clear ang
%                         mmin=tempdist(ind);
%                         if mmin<=a
%                             line(i,3:4)=line(ind,1:2);
%                             link(i).x=[link(i).x,pointX(ind,end:-1:1)];
%                             link(i).y=[link(i).y,pointY(ind,end:-1:1)];
%                             crackCom = [crackCom;flipud(crackRaw{ind})];
%                             line(ind,:)=zeros(1,4);
%                             templine(ind,:)=zeros(1,4);
%                             detind=[detind, ind];
%                         else
%                             break                               % Added newly
%                         end
%                     else
%                         break
%                     end
%                 end
%              end
%             plot(link(i).y,link(i).x,'LineWidth',2)
%         end

        if ~isempty(line) || ~isempty(h_line)
            if ~isempty(line)
                detind=[];
                templine=line;

                endlogi_t = reshape(spdist(cp,[line(:,1) line(:,2);line(:,3) line(:,4)])<s*sqrt(2),[],2) & endlogi;            

                %if isequal(cp,acp)
                    %ll= sum(endlogi_t,2)==max(sum(endlogi_t,2));
                %else
                ll= sum(endlogi,2)>0;
                %end

                tt=reshape([templine(ll,[1,3]),templine(ll,[2,4])],[],2);

                % Unfilled Crack 
                %ee=endlogi(ll,:);
                %if any(sum(endlogi,2)==2 &sum(endlogi_a,2)) || spdist(cp,acp)<a
                    %ll= sum(endlogi,2)==max(sum(endlogi,2));
                %end
                if isequal(cp,acp)
                ll= sum(endlogi_t,2)==max(sum(endlogi_t,2));
                end

                endlogi_t=endlogi;endlogi_t(~(ll),:)=0;ee=endlogi_t;
                tt2=tt(logical(ee(:)),:);  %True End points
                [cd,ee1] = min(spdist(cp,tt2));[~,ee]=ismember(tt2(ee1,:),tt,'rows');  %Finds distance between the current point and the true end points

                %%%% Check if the point is an intersection point
                while(1)
                    if sum(ismember(tt,tt(ee,:),'rows'))>1
                        tt2(ismember(tt2,tt2(ee1,:),'rows'),:)=[];
                        [cd,ee1] = min(spdist(cp,tt2));[~,ee]=ismember(tt2(ee1,:),tt,'rows');
                    else
                        break 
                    end
                end
                %%%%

                if cd<=s*sqrt(2)
                    %[~,ee] = dsearchn(tt(ee,:),realEndP);ee=ee<6;
                    ee=spdist(tt(ee,:),realEndP);ee=ee==min(ee);
                    [~,d]=dsearchn(realEndP(ee,:),[templine(:,1) templine(:,2);templine(:,3) templine(:,4)]);d=d<6;d=reshape(d,[],2);
                    d(~(sum(d,2)&ll),:)=0;ll=find(sum(d,2));pp1=[];

                    if length(ll)>1
                        for lll=ll'
                            [~,pp]=dsearchn(crackRaw{lll},acp);pp1=[pp1;pp];
                        end
                        d(ll(~(min(pp1)==pp1)),:)=0;
                    end

                    if any(d,'all')%any(d(:,1))
                        intt= find(d(:,1));if isempty(intt); intt= find(d(:,2));else intt=intt(1);end 
                        crackRawt=crackRaw;
                        if find(d(intt,:))==1
                            link(intt).x=pointX(intt,:);
                            link(intt).y=pointY(intt,:);
                        else
                            for intt_l=reshape(intt,1,[])
                                link(intt_l).x=fliplr(pointX(intt_l,:));
                                link(intt_l).y=fliplr(pointY(intt_l,:));
                                endlogi(intt_l,:)= fliplr(endlogi(intt_l,:));
                                crackRaw{intt_l}=flipud(crackRaw{intt_l});
                                line(intt_l,:)=[line(intt_l,(3:4)),line(intt_l,(1:2))];
                            end
                        end
                        templine(intt,:)=0;indt=intt;

                        if endlogi(intt,2)~=0 && isequal(cp,acp)   % Current line has realEndpoint, connect to another line  && isequal function stops addind new line while on filling line
                            endlogi = reshape(ismember([templine(:,1) templine(:,2);templine(:,3) templine(:,4)],realEndP,'rows'),[],2);
            %                 [~,dd]=dsearchn(realEndP,[templine(:,1) templine(:,2);templine(:,3) templine(:,4)]);dd=dd<6;endlogi=endlogi|reshape(dd,[],2)
                            ss=sum(endlogi,2);
                            if any(ss(1:size(ss,1),:)==2)             % Checking if any other line with two end points
                                intt2=find(ss(1:size(ss,1),:)==2)';
                                detind=[detind,find(~ismember(1:size(line,1),intt2) & ~ismember(1:size(line,1),intt))];
                                templine(setdiff(1:end,intt2),:)=0;

                                while(1)

                %                     for kk = 1:length(intt2)
                % 
                                    tempdist= spdist([link(intt).x(end),link(intt).y(end)],templine(:,1:2));
                %                     inst=find(strlength(string(round(tempdist(:))))<=2);
                                    inst=find(tempdist<=a);
                                    if ~isempty(inst)
                                        inst=find(tempdist==min(tempdist(tempdist<=a)));inst=inst(1);
                                        v1=crackRawt{indt}(1,:)-crackRawt{indt}(end,:);
                                        v2=crackRaw{intt}(1,:)-crackRaw{intt}(end,:);
                                        v3=crackRaw{inst}(end,:)-crackRaw{inst}(1,:);
                                        ang=ab2v(v1,v3);indt=inst;ang2=ab2v(v2,v3);
                                    end
                                    if ~isempty(inst) && ~(ang<45 || ang >360-45) && ~(ang2<45 || ang2 >360-45)
                                        [~,d]=min(tempdist);
                                        link(intt).x=[link(intt).x, pointX(d,:)];
                                        link(intt).y=[link(intt).y, pointY(d,:)];
                                        line(intt,3:4)=[link(intt).x(end),link(intt).y(end)];
                                        crackRaw{intt}=[crackRaw{intt};crackRaw{d}];
                                        templine(d,:)=0;
                                        detind=[detind,d];intt2(intt2==d)=[];
                                    else
                                        tempdist= spdist([link(intt).x(end),link(intt).y(end)],templine(:,3:4));
                                        inst=find(tempdist<=a);
                                        if ~isempty(inst)
                                            inst=find(tempdist==min(tempdist(tempdist<=a)));inst=inst(1);
                                            v1=crackRawt{indt}(1,:)-crackRawt{indt}(end,:);
                                            v2=crackRaw{intt}(1,:)-crackRaw{intt}(end,:);
                                            v3=crackRaw{inst}(1,:)-crackRaw{inst}(end,:);
                                            ang=ab2v(v1,v3);indt=inst;ang2=ab2v(v2,v3);
                                        end
                                        if ~isempty(inst) && ~(ang<45 || ang >360-45) && ~(ang2<45 || ang2 >360-45)
                                            [~,d]=min(tempdist);
                                            link(intt).x=[link(intt).x, fliplr(pointX(d,:))];
                                            link(intt).y=[link(intt).y, fliplr(pointY(d,:))];
                                            line(intt,3:4)=[link(intt).x(end),link(intt).y(end)];
                                            crackRaw{intt}=[crackRaw{intt};flipud(crackRaw{d})];
                                            crackRawt{d}=flipud(crackRawt{d});
                                            templine(d,:)=0;
                                            detind=[detind,d];intt2(intt2==d)=[];
                                        else
                                            intt=intt2(1);indt=intt;
                                            link(intt).x=[link(intt).x, pointX(intt,:)];
                                            link(intt).y=[link(intt).y, pointY(intt,:)];
                                            line(intt,3:4)=[link(intt).x(end),link(intt).y(end)];
                                            templine(intt,:)=0;
                                            intt2(intt2==intt)=[];

    %                                         detind=find(all(cell2mat(arrayfun(@(x) structfun(@isempty, x), link, 'UniformOutput', false)),1));
    %                                         templine(detind,:)=0;
                                        end
                                    end
                                    if any(templine,'all')
                                        continue
                                    else
                                        break
                                    end
                                end
            %                     
            %                     
            %                     for jj=intt2
            %                         link(jj).x=pointX(jj,:);
            %                         link(jj).y=pointY(jj,:);
            %                     end
            %                     detind=[detind,find(~ismember(1:size(d,1),intt2) & ~ismember(1:size(d,1),intt))]
                            elseif ss==0
                                detind=[detind,find(1:size(d,1)~=intt)];
                            else      
                                ddist=[];
                                for ll=reshape(find((1:size(endlogi,1)~=intt)' & ss),1,[])
                    %                 if sum(endlogi(ll,:))==1                                     % New line connected is 
                                        ii = find(endlogi(ll,:));
                                        ddist=[ddist;ll,spdist([link(intt).x(end),link(intt).y(end)],templine(ll,2*ii-1:2*ii))];
                                end
                                [n,m]=min(ddist(:,2));%%%
                                if n<=a
                                    if find(endlogi(ddist(m,1),:))==1
                                        link(intt).x=[link(intt).x,pointX(ddist(m,1),:)];
                                        link(intt).y=[link(intt).y,pointY(ddist(m,1),:)];
                                        line(intt,3:4)=[link(intt).x(end),link(intt).y(end)];
                                        crackRaw{intt}=[crackRaw{intt};crackRaw{ddist(m,1)}];
                                        detind=[detind,find(1:size(d,1)~=intt)];
                                    else
                                        link(intt).x=[link(intt).x,fliplr(pointX(ddist(m,1),:))];
                                        link(intt).y=[link(intt).y,fliplr(pointY(ddist(m,1),:))];
                                        line(intt,3:4)=[link(intt).x(end),link(intt).y(end)];
                                        crackRaw{intt}=[crackRaw{intt};flipud(crackRaw{ddist(m,1)})];
                                        detind=[detind,find(1:size(d,1)~=intt)];
                                    end
                                else
                                    detind=[detind,find(1:size(d,1)~=intt)];
                                end
            %                     break
                            end




    %                     elseif max(sum(endlogi,2))==2 && isequal(cp,acp)
    %                         ll= sum(endlogi,2)==max(sum(endlogi,2));
    %                         tt=reshape([templine(ll,[1,3]),templine(ll,[2,4])],[],2);
    % 
    %                         % Unfilled Crack 
    %                         ee=endlogi(ll,:);
    %                         tt2=tt(ee(:),:);
    %                         [cd,ee] = min(spdist(cp,tt2));[~,ee]=ismember(tt2(ee),tt);
    % 
    %                         [~,ee] = dsearchn(tt(ee,:),realEndP);ee=ee<6;
    %                         [~,d]=dsearchn(realEndP(ee,:),[templine(:,1) templine(:,2);templine(:,3) templine(:,4)]);d=d<6;d=reshape(d,[],2);
    %                         d(~(sum(d,2)&ll),:)=0;ll=find(sum(d,2));pp1=[];
    % 
    %                         if length(ll)>1
    %                             for lll=ll'
    %                                 [~,pp]=dsearchn(crackRaw{lll},acp);pp1=[pp1;pp];
    %                             end
    %                             d(ll(~(min(pp1)==pp1)),:)=0;
    %                         end
    % 
    %                         if any(d,'all')%any(d(:,1))
    %                             intt= find(d(:,1));if isempty(intt); intt= find(d(:,2));end 
    %                             if find(d(intt,:))==1
    %                                 link(intt).x=pointX(intt,:);
    %                                 link(intt).y=pointY(intt,:);
    %                             else
    %                                 link(intt).x=fliplr(pointX(intt,:));
    %                                 link(intt).y=fliplr(pointY(intt,:));
    %                                 endlogi(intt,:)= fliplr(endlogi(intt,:));
    %                                 crackRaw{intt}=flipud(crackRaw{intt});
    %                             end
    %                             detind=[detind,find(1:size(d,1)~=intt)];
    %                         else
    %                             detind=1:size(line,1);
    %                         end




                        elseif endlogi(intt,2)==0 || ~isequal(cp,acp)                                                % Current line has contEndpoint, remove all other lines
                            detind=[detind,find(1:size(d,1)~=intt)];%add1=false;
            %                 break
                        end

                    end

                elseif max(sum(endlogi,2))==2 && isequal(cp,acp)
                    ll= sum(endlogi,2)==max(sum(endlogi,2));
                    tt=reshape([templine(ll,[1,3]),templine(ll,[2,4])],[],2);

                    % Unfilled Crack 
                    ee=endlogi(ll,:);
                    tt2=tt(ee(:),:);
                    % [cd,ee] = min(spdist(cp,tt2));[~,ee]=ismember(tt2(ee,:),tt,'rows');

                    [cd,ee1] = min(spdist(cp,tt2));[~,ee]=ismember(tt2(ee1,:),tt,'rows');  %Finds distance between the current point and the true end points

                    %%%% Check if the point is an intersection point
                    while(1)
                        if sum(ismember(tt,tt(ee,:),'rows'))>1
                            tt2(ismember(tt2,tt2(ee1,:),'rows'),:)=[];
                            [cd,ee1] = min(spdist(cp,tt2));[~,ee]=ismember(tt2(ee1,:),tt,'rows');
                        else
                            break 
                        end
                    end
                    %%%%

                    [~,ee] = dsearchn(tt(ee,:),realEndP);ee=ee<6;
                    [~,d]=dsearchn(realEndP(ee,:),[templine(:,1) templine(:,2);templine(:,3) templine(:,4)]);d=d<6;d=reshape(d,[],2);
                    d(~(sum(d,2)&ll),:)=0;ll=find(sum(d,2));pp1=[];

                    if length(ll)>1
                        for lll=ll'
                            [~,pp]=dsearchn(crackRaw{lll},acp);pp1=[pp1;pp];
                        end
                        d(ll(~(min(pp1)==pp1)),:)=0;
                    end

                    if any(d,'all')%any(d(:,1))
                        intt= find(d(:,1));if isempty(intt); intt= find(d(:,2));end 
                        if find(d(intt,:))==1
                            link(intt).x=pointX(intt,:);
                            link(intt).y=pointY(intt,:);
                        else
                            link(intt).x=fliplr(pointX(intt,:));
                            link(intt).y=fliplr(pointY(intt,:));
                            endlogi(intt,:)= fliplr(endlogi(intt,:));
                            crackRaw{intt}=flipud(crackRaw{intt});
                        end
                        detind=[detind,find(1:size(d,1)~=intt)];
                    else
                        detind=1:size(line,1);%add1=false;
                    end


                else
                    detind=1:size(line,1);%add1=false;
                end

                line(detind,:)=[];
                link(all(cell2mat(arrayfun(@(x) structfun(@isempty, x), link, 'UniformOutput', false)),1)) = []; %link(detind)=[];
                crackRaw(detind)=[];
                
                fP(detind)=[];
            end

            % Adding edges on hold if add1 is true 
            if ~isempty(h_line)%exist('h_line','var')% add1
                line = [line;h_line];
                crackRaw=[crackRaw,h_crackRaw];fP=[fP;h_fP];
                for kk=1:size(h_line,1)
                    link(end+1).x=fliplr(h_pointX(kk,:));
                    link(end).y=fliplr(h_pointY(kk,:));
                end
            end
            crackR=crackRaw;
%             if exist('h_line','var')
%                 if size(h_line,1)>1
%                     dd=[spdist(cp,h_line(:,[1,2])),spdist(cp,h_line(:,[3,4]))];
%                     h_line=h_line(sum(dd==min(min(dd)),2)>0,:);
%                     h_pointX=h_pointX(sum(dd==min(min(dd)),2)>0,:);
%                     h_pointY=h_pointY(sum(dd==min(min(dd)),2)>0,:);
%                     h_crackRaw={h_crackRaw{find(sum(dd==min(min(dd)),2)>0)}};
%                 end
%                 t_line = [line;h_line];
%                 dd=[spdist(cp,t_line(:,[1,2])),spdist(cp,t_line(:,[3,4]))];
%                 ttt=sum(dd==min(min(dd)),2);add1=ttt(end);%
%                 if add1
%                     crackRaw=h_crackRaw;fP=h_fP;
%                     link.x=h_pointX;
%                     link.y=h_pointY;
%                 end
%             end
            
            if ~isempty(link) && worki
                for p=1:length(link);plot(link(p).y,link(p).x,'LineWidth',2);end
            end
            
            if ~isempty(preCrack)
                if isequal(crackRaw,preCrack)
                    fline=[];  %to skip the whole function
                end
            end
            
          
           %% To solve the cracks with sharp acute angle
            %%%% 
            for l=1:length(crackRaw)
                if total_length(crackRaw{l})>2*a*sqrt(2)
                    aRan=polybuffer(crackRaw{l}([1,end],:),'points',a);%plot(polybuffer(fliplr(crackRaw{l}([1,end],:)),'points',a))
                    crackW=crackRaw{l}(~ismember(crackRaw{l},rmmissing(intersect(aRan,rmmissing(crackRaw{l}))),'rows'),:);
                    if total_length(crackW)>a % New ~isempty(crackW)
                        mov=crackW-crackW(1,:);
                    else
                        mov=crackRaw{l}-crackRaw{l}(1,:);
                    end
                else
                    mov=crackRaw{l}-crackRaw{l}(1,:);%plot(mov(:,2),mov(:,1));plot(mov(end,2),mov(end,1),'r*')
                end
                v1=[0,yy];
                v2=mov(end,:);
                ang=ab2v(v1,v2);
                rotv=[cos(deg2rad(ang)) sin(deg2rad(ang)); -sin(deg2rad(ang)) cos(deg2rad(ang))];
                rv2=(rotv*[mov]')';%plot(rv2(:,2),rv2(:,1))
                [~,d]=min(rv2(:,1)); peakPts_idx=d;
                [~,d]=max(rv2(:,1)); peakPts_idx=[peakPts_idx;d];
                [~,d]=max(abs(rv2(peakPts_idx,1)));peakPts_idx=peakPts_idx(d);%plot(rv2(peakPts_idx,2),rv2(peakPts_idx,1),'r*')
                v1=-rv2(peakPts_idx,:);v2=rv2(end,:)-rv2(peakPts_idx,:);%plot([v1(2),0],[v1(1),0]);plot([v2(2),0],[v2(1),0])
                ang=ab2v(v1,v2)/2; %if angle <45 or >360-45 change 'a' and skip edge 
                if (ang<45 || ang >360-45)&& ang~=0
%                    disp(ang)
%                    at=fP(l);
%                    while(1)
%                       ht=at/tan(deg2rad(ang)); 
%                       if ht>a;at=at-.25;else; break;end
%                    end
%                    fP(l)=at;%det=[det;l];
                    fP(l)=tan(deg2rad(ang))*a;
                end
            end
           
           %%
            if ~isempty(line)
                y=[line(:,1);line(:,3)];
                x=[line(:,2);line(:,4)];

                if worki;for b=1:length(link);plot(link(b).y,link(b).x,'LineWidth',2);end;end
                ss=struct2cell(link);
                llink = [horzcat(ss{1,1,:});horzcat(ss{2,1,:})]';clear ss

                % plot(x',y');hold off

                %% End point check 

                endlogi = reshape(ismember([line(:,1) line(:,2);line(:,3) line(:,4)],realEndP,'rows'),[],2);
                [~,d]=dsearchn(realEndP,[line(:,1) line(:,2);line(:,3) line(:,4)]);d=d<6;endlogi=endlogi|reshape(d,[],2);
                line(~(endlogi(:,1)|endlogi(:,2)),:)=[];
                link(~(endlogi(:,1)|endlogi(:,2)))=[];



                %% Algorithm Line 4 & 5 

                endPoints=unique([line(:,1) line(:,2);line(:,3) line(:,4)],'rows');endNodes=[];endPoints_t=endPoints;
                e=1;
                while(~isempty(endPoints_t)) 
                    %TestPoints = endPoints(1:end~=e,:);
                    endlogi = reshape(ismember([line(:,1) line(:,2);line(:,3) line(:,4)],endPoints_t(e,:),'rows'),[],2);
                    endlogi=sum(endlogi,2);
                    TestPoints = endPoints_t(~ismember(endPoints_t,[line(logical(endlogi),1:2);line(logical(endlogi),3:4)],'rows'),:);
                    if all(spdist(endPoints_t(e,:),TestPoints)>a)
                        endNodes = [endNodes;endPoints_t(e,:)]; endPoints_t(e,:)=[];
                    else
                        endPoints_t(e,:)=[];
                    end
                end

                %% Algorithm Line 6: Minkowsky Sum

                % figure, imshow(~BW3)
                % hold on
                n=size(line,1);

                % nnn=length(crackRaw);
                % for i=1:nnn
                %     crack=crackRaw{i};
                %     x=crack(:,1);
                %     y=crack(:,2);
                %     % plot (y,x,'linewidth',3,'color',colors{mod(i,6)+1})
                % end

                for i=1:n
                    poly = polybuffer([link(i).y',link(i).x'],'line',fP(i));%a
        %             plot(poly,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha', 0.3)
                    [px,py]=poly.boundary;
                    if ~(px(1)==px(end)&&py(1)==py(end));px(end+1)=px(1);py(end+1)=py(1);end
                    poly.Vertices=DecimatePoly([px,py],[1 1],false);
                    S1(i).P.x = poly.Vertices(:,2);
                    S1(i).P.y = poly.Vertices(:,1);
                    S1(i).P.hole = poly.NumHoles;
                    S(i).P(1) = poly;
                    if i==1
                        u=poly;
                        v(i)=poly;
                    else
                        u = union(u,poly);                  % Combining all the polygons
                        v(i)=poly;
                    end

                end

                % m=length(endP_row);

                %% Algorithm line 9: Shorten the endpoints
% 
%                 for e=1:size(endNodes,1)
%                     aRan=polybuffer(endNodes(e,:),'points',a);
%                     for c=1:size(crackRaw,2)
%                         if any(isinterior(aRan,crackRaw{c}([1,end],:)))%spdist(endNodes(e,:),crackRaw{c}(1,:))<10||spdist(endNodes(e,:),crackRaw{c}(end,:))<10
%                             [~,out]=intersect(aRan,crackRaw{c});
%                             if ~isempty(out)                                            %QuickFix 
%                                 crackRaw{c}=out;ee=crackRaw{c}([1,end],:);
%                                 ddd=min(spdist(endNodes(e,:),ee))==spdist(endNodes(e,:),ee);
%                                 endNodes(e,:)=ee(ddd,:);
%                             end
%                         end
%                     end
%                 end
                
% % %                 for e=1:size(endNodes,1)
% % %                     aRan=polybuffer(endNodes(e,:),'points',a);
% % %                     logi_cell=cell2mat(cellfun(@(x) ~isempty(x),cellfun(@(x) intersect(aRan,x),crackRaw,'un',0),'un',0));
% % %                     if sum(logi_cell)==1
% % %                         if any(isinterior(aRan,crackRaw{logi_cell}([1,end],:)))%spdist(endNodes(e,:),crackRaw{c}(1,:))<10||spdist(endNodes(e,:),crackRaw{c}(end,:))<10
% % %                             [~,out]=intersect(aRan,crackRaw{logi_cell});
% % %                             if ~isempty(out)                                            %QuickFix 
% % %                                 crackRaw{logi_cell}=out;ee=crackRaw{logi_cell}([1,end],:);
% % %                                 ddd=min(spdist(endNodes(e,:),ee))==spdist(endNodes(e,:),ee);
% % %                                 endNodes(e,:)=ee(ddd,:);
% % %                             end
% % %                         end
% % %                     end
% % %                 end

% cellfun(@(x) intersect(aRan,x),crackRaw(1:length(crackRaw)~=c),'un',0);cell2mat(ans)
% cellfun(@(x) intersect(aRan,x),crackRaw,'un',0);
% sum(cell2mat(cellfun(@(x) ~isempty(x),cellfun(@(x) intersect(aRan,x),crackRaw,'un',0),'un',0)))

                %% Algorithm line 7 & 8: Calculate the areas and find the overlapping areas

                TF = overlaps(v);
                if any(TF.*~eye(size(TF)),'all')
                    Display_result = 0;
                    Accuracy       = 1e-3;
                    Geo=Polygons_intersection_vv1(S,Display_result,Accuracy);
%                     Geo=Polygons_intersection_v_clipper(S1,Display_result,Accuracy);
                    %for p=1:length(Geo);Geo(p).P.plot;end
                    %%% Computing the centroinds of the verlapping areas and removing the nodes that has dstance less than the robots footprint.
                    N=length(Geo);
                    Ind=[];
                    nodes=[];
                    for i=1:N
                        index=Geo(i).index;
                        inum=length(index);
                        if inum>=2                                  % Selecting the overlapping area index
                            pnum = length(Geo(i).P);
                            for j = 1:pnum
                                Ind=[Ind;inum];
%                                 tempx=Geo(i).P(j).x;
%                                 tempy=Geo(i).P(j).y;
%                                 [cx,cy]=getcentroid(tempx,tempy);
                                [cy,cx]=centroid(Geo(i).P(j));
                                nodes=[nodes;cx,cy];           % Centroid Nodes of the overlapping areas
                            end                
                        end
                    end
                    
                    [~,Index]=sort(Ind,'descend');
                    SortNode=nodes(Index,:);
        %             plot(SortNode(:,2),SortNode(:,1),'pg','MarkerSize',5)

                    NodeCan = [SortNode;endNodes];node=[];

                    for n=1:size(NodeCan,1)
                        if ~isempty(node)
                            if all(spdist(NodeCan(n,:),node)>a)
                                node = [node;NodeCan(n,:)];
                            end
                        else
                            node = [node;NodeCan(n,:)];
                        end
                    end


    %                 open=SortNode;
    %                 i=1;
    %                 while ~isempty(open)                            % Removing the nodes that has distance less than the footprint of the robot
    %                     Node_cut(i,:)=open(1,:);
    %                     open(1,:)=[];
    %                     D = pdist2(Node_cut(i,:),open);
    %                     ii=find (D<a);
    %                     open(ii,:)=[];
    %                     i=i+1;
    %                 end
    % 
    %                 % %% Adding the end points to the nodes list
    %                 [mm,~]=size(endP);
    %                 for i=1:mm                                      % Chekcing if the disctance from the endpoints to nodes is grater than the robot footprint
    %                     inn=spdist(endP(i,:),Node_cut)<a;
    %                     Node_cut(inn,:)=[];
    %                 end
    %                 node=[Node_cut;llink(dsearchn(llink,endP),:)];
    %     %             [mm,~]=size(Node_cut);
    %     %             for i=1:mm                                      % Chekcing if the disctance from the endpoints to nodes is grater than the robot footprint
    %     %                 inn=spdist(Node_cut(i,:),endP)<a/2;
    %     %                 endP(inn,:)=[];
    %     %             end
    %     %             [f,d]=dsearchn(llink,endP);
    %     %             node=[Node_cut;llink(f(d<a),:)];


                else
    %                 node=llink(dsearchn(llink,endP),:);
                    node=endNodes;
                end

    %             node=unique(node,'rows');                       % Removes the duplicate nodes, distint nodes.
    %             n=size(node,1);

                %% Algorithm line 9: Shorten the endpoints (Moved here)
                for e=1:size(node,1)
                    aRan=polybuffer(node(e,:),'points',a/sqrt(2));%plot(polybuffer(fliplr(node(e,:)),'points',a))
                    logi_cell=cell2mat(cellfun(@(x) ~isempty(x),cellfun(@(x) intersect(aRan,rmmissing(x)),crackRaw,'un',0),'un',0));
                    if sum(logi_cell)==1&&ismember(node(e,:),[realEndP;fliplr(realEndP)],'rows')
                        if any(isinterior(aRan,crackRaw{logi_cell}([1,end],:)))%spdist(endNodes(e,:),crackRaw{c}(1,:))<10||spdist(endNodes(e,:),crackRaw{c}(end,:))<10
                            [~,out]=intersect(aRan,rmmissing(crackRaw{logi_cell}));
                            if ~isempty(out)                                            %QuickFix 
                                crackRaw{logi_cell}=out;ee=crackRaw{logi_cell}([1,end],:);
                                ddd=min(spdist(node(e,:),ee))==spdist(node(e,:),ee);
                                node(e,:)=ee(ddd,:);
                            end
                        end
                    end
                end
   
                %% Visibility Graph

                % S variable has the independent objects
                % node variable has all the nodes
                % figure('position', pos),hold on
                % axis([0 colBW 0 rowBW])
                % set(gca,'Ydir','reverse')

                v={};vgNE={};vgEE={};iT=[];%ch=false;
                for i = 1: length(S)
                    in = inpolygon(node(:,1),node(:,2),S(i).P.Vertices(:,2),S(i).P.Vertices(:,1));  % Finds the vertices inside the polygon
                    if any(in) %&& S(i).P.area>aP.area
                        v(i).P = [find(in),node(in,:)]; 
                        mem=ismember(v(i).P(:,2:3),[link(i).x',link(i).y'],'rows');
                        [~,d]=dsearchn([link(i).x',link(i).y'],v(i).P(:,2:3));d=d<a;    %a/2
                        mem = mem|d;
                        temp = sortrows([dsearchn([link(i).x',link(i).y'],v(i).P(mem,2:3)),v(i).P(mem,:)],1,'ascend');  % Sorts the nodes along the polygin %
                        if length(temp(:,1))-1>0
                            for j = 1:length(temp(:,1))-1                          % Finds the edge that intersects with the polygon
                                vgNE(i).S(j,:) = [temp(j,3:4) temp(j+1,3:4)];
                                vgEE(i).S(j,:) = [temp(j,2) temp(j+1,2)];
                            end
                        else
                            vgNE(i).S=[];vgEE(i).S=[];
                        end

                        if S(i).P.NumHoles>0
                            [~,d]=dsearchn([link(i).x(temp(end,1):end)',link(i).y(temp(end,1):end)'],v(i).P(mem,2:3));d=d<a;mem1=d;
                            temp1 = sortrows([dsearchn([link(i).x',link(i).y'],v(i).P(mem1,2:3)),v(i).P(mem1,:)],1,'ascend');  % Sorts the nodes along the polygin %
                            if length(temp1(:,1))-1>0
                                for j = 1:length(temp1(:,1))-1                          % Finds the edge that intersects with the polygon
                                    vgNE(i).S = [vgNE(i).S;temp1(j,3:4) temp1(j+1,3:4)];
                                    vgEE(i).S = [vgEE(i).S;temp1(j,2) temp1(j+1,2)];
                                end
                            end
                        end
                        temp2 = dsearchn(temp(:,3:4),v(i).P(~mem,2:3));  % Sorts the nodes along the polygin %
                        vgNE(i).S = [vgNE(i).S;v(i).P(~mem,2:3),temp(temp2,3:4)];
                        vgEE(i).S = [vgEE(i).S;v(i).P(~mem,1),temp(temp2,2)];
                    else
                        iT=[iT,i];vgNE(i).S=[];vgEE(i).S=[];%ch=true;
                    end
                end
                %if ch
                    S(iT)=[];vgEE(iT)=[];vgNE(iT)=[];
                    %vgEE(all(cell2mat(arrayfun(@(x) structfun(@isempty, x), vgEE, 'UniformOutput', false)),1)) = []; 
                    %vgNE(all(cell2mat(arrayfun(@(x) structfun(@isempty, x), vgNE, 'UniformOutput', false)),1)) = [];
                %end

                for i=1:length(S)
                    visibility(i).S = line_of_sight2(vgNE(i).S(:,1:2),vgNE(i).S(:,3:4),[S(i).P.Vertices(:,2)'; S(i).P.Vertices(:,1)']);
                    %S(i).P.plot;hold on;plot([node(vgEE(i).S(:,1),2) node(vgEE(i).S(:,2),2)]',[node(vgEE(i).S(:,1),1) node(vgEE(i).S(:,2),1)]','*--');
                    for j=1:length(visibility(i).S)
                        if ~visibility(i).S(j)
                            start=vgNE(i).S(j,1:2);goal=vgNE(i).S(j,3:4);boundary=[S(i).P.Vertices(:,2)'; S(i).P.Vertices(:,1)']';
                            startn=vgEE(i).S(j,1);goaln=vgEE(i).S(j,2);
                            % vgNE(i).S(~logical(visibility(i).S),:)=[];vgEE(i).S(~logical(visibility(i).S),:)=[]; % Removes the edges
                            slen = length(node(:,1));
                            
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            % New Solution to optimize the visibility graph
                            % Keep the nodes between the start and the goal
                            % on the boundary and reduces the resolution of
                            % the boundary outside
%                             poly=polyshape(boundary);
%                             in=intersect(poly,[start;goal]);
%                             if any(isnan(in(:,1)))
%                                 l_n=find(isnan(in(:,1)));
%                                 st_p=in(l_n-1,:);
%                                 en_p=in(l_n+1,:);
%                                 ttt=dsearchn(boundary,[st_p;en_p]);
%                                 if ttt(1)~=ttt(2)
%                                     if (max(ttt)-min(ttt))<=(size(boundary,1)-max(ttt)+min(ttt))         
%                                         nBoundary=circshift(boundary,size(boundary,1)-(min(ttt))+1);
%                                         t1=nBoundary(1:(max(ttt)-min(ttt)),:);
%                                         t2=nBoundary((max(ttt)-min(ttt))+1:end,:);
%                                         t3=t2(round(linspace(1,size(t2,1),10)),:);
%                                         nBoundary=[t1;t3;t1(1,:)];bt=5;
%                                         while(length(regions(polyshape(nBoundary))))>1
%                                             t3=t2(round(linspace(1,size(t2,1),10+bt)),:);
%                                             nBoundary=[t1;t3;t1(1,:)];
%                                             bt=bt+5;
%                                         end
%                                     else
%                                         nBoundary=circshift(boundary,size(boundary,1)-(max(ttt))+1);
%                                         t1=nBoundary(1:(size(boundary,1)-max(ttt)+min(ttt)),:);
%                                         t2=nBoundary((size(boundary,1)-max(ttt)+min(ttt))+1:end,:);
%                                         t3=t2(round(linspace(1,size(t2,1),10)),:);
%                                         nBoundary=[t1;t3;t1(1,:)];bt=5;
%                                         while(length(regions(polyshape(nBoundary))))>1
%                                             t3=t2(round(linspace(1,size(t2,1),10+bt)),:);
%                                             nBoundary=[t1;t3;t1(1,:)];
%                                             bt=bt+5;
%                                         end
%                                     end
%                                 else
%                                     nBoundary=circshift(boundary,size(boundary,1)-(min(ttt))+1);
%                                     t2=nBoundary((max(ttt)-min(ttt))+1:end,:);
%                                     t3=t2(round(linspace(1,size(t2,1),10)),:);
%                                     nBoundary=[t3;t3(1,:)];
%                                 end
%                             end
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            
% %                             %%%%%%%%%%%%
% %                             poly=S(i).P;%plot(polyshape(fliplr(boundary)));
% %                             in=intersect(poly,fliplr([start;goal]));
% %                             if any(isnan(in(:,1)))
% %                                 l_n=find(isnan(in(:,1)));
% % 
% %                                 st_p=fliplr(in(l_n-1,:));
% %                                 en_p=fliplr(in(l_n+1,:));
% %                                 ttt=dsearchn(boundary,[st_p;en_p]);ttt=sort(ttt)';
% %                                 % plot(boundary(ttt(1):ttt(2),2),boundary(ttt(1):ttt(2),1))
% %                                 
% %                                 ttt=[1,ttt,size(boundary,1)];
% %                                 bt=0;
% %                                 while true
% %                                     nBoundary=[];
% %                                     for m=1:length(ttt)-1
% %                                         if mod(m,2)
% %                                             t2=boundary(round(linspace(ttt(m),ttt(m+1),10+bt)),:);
% %                                         else
% %                                             t2=boundary(ttt(m):ttt(m+1),:);
% %                                         end
% %                                         nBoundary=[nBoundary;t2];
% %                                     end
% % 
% %                                     if (length(regions(polyshape(nBoundary))))==1
% %                                         break
% %                                     else
% %                                         bt=bt+5;
% %                                     end
% %                                 end
% %                             end
% %                             %%%%%%%%%%%%
%                             nBoundary=DecimatePoly([boundary;boundary(1,:)],[1 1]);
                            [waypoints,~]=pathfinder(start,goal,boundary);nn=waypoints(2:end-1,:);%disp(wt)
%                             plot(waypoints(:,2),waypoints(:,1),'*--')
                            node = [node;nn];elen = length(node(:,1));
                            if ~isempty(nn)
                                vgNE(i).S = [vgNE(i).S;start,nn(1,:)];
                                vgEE(i).S = [vgEE(i).S;startn,slen+1];
                                if length(nn(:,1))>1
                                    for k = 2:size(nn,1)
                                        vgNE(i).S = [vgNE(i).S;vgNE(i).S(end,3:4),nn(k,:)];
                                        vgEE(i).S = [vgEE(i).S;vgEE(i).S(end,2),slen+k];
                                    end
                                end
                                vgNE(i).S = [vgNE(i).S;vgNE(i).S(end,3:4),goal];
                                vgEE(i).S = [vgEE(i).S;vgEE(i).S(end,2),goaln];
                            else
                                vgNE(i).S = [vgNE(i).S;start,goal];
                                vgEE(i).S = [vgEE(i).S;startn,goaln];
                            end
                        end
                    end
                    vgNE(i).S(~logical(visibility(i).S),:)=[];vgEE(i).S(~logical(visibility(i).S),:)=[]; % Removes the edges
                end

                edgeList = [];
                for i=1: size(vgEE,2)
                    edgeList = [edgeList ;vgEE(i).S];
                end
                % text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
                % plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','*--')

                %% Shorten the endpoints, Algorithm line 9

    %             if any(TF.*~eye(size(TF)),'all')
    %                 G=graph(edgeList(:,1),edgeList(:,2));n=find(degree(G)==1);clear G
    %                 detind=[];
    %                 for i=n'                                               
    %                     ind=i;
    %                     if ~isempty(ind)
    %                         ind2=find(edgeList(:,1)==ind);
    %                         dir=(node(edgeList(ind2,2),:)-node(i,:))./norm(node(edgeList(ind2,2),:)-node(i,:));
    %                         if isempty(ind2)
    %                             ind2=find(edgeList(:,2)==ind);
    %                             dir=(node(edgeList(ind2,1),:)-node(i,:))./norm(node(edgeList(ind2,1),:)-node(i,:));
    %                         end
    %                         if length(ind2)<2
    %                             node(ind,:)=node(i,:)+a*dir;
    %                             if sum((node(ind,1) - Node_cut(:,1)).^2+ (node(ind,2) - Node_cut(:,2)).^2<a^2/4)
    %                                 detind=[detind ind];
    %                             end
    %                         end
    %                         if length(ind2)==2
    %                             P1=node(edgeList(ind2(1),2),:);
    %                             P2=node(edgeList(ind2(2),2),:);
    %                             mid=midP(P1,P2);
    %                             dir=(mid-node(i,:))./norm(mid-node(i,:));
    %                             node(ind,:)=node(i,:)+a*dir;
    %                         end
    %                     end
    %                 end
    %             end

                % figure('position', pos)
                % axis([0 colBW 0 rowBW])
                % hold on
                % set(gca,'Ydir','reverse')
                % plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)

                % text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
                % plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','*--')

                %% Plot node and edges 

                % figure('position', pos),hold on
                % plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)
                % axis([0 colBW 0 rowBW])
                % set(gca,'Ydir','reverse')
                % No_node=length(node);
                if worki
                    text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
                    plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','--')
                end

                edgeList = unique(sort(edgeList, 2),'rows');

                %%% Test
                G=graph(edgeList(:,1),edgeList(:,2));
    %             pp=polybuffer(ppath,'line',a);in=isinterior(pp,fliplr(node));in=find(in);
    %             if length(in)>1
    %                for ii=1:length(in)
    %                    n=neighbors(G,in(ii));
    %                    for nn=1:length(n)
    %                        if ismember(n(nn),in)
    %                            G=rmedge(G,in(ii),n(nn));
    %                        end
    %                    end
    %                end
    %             end
    %             nn=unique(G.Edges.EndNodes);edgeList=G.Edges.EndNodes;
                %%%
                
                %% No Neighbor node correction                
                while any(~ismember(1:numel(node(:,1)),unique(G.Edges.EndNodes)))
                    nnode=find(~ismember(1:numel(node(:,1)),unique(G.Edges.EndNodes))); %finding nodes with no neighbor
                    [~,ddd]=min(spdist(cp,node(nnode,:)));                              %selecting closest node to the CP
                    dd= spdist(node(nnode(ddd),:),node);dd(dd==0)=inf;[~,dd]=min(dd);
                    G=addedge(G,nnode(ddd),dd);
                    edgeList=[edgeList;nnode(ddd),dd];
                end
                
                if 0
                    text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
                    plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','--')
                end

                %% Get the graph

                if  size(edgeList,1)>1%(size(realEndP,1)>1 || size(contEndP,1)>1) &&

                    %%%%%%% Remove Continuining endpoints connections 
        %             G=graph(edgeList(:,1),edgeList(:,2));
        %             b_n=1*(mod(degree(G),2)~=0);
        %             combEdge = combnk(find(b_n),2);  
        % 
        %             colOR = @(x) x(:,1)|x(:,2);
                    adj=full(adjacency(G));b_n=mod(sum(adj,2),2)~=0;
        %             remEdge=combEdge(colOR(combEdge==find(ismember(node,contEndP,'rows'))'),:);
        %             remEdge=[remEdge;fliplr(remEdge)];
                    remNode = find(ismember(node,contEndP,'rows'));eeP=[line(:,[1,2]);line(:,[3,4])];
%                     [~,d]=dsearchn(cp,node);[~,d]=min(d);%d=nn(d);
                    [~,d]=min(spdist(cp,eeP));
%                     d=find(spdist(cp,node)==min(spdist(cp,node(b_n,:))));
                    %%%New selecting nodes with neighbor
                    nodewNeighbor=node(unique(G.Edges.EndNodes(:)),:);
                    [~,d]=min(spdist(eeP(d,:),nodewNeighbor));
                    [~,d]=ismember(nodewNeighbor(d),node);
                    %%%%%%%
                    
                    %No_node=length(node);
                    %edgeListG=[edgeList;edgeList(:,2),edgeList(:,1)];
                    %adj = full(sparse(edgeListG(:, 1), edgeListG(:, 2), 1, No_node, No_node));
                    
                    Dist =squareform(pdist(node));
                    AdjMax=adj.*Dist;
                    % [Path, weight, add,st]=ChinesePostman(adj,AdjMax,Dist);
                    [Path, ~, add,st]=ChinesePostman(adj,AdjMax,Dist,[],remNode,d);
                    if ~isempty(st); add(ismember(add,st,'rows'),:)=[]; end
    %                 n=size(add,1);
                    %edgeList = [edgeList;add];

    %                 for i=1:n
    %                     poly = polybuffer(fliplr(node(add(i,:),:)),'line',a);
    %                     u=union(u,poly);
    %                 end
                else
                    % Path = [edgeList,edgeList(1)];
                    [~,d]=dsearchn(cp,node);[~,d]=min(d);
                    Path = [edgeList(d),edgeList(1:end~=d)];
                end

                %% Ploting the Path

%                 if Path(end)==Path(1);Path(end)=[];end

%                 text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
%                 plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','--')

                waypoint=node(Path,:);

                waypoint_coords=waypoint;
                [~,d1]=dsearchn(waypoint_coords(1,:),realEndP);d1=d1<=a+5;[~,d2]=dsearchn(waypoint_coords(end,:),realEndP);d2=d2<=a+5;%realEndP
                endPP = [waypoint_coords(1,:),any(d1);waypoint_coords(end,:),any(d2)];
                
%                 %%% Test
%                 pp=polybuffer(waypoint_coords,'line',44*sqrt(2)); %44 is the footprint radius
%                 %plot(crackRaw{1}(:,1),crackRaw{1}(:,2))
%                 dd=[];a_t=a;
%                 for ii=1:length(crackRaw)
%                     t = isinterior(pp,crackRaw{ii});dd=[dd;all(t)];
%                 end
%                 if ~all(dd)
%                     pp=polybuffer(waypoint_coords,'line',(44+10)*sqrt(2));dd=[];
%                     for ii=1:length(crackRaw)
%                         t = isinterior(pp,crackRaw{ii});dd=[dd;all(t)];
%                     end
%                     if ~all(dd)
%                         a_t=a_t-10;
%                         [waypoint_coords,flag,~] = image_planning_func2(BW,a_t,s,cp,acp,endP,realEndP,contEndP,ppath,[]);
%                     end
%                 end
%                 %%%

%%
                if any(endPP(:,3)) && a==a%44  %%%Not sure why
                    flag = 1;
                    if endPP(1,3)~=endPP(2,3)
                        if endPP(2,3)
                            waypoint_coords=flipud(waypoint_coords);
                            % waypoint_coords=[cp;waypoint_coords];
                            % Adding equidistant points along the path
                            [marker_x1,marker_y1] = addPtsLin([cp(1),waypoint_coords(1,1)],[cp(2),waypoint_coords(1,2)],a+5);
                            marker_x2=[];marker_y2=[];
                            for h=1:size(waypoint_coords,1)-1
                                [m,n] = addPtsLin(waypoint_coords([h h+1],1)',waypoint_coords([h h+1],2)',a);
                                marker_x2=[marker_x2,[waypoint_coords(h,1) m]];marker_y2=[marker_y2,[waypoint_coords(h,2) n]];
                            end 
                            % [marker_x2,marker_y2] = addPtsLin(waypoint_coords(:,1)',waypoint_coords(:,2)',a);
                            if ~isempty(marker_x1);b=1;else;b=[];end
                            dd=false(1,length(marker_x2));
                            for ii=1:length(crackRaw)
                                [~,t]=dsearchn(crackRaw{ii},[marker_x2;marker_y2]');dd=dd|t'<=a*sqrt(2);
                            end
                            waypoint_coords=[[marker_x1;marker_y1;[zeros(1,size(marker_x1,2)-1) b]]';[marker_x2;marker_y2;dd]';[waypoint_coords(end,:),1]];  % Since the first point is the current point, cp; ones(1,size(marker_x2,2))
                        else
                            % waypoint_coords=[cp;waypoint_coords];
                            % Adding equidistant points along the path
                            [marker_x1,marker_y1] = addPtsLin([cp(1),waypoint_coords(1,1)],[cp(2),waypoint_coords(1,2)],a+5);
                            marker_x2=[];marker_y2=[];
                            for h=1:size(waypoint_coords,1)-1
                                [m,n] = addPtsLin(waypoint_coords([h h+1],1)',waypoint_coords([h h+1],2)',a);
                                marker_x2=[marker_x2,[waypoint_coords(h,1) m]];marker_y2=[marker_y2,[waypoint_coords(h,2) n]];
                            end 
                            % [marker_x2,marker_y2] = addPtsLin(waypoint_coords(:,1)',waypoint_coords(:,2)',a);
                            if ~isempty(marker_x1);b=1;else;b=[];end
                            dd=false(1,length(marker_x2));
                            for ii=1:length(crackRaw)
                                [~,t]=dsearchn(crackRaw{ii},[marker_x2;marker_y2]');dd=dd|t'<=a*sqrt(2);
                            end
                            waypoint_coords=[[marker_x1;marker_y1;[zeros(1,size(marker_x1,2)-1) b]]';[marker_x2;marker_y2;dd]';[waypoint_coords(end,:),1]];
                        end
                    else
                        [~,d]=min(spdist(cp,endPP(:,[1,2])));
                        if d==2
                            waypoint_coords=flipud(waypoint_coords);
                            % waypoint_coords=[cp;waypoint_coords];
                            % Adding equidistant points along the path
                            [marker_x1,marker_y1] = addPtsLin([cp(1),waypoint_coords(1,1)],[cp(2),waypoint_coords(1,2)],a+5);
                            marker_x2=[];marker_y2=[];
                            for h=1:size(waypoint_coords,1)-1
                                [m,n] = addPtsLin(waypoint_coords([h h+1],1)',waypoint_coords([h h+1],2)',a);
                                marker_x2=[marker_x2,[waypoint_coords(h,1) m]];marker_y2=[marker_y2,[waypoint_coords(h,2) n]];
                            end 
                            % [marker_x2,marker_y2] = addPtsLin(waypoint_coords(:,1)',waypoint_coords(:,2)',a);
                            if ~isempty(marker_x1);b=1;else;b=[];end
                            dd=false(1,length(marker_x2));
                            for ii=1:length(crackRaw)
                                [~,t]=dsearchn(crackRaw{ii},[marker_x2;marker_y2]');dd=dd|t'<=a*sqrt(2);
                            end
                            waypoint_coords=[[marker_x1;marker_y1;[zeros(1,size(marker_x1,2)-1) b]]';[marker_x2;marker_y2;dd]';[waypoint_coords(end,:),1]];  % Since the first point is the current point, cp
                        else
                            % waypoint_coords=[cp;waypoint_coords];
                            % Adding equidistant points along the path
                            [marker_x1,marker_y1] = addPtsLin([cp(1),waypoint_coords(1,1)],[cp(2),waypoint_coords(1,2)],a+5);
                            marker_x2=[];marker_y2=[];
                            for h=1:size(waypoint_coords,1)-1
                                [m,n] = addPtsLin(waypoint_coords([h h+1],1)',waypoint_coords([h h+1],2)',a);
                                marker_x2=[marker_x2,[waypoint_coords(h,1) m]];marker_y2=[marker_y2,[waypoint_coords(h,2) n]];
                            end 
                            % [marker_x2,marker_y2] = addPtsLin(waypoint_coords(:,1)',waypoint_coords(:,2)',a);
                            if ~isempty(marker_x1);b=1;else;b=[];end
                            dd=false(1,length(marker_x2));
                            for ii=1:length(crackRaw)
                                [~,t]=dsearchn(crackRaw{ii},[marker_x2;marker_y2]');dd=dd|t'<=a*sqrt(2);
                            end
                            waypoint_coords=[[marker_x1;marker_y1;[zeros(1,size(marker_x1,2)-1) b]]';[marker_x2;marker_y2;dd]';[waypoint_coords(end,:),1]];
                        end

                    end
                else 
                    
                    flag = 0;
                end
                
%                 %%% Test
%                 pp=polybuffer(waypoint_coords(logical(waypoint_coords(:,3)),1:2),'line',44*sqrt(2)); %44 is the footprint radius
%                 %plot(crackRaw{1}(:,1),crackRaw{1}(:,2))
%                 dd=[];
%                 for ii=1:length(crackRaw)
%                     t = isinterior(pp,crackRaw{ii});dd=[dd;all(t)];
%                 end
%                 if ~all(dd)
%                     a=a-10;
%                     [waypoint_coords,flag,~] = image_planning_func2(BW,a,s,cp,acp,endP,realEndP,contEndP,ppath,[]);
%                 end
%                 %%%

                % save knownCrackG.mat node edgeList

            else
                waypoint_coords=[];%crackR=[];
                flag = 0;
            end
        else
            waypoint_coords=[];crackR=[];
            flag = 0;
        end
    else
    	waypoint_coords=[];crackR=[];
        flag = 0;
    end
%     waypoint_coords(:,3)=1;
ttt=toc;
end


%% Functions

function ang = ab2v(a,b) 
    theta = rad2deg(atan2(norm(cross([1,0,0],[a,0])), dot([1,0,0],[a,0])));
    if a(2)<0 theta=360-theta; end
    
    aR = a*[1;1i]*exp(-1i*theta*pi/180); aR=[real(aR) imag(aR)];
    bR = b*[1;1i]*exp(-1i*theta*pi/180); bR=[real(bR) imag(bR)];

    ang=rad2deg(atan2(norm(cross([aR,0],[bR,0])), dot([aR,0],[bR,0])));
    if bR(2)<0 ang=360-ang; end
end

function y = bound(x,bl,bu)
  % return bounded value clipped between bl and bu
  y=min(max(x,bl),bu);
end

function [marker_x,marker_y] = addPtsLin(x,y,marker_dist)
    % Adding equidistant points along Path
    dist_from_start = cumsum( [0, sqrt((x(2:end)-x(1:end-1)).^2 + (y(2:end)-y(1:end-1)).^2)] );marker_x=[];marker_y=[];
    marker_locs = marker_dist : marker_dist : dist_from_start(end);   %replace with specific distances if desired
    if length(marker_locs)>1 %~isempty(marker_locs)
        marker_indices = interp1( dist_from_start, 1 : length(dist_from_start), marker_locs);
        marker_base_pos = floor(marker_indices);
        weight_second = marker_indices - marker_base_pos;
        marker_x = [marker_x, x(marker_base_pos) .* (1-weight_second) + x(marker_base_pos+1) .* weight_second];
        marker_y = [marker_y, y(marker_base_pos) .* (1-weight_second) + y(marker_base_pos+1) .* weight_second];
    end
end
