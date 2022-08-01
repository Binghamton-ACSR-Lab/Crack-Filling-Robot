%% Image Planning Function for SCC
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [node,edgeList,ttt]=Image_planning_func_SCC(img_n)
%
% Performs Image processing, extacts cracks and metadata, and computes the waypoints for the crackGraph using Visibility Graph.
%
% INPUTS:
%   img_n = RGB or Binary image of the workspace. 
%
% OUTPUTS:
%   node = [x0; y0] = nodes list of all edges in crackGraph in undirected graph format. 
%   edgeList = [node1; node2] = edges list of all edges in crackGraph in undirected graph format.
%   ttt = Computation time.

% clc
% 
% close all 
% clear all
spdist = @(P,Ps) sqrt((P(1,1)-Ps(:,1)).^2 + (P(1,2)-Ps(:,2)).^2);
midP= @(P1,P2) (P1(:)+ P2(:)).'/2;
total_length = @(Ps) sum(sqrt(sum(diff(Ps).*diff(Ps),2)));

inpxMap = @(x) fix(((x)*25.4)/2);
pxinMap = @(x) round((x)*2/25.4,1);

botD = 48;                                       % 48" Diameter
footD= 7;                                        % 7" Diameter
sensD= 4.5*12;                                   % 4.5*12" Diameter

r1=inpxMap(botD/2);                      % Robot Radius 
a=inpxMap(footD/2);                      % Footprint Radius
s=inpxMap(sensD/2);                      % Sensor Range Radius

% tic
% %%  Import Crack Image and extract the crack information
worki=false;
plt = false;
% img_n='myCrack2_45';

if ischar(img_n)
    BW = imread(['Crack Maps/Uniform/' img_n '.png']);      % Importing the map w/ crack information imread([img_n '.png']);%
    BW = imbinarize(BW);  % Converting the image into a binary image
    BW = BW(:,:,1);
    % figure, imshow(BW)

    BW = bwareaopen(BW, 50);        % Removes all objects that have fewer than 50 pixels from the binary image BW
    % BW = padarray(~BW,[1 1]);        % Adds a boundary around the map
    BW = ~BW;                       % Reverses the binary
    % figure, imshow(BW)

    IM2 = imcomplement(BW);         % Computes the complement of the image BW
    % figure, imshow(IM2);
    rawBW = BW;

else
	BW = img_n;  
end

% BW2 = bwmorph(BW,'thin', inf);  % bwskel(BW);% Thins objects to lines, making up the image skeleton
% figure, imshow(BW2);
BW2=bwmorph(BW,'fill');             % New Fills 1 pixel islands
BW2 = bwskel(BW2>0);

BW3 = bwmorph(BW2, 'spur', a); % Removes spur pixels


%%%%%%%%%%%%%%%
% load VISHNU.mat
% % IM2 = imdilate(IM,strel('square',10));
% % IM2 = bwmorph(IM2,'fill',inf);
% % % IM2 = bwmorph(IM2,'thin', inf);  % bwskel(BW);% Thins objects to lines, making up the image skeleton
% % IM2 = bwmorph(IM2,'skel',inf);
% % IM2 = bwmorph(IM2, 'spur', a);
% % IM2 = bwmorph(IM2,'thin', inf);
% % BW3=IM2;
%%%%%%%%%%%%%%%

% montage({BW,BW3},'BackgroundColor','blue','BorderSize',5)
if worki; figure, imshow(~BW3); hold on; end
if plt; figure, imshow(~BW3); hold on; end
[rowBW, colBW]= size(BW3);
% pos=get(gcf, 'Position');

skelBW = BW3;

endPoints = bwmorph(BW3, 'endpoints');  % Finds end points of skeleton
[endP_row, endP_col]=find(endPoints);
if worki; plot(endP_col,endP_row,'b*'); end

intPoints = bwmorph(BW3, 'branchpoints');
[intP_row, intP_col]=find(intPoints);
if worki; plot(intP_col,intP_row,'r*'); end

I=BW3;%imshow(~BW3)

endP=[endP_row endP_col];

dir_map=[-1 -1;-1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0 ; 1 1];
dir_can=[1 2 4 3 6;2 1 3 4 5; 3 2 5 1 8; 4 1 6 2 7 ; 5 3 8 2 7; 6 4 7 1 8; 7 6 8 4 5; 8 5 7 3 6];
colors={'y','m','c','r','g','b'};

[crackRaw,line,pointX,pointY] = compCrack(I,endP,dir_map,colors);     % Extract the crack information from the image
% save cc.mat crackRaw

% %%% Removing small pixels
% ind = cell2mat(cellfun(@(x) length(x),crackRaw,'un',0))<a;
% % cellfun(@(x) plot(x(:,2),x(:,1)),crackRaw(ind),'un',0)
% crackRaw(ind)=[];line(ind,:)=[];pointX(ind,:)=[];pointY(ind,:)=[];
% %%%

tfun=tic;

%%%% Alternative BWMorph Spur
c_chq=cellfun(total_length,crackRaw);c_chq=c_chq<a;
line(c_chq,:)=[];
pointX(c_chq,:)=[];pointY(c_chq,:)=[];
crackRaw(c_chq)=[];
%%%%

%if worki; cellfun(@(x) plot(x(:,2),x(:,1)),crackRaw,'un',0); end


% ee=reshape([line(:,[1,3]),line(:,[2,4])],length(line(:,1))*2,2);
% plot(ee(:,2),ee(:,1),'pr')
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
endPoints=[endP_row,endP_col];
% plot(endP(:,2),endP(:,1),'pr');hold off
%%%%%%%%

%%% Saving Crack Information 
% save crackRaw.mat crackRaw line pointX pointY endPoints intPoints
% save crackBW.mat rawBW skelBW

%         %% End point check 
%         
% %         endlogi = reshape(ismember([line(:,1) line(:,2);line(:,3) line(:,4)],realEndP,'rows'),[],2);
%     if ~isempty(realEndP)
%         endlogi = reshape(sum(cell2mat(cellfun(@(s) spdist(s,[line(:,1) line(:,2);line(:,3) line(:,4)]),num2cell(realEndP, 2),'un',0)')<5,2)>0,[],2);
%         endlogi_a = reshape(spdist(cp,[line(:,1) line(:,2);line(:,3) line(:,4)])<a,[],2);
%     else
%         endlogi = zeros(size(line,1),2);endlogi_a = zeros(size(line,1),2);
%     end
% %         [~,d]=dsearchn(realEndP,[line(:,1) line(:,2);line(:,3) line(:,4)]);d=d<6;endlogi=endlogi|reshape(d,[],2);
%     line(~(endlogi(:,1)|endlogi(:,2)),:)=[];crackRaw(~(endlogi(:,1)|endlogi(:,2)))=[];
%     pointX(~(endlogi(:,1)|endlogi(:,2)),:)=[];pointY(~(endlogi(:,1)|endlogi(:,2)),:)=[];
%     endlogi_a(~logical(sum(endlogi,2)),:)=[];endlogi(~logical(sum(endlogi,2)),:)=[];
    link=struct('x',cell(1,size(line,1)),'y',cell(1,size(line,1)));


%% To solve the cracks with sharp acute angle
%%%% 
fP=repmat(a,[size(line,1),1]);
[~,yy]=size(BW3);det=[];
for l=1:length(crackRaw)
    if total_length(crackRaw{l})>2*a*sqrt(2)
        aRan=polybuffer(crackRaw{l}([1,end],:),'points',a);%plot(polybuffer(fliplr(crackRaw{l}([1,end],:)),'points',a))
        crackW=crackRaw{l}(~ismember(crackRaw{l},rmmissing(intersect(aRan,rmmissing(crackRaw{l}))),'rows'),:);
        if ~isempty(crackW)
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
% det=det(sum(endlogi(det,:))>0); % Check atleat one node is a real endpoint
if ~isempty(det)
    h_line = line(det,:);
    h_pointX=pointX(det,:);h_pointY=pointY(det,:);
    h_crackRaw=crackRaw(det);h_fP=fP(det);

    line(det,:)=[];link(det)=[];
    pointX(det,:)=[];pointY(det,:)=[];
    crackRaw(det)=[];fP(det)=[]; %endlogi(det,:)=[]; endlogi_a(det,:)=[];  
end
%%%%

%% Connecting the Crack Graph

% figure
% imshow(BW3);
nnn=size(line,1);
detind=[];
templine=line;
% % for i=1:nnn
% %     crackCom=crackRaw{i};
% %      link(i).x=pointX(i,:);
% %      link(i).y=pointY(i,:);
% % 
% %      if sum(templine(i,:))==0
% %         continue;
% %      end
% %      templine(i,:)=zeros(1,4);
% %      while(1)
% %         tempdist= spdist([link(i).x(end),link(i).y(end)],templine(:,1:2));
% %         % inst=find(strlength(string(round(tempdist(:))))<=2);
% %         inst=find(tempdist(:)<=5);
% %         if ~isempty(inst)
% %             v1=crackCom(bound(length(crackCom)-4,1,length(crackCom)),:)-crackCom(end,:);
% %             for l=1:length(inst)
% %                 v2=crackRaw{inst(l)}(4,:)-crackRaw{inst(l)}(1,:);
% %                 ang(l)=ab2v(v1,v2);
% %             end
% %             ind=inst(max(ang)==ang);clear ang
% %             mmin=tempdist(ind);
% %             if mmin<=a
% %                 line(i,3:4)=line(ind,3:4);
% %                 link(i).x=[link(i).x,pointX(ind,:)];
% %                 link(i).y=[link(i).y,pointY(ind,:)];
% %                 crackCom = [crackCom;crackRaw{ind}];
% %                 line(ind,:)=zeros(1,4);
% %                 templine(ind,:)=zeros(1,4);
% %                 detind=[detind, ind];
% %             else
% %                 tempdist= spdist([link(i).x(end),link(i).y(end)],templine(:,3:4));
% %                 % inst=find(strlength(string(round(tempdist(:))))<=2);
% %                 inst=find(tempdist(:)<=5);
% %                 if ~isempty(inst)
% %                     v1=crackCom(bound(length(crackCom)-4,1,length(crackCom)),:)-crackCom(end,:);
% %                     for l=1:length(inst)
% %                         v2=crackRaw{inst(l)}(bound(length(crackRaw{inst(l)})-4,1,length(crackRaw{inst(l)})),:)-crackRaw{inst(l)}(end,:);
% %                         ang(l)=ab2v(v1,v2);
% %                     end
% %                     ind=inst(max(ang)==ang);clear ang
% %                     mmin=tempdist(ind);
% %                     if mmin<=a
% %                         line(i,3:4)=line(ind,1:2);
% %                         link(i).x=[link(i).x,pointX(ind,end:-1:1)];
% %                         link(i).y=[link(i).y,pointY(ind,end:-1:1)];
% %                         crackCom = [crackCom;flipud(crackRaw{ind})];
% %                         line(ind,:)=zeros(1,4);
% %                         templine(ind,:)=zeros(1,4);
% %                         detind=[detind, ind];
% %                     end
% %                 else
% %                     break
% %                 end
% %             end
% %         else
% %             tempdist= spdist([link(i).x(end),link(i).y(end)],templine(:,3:4));
% %             % inst=find(strlength(string(round(tempdist(:))))<=2);
% %             inst=find(tempdist(:)<=5);
% %             if ~isempty(inst)
% %                 v1=crackCom(bound(length(crackCom)-4,1,length(crackCom)),:)-crackCom(end,:);
% %                 for l=1:length(inst)
% %                     v2=crackRaw{inst(l)}(bound(length(crackRaw{inst(l)})-4,1,length(crackRaw{inst(l)})),:)-crackRaw{inst(l)}(end,:);
% %                     ang(l)=ab2v(v1,v2);
% %                 end
% %                 ind=inst(max(ang)==ang);clear ang
% %                 mmin=tempdist(ind);
% %                 if mmin<=a
% %                     line(i,3:4)=line(ind,1:2);
% %                     link(i).x=[link(i).x,pointX(ind,end:-1:1)];
% %                     link(i).y=[link(i).y,pointY(ind,end:-1:1)];
% %                     crackCom = [crackCom;flipud(crackRaw{ind})];
% %                     line(ind,:)=zeros(1,4);
% %                     templine(ind,:)=zeros(1,4);
% %                     detind=[detind, ind];
% %                 else
% %                     break
% %                 end
% %             else
% %                 break
% %             end
% %         end
% %      end
% %     if worki; plot(link(i).y,link(i).x,'LineWidth',2);end
% % end

%for intt=1:nnn
    intt=1;
    crackRawt=crackRaw;
    link(intt).x=pointX(intt,:);
    link(intt).y=pointY(intt,:);

%     if sum(templine(intt,:))==0
%     continue;
%     end
    templine(intt,:)=zeros(1,4);intt2=(intt+1:size(line,1))';indt=intt;
    while(~isempty(intt2))
        tempdist= spdist([link(intt).x(end),link(intt).y(end)],templine(:,1:2));
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
            end
        end
        if any(templine,'all')
            continue
        else
            break
        end
    end
    
%end


line(detind,:)=[];
link(all(cell2mat(arrayfun(@(x) structfun(@isempty, x), link, 'UniformOutput', false)),1)) = []; %link(detind)=[];
crackRaw(detind)=[];

fP(detind)=[];

% Adding edges on hold if add1 is true 
if exist('h_line','var')% add1
    line = [line;h_line];
    crackRaw=[crackRaw,h_crackRaw];fP=[fP;h_fP];
    for kk=1:size(h_line,1)
        link(end+1).x=fliplr(h_pointX(kk,:));
        link(end).y=fliplr(h_pointY(kk,:));
    end
end

if ~isempty(link) && worki
    for p=1:length(link);plot(link(p).y,link(p).x,'LineWidth',2);end
end

y=[line(:,1) line(:,3)];
x=[line(:,2) line(:,4)];

ss=struct2cell(link);
llink = [horzcat(ss{1,1,:});horzcat(ss{2,1,:})]';clear ss

% plot(x',y');hold off
%% To solve the cracks with sharp acute angle
%%%% 
for l=1:length(crackRaw)
    if total_length(crackRaw{l})>2*a*sqrt(2)
        aRan=polybuffer(crackRaw{l}([1,end],:),'points',a);%plot(polybuffer(fliplr(crackRaw{l}([1,end],:)),'points',a))
        crackW=crackRaw{l}(~ismember(crackRaw{l},rmmissing(intersect(aRan,rmmissing(crackRaw{l}))),'rows'),:);
        if ~isempty(crackW)
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
% y=[line(:,1) line(:,3)];
% x=[line(:,2) line(:,4)];

nnn=length(crackRaw);
for i=1:nnn
    crack=crackRaw{i};
    x=crack(:,1);
    y=crack(:,2);
%     plot (y,x,'linewidth',3,'color',colors{mod(i,6)+1})
end

% u=polyshape();
for i=1:n
%     [x,y] = thick_line(link(i).x,link(i).y,a);
%     p_obj = patch(y, x, [0.7, 0.1, 0.1]);
    poly = polybuffer([link(i).y',link(i).x'],'line',a);
    [px,py]=poly.boundary;
    if ~(px(1)==px(end)&&py(1)==py(end));px(end+1)=px(1);py(end+1)=py(1);end
    poly.Vertices=DecimatePoly([px,py],[1 1],false);
    S1(i).P.x = poly.Vertices(:,2);
    S1(i).P.y = poly.Vertices(:,1);
    S1(i).P.hole = poly.NumHoles;
    S(i).P(1) = poly;
    if i==1
%         ux=x;uy=y;
        u=poly;
        v(i)=poly;
    else
%         [ux,uy] = polybool('union',ux,uy,x,y);          % Combining all the polygons
        u = union(u,poly);
        v(i)=poly;
    end
    
end
m=length(endP_row);
% hold off
% box on

if plt; plot(u,'FaceColor','w','FaceAlpha',0.1,'LineStyle','-') ; end

%% Algorithm line 7 & 8: Calculate the areas and find the overlapping areas

TF = overlaps(v);
if any(TF.*~eye(size(TF)),'all')
    Display_result = 0;
    Accuracy       = 1e-3;
%     tic
    Geo=Polygons_intersection_vv1(S,Display_result,Accuracy);
%     Geo=Polygons_intersection_v_clipper(S1,Display_result,Accuracy);
%     arrayfun(@(s) plot(s.P),Geo);  %Plotting Geo 
%     toc
    %%% Computing the centroinds of the verlapping areas and removing the nodes that has dstance less than the robots footprint.

    % hold on

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
%                 if any(ismember([Geo(i).P(j).x',Geo(i).P(j).y'],[0,0],'rows'))
%                     temp=rmoutliers([Geo(i).P(j).x',Geo(i).P(j).y']);
%                     Geo(i).P(j).x=temp(:,1)';Geo(i).P(j).y=temp(:,2)';
%                 end
%                 temp=[Geo(i).P(j).x',Geo(i).P(j).y'];
%                 temp=temp(~ismember(temp,[0,0],'rows'),:);
%                 Geo(i).P(j).x=temp(:,1)';Geo(i).P(j).y=temp(:,2)';
% %                 tempx=Geo(i).P(j).x;
% %                 tempy=Geo(i).P(j).y;
% %                 [cx,cy]=getcentroid(tempx,tempy);
                [cy,cx]=centroid(Geo(i).P(j));
                nodes=[nodes;cx,cy];           % Centroid Nodes of the overlapping areas
            end                
        end
    end

    [~,Index]=sort(Ind,'descend');
    SortNode=nodes(Index,:);%SortNode=rmmissing(SortNode);
    
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

%     open=SortNode;
%     i=1;
%     while ~isempty(open)                            % Removing the nodes that has distance less than the footprint of the robot
%         Node_cut(i,:)=open(1,:);
%     %     plot(Node_cut(i,2),Node_cut(i,1),'+')
%         open(1,:)=[];
%         D = pdist2(Node_cut(i,:),open);
%         ii=find (D<a);
%         open(ii,:)=[];
%         i=i+1;
%     end
% 
%     % %% Adding the end points to the nodes list
% 
% % %     [mm,~]=size(endP);
% % %     for i=1:mm                                      % Chekcing if the disctance from the endpoints to nodes is grater than the robot footprint
% % %         inn=spdist(endP(i,:),Node_cut)<a/2;
% % %         Node_cut(inn,:)=[];
% % %     end
%     [mm,~]=size(Node_cut);
%     for i=1:mm                                      % Chekcing if the disctance from the endpoints to nodes is grater than the robot footprint
%         inn=spdist(Node_cut(i,:),endP)<a;
%         endP(inn,:)=[];
%     end
%     
%     % plot(Node_cut(:,2),Node_cut(:,1),'+')
% 
% 
%     node=[Node_cut;llink(dsearchn(llink,endP),:)];

else
%     node=llink(dsearchn(llink,endP),:);
    node=endNodes;
end

% node=unique(node,'rows');                       % Removes the duplicate nodes, distint nodes.
% 
% n=size(node,1);

% plot(node(:,2),node(:,1),'pr')

%% Algorithm line 9: Shorten the endpoints (Moved here)
for e=1:size(node,1)
    aRan=polybuffer(node(e,:),'points',a/sqrt(2));%plot(polybuffer(fliplr(node(e,:)),'points',a))
    logi_cell=cell2mat(cellfun(@(x) ~isempty(x),cellfun(@(x) intersect(aRan,rmmissing(x)),crackRaw,'un',0),'un',0));
    if sum(logi_cell)==1
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

v={};vgNE={};vgEE={};
for i = 1: length(S)
%     plot(S(i).P,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3);hold on
    in = inpolygon(node(:,1),node(:,2),S(i).P.Vertices(:,2),S(i).P.Vertices(:,1));  % Finds the vertices inside the polygon
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
%     plot([node(vgEE(i).S(:,1),2) node(vgEE(i).S(:,2),2)]',[node(vgEE(i).S(:,1),1) node(vgEE(i).S(:,2),1)]','*--')
end

% v={};vgedge={};
% for i = 1: length(S)
%     in = inpolygon(node(:,1),node(:,2),S(i).P.x,S(i).P.y);  % Finds the vertices inside the polygon
%     v(i).P = node(in,:);
%     v(i).P = sortrows([v(i).P],2,'ascend');
%     for j = 1:length(v(i).P)-1                          % Finds the edge that intersects with the polygon
%         vgedge(i).S(j,:) = [v(i).P(j,:) v(i).P(j+1,:)];
%     end
% end 

%GPUarray
%vv=bsxfun(@line_of_sightv2,vgNE(i).S,polyshape(S(i).P.x, S(i).P.y))


for i=1:length(S)
    visibility(i).S = line_of_sight2(vgNE(i).S(:,1:2),vgNE(i).S(:,3:4),[S(i).P.Vertices(:,2)'; S(i).P.Vertices(:,1)']);
%     S(i).P.plot;hold on;%plot([node(vgEE(i).S(:,1),2) node(vgEE(i).S(:,2),2)]',[node(vgEE(i).S(:,1),1) node(vgEE(i).S(:,2),1)]','*--');
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
%             poly=polyshape(boundary);
%             in=intersect(poly,[start;goal]);
%             if any(isnan(in(:,1)))
%                 l_n=find(isnan(in(:,1)));
%                 st_p=in(l_n-1,:);
%                 en_p=in(l_n+1,:);
%                 ttt=dsearchn(boundary,[st_p;en_p]);
%                 if (max(ttt)-min(ttt))<=(size(boundary,1)-max(ttt)+min(ttt))         
%                     nBoundary=circshift(boundary,size(boundary,1)-(min(ttt))+1);
%                     t1=nBoundary(1:(max(ttt)-min(ttt)),:);
%                     t2=nBoundary((max(ttt)-min(ttt))+1:end,:);
%                     t3=t2(round(linspace(1,size(t2,1),10)),:);
%                     nBoundary=[t1;t3;t1(1,:)];bt=5;
%                     while(length(regions(polyshape(nBoundary))))>1
%                         t3=t2(round(linspace(1,size(t2,1),10+bt)),:);
%                         nBoundary=[t1;t3;t1(1,:)];
%                         bt=bt+5;
%                     end
%                 else
%                     nBoundary=circshift(boundary,size(boundary,1)-(max(ttt))+1);
%                     t1=nBoundary(1:(size(boundary,1)-max(ttt)+min(ttt)),:);
%                     t2=nBoundary((size(boundary,1)-max(ttt)+min(ttt))+1:end,:);
%                     t3=t2((linspace(1,size(t2,1),10)),:);
%                     nBoundary=[t1;t3;t1(1,:)];
%                     while(length(regions(polyshape(nBoundary))))>1
%                         t3=t2(round(linspace(1,size(t2,1),10+bt)),:);
%                         nBoundary=[t1;t3;t1(1,:)];
%                         bt=bt+5;
%                     end
%                 end
%             end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
%             %%%%%%%%%%%%
%             poly=S(i).P;%polyshape(boundary);
%             in=intersect(poly,fliplr([start;goal]));
%             if any(isnan(in(:,1)))
%                 l_n=find(isnan(in(:,1)));
% 
%                 st_p=fliplr(in(l_n-1,:));
%                 en_p=fliplr(in(l_n+1,:));
%                 ttt=dsearchn(boundary,[st_p;en_p]);
% 
%                 ttt=[1,sort(ttt)',size(boundary,1)];
%                 bt=0;
%                 while true
%                     nBoundary=[];
%                     for m=1:length(ttt)-1
%                         if mod(m,2)
%                             t2=boundary(round(linspace(ttt(m),ttt(m+1),10+bt)),:);
%                         else
%                             t2=boundary(ttt(m):ttt(m+1),:);
%                         end
%                         nBoundary=[nBoundary;t2];
%                     end
% 
%                     if (length(regions(polyshape(nBoundary))))==1
%                         break
%                     else
%                         bt=bt+5;
%                     end
%                 end
%             end
%             %%%%%%%%%%%%
            
            % plot(polyshape(fliplr(nBoundary)))
            waypoints=pathfinder(start,goal,boundary);nn=waypoints(2:end-1,:);
%             plot(waypoints(:,2),waypoints(:,1),'r')
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
%     plot([node(vgEE(i).S(:,1),2) node(vgEE(i).S(:,2),2)]',[node(vgEE(i).S(:,1),1) node(vgEE(i).S(:,2),1)]','*--')  
    if plt
        plot([node(vgEE(i).S(:,1),2) node(vgEE(i).S(:,2),2)]',[node(vgEE(i).S(:,1),1) node(vgEE(i).S(:,2),1)]','-','LineWidth',2)  
    end
end


% node=unique(node,'rows');                       % Removes the duplicate nodes, distint nodes.
% node=sortrows(node,2,'descend');
% n=size(node,1);

% for i=1:n                                       % Plots all the nodes and numbers them
%     plot (node(i,2),node(i,1),'*')
%       text(node(i,2)+5,node(i,1)+5,int2str(i))
% end

% v={};vgedge={};
% for i = 1: length(S)
%     in = inpolygon(node(:,1),node(:,2),S(i).P.x,S(i).P.y);  % Finds the vertices inside the polygon
%     v(i).P = find(in);
%     for j = 1:length(v(i).P)-1                          % Finds the edge that intersects with the polygon
%         vgedge(i).S(j,:) = [v(i).P(j,:) v(i).P(j+1,:)];%[v(i).P(j,:) v(i).P(j+1,:)];
%     end
% end

% v={};vgedge={};
% for i = 1: length(S)
%     in = inpolygon(node(:,1),node(:,2),S(i).P.x,S(i).P.y);  % Finds the vertices inside the polygon
%     v(i).P = [find(in),node(in,:)];
%     v(i).P = sortrows([dsearchn([link(i).x',link(i).y'],v(i).P(:,2:3)),v(i).P],1,'ascend');  % Sorts the nodes along the polygin %
%     v(i).P = v(i).P(:,2);
%     for j = 1:length(v(i).P)-1                          % Finds the edge that intersects with the polygon
%         vgedge(i).S(j,:) = [v(i).P(j,:) v(i).P(j+1,:)];%[v(i).P(j,:) v(i).P(j+1,:)];
%     end
% end

edgeList = [];
for i=1: size(vgEE,2)
    edgeList = [edgeList ;vgEE(i).S];
end
% text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
% plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','*--')

% ind=[];
% for i= 1: size(node,1)
%     for j = 1: size(endP,1)
%        if isequal(node(i,:),endP(j,:))
%           ind = [ind i]; 
%        end
%     end
% end

% for i= 1: size(edgeList,1)                             
%    for j = 1: size(ind,2) 
%       if isequal(edgeList(i,2),ind(j))
%          edgeList(i,:)=[edgeList(i,2),edgeList(i,1)];
%       end
%    end
% end G=graph(edgeList(:,1),edgeList(:,2))

% ind=find(ismember(node,endPoints,'rows'));
% ind2=ismember(edgeList(:,2),ind);
% edgeList(ind2,:)=fliplr(edgeList(ind2,:));              % Reversing the endpoint node edges direction 

%% Shorten the endpoints, Algorithm line 9

% n=length(endPoints(:,1));
% detind=[];
% for i=1:n                                               
%     ind=find(ismember(node,endPoints(i,:),'rows'));
%     if ~isempty(ind)
%         ind2=find(edgeList(:,1)==ind);
%         dir=(node(edgeList(ind2,2),:)-endPoints(i,:))./norm(node(edgeList(ind2,2),:)-endPoints(i,:));
%         if isempty(ind2)
%             ind2=find(edgeList(:,2)==ind);
%             dir=(node(edgeList(ind2,1),:)-endPoints(i,:))./norm(node(edgeList(ind2,1),:)-endPoints(i,:));
%         end
%         if length(ind2)<2
%             node(ind,:)=endPoints(i,:)+a*dir;
%             if sum((node(ind,1) - Node_cut(:,1)).^2+ (node(ind,2) - Node_cut(:,2)).^2<a^2/4)
%                 detind=[detind ind];
%             end
%         end
%         if length(ind2)==2
%             P1=node(edgeList(ind2(1),2),:);
%             P2=node(edgeList(ind2(2),2),:);
%             mid=midP(P1,P2);
%             dir=(mid-endPoints(i,:))./norm(mid-endPoints(i,:));
%             node(ind,:)=endPoints(i,:)+a*dir;
%         end
%     end
% end

% n=length(endPoints(:,1));

% % % % % if any(TF.*~eye(size(TF)),'all')
% % % % %     G=graph(edgeList(:,1),edgeList(:,2));n=find(degree(G)==1);clear G
% % % % %     detind=[];
% % % % %     for i=n'                                               
% % % % %         ind=i;
% % % % %         if ~isempty(ind)
% % % % %             ind2=find(edgeList(:,1)==ind);
% % % % %             dir=(node(edgeList(ind2,2),:)-node(i,:))./norm(node(edgeList(ind2,2),:)-node(i,:));
% % % % %             if isempty(ind2)
% % % % %                 ind2=find(edgeList(:,2)==ind);
% % % % %                 dir=(node(edgeList(ind2,1),:)-node(i,:))./norm(node(edgeList(ind2,1),:)-node(i,:));
% % % % %             end
% % % % %             if length(ind2)<2
% % % % %                 node(ind,:)=node(i,:)+a*dir;
% % % % %                 if sum((node(ind,1) - Node_cut(:,1)).^2+ (node(ind,2) - Node_cut(:,2)).^2<a^2/4)
% % % % %                     detind=[detind ind];
% % % % %                 end
% % % % %             end
% % % % %             if length(ind2)==2
% % % % %                 P1=node(edgeList(ind2(1),2),:);
% % % % %                 P2=node(edgeList(ind2(2),2),:);
% % % % %                 mid=midP(P1,P2);
% % % % %                 dir=(mid-node(i,:))./norm(mid-node(i,:));
% % % % %                 node(ind,:)=node(i,:)+a*dir;
% % % % %             end
% % % % %         end
% % % % %     end
% % % % % end

% figure('position', pos)
% axis([0 colBW 0 rowBW])
% hold on
% set(gca,'Ydir','reverse')
% plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)

% No_edge=length(edgeList);
% for i=1:No_edge
%     plot([node(edgeList(i,1),2),node(edgeList(i,2),2)],[node(edgeList(i,1),1),node(edgeList(i,2),1)],'-')
% end
% 
% for i=1:No_node
%     plot (node(i,2),node(i,1),'*')
%     text(node(i,2)+5,node(i,1)+5,int2str(i))
% end
% text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
% plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','*--')

%% Plot node and edges 
% 
% figure('position', pos),hold on
% plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)
% axis([0 colBW 0 rowBW])
% set(gca,'Ydir','reverse')
% No_node=length(node);
% 
% % No_edge=length(edgeList);
% % for i=1:No_edge
% %     plot([node(edgeList(i,1),2)    node(edgeList(i,2),2)],[node(edgeList(i,1),1)    node(edgeList(i,2),1)],'*--','linewidth',2)
% % end
% % for i=1:No_node
% %     plot (node(i,2),node(i,1),'*')
% %     text(node(i,2)+5,node(i,1)+5,int2str(i))%,'FontSize',20
% % end
% text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
% plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','*--')


%% Get the graph
% 
% edgeListG=[edgeList;edgeList(:,2),edgeList(:,1)];
% 
% adj = full(sparse(edgeListG(:, 1), edgeListG(:, 2), 1, No_node, No_node));
% Dist =squareform( pdist(node));
% AdjMax=adj.*Dist;
% % [Path, weight, add,st]=ChinesePostman(adj,AdjMax,Dist);
% [Path, weight, add,st]=ChinesePostman2(AdjMax,Dist);
% add(ismember(add,st,'rows'),:)=[];
% % n=length(add);
% n=size(add,1);
% % edgeList = [edgeList;add];
% 
% for i=1:n
%     poly = polybuffer(fliplr(node(add(i,:),:)),'line',a);
%     u=union(u,poly);
% end
% 
% figure('position', pos),hold on
% plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)
% % G = digraph(edgeList(:,1),edgeList(:,2));
% % plot(G,'XData', node(:,2), 'YData', node(:,1))
% axis([0 colBW 0 rowBW])
% set(gca,'Ydir','reverse')
% % plot(node(:,2),node(:,1),'*');
% text(node(:,2)+10,node(:,1)+10,int2str([1:length(node)]'));
% plot([node(edgeList(:,1),2) node(edgeList(:,2),2)]',[node(edgeList(:,1),1) node(edgeList(:,2),1)]','*--')
% % for i=1:No_node
% %     plot(node(i,2),node(i,1),'*')
% %     text(node(i,2)+5,node(i,1)+5,int2str(i))%,'FontSize',20
% % end
% % No_edge=length(edgeList);
% % for i=1:No_edge/2
% %     plot([node(edgeList(i,1),2)    node(edgeList(i,2),2)],[node(edgeList(i,1),1)    node(edgeList(i,2),1)],'*--')
% % end

%%
% find intersecting path with Union areas
% use visibility graph find min path inside the areas
% for i=1:n
% start_point=node(1,:);%(Path(14),:);
% end_point=node(10,:);%(Path(14+1),:);
% waypoint_coordinates1 = pathfinder(start_point, end_point, boundary);
% % end
% start_point=node(Path(16),:);
% end_point=node(Path(16+1),:);
% waypoint_coordinates2 = pathfinder(start_point, end_point, [S(12).P.x;S(12).P.y]);

%% Ploting the Path


% figure('position', pos),hold on
% plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)
% axis([0 colBW 0 rowBW])
% set(gca,'Ydir','reverse')
% 
% % waypoint=[];
% if Path(end)==Path(1);Path(end)=[];end
% waypoint=node(Path,:);
% waypoint_coords=[waypoint];%;waypoint_coordinates1(2:end,:)];
% n = length(waypoint_coords);
% 
% edgeList=[];
% for i=1:n-1
%     plot([waypoint_coords(i,2) waypoint_coords(i+1,2)],[waypoint_coords(i,1) waypoint_coords(i+1,1)],'*-','linewidth',3)
%     edgeList=[edgeList;Path(i),Path(i+1)];
% end
% 
% text(waypoint_coords(:,2)+10,waypoint_coords(:,1)+10,int2str([1:length(waypoint_coords)]'));
% box on
% 
% fprintf('Node List = \n'),disp(node)
% fprintf('Waypoints = \n'),disp(waypoint_coords)

%% Ploting the diGraph

% figure('position', pos),hold on
% plot(u,'FaceColor',[0.7, 0.1, 0.1],'FaceAlpha',0.3)
% axis([0 colBW 0 rowBW])
% set(gca,'Ydir','reverse')
% 
% G = graph(edgeList(:,1),edgeList(:,2));
% p=plot(G,'XData', node(:,2), 'YData', node(:,1));
% highlight(p,[Path(1)],'NodeColor','r'),
% highlight(p,[Path(end)],'NodeColor','g')
% highlight(p,Path,'EdgeColor',[0.8500 0.3250 0.0980],'LineWidth',1.5)

%% Waypoint Edge Minkowsky Sum 

% totCrack=[];
% for i=1:length(crackRaw)
%     totCrack = [totCrack' crackRaw{1,i}']';
% end
% 
% figure('position', pos),hold on
% axis([0 colBW 0 rowBW])
% set(gca,'Ydir','reverse')
% 
% for i=1:n-1
%     plot([waypoint_coords(i,2) waypoint_coords(i+1,2)],[waypoint_coords(i,1) waypoint_coords(i+1,1)],'g--','linewidth',1.5)
% end
% plot(node(:,2),node(:,1),'rp','MarkerSize',10,'MarkerFaceColor','r')
% 
% % for i=1:n
% pathMina = polybuffer([waypoint_coords(:,2),waypoint_coords(:,1)],'line',a);
% pathMins = polybuffer([waypoint_coords(:,2),waypoint_coords(:,1)],'line',s);
% %     [ux,uy] = polybool('union',ux,uy,x,y);
% % end
% 
% plot(pathMina,'FaceColor','y','FaceAlpha',0.3,'LineStyle','none')
% plot(pathMins,'FaceColor','y','FaceAlpha',0.3,'LineStyle','none')
% text(waypoint_coords(:,2)+10,waypoint_coords(:,1)+10,int2str([1:length(waypoint_coords)]'));
% box on

% save knownCrackG.mat node edgeList
% save knownCrackG_Vishnu.mat node edgeList

% save(['CrackGT/knownCrackG_' img_n '.mat'], 'node', 'edgeList')
ttt=toc(tfun);
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