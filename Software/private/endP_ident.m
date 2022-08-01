%% endP_ident to iendtify the endpoints in a Binary Image
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [eP,rP,cP] = endP_ident(BW3,BW,org)
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

    % Version2

    if sum(sum(BW3))>5
        
        spdist = @(P,Ps) sqrt((P(1,1)-Ps(:,1)).^2 + (P(1,2)-Ps(:,2)).^2);
%         if ~isempty(org)
%             if any(org==0)
%                 if org(1)==0 && org(2)==0
%                     BW3=padarray(BW3,[0,1],'post');
%                     BW3=padarray(BW3,[1,0],'post');
%                 elseif org(1)==0 && org(2)~=0
%                     BW3=padarray(BW3,[0,1],'post');
%                     BW3=padarray(BW3,[1,0]);
%                 elseif org(2)==0 && org(1)~=0
%                     BW3=padarray(BW3,[0,1]);
%                     BW3=padarray(BW3,[1,0],'post');
%                 end
%             else
%                 BW3 = padarray(BW3,[1 1]);        % Adds a boundary around the map
%             end
%         else
%             BW3 = padarray(BW3,[1 1]);
%         end
        if ~isempty(BW)
            BW3=padarray(BW3,[0,1],'post');
            BW3=padarray(BW3,[1,0],'post');
%         else
%             BW3 = padarray(BW3,[1 1]);
        end

        % figure, imshow(~BW3); hold on
        % [rowBW, colBW]= size(BW3);
        
%         BW3 = bwmorph(BW3,'thin', inf);
        
% % % % % % % % %         endPoints = bwmorph(BW3, 'endpoints');  % Finds end points of skeleton
% % % % % % % % %         [endP_row, endP_col]=find(endPoints);
% % % % % % % % % %         intPoints = bwmorph(BW3, 'branchpoints');
% % % % % % % % %         endP=[endP_row endP_col];
        
        [~,endP]=bwmorph_v2(BW3,1);  %1: End Points

        %%%%%%%% Block commented after cleaning intersection points
% % % %         clear endP
% % % %         endPoints(intPoints)=1;
% % % %         [endP(:,1),endP(:,2)]=find(endPoints);
% % % %         n=1;
% % % %         while n<=length(endP)
% % % %             if sum(spdist(endP(n,:),endP(1:end ~= n,:))<5)>0
% % % %                 endP(n,:)=[];
% % % %             end
% % % %             n=n+1;
% % % %         end

        endPoints = endP;post=[]; eP=endP;
        
        %[endP_row,endP_col];
%         pre=[];
%         for i = 1:length(endPoints(:,1))
%             eP=endPoints(i,:);
%             row=eP(1);
%             col=eP(2);
%             I = BW3;
%             b = [I(row-1,col-1:col+1) ...
%                     I(row, col-1) I(row, col+1)...
%                     I(row+1,col-1:col+1)];
%             pre = [pre;b];
%         end

        %%%%
        %%%%
        I=BW;I_te=I;
        while true
% % % % % % % % %             intPoints = bwmorph(I, 'branchpoints');
% % % % % % % % %             I_te(intPoints)=0;
% % % % % % % % %             I_te=bwmorph(I_te,'clean');
            intPoints=bwmorph_v2(I,2); %2: Branchpoints
            I_te(intPoints>0)=0;
            pixelP=bwmorph_v2(I_te,0); %0: Single Pixels
            I_te(pixelP>0)=0;
            
            if ~isequal(I,I_te) 
               I=I_te;
            else
               break;
            end            
        end
        BW=I;
        %%%%
        %%%%


        if ~isempty(BW)
            I = BW;
            for i = 1:length(endPoints(:,1))
                ePp=endPoints(i,:);
                row=ePp(1);
                col=ePp(2);
                b = [I(row-1,col-1:col+1) ...
                        I(row, col-1) I(row, col+1)...
                        I(row+1,col-1:col+1)];
                post = [post;b];
            end

            post=sum(post,2);
            realEndP = endPoints(post~=2,:);rP=realEndP;%post==1|post>2
            contEndP = endPoints(post==2,:);cP=contEndP;
        else
            rP=[];
            cP=[];
        end
        
        
        %%% New TEST funtion to increase the efficiency of the image processing
% %         I=BW3;I_te=I;
% %         while true
% %             intPoints = bwmorph(I, 'branchpoints');
% %             I_te(intPoints)=0;
% %             I_te=bwmorph(I_te,'clean');
% %             if ~isequal(I,I_te) 
% %                I=I_te;
% %             else
% %                break;
% %             end            
% %         end
% %         endPoints = bwmorph(I, 'endpoints');  % Finds end points of skeleton
% %         [endP_row, endP_col]=find(endPoints);
% %         endP=[endP_row endP_col];
% %         %%%
        
% %         rPt=[];
% %         for r = 1:size(rP,1)
% %             rPt=[rPt;endP(spdist(rP(r,:),endP)<5,:)];
% %         end
% %         rP=unique(rPt,'rows');
% %         eP=endP;
        % realEndP = endPoints(~any(ismember(endPoints,[2 colBW-1 rowBW-1])')',:);rP=realEndP;% plot(realEndP(:,2),realEndP(:,1),'pr');
        % contEndP = endPoints(any(ismember(endPoints,[2 colBW-1 rowBW-1])')',:);cP=contEndP;% plot(contEndP(:,2),contEndP(:,1),'pg');
        % plot(endP(:,2),endP(:,1),'pg');%hold off
        %%%%%%%%

    else
        eP=[];rP=[];cP=[];
    end