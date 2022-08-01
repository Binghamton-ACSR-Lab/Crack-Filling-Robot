%% Refines a polygon
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% October 2019, Last Revision: 20-October-2019

function polyout = refinePoly(polyin,times,smoothflag)
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

    switch nargin
        case 2
            smoothflag=false;
    end
    polyout=polyshape();
    
    for n = 1:times
        for p = 1:length(polyin)
            boundary=rmholes(polyin(p));
            jj=regions(boundary);
            newBoundary = [];

            for i = 1:length(jj)
                if ~isempty(regions(jj(i)))
                    if smoothflag
                        jj(i)=polybuffer(polybuffer(jj(i),-2),2);
                        jj(i) = rmslivers(jj(i),0.00001);
                        x=jj(i).Vertices(:,1);
                        y=jj(i).Vertices(:,2);
                        smoothX = smooth(x);smoothX = smooth(smoothX);% added span %known ,'sgolay',3
                        smoothY = smooth(y);smoothY = smooth(smoothY);% added span %known ,'sgolay',3
                        jj(i) =polyshape([smoothX,smoothY]);%boundary.plot
                        if length(regions(jj(i)))>1
                            jj(i)=polyshape([x,y]);
                        end
                    end
                    [Bound(:,1),Bound(:,2)] =jj(i).boundary;Bound(end,:)=[];
                    [values(:,1),values(:,2)] =jj(i).boundary;values(end,:)=[];
                    ii=0;
                    for j = 1:length(values(:,1))
                            P1=round(values(j,:),5);
                            P2=round(values(mod(j,length(values(:,1)))+1,:),5);
                            mid=(P1(:)+ P2(:)).'/2;
                            Bound=[Bound(1:j+ii,:) ;mid; Bound(1+j+ii:end,:)];
                            ii=ii+1;
                    end

                    if i==1
                        newBoundary = [newBoundary; Bound(end,:);Bound];clear values Bound;%
                        newBoundary=circshift(newBoundary,fix(size(newBoundary,1)/10));
                    else
                        newBoundary = [newBoundary; NaN,NaN;Bound(end,:);Bound];clear values Bound;%
                        newBoundary=circshift(newBoundary,fix(size(newBoundary,1)/10));
                    end
                end
            end

            ext = polyshape(newBoundary,'Simplify',false);

            jj=holes(polyin(p));
            newBoundary = [];

            for i = 1:length(jj)
                if smoothflag
                    jj(i)=polybuffer(polybuffer(jj(i),-1),1);
                    jj(i) = rmslivers(jj(i),0.00001);
                    x=jj(i).Vertices(:,1);
                    y=jj(i).Vertices(:,2);
                    smoothX = smooth(x);smoothX = smooth(smoothX);% known ,'sgolay',3
                    smoothY = smooth(y);smoothY = smooth(smoothY);% known ,'sgolay',3
                    jj(i) =polyshape([smoothX,smoothY]);%boundary.plot
                end
                [Bound(:,1),Bound(:,2)] =jj(i).boundary;Bound(end,:)=[];
                [values(:,1),values(:,2)] =jj(i).boundary;values(end,:)=[];
                ii=0;
                for j = 1:length(values(:,1))
                        P1=round(values(j,:),5);
                        P2=round(values(mod(j,length(values(:,1)))+1,:),5);
                        mid=(P1(:)+ P2(:)).'/2;
                        Bound=[Bound(1:j+ii,:) ;mid; Bound(1+j+ii:end,:)];
                        ii=ii+1;
                end

                if i==1
                    newBoundary = [newBoundary; Bound(end,:);Bound];clear values Bound;%
                    newBoundary=circshift(newBoundary,fix(size(newBoundary,1)/10));
                else
                    newBoundary = [newBoundary; nan,nan;Bound(end,:);Bound];clear values Bound;%
                    newBoundary=circshift(newBoundary,fix(size(newBoundary,1)/10));
                end
            end

            if ~isempty(newBoundary); hol = polyshape(newBoundary,'Simplify',false);else;hol = polyshape(); end

            subpolyout = subtract(ext,hol,'KeepCollinearPoints',true);

            if length(polyout)>1
                polyout=[polyout;subpolyout];
            end
        end

        if length(polyin)==1
            polyout=subpolyout;
        end
        polyin=polyout;
    end
end