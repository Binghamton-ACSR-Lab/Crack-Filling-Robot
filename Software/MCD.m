%% Morse Decomposition Funcion 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [critPT,polyout_work,polyout,splitEdge]=MCD(polyin_buffed,polyin_work,polyin,nodeend,flag)
%
% Computes a Morse Decomposition of the workspace. 
%
% INPUTS:
%   polyin_buffed = Polygon in the shape of the workspace with a minkowski sum of 'a'.  
%   polyin_work = Polygon in the shape of the workspace, working. 
%   polyin = Polygon in the shape of the workspace.
%   nodeend = Current Position of the robot. 
%   flag = Flag to smooth the edges of the imput polygon. 
%
% OUTPUTS:
%   critPT = Critcal points in the workspace. 
%   polyout_work = Decomposed cells in the workspace. 
%   polyout = Decomposed cells in the workspace.
%   splitEdge = Splitline at the critial points. 

    if length(polyin_buffed)==1; polyin_buffed=regions(polyin_buffed);end
    if length(polyin_buffed)>1;polyin_work=regions(polyin_work);end
    critPT =[];splitEdge=[];
    for p = 1:length(polyin_buffed)
        style='r*';nodeend=fliplr(nodeend);t=0.5;
        boundary=rmholes(polyin_buffed(p));
        spdist = @(P,Ps) sqrt((P(1,1)-Ps(:,1)).^2 + (P(1,2)-Ps(:,2)).^2);
        critP =[];cvex=[];
        % Smooth Polyshape Edge
        newBoundary = [];
       
        boundary = refinePoly(boundary,2,flag);
        [newBoundary(:,1),newBoundary(:,2)] = boundary.boundary;

        % Forward boundary scan
        TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');sN=find(TF(:,1));
        while isempty(sN)
            clear newBoundary
            boundary = refinePoly(boundary,1,flag);[newBoundary(:,1),newBoundary(:,2)] = boundary.boundary;
            TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');sN=find(TF(:,1));
        end
        sN=sN(1);newBoundary=circshift(newBoundary,sN);
        TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');
        TF([find(isnan(newBoundary(:,1)))+1 ; find(isnan(newBoundary(:,1)))-1],:)=0;
        subcritP=newBoundary(TF(:,1),:);inPoly=polyshape(newBoundary);%inPoly.plot
        
        if isempty(subcritP)
            critP =[];cvex=[];
            % Smooth Polyshape Edge
            newBoundary = [];
            boundary=rmholes(polyin_buffed(p));
            boundary = refinePoly(boundary,2,~flag);
            [newBoundary(:,1),newBoundary(:,2)] = boundary.boundary;

            % Forward boundary scan
            TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');sN=find(TF(:,1));
            while isempty(sN)
                clear newBoundary
                boundary = refinePoly(boundary,1,~flag);[newBoundary(:,1),newBoundary(:,2)] = boundary.boundary;
                TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');sN=find(TF(:,1));
            end
            sN=sN(1);newBoundary=circshift(newBoundary,sN);
            TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');
            TF([find(isnan(newBoundary(:,1)))+1 ; find(isnan(newBoundary(:,1)))-1],:)=0;
            subcritP=newBoundary(TF(:,1),:);inPoly=polyshape(newBoundary);
        end

        for j=find(TF(:,1))'
            in1 = isinterior(inPoly,[newBoundary(j,1)-5 newBoundary(j,2)]); 
            in2 = isinterior(inPoly,[newBoundary(j,1)+5 newBoundary(j,2)]);
            if in1&&~in2; cvex=[cvex;0];elseif ~in1&&in2 ; cvex=[cvex;1];else; cvex=[cvex;1]; end
        end
        critP = [critP;subcritP];%cave=[cave;ones(length(subcritP(:,1)),1)];
        % plot(critP(:,1),critP(:,2),'r*');plot(critP(find(cvex),1),critP(find(cvex),2),'k*')

        % Holes boundary scan
        obj = holes(polyin_buffed(p));
        obj = polybuffer(obj, 10);%obj.plot
        obj = polybuffer(obj,-10,'JointType','miter','MiterLimit',4);%obj.plot

        poly=obj;

        for i= 1: length(poly)
            [values(:,1),values(:,2)] =poly(i).boundary;%spcrv(poly(i).Vertices',3);values=values';
%             bb2=convhull(values);
            TF=islocalmin(values(:,1));TF([find(isnan(values(:,1)))+1 ; find(isnan(values(:,1)))-1],:)=0;
            subcritP=values(TF(:,1),:);
%             plot(subcritP(:,1),subcritP(:,2),style)
            critP = [critP;subcritP];
%             for j = find(TF(:,1))'
%                 v1=values(mod(j+4-1,length(TF(:,1)))+1,:)-values(j,:);%vectarrow(values(j,:),values(j+4,:))
%                 v2=values(mod(j-4-1,length(TF(:,1)))+1,:)-values(j,:);%vectarrow(values(j,:),values(j-4,:))
%                 if ab2v(v1,v2)<180
%                     cvex=[cvex;1];
%                 else
%                     cvex=[cvex;0];
%                 end
%             end
            for j=find(TF(:,1))'                            % Checking convex or concave
%                 [in,out] = intersect(poly,[values(j,1) values(j,2)+5;values(j,1) values(j,2)-5]);%inter=fliplr(inter);
%                 if isempty(in)||sum(~ismember(in(:,2),out(:,2)))==0; cvex=[cvex;1];else; cvex=[cvex;0]; end
                in1 = ~isinterior(poly(i),[values(j,1)-5 values(j,2)]); 
                in2 = ~isinterior(poly(i),[values(j,1)+5 values(j,2)]);
                if in1&&~in2; cvex=[cvex;0];elseif ~in1&&in2 ; cvex=[cvex;1];else; cvex=[cvex;1]; end
%                 if isempty(out); cvex=[cvex;0];end
            end
%             cvex=[cvex;ismember(find(TF(:,1)),bb2)];
            TF=islocalmax(values);TF([find(isnan(values(:,1)))+1 ; find(isnan(values(:,1)))-1],:)=0;
            subcritP=values(TF(:,1),:);
%             plot(subcritP(:,1),subcritP(:,2),style)
            critP = [critP;subcritP];
%             for j = find(TF(:,1))'
%                 v1=values(mod(j+4-1,length(TF(:,1)))+1,:)-values(j,:);%vectarrow(values(j,:),values(j+4,:))
%                 v2=values(mod(j-4-1,length(TF(:,1)))+1,:)-values(j,:);%vectarrow(values(j,:),values(j-4,:))
%                 if ab2v(v1,v2)<180
%                     cvex=[cvex;1];
%                 else
%                     cvex=[cvex;0];
%                 end
%             end
            for j=find(TF(:,1))'
%                 [in,out] = intersect(poly,[values(j,1) values(j,2)+5;values(j,1) values(j,2)-5]);%inter=fliplr(inter);
%                 if isempty(in)||sum(~ismember(in(:,2),out(:,2)))==0; cvex=[cvex;1];else; cvex=[cvex;0]; end
                in1 = ~isinterior(poly(i),[values(j,1)-5 values(j,2)]); 
                in2 = ~isinterior(poly(i),[values(j,1)+5 values(j,2)]);
                if in1&&~in2; cvex=[cvex;1];elseif ~in1&&in2 ; cvex=[cvex;0];else; cvex=[cvex;1]; end
%                 if isempty(out); cvex=[cvex;0];end
            end
%             cvex=[cvex;ismember(find(TF(:,1)),bb2)];
            clear values
        end

        % Backward boundary scan
        TF=islocalmax(round(newBoundary,5),'FlatSelection', 'center');sN=find(TF(:,1));
        if ~(any(TF(:,1)))
           newBoundary=circshift(newBoundary,10);TF=islocalmax(round(newBoundary,5),'FlatSelection', 'center');
        end
% %         while isempty(sN)
% %             clear newBoundary
% %             boundary = refinePoly(boundary,1,flag);[newBoundary(:,1),newBoundary(:,2)] = boundary.boundary;
% %             TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');sN=find(TF(:,1));
% %         end
% %         sN=sN(1);newBoundary=circshift(newBoundary,sN);
        %TF=islocalmin(round(newBoundary,5),'FlatSelection', 'center');
        TF([find(isnan(newBoundary(:,1)))+1 ; find(isnan(newBoundary(:,1)))-1],:)=0;
        subcritP=newBoundary(TF(:,1),:);
%         plot(newBoundary(TF(:,1),1),newBoundary(TF(:,1),2),style)
        critP = [critP;subcritP];%cave=[cave;ones(length(subcritP(:,1)),1)];
% 
%         for j=find(TF(:,1))'
%             [in,out] = intersect(inPoly,[newBoundary(j,1) newBoundary(mod(j+4-1,length(TF(:,1)))+1,2);newBoundary(j,1) newBoundary(mod(j-4-1,length(TF(:,1)))+1,2)]);%inter=fliplr(inter);
%             if isempty(in); cvex=[cvex;1];end
%             if isempty(out); cvex=[cvex;0];end
%         end

        for j=find(TF(:,1))'
%             [in,out] = intersect(inPoly,[newBoundary(j,1) newBoundary(j,2)+5;newBoundary(j,1) newBoundary(j,2)-5]);%inter=fliplr(inter);
%             if isempty(in)||sum(~ismember(in(:,2),out(:,2)))==0; cvex=[cvex;1];else; cvex=[cvex;0]; end
            in1 = isinterior(inPoly,[newBoundary(j,1)-5 newBoundary(j,2)]); 
            in2 = isinterior(inPoly,[newBoundary(j,1)+5 newBoundary(j,2)]);
            if in1&&~in2; cvex=[cvex;1];elseif ~in1&&in2 ; cvex=[cvex;0];else; cvex=[cvex;1]; end
%             if isempty(out); cvex=[cvex;0];end
        end
        
%         cvex=[cvex;ismember(find(TF(:,1)),bb1)];
        % plot(critP(:,1),critP(:,2),'r*');plot(critP(find(cvex),1),critP(find(cvex),2),'k*')
        
%         plot(critP(:,1),critP(:,2),style)
%         plmcdcrtt=text(critP(:,1)+10,critP(:,2)+10,[repmat('C',length(critP),1) int2str([1:length(critP)]')]);

        % Decomposing the cells
        ymin = round(min(polyin_work(p).Vertices(:,2)));ymax=round(max(polyin_work(p).Vertices(:,2)));  %%%polyin_buffed
        for c = 1:length(critP(:,1))
                bool=false;boo11=false;boo12=false;
                [in,~] = intersect(polyin_work(p),[critP(c,1),ymin;critP(c,1),ymax]);%in=fliplr(in);out=fliplr(out);    %%%polyin_buffed
                if ~isempty(in); if ymax-in(end,2)<50; in(end,2)=ymax;end; end                              % Keep in mind
                if ~isempty(in); if in(1,2)-ymin<50; in(1,2)=ymin;end; end                                  % Keep in mind
                if cvex(c)==0
%                     if sum(isnan(out(:,1)))==0 
                    if sum(isnan(in(:,1)))==0 
                        splitEdge = [splitEdge ; polybuffer([in(1,:);in(end,:)],'lines',t)];%cave(c)=0;
                    else
%                         oo = rmmissing(out);
%                         for k = 1:sum(isnan(out(:,1)))+1
%                             if oo(2*k-1,2)<critP(c,2) && oo(2*k,2)>critP(c,2)
%                                 bool=true;
%                                 break;
%                             end
%                         end
%                         if bool 
%                             splitEdge = [splitEdge ; polybuffer([in((find(oo(2*k-1,2)==in(:,2))+1)-2,:);in((find(oo(2*k-1,2)==in(:,2))+1)+2,:)],'lines',t)];
%                             %cave(c)=0;
%                         end

                        %%% New : for prominent identification closer to
                        %%% NaN
                        in(find(isnan(in(:,2)))-1,2)=in(find(isnan(in(:,2)))-1,2)-1;
                        in(find(isnan(in(:,2)))+1,2)=in(find(isnan(in(:,2)))+1,2)+1;
                        %%%
                        
                        oo = rmmissing(in);
                        for k = 1:sum(isnan(in(:,1)))+1
                            if oo(2*k-1,2)<critP(c,2) && oo(2*k,2)>critP(c,2)
                                bool=true;%ii=[oo(2*k-1,:);oo(2*k,:)];
                                boo11=true;
                                break;
                            end
                        end
                        if ~bool
%                            [~,kk]=dsearchn(critP(c,2),oo(:,2));%kk=kk<20;kk=find(kk);
%                            kkk=sort(kk);kk=ismember(kk,kkk(1:2));kk=find(kk);
                           kk=find(diff((oo(:,2)-critP(c,2))>=0),1);kk=[kk;kk+1];           %%New find crit point locaton on in
                           bool=true;
                           boo12=true;
                        end
                        if bool 
                            % splitEdge = [splitEdge ; polybuffer([in((find(oo(2*k-1,2)==in(:,2))+1)-2,:);in((find(oo(2*k-1,2)==in(:,2))+1)+2,:)]+[0,-2;0,2],'lines',t)];
                            if boo11; splitEdge = [splitEdge ; polybuffer([oo(2*k-1,:);oo(2*k,:)]+[0,-10;0,10],'lines',t)];end
                            if boo12; splitEdge = [splitEdge ; polybuffer([oo(kk(1)-1,:);oo(kk(2)+1,:)]+[0,-10;0,10],'lines',t)];end
                            % splitEdge = [splitEdge ; polybuffer(ii,'lines',t)];
                        end
                    end
                end
        end

        distt=[];
        for ii = 1:size(critP,1)
           distt = [distt,spdist(critP(ii,:),critP)];
        end
        distt(distt==0)=inf;
        if min(distt(:))<=25
            A=min(distt(:))==distt;A(find(cvex),:)=[];
            for ii=1:length(A(:,1))
               critP(A(ii,:),:)=[]; 
            end
        end
        critPT=[critPT;critP];
    end
    
    if ~isempty(splitEdge)
        for i=1:length(splitEdge)
            polyin_work=subtract(polyin_work,splitEdge(i));%splitEdge.plot          % Splits the Poly region into different cells, Decompsition 
            polyin=subtract(polyin,splitEdge(i));        
        end
    end
    polyin_work = sortregions(polyin_work,'centroid','ascend');
    polyin = sortregions(polyin,'centroid','ascend');

    if length(polyin_work)>1; polyin_work=regJoin(polyin_work);end
    polyout_work=regions(polyin_work);
    polyout=regions(polyin);

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