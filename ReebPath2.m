%% Reeb Path Planner Master 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [Path,wall_fol,adj]=ReebPath2(adj,critP,reebEdge,cells,Start)
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

    Path=[];wall_fol=[];global spdist
    
%     if length(conncomp(graph(reebEdge(:,1),reebEdge(:,2)), 'OutputForm', 'cell'))>1
%         Dist =squareform(pdist(critP));
%         AdjMax=adj.*Dist;G=graph(AdjMax);
%         combEdge = combnk(unique(G.Edges.EndNodes),2);
%         combEdge(ismember(combEdge,G.Edges.EndNodes,'rows'),:)=[];
%         EdgeTable=table(combEdge,diag(Dist(combEdge(:,1),combEdge(:,2))),'VariableNames',{'EndNodes','Weight'});
%         G_a=addedge(G,EdgeTable);G_a=simplify(G_a);
%               
%         T = minspantree(G_a);idx=find(ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'));T=rmedge(T,idx);
%         idx=find(max(T.Edges.Weight(~ismember(T.Edges.EndNodes,G.Edges.EndNodes,'rows'),:))==T.Edges.Weight);T=rmedge(T,idx);
%         reebEdge=[reebEdge;T.Edges.EndNodes];G=graph(reebEdge(:,1),reebEdge(:,2));adj=full(adjacency(G));
%     end
    
    [tPath,twall_fol,adj]=ReebPath(adj,critP,reebEdge,cells,Start);
    Path=[Path;tPath];
    wall_fol = [wall_fol;twall_fol];wall_fol(end)=0;
    
    while sum(ismember(reebEdge,Path(:,1:2),'rows')|ismember(fliplr(reebEdge),Path(:,1:2),'rows'))~=size(reebEdge,1)
    
        Start = Path(end,2);
        t=find(~ismember(1:length(critP),reshape(Path(:,1:2),1,[])));
        
        [~,ind]=min(spdist(critP(Start,:),critP(t,:)));
        Path=[Path;[Start,t(ind),0]];
        wall_fol = [wall_fol;0];
        Start=t(ind);
        
        [tPath,twall_fol,adj]=ReebPath(adj,critP,reebEdge,cells,Start);            % Function Loops to find the path 
        Path=[Path;tPath];
        wall_fol = [wall_fol;twall_fol];wall_fol(end)=0;

    end
end
