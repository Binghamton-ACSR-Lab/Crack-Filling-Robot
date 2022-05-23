%% Line of Sight Function 2 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [visibility,in,out] = line_of_sight2(observer_node, target_node, external_boundaries)
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

%Functions determines whether the current target node is visible from the
%current observer position
on = observer_node;
tn = target_node;
ed = polyshape(external_boundaries(1,:),external_boundaries(2,:));

% text(observer_node(:,2)+10,observer_node(:,1)+10,int2str([1:length(observer_node)]'));

visibility = ones(size(on,1),1);
for i = 1: size(on,1)
        [in,out]=intersect(ed,[on(i,:);tn(i,:)]);
%         plot(in(:,2),in(:,1),'b',out(:,2),out(:,1),'r')
        if ~isempty(out)
            visibility(i)=0;    % The Target node is not visile from the observing node
        end  
end

end










