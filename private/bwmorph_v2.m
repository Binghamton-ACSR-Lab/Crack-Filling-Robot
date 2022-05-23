%% Modified bwmorph function 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function [BW_out,Points] = bwmorph_v2(BW,operation)
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

% Operation:
% 0: Single pixels 
% 1: End Points 
% 2: Intersection Points 

[ll,kk]=find(BW);
if operation < 2
    test=find(sum(cell2mat(arrayfun(@(row,col) [BW(row-1,col-1:col+1) ...
                    BW(row, col-1) BW(row, col+1)...
                    BW(row+1,col-1:col+1)],ll,kk,'UniformOutput',false)),2)==operation);
else
    test=find(sum(cell2mat(arrayfun(@(row,col) [BW(row-1,col-1:col+1) ...
                    BW(row, col-1) BW(row, col+1)...
                    BW(row+1,col-1:col+1)],ll,kk,'UniformOutput',false)),2)>operation);
end

Points=[ll(test) kk(test)];

BW_out=full(sparse([ll(test);size(BW,1)],[kk(test);size(BW,2)],1));BW_out(end,end)=0;
