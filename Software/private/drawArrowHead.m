%% Modified drawArrow Function 
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function hArrow = drawArrowHead(p0,p1,color)
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

if nargin == 2
   color = 'k'; 
end
% Parameters:
W1 = 0.08;   % half width of the arrow head, normalized by length of arrow
W2 = 0.014;  % half width of the arrow shaft
L1 = 0.18;   % Length of the arrow head, normalized by length of arrow
L2 = 0.13;  % Length of the arrow inset
% Unpack the tail and tip of the arrow
x0 = p0(1);
y0 = p0(2);
x1 = p1(1);
y1 = p1(2);
% Start by drawing an arrow from 0 to 1 on the x-axis
% P = [...
%     0, (1-L2), (1-L1), 1, (1-L1), (1-L2), 0;
%     W2,    W2,     W1, 0,    -W1,    -W2, -W2];
P = [...
    (L1-L2), (L1-L1), L1, (L1-L1), (L1-L2);
    W2,     W1, 0,    -W1,    -W2];
P(1,:)=P(1,:)-L1;
% Scale,rotate, shift and plot:
dx = x1-x0;
dy = y1-y0;
Length = sqrt(dx*dx + dy*dy);
Angle = atan2(-dy,dx);
P = 342*P;   %Scale
P = [cos(Angle), sin(Angle); -sin(Angle), cos(Angle)]*P;  %Rotate
P = p1(:)*ones(1,5) + P;  %Shift
% Plot!
hArrow = patch('Faces',1:5,'Vertices',P','FaceColor',color,'EdgeColor',color);  axis equal;
% hArrow.EdgeColor = color;
end