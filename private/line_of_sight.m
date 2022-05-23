%% Line of Sight Function
% Author: Vishnu Veeraraghavan,
% Automated Control Systems and Robotics Lab.
% Email: vveerar1@binghamton.edu.
% July 2019, Last Revision: 25-Sep-2019

function visibility = line_of_sight(observer_state, current_target_node, external_boundaries)
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
%global external_boundaries
%Dissect observer state into coordinates
x_observer = observer_state(:,1);
y_observer = observer_state(:,2);

%Dissect current target node ccordinates
x_current_target = current_target_node(:,1);
y_current_target = current_target_node(:,2);

% external_boundaries=gpuArray(external_boundaries);
beam_direction_vector = [(x_current_target - x_observer) , (y_current_target - y_observer)];
intersection_check = zeros(size(x_current_target,1),1);

%Create empty distance array and initialize counter
distance_array = zeros(size(observer_state,1),size(external_boundaries,1));
i = 0;

x_1 = external_boundaries(:,1);
y_1 = external_boundaries(:,2);
x_2 = [external_boundaries(2:end,1);external_boundaries(1,1)];
y_2 = [external_boundaries(2:end,2);external_boundaries(1,2)];
f=@() deal(x_observer,y_observer,x_current_target,y_current_target,beam_direction_vector);K=repmat({f},length(x_1),1);

tt = arrayfun(@loopFun,x_1,y_1,x_2,y_2,K,'UniformOutput',false);
distance_array=cell2mat(tt');


%Get the minimum value only from the first i entries; the other entries may be zero
%or residual from previous iterations
distance_array(distance_array==0)=Inf;
p_min = min(distance_array,[],2);

visibility = zeros(size(distance_array,1),1);
x_midpoint = zeros(size(distance_array,1),1);
y_midpoint = zeros(size(distance_array,1),1);

   
%Establish whether the path from the current to the target node passes beyond the boundaries
ind = p_min >= 1;
%Calculate the midpoint between the current and target nodes
x_midpoint(ind) = 0.5.*(x_observer(ind) + x_current_target(ind));
y_midpoint(ind) = 0.5.*(y_observer(ind) + y_current_target(ind));

% x_midpoint=gpuArray(x_midpoint);y_midpoint=gpuArray(y_midpoint);
%Determine if the midpoint is within the boundaries
% [IN] = inpolygon_for_gpu(x_midpoint,y_midpoint,external_boundaries(:,1),external_boundaries(:,2));
[IN,ON] = inpolygon(x_midpoint,y_midpoint,external_boundaries(:,1),external_boundaries(:,2));

% [IN] = isinterior(polyshape(external_boundaries),x_midpoint,y_midpoint);
%If the path between the current and target nodes lie within the boundaries

visibility(IN|ON) = 1; %Robot has a direct line of sight to the target node

end

function p=loopFun(x_1,y_1,x_2,y_2,f)

%     if k < size(external_boundaries,1)
%         
%         %Assign two adjacent points as wall ends (x_1;y_1) and (x_2;y_2)
%         
%         point_1 = [external_boundaries(k,1);external_boundaries(k,2)];
%         x_1 = point_1(1);
%         y_1 = point_1(2);
%         
%         point_2 = [external_boundaries(k+1,1);external_boundaries(k+1,2)];
%         x_2 = point_2(1);
%         y_2 = point_2(2);
%         
%     elseif k == size(external_boundaries,1)
%         
%         %Assign the last point as (x_1;y_1) and the first point in the list as (x_2;y_2)
%         
%         point_1 = [external_boundaries(k,1);external_boundaries(k,2)];
%         x_1 = point_1(1);
%         y_1 = point_1(2);
%         
%         point_2 = [external_boundaries(1,1);external_boundaries(1,2)];
%         x_2 = point_2(1);
%         y_2 = point_2(2);
%         
%     end
    
    %Calculate beam and wall direction vectors
    % beam_direction_vector = [(x_current_target - x_observer) , (y_current_target - y_observer)];
    wall_dir = [(x_2-x_1), (y_2-y_1)];
%     wall_direction_vector(1,1) = (x_2-x_1);
%     wall_direction_vector(1,2) = (y_2-y_1);
    [x_observer,y_observer,x_current_target,y_current_target,beam_direction_vector]=f{1}();%f();
    %beam_direction_vector = f();
    %Check for cosine of angle between the lines
    %intersection_check = sum(beam_direction_vector.*repmat(wall_dir,size(beam_direction_vector,1),1),2)./(vecnorm(beam_direction_vector')'.*norm(wall_dir));
    %intersection_check = sum(beam_direction_vector.*wall_dir,2)./(vecnorm(beam_direction_vector')'.*norm(wall_dir));
    
    a=beam_direction_vector.*wall_dir;
    intersection_check = (a(:,1)+a(:,2))./((((beam_direction_vector(:,1)).^2+(beam_direction_vector(:,2)).^2).^(1/2)).*((wall_dir(1)^2+wall_dir(2)^2)^(1/2)));
    indd=intersection_check ~= 1 & intersection_check ~= -1;

    p_calculation_numerator = (x_2 - x_1).*(y_1 - y_observer) - (y_2 - y_1).*(x_1 - x_observer);
    p_calculation_denominator = (x_2 - x_1).*(y_current_target - y_observer) - (y_2 - y_1).*(x_current_target - x_observer);
    p = p_calculation_numerator./p_calculation_denominator;

%     q=zeros(length(p),1,'gpuArray');
    q=Inf*ones(length(p),1);

    ind = (p >= 0 & (y_2 - y_1) == 0) & indd;
    q(ind) = ( x_observer(ind) - x_1 + p(ind).*(x_current_target(ind) - x_observer(ind)) )/(x_2 - x_1);

    ind = (p >= 0 & (x_2 - x_1) == 0) & indd;
    q(ind) = ( y_observer(ind) - y_1 + p(ind).*(y_current_target(ind) - y_observer(ind)) )/(y_2 - y_1);

    ind = (p >= 0 & ~((x_2 - x_1) == 0|(y_2 - y_1)==0)) & indd;
    q(ind) = ( y_observer(ind) - y_1 + p(ind).*(y_current_target(ind) - y_observer(ind)) )/(y_2 - y_1);

    ind=logical((q >= 0 & q <= 1));p(~ind)=0;
    %distance_array(:,i) = distance_array(:,i)+p;

end





