function S = Polygons_intersection_Polygon_cleanup_vv(S,accuracy)
if nargin == 1
    accuracy = 1e-9;
end
if accuracy<eps
    error('Polygons_intersection_Polygon_cleanup:e0',...
          'Accuracy must be a positive real');
end

% for i=1:numel(S)
%     To_delete = S(i).area < accuracy;
%     if To_delete
%         S(i) = [];
%     end
% end

S(arrayfun(@(s) isempty(s.area),S))=[];  %Test - Removes empty cells 

S([S.area]<accuracy) = [];
