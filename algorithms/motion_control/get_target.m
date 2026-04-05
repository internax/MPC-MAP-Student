function [target] = get_target(pose, path)
                                                                                                                                                                                                                     
  if isempty(path)
      target = pose(1:2);                                                                                                                                                                                  
      return      
  end                                                                                                                                                                                                                
   
  % nejblizsk bod                                                                                                                                                                                  
  distances = sqrt((path(:,1) - pose(1)).^2 + (path(:,2) - pose(2)).^2);
  [~, index] = min(distances);                                                                                                                                                                                             
   
  % cilovy bod                                                                                                                                                                                   
  lookahead_idx = min(index + 5, size(path, 1));
  target = path(lookahead_idx, :);                                                                                                                                                                                   
             
