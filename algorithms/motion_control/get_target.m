function [target] = get_target(pose, path)

  if isempty(path)
      target = pose(1:2);
      return
  end
%hledání ve vzdálenosti lookahed_dist
  lookahead_dist = 0.5; % metry

  % Nejblizsí bod na ceste
  distances = sqrt((path(:,1) - pose(1)).^2 + (path(:,2) - pose(2)).^2);
  [~, nearest_idx] = min(distances);

  % Hledej prvni bod ktery je alespon lookahead_dist daleko od robota
  target = path(end, 1:2);
  for i = nearest_idx:size(path, 1)
      d = sqrt((path(i,1) - pose(1))^2 + (path(i,2) - pose(2))^2);
      if d >= lookahead_dist
          target = path(i, 1:2);
          return;
      end
  end
