function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Naplánuje cestu pomocí A* - plánuje pouze jednou (když je cesta prázdná)

if isempty(public_vars.path)
    path = astar(read_only_vars, public_vars);
    path = smooth_path(path);
else
    path = public_vars.path;
end

end
