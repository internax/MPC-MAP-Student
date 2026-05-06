function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Iterativni vyhlazeni cesty metodou gradient descent

if isempty(old_path) || size(old_path,1) < 3
    new_path = old_path;
    return;
end

alpha = 0.4;
beta  = 0.3;
tol   = 1e-5;

new_path = old_path;
n        = size(old_path, 1);

change = tol + 1;
while change > tol
    change = 0;
    for i = 2:n-1
        for j = 1:2
            old_val       = new_path(i,j);
            new_path(i,j) = new_path(i,j) ...
                + alpha * (old_path(i,j)   - new_path(i,j)) ...
                + beta  * (new_path(i-1,j) + new_path(i+1,j) - 2*new_path(i,j));
            change = change + abs(new_path(i,j) - old_val);
        end
    end
end

end
