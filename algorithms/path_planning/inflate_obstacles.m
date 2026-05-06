function inflated = inflate_obstacles(map, n)
    kernel   = ones(2*n+1, 2*n+1);
    inflated = conv2(double(map), kernel, 'same') > 0;
end
