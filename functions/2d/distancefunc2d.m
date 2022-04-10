function d = distancefunc2d(x, pointcloud_sampled, orientednormal_sampled)
    v = pointcloud_sampled - reshape(x,[1,2]);
    vec_norm = sum(v.*v, 2);
    [~, i] = sort(vec_norm);
    
    d_p2po = norm(v(i(1), :));
    if v(i(1), :) * orientednormal_sampled(i(1), :)' < 0
        d_p2po = -d_p2po;
    end
    
    d = d_p2po;
end
