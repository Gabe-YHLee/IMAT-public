function cost = boundary_matching_obj(c, r, Index, pc_nosie, ...
    orientedNormal_sampled, margin, scale_factor) 
    s = size(size(Index,2),1);
    for j=1:size(Index,2)
        s(j)=(norm(c- pc_nosie(Index(j),:)) - r)^2;
    end
    t= margin;
    cost = mean(s) ...
        + scale_factor * (max(distancefunc2d(c, pc_nosie, orientedNormal_sampled) + r, t)^2 ...
        + scale_factor * max(-r,0)^2);
    
end

