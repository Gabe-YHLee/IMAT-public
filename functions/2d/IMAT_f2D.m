function [c, r, frames] = IMAT_f2D(pointcloud_noise, orientednormal_sampled, ...
    num_fixed_balls, lambda, inner_lambda, s_p, step_size1, step_size2, ...
    margin, num_iter, plot, movie)

fprintf('-------------------- Initialization begins \n');

r_init = zeros(num_fixed_balls, 1);
c_init = zeros(num_fixed_balls,2);

for i=1:num_fixed_balls
    c = [rand(1), rand(1)];
    r = 0.1 * rand(1);
    r_init(i) = r;
    c_init(i,:) = c;
end
 
fprintf('-------------------- Initialization ends \n');

num_circles = num_fixed_balls;

%%%%%% optimization param and initial param re-setting %%%%%%
position_centers = cell(num_iter+1, 1);
radius = cell(num_iter+1, 1);

position_centers(1) = {c_init};
radius(1) = {r_init};

fprintf('-------------------- optimization begins \n');

for k=1:num_iter
    fprintf('-------------------- %d -th iter \n', k);
    clear c r;
    c = cell2mat(position_centers(k));
    r = cell2mat(radius(k));
    m = num_circles;
    %%%%%%%%%%%%%%% reassignment step %%%%%%%%%%%%%%%%
    Index_matrix = zeros(size(c,1), size(pointcloud_noise,1));
    for i=1:size(c,1)
        for j=1:size(pointcloud_noise,1)
            Index_matrix(i,j)=(norm(c(i,:)-pointcloud_noise(j,:))-r(i))^2;
        end
    end
    clear Index;
    [~, Index]=min(Index_matrix);

    %%%%%%%%%%%%%%% reduction step %%%%%%%%%%%%%%%%
    clear a num Num;

    Num = cell(num_fixed_balls);
    for i=1:m
        num = 1:size(pointcloud_noise, 1);
        Num(i) = {num(Index==i)};  
    end

    %%%%%%%%%%%%%%% boundary matching %%%%%%%%%%%%%%%%
    cost_prev = zeros(m,1);
    for i=1:m
        temp_c = c(i,:);
        temp_r = r(i);
        temp_Num = Num(i);
        scale_factor = 1;
        for j=1:lambda
            for l=1:inner_lambda
                Grad = grad_boundary_matching_obj(...
                    temp_c, temp_r, cell2mat(temp_Num), ...
                    pointcloud_noise, orientednormal_sampled, margin, scale_factor);
                temp_c = temp_c - step_size1 * Grad(1:2)/norm(Grad);
                temp_r = temp_r - step_size1 * Grad(3)/norm(Grad);
            end
            scale_factor = scale_factor * s_p;
        end
       c(i, :) = temp_c;
       r(i) = temp_r;
       cost_prev(i) = boundary_matching_obj(...
            temp_c, temp_r, cell2mat(temp_Num), ...
            pointcloud_noise, orientednormal_sampled, margin, scale_factor);
    end

    %%%%%%%%%%%%%%% Volume maximizing %%%%%%%%%%%%%%%%
    for i=1:m
        temp_c = c(i,:);
        temp_r = r(i);
        temp_Num = Num(i);
        temp_cost = cost_prev(i);
        scale_factor = 1;
        for j=1:lambda
            for l=1:inner_lambda
                Grad = grad_vol_max_obj(temp_c, temp_r, temp_cost, cell2mat(temp_Num), ...
                    pointcloud_noise, orientednormal_sampled, margin, scale_factor);
                temp_c = temp_c - step_size2 * Grad(1:2)/norm(Grad);
                temp_r =temp_r - step_size2 * Grad(3)/norm(Grad);
            end
            scale_factor = scale_factor * s_p;
        end
        c(i, :) = temp_c;
        r(i) = temp_r;
    end

position_centers(k+1) = {c};
radius(k+1) = {r};
end

fprintf('-------------------- optimization ends \n');

if plot == 1
    figure()    
    hold on;
    title('Approximate medial axis')
    scatter(pointcloud_noise(:,1),pointcloud_noise(:,2),4,'filled','k');
    scatter(c(:,1),c(:,2),20,'MarkerEdgeColor',[0.0,0.0,1],'MarkerFaceColor',[0.0,0.0,1]);
    axis([-0.1,1.1,-0.1,1.1]);

    figure()    
    for i=1:size(c,1)
        hold on;
        drawfilledcircle(c(i,1),c(i,2),r(i),[0,0.6,0.9]);
    end
    hold on;
    title('Approximate medial balls')
    scatter(pointcloud_noise(:,1),pointcloud_noise(:,2),4,'filled','k');
    axis([-0.1,1.1,-0.1,1.1]);
end

if movie == 1
    figure()
    nr_fr = num_iter;
    frames = moviein(nr_fr); 

    for i = 1 : nr_fr
        fprintf('%d-th frame is being recorded \n', i)
        clear c r;
        c = cell2mat(position_centers(i));
        r = cell2mat(radius(i));
        figure()
        for j=1:size(c,1)
            hold on;
            drawfilledcircle(c(j,1),c(j,2),r(j),[0,0.6,0.9]);
        end
        hold on;
        scatter(c(:,1),c(:,2),20,'MarkerEdgeColor',[0.0,0.0,1],'MarkerFaceColor',[0.0,0.0,1]);
        scatter(pointcloud_noise(:,1),pointcloud_noise(:,2),4,'filled','k');
        axis([-0.1,1.1,-0.1,1.1]);
        title('Recording movie...')
        frames(:, i) = getframe;
    end

else
    frames = 1;
end

end

