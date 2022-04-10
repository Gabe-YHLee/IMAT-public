function [pointcloud_sampled, orientednormal_sampled, pointcloud_nosie] = ...
    image2orientedpointcloud(image, sampling_ratio, noise_level, plot)

Test_image = image;

[Y_size, X_size] = size(Test_image);
[Gx, Gy] = imgradientxy(Test_image);
[x, y] = meshgrid(1:X_size, 1:Y_size);

G_norm = sqrt(Gx.^2 + Gy.^2);

num_nonzeros_G_norm = sum(sum(G_norm~=0));
pointCloud = zeros(num_nonzeros_G_norm, 2);
orientedNormal = zeros(num_nonzeros_G_norm, 2);

num = 0;
for i=1:Y_size
    for j=1:X_size
        if G_norm(i,j)~=0
            num = num + 1;
            pointCloud(num,:) = [x(i,j), y(i,j)];
            orientedNormal(num,:) = [-Gx(i,j), -Gy(i,j)];
        end
    end
end


pointCloud = pointCloud*[1/X_size, 0; 0, 1/Y_size]*[0, -1; 1, 0]*[0, -1; 1, 0] + [1, 1];
orientedNormal = orientedNormal*[1/X_size, 0; 0, 1/Y_size]*[0, -1; 1, 0]*[0, -1; 1, 0];
orientedNormal = orientedNormal./vecnorm(orientedNormal')';

num_sample=int16(sampling_ratio*size(pointCloud, 1));
p = randperm(size(pointCloud, 1));
p = p(1:num_sample);

pointcloud_sampled = pointCloud(p,:);
orientednormal_sampled = orientedNormal(p,:);

sigma = noise_level * sqrt(2);
noist_theta = rand(num_sample, 1)*2*pi;
noise_direction = [cos(noist_theta), sin(noist_theta)];
pointcloud_nosie = pointcloud_sampled + sigma*randn(num_sample,1).*noise_direction;

if plot == 1
    figure()
    hold on;
    title('Point cloud and oriented normal (entire points)')
    scatter(pointCloud(:,1), pointCloud(:,2), 3, 'filled', 'k');
    quiver(pointCloud(:,1), pointCloud(:,2), orientedNormal(:,1), orientedNormal(:,2));

    figure()
    hold on;
    title('Point cloud and oriented normal (subsampled points)')
    scatter(pointcloud_sampled(:,1), pointcloud_sampled(:,2), 1, 'filled');
    quiver(pointcloud_sampled(:,1), pointcloud_sampled(:,2), orientednormal_sampled(:,1), orientednormal_sampled(:,2));

    figure()
    hold on;
    title('Noisy point cloud and oriented normal (subsampled points)')
    scatter(pointcloud_nosie(:,1),pointcloud_nosie(:,2),3,'filled','k');
    quiver(pointcloud_nosie(:,1),pointcloud_nosie(:,2),orientednormal_sampled(:,1),orientednormal_sampled(:,2));
end

