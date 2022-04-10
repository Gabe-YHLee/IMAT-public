clc;
close all;

%% Add path
addpath('utils\2d');
addpath('data\2dimages');
addpath('functions\2d');


%% Data preproseccing
Test_image = imread('anim0.pgm');
sampling_ratio = 0.2;
noise_level = 0.001;
plot = 1;
[pointcloud_sampled, orientednormal_sampled, pointcloud_noise] = image2orientedpointcloud(...
    Test_image, sampling_ratio, noise_level, plot);

%% IMAT

init_num_circles = 10000;
lambda = 2;
inner_lambda = 5;
s_p = 10;
step_size1 = 0.01;
step_size2 = 0.002;
margin = noise_level * sqrt(2);
num_iter = 25;

plot = 1;
mov = 0;

[c, r, frames] = IMAT2D(pointcloud_noise, orientednormal_sampled, ...
    init_num_circles, lambda, inner_lambda, s_p, step_size1, step_size2, ...
    margin, num_iter, plot, mov);

if mov == 1
    figure()
    title('Movie being played back...');
    movie(frames, 1, 2);
end