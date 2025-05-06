clc;
clear;

figure(1)
im = imread('hello.jpg');
imshow(im);

figure(2)

% Convert a color image to a gray image
gray_im = (0.2989 * double(im(:, :, 1)) + 0.5870 * double(im(:, :, 2)) + 0.1140 * double(im(:, :, 3))) / 255;
imshow(gray_im);

figure(3)
BWs = edge(gray_im, 'sobel'); % Sobel filter
imshow(BWs)