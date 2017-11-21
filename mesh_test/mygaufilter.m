function [ result ] = mygaufilter( image, window_size, sigma )
%Performs gaussian filtering on an image
%   image: image to be filtered
%   window_size: dimension of one side of the square filter.
%   sigma: the standard deviation of the gaussian distribution

    gau_filter = fspecial('gaussian', window_size, sigma);
    result = imfilter(image, gau_filter);


end
