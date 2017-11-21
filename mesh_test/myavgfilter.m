function [ result ] = myavgfilter( image, window_size )
%Performs average filtering on an image
%   image: image to be filtered
%   window_size: dimension of one side of the square filter
    window = double(ones(window_size, window_size));
    window = window/(window_size^2);
    result = imfilter(image, window);
end
