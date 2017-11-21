function [ result ] = mymedfilter( image, window_size )
%Performs median filtering on an image
%   image: image to be filtered
%   window_size: dimension of one side of the square filter. Must be odd.
    sz_image = size(image);
    pad_size = floor(window_size/2);
    pad_image = padarray(image, [pad_size, pad_size], 'replicate','both');
    im_blocks = im2col(pad_image, [window_size, window_size], 'sliding');
    median_vals = median(im_blocks, 1);
    result = reshape(median_vals, sz_image(1), sz_image(2));

end
