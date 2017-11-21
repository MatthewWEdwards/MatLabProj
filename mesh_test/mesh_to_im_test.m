close all
clear

test_im = obj_to_im('downloads/mattreduced.obj', 128, 128, 0, 90);

figure
imshow(test_im);
title('Output of obj_to_im.m')

figure
filtered_im = test_im;
filtered_im(:,:,1) = myavgfilter(test_im(:,:,1), 5);
filtered_im(:,:,2) = myavgfilter(test_im(:,:,2), 5);
filtered_im(:,:,3) = myavgfilter(test_im(:,:,3), 5);
imshow(filtered_im)
title('Average filtered')

figure
filtered_im = test_im;
filtered_im(:,:,1) = mymedfilter(test_im(:,:,1), 5);
filtered_im(:,:,2) = mymedfilter(test_im(:,:,2), 5);
filtered_im(:,:,3) = mymedfilter(test_im(:,:,3), 5);
imshow(filtered_im)
title('Median filtered')

figure
filtered_im = test_im;
filtered_im(:,:,1) = mygaufilter(test_im(:,:,1), 5, 1);
filtered_im(:,:,2) = mygaufilter(test_im(:,:,2), 5, 1);
filtered_im(:,:,3) = mygaufilter(test_im(:,:,3), 5, 1);
imshow(filtered_im)
title('Gaussian filtered')
