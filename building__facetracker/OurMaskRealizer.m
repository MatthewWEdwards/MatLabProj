classdef OurMaskRealizer
    %OURMASKREALIZER 
    
    properties
        MaskMapAz
    end
    
    methods
        function obj = OurMaskRealizer()
            load('MaskMap_Azimuth.mat');
            obj.MaskMapAz = maskMap;
        end
        
        function videoFrame = AddMask(obj, videoFrame, faceTracker)
            try 
                [az_est, ~] = faceTracker.EstimateFaceOrientation();
                maskIm = obj.maskImFromAz(az_est);
                matchedMask = MatchMaskToFace(maskIm, faceTracker);
                videoFrame = blendMaskWithFrame(videoFrame, matchedMask);
            catch EX
                
            end
        end
        
        function maskIm = maskImFromAz(obj, az)
            % Retrieve image based on orientation of the face
            sampled_azimuth = int8((180.0*az/pi)/1)*1;
            if sampled_azimuth > 50
                sampled_azimuth =  50;
            end
            if sampled_azimuth < -50
                sampled_azimuth = -50;
            end
            maskIm = obj.MaskMapAz(sampled_azimuth);
        end
        
    end
end

function videoFrame = blendMaskWithFrame(videoFrame, matchedMask)
% Create logical mask from the image mask overlay
mask = sum(matchedMask, 3) > 0.2;
masked_frame = videoFrame.*uint8(mask);

% Adjust mask luminance based on the luminance in the videoframe
face_avg_lum = double(mean(mean(mean(masked_frame))))/255;
mask_avg_lum = double(mean(mean(mean(matchedMask))));
matchedMask(:,:,1) = matchedMask(:,:,1)*(face_avg_lum/mask_avg_lum);
matchedMask(:,:,2) = matchedMask(:,:,2)*(face_avg_lum/mask_avg_lum);
matchedMask(:,:,3) = matchedMask(:,:,3)*(face_avg_lum/mask_avg_lum);

% Add mask to videoframe
videoFrame = videoFrame .* uint8(~mask);
videoFrame = videoFrame + uint8(255*matchedMask);
end

function matchedMask = MatchMaskToFace(maskIm, faceTracker)
[b_center, hor_length, ver_length, top_line_slope] = ExtractMaskData(faceTracker);
frame_sz = faceTracker.FrameSize;

% Resize mask based on detected face
mask_resize = imresize(maskIm, [ver_length, hor_length]);
mask_resize = imrotate(mask_resize, atan(top_line_slope)*90/pi);
pad_size_pre = double([b_center(2) - (ver_length/2), b_center(1) - (hor_length/2)]);
pad_size_post = double([frame_sz(1) - b_center(2) - (ver_length/2), frame_sz(2) - b_center(1) - (hor_length/2)]);
matchedMask = padarray(mask_resize, pad_size_pre, 0,'pre');
matchedMask = padarray(matchedMask, pad_size_post, 0,'post');
matchedMask = imresize(matchedMask,[frame_sz(1) frame_sz(2)]);
end

function [b_center, hor_length, ver_length, top_line_slope] = ExtractMaskData(faceTracker)
% Get points from the face bounding box
box_points = int32(faceTracker.FaceDetector.BboxPts);
b_tl = box_points(1, :);
b_tr = box_points(2, :);
b_br = box_points(3, :);
b_bl = box_points(4, :);

% Bounding box information from points
b_center = floor((b_br + b_tl)/2);
hor_length = int32(sqrt(double(((b_tl(1) - b_tr(1))^2 + (b_tl(2) - b_tr(2))^2))));
ver_length = int32(sqrt(double(((b_tl(1) - b_bl(1))^2 + (b_tl(2) - b_bl(2))^2))));
top_line_slope = double(b_tr(2) - b_tl(2))/double(b_tl(2) - b_tl(1));
end
