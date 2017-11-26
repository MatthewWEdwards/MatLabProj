close all
clear

load('mask_im.mat')

global bboxPolygonEyes;
global bboxPolygonNose;
global azimuth;
global elevation;

azimuth = 0;

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();
eyeDetector = vision.CascadeObjectDetector('EyePairSmall','UseROI',true);
noseDetector = vision.CascadeObjectDetector('Nose','UseROI',true);

% Create the point tracker object.
pointTrackerFace = vision.PointTracker('MaxBidirectionalError', 2);
pointTrackerEyes = vision.PointTracker('MaxBidirectionalError', 2);
pointTrackerNosee = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
maskFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
videoPlayer2 = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPtsFace = 0;
numPtsEyes = 0; 
numPtsNose = 0;
frameCount = 0;

while runLoop && frameCount < 400

    % Get the next frame.
    videoFrame = snapshot(cam);
    maskFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPtsFace < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);

        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;

            numPtsFace = size(xyPoints,1);
            release(pointTrackerFace);
            initialize(pointTrackerFace, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTrackerFace, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPtsFace = size(visiblePoints, 1);

        if numPtsFace >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTrackerFace, oldPoints);
        end
    end
    
    if numPtsEyes < 10
        % Detection mode.
        if(size(bbox) ~= 0)
            bboxEyes = eyeDetector.step(videoFrameGray,bbox(1, :));
        else
            size(videoFrameGray)
            frameSize_ = frameSize
            bboxEyes = eyeDetector.step(videoFrameGray,[30, 30, frameSize(2)-30, frameSize(1)-30]);
        end
        
        if ~isempty(bboxEyes)
            % Find corner points inside the detected region.
            pointsEyes = detectMinEigenFeatures(videoFrameGray, 'ROI', bboxEyes(1, :));

            % Re-initialize the point tracker.
            xyPointsEyes = pointsEyes.Location;

            numPtsEyes = size(xyPointsEyes,1);
            release(pointTrackerEyes);
            initialize(pointTrackerEyes, xyPointsEyes, videoFrameGray);

            % Save a copy of the points.
            oldPointsEyes = xyPointsEyes;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPointsEyes = bbox2points(bboxEyes(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygonEyes = reshape(bboxPointsEyes', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygonEyes, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPointsEyes, '+', 'Color', 'blue');
        end

    else
        % Tracking mode.
        [xyPointsEyes, isFoundEyes] = step(pointTrackerEyes, videoFrameGray);
        visiblePointsEyes = xyPointsEyes(isFoundEyes, :);
        oldInliersEyes = oldPointsEyes(isFoundEyes, :);

        numPtsEyes = size(visiblePointsEyes, 1);

        if numPtsEyes >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform2, oldInliersEyes, visiblePointsEyes] = estimateGeometricTransform(...
                oldInliersEyes, visiblePointsEyes, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPointsEyes = transformPointsForward(xform2, bboxPointsEyes);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygonEyes = reshape(bboxPointsEyes', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygonEyes, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePointsEyes, '+', 'Color', 'blue');

            % Reset the points.
            oldPointsEyes = visiblePointsEyes;
            setPoints(pointTrackerEyes, oldPointsEyes);
        end

    end
    
    
    if numPtsNose < 10
        % Detection mode.
        if(size(bbox) ~= 0)
            bboxNose = noseDetector.step(videoFrameGray,bbox(1, :));
        else
            bboxNose = noseDetector.step(videoFrameGray,[30, 30, frameSize(2)-30, frameSize(1)-30]);
        end

        if ~isempty(bboxNose)
            % Find corner points inside the detected region.
            pointsNose = detectMinEigenFeatures(videoFrameGray, 'ROI', bboxNose(1, :));

            % Re-initialize the point tracker.
            xyPointsNose = pointsNose.Location;

            numPtsNose = size(xyPointsNose,1);
            release(pointTrackerNosee);
            initialize(pointTrackerNosee, xyPointsNose, videoFrameGray);

            % Save a copy of the points.
            oldPointsNose = xyPointsNose;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPointsNose = bbox2points(bboxNose(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygonNose = reshape(bboxPointsNose', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygonNose, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPointsNose, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPointsNose, isFound5] = step(pointTrackerNosee, videoFrameGray);
        visiblePoints5 = xyPointsNose(isFound5, :);
        oldInliers5 = oldPointsNose(isFound5, :);

        numPtsNose = size(visiblePoints5, 1);

        if numPtsNose >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform5, oldInliers5, visiblePoints5] = estimateGeometricTransform(...
                oldInliers5, visiblePoints5, 'similarity', 'MaxDistance', 3);

            % Apply the transformation to the bounding box.
            bboxPointsNose = transformPointsForward(xform5, bboxPointsNose);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygonNose = reshape(bboxPointsNose', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygonNose, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints5, '+', 'Color', 'blue');

            % Reset the points.
            oldPointsNose = visiblePoints5;
            setPoints(pointTrackerNosee, oldPointsNose);
        end

    end
    
    
    if(size(bboxPolygonEyes,2) == 8 && size(bboxPolygonNose,2) == 8)
        %[x1 y1 x2 y2 x3 y3 x4 y4]
      %  answer1 = bboxPolygonEyes
        xEyesAvg = (bboxPolygonEyes(1) + bboxPolygonEyes(3) + bboxPolygonEyes(5) + bboxPolygonEyes(7)) / 4;
        yEyesAvg = (bboxPolygonEyes(2) + bboxPolygonEyes(4) + bboxPolygonEyes(6) + bboxPolygonEyes(8)) / 4;
      %  answer2 = bboxPolygonNose
        xNoseAvg = (bboxPolygonNose(1) + bboxPolygonNose(3) + bboxPolygonNose(5) + bboxPolygonNose(7)) / 4;
        yNoseAvg = (bboxPolygonNose(2) + bboxPolygonNose(4) + bboxPolygonNose(6) + bboxPolygonNose(8)) / 4;
        azimuth = atan((xNoseAvg - xEyesAvg)/(yNoseAvg - yEyesAvg));
        elevation = atan((yNoseAvg - yEyesAvg)/(10));
       % answer = [azimuth elevation];
    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    
    % Add mask to videoFrame2
    if exist('mask_im', 'var') == 1
        frame_sz = size(maskFrame);
       
        % Get points from the face bounding box
        box_points = int32(bboxPoints);
        b_tl = box_points(1, :);
        b_tr = box_points(2, :);     
        b_br = box_points(3, :);
        b_bl = box_points(4, :);
        
        % Bounding box information from points
        b_center = floor((b_br + b_tl)/2);
        hor_length = int32(sqrt(double(((b_tl(1) - b_tr(1))^2 + (b_tl(2) - b_tr(2))^2))));
        ver_length = int32(sqrt(double(((b_tl(1) - b_bl(1))^2 + (b_tl(2) - b_bl(2))^2))));
        top_line_slope = double(b_tr(2) - b_tl(2))/double(b_tl(2) - b_tl(1));
        
        % Retrieve image based on orientation of the face
        sampled_azimuth = int8((180.0*azimuth/pi)/1)*1;
        if sampled_azimuth > 50 
           sampled_azimuth =  50;
        end
        if sampled_azimuth < -50
           sampled_azimuth = -50;
        end
        mask = double(imread(strcat('mesh_test/2mask_im_az',num2str(sampled_azimuth),'.png')))./(255.0);
        
        % Resize mask based on detected face
        mask_resize = imresize(mask, [ver_length, hor_length]);
        mask_resize = imrotate(mask_resize, atan(top_line_slope)*90/pi);
        pad_size_pre = double([b_center(2) - (ver_length/2), b_center(1) - (hor_length/2)]);
        pad_size_post = double([frame_sz(1) - b_center(2) - (ver_length/2), frame_sz(2) - b_center(1) - (hor_length/2)]);
        mask_fullframe = padarray(mask_resize, pad_size_pre, 0,'pre');
        mask_fullframe = padarray(mask_fullframe, pad_size_post, 0,'post');
        mask_fullframe = imresize(mask_fullframe,[frame_sz(1) frame_sz(2)]);
      
        % Create logical mask from the image mask overlay
        mask = sum(mask_fullframe, 3) > 0.2; 
        masked_frame = maskFrame.*uint8(mask);
        
        % Adjust mask luminance based on the luminance in the videoframe
        face_avg_lum = double(mean(mean(mean(masked_frame))))/255;
        mask_avg_lum = double(mean(mean(mean(mask_fullframe))));
        mask_fullframe(:,:,1) = mask_fullframe(:,:,1)*(face_avg_lum/mask_avg_lum);
        mask_fullframe(:,:,2) = mask_fullframe(:,:,2)*(face_avg_lum/mask_avg_lum);
        mask_fullframe(:,:,3) = mask_fullframe(:,:,3)*(face_avg_lum/mask_avg_lum);
        
        % Add mask to videoframe
        maskFrame = maskFrame .* uint8(~mask);
        maskFrame = maskFrame + uint8(255*mask_fullframe);
        
    end
    step(videoPlayer2, maskFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(videoPlayer2);
release(pointTrackerFace);
release(faceDetector);