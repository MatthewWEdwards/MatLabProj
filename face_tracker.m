close all
clear

load('mask_im.mat')

figure

global bboxPolygonEyes;
global bboxPolygonNose;


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
videoFrame2 = snapshot(cam);
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
    videoFrame2 = snapshot(cam);
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
        answer = [azimuth elevation]
    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);
    
    
    % Add mask to videoFrame2
    if exist('mask_im', 'var') == 1
        top_line_slope = (bboxPoints(2,2) - bboxPoints(1, 2))/(bboxPoints(1,2) - bboxPoints(1,1));
        left_most_point_val = min(xyPoints(:, 1));
        right_most_point_val = max(xyPoints(:,1));
        top_most_point_val = min(xyPoints(:, 2));
        bottom_most_point_val = max(xyPoints(:,2));
        
        % Assume: mask_im is of type double
        box_points = uint16(bboxPoints);
        box_left_pt = min(box_points(:,1));
        box_right_pt = max(box_points(:,1));
        box_top_pt = min(box_points(:,2));
        box_bottom_pt = max(box_points(:,2));
        
        num_rows = box_bottom_pt - box_top_pt;
        num_cols = box_right_pt - box_left_pt;
        mask_resize = imresize(mask_im, [num_rows, num_cols]);
        mask = sum(mask_resize, 3) > 0.2; 
        videoFrame2(box_top_pt:box_bottom_pt-1, box_left_pt:box_right_pt-1, :) = ...
            videoFrame2(box_top_pt:box_bottom_pt-1, box_left_pt:box_right_pt-1, :) .* ...
            uint8(~mask);
        videoFrame2(box_top_pt:box_bottom_pt-1, box_left_pt:box_right_pt-1, :) = ...
            videoFrame2(box_top_pt:box_bottom_pt-1, box_left_pt:box_right_pt-1, :)+   ... 
            uint8(255*mask_resize);
        figure(1)
        imshow(videoFrame2)
    end
    step(videoPlayer2, videoFrame2);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(videoPlayer2);
release(pointTrackerFace);
release(faceDetector);