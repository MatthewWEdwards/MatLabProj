% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();
leftEyeDetector = vision.CascadeObjectDetector('LeftEyeCART','UseROI',true);
rightEyeDetector = vision.CascadeObjectDetector('RightEyeCART','UseROI', true);
mouthDetector = vision.CascadeObjectDetector('Mouth','UseROI',true);
noseDetector = vision.CascadeObjectDetector('Nose','UseROI',true);


% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
pointTracker2 = vision.PointTracker('MaxBidirectionalError', 2);
pointTracker3 = vision.PointTracker('MaxBidirectionalError', 2);
pointTracker4 = vision.PointTracker('MaxBidirectionalError', 2);
pointTracker5 = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPts = 0;
numPts2 = 0;
numPts3 = 0;
numPts4 = 0;
numPts5 = 0;
frameCount = 0;

while runLoop && frameCount < 400

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);

        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;

            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

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
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
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
            setPoints(pointTracker, oldPoints);
        end

    end
    
    if numPts2 < 10
        % Detection mode.
        bbox2 = leftEyeDetector.step(videoFrameGray,bbox(1, :));

        if ~isempty(bbox2)
            % Find corner points inside the detected region.
            points2 = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox2(1, :));

            % Re-initialize the point tracker.
            xyPoints2 = points2.Location;

            numPts2 = size(xyPoints2,1);
            release(pointTracker2);
            initialize(pointTracker2, xyPoints2, videoFrameGray);

            % Save a copy of the points.
            oldPoints2 = xyPoints2;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints2 = bbox2points(bbox2(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon2 = reshape(bboxPoints2', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon2, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints2, '+', 'Color', 'blue');
        end

    else
        % Tracking mode.
        [xyPoints2, isFound2] = step(pointTracker2, videoFrameGray);
        visiblePoints2 = xyPoints2(isFound2, :);
        oldInliers2 = oldPoints2(isFound2, :);

        numPts2 = size(visiblePoints2, 1);

        if numPts2 >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform2, oldInliers2, visiblePoints2] = estimateGeometricTransform(...
                oldInliers2, visiblePoints2, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints2 = transformPointsForward(xform2, bboxPoints2);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon2 = reshape(bboxPoints2', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon2, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints2, '+', 'Color', 'blue');

            % Reset the points.
            oldPoints2 = visiblePoints2;
            setPoints(pointTracker2, oldPoints2);
        end

    end
    
    if numPts3 < 10
        % Detection mode.
        bbox3 = rightEyeDetector.step(videoFrameGray,bbox(1, :));

        if ~isempty(bbox3)
            % Find corner points inside the detected region.
            points3 = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox3(1, :));

            % Re-initialize the point tracker.
            xyPoints3 = points3.Location;

            numPts3 = size(xyPoints3,1);
            release(pointTracker3);
            initialize(pointTracker3, xyPoints3, videoFrameGray);

            % Save a copy of the points.
            oldPoints3 = xyPoints3;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints3 = bbox2points(bbox3(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon3 = reshape(bboxPoints3', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon3, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints3, '+', 'Color', 'green');
        end

    else
        % Tracking mode.
        [xyPoints3, isFound3] = step(pointTracker3, videoFrameGray);
        visiblePoints3 = xyPoints3(isFound3, :);
        oldInliers3 = oldPoints3(isFound3, :);

        numPts3 = size(visiblePoints3, 1);

        if numPts3 >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform3, oldInliers3, visiblePoints3] = estimateGeometricTransform(...
                oldInliers3, visiblePoints3, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints3 = transformPointsForward(xform3, bboxPoints3);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon3 = reshape(bboxPoints3', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon3, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints3, '+', 'Color', 'green');

            % Reset the points.
            oldPoints3 = visiblePoints3;
            setPoints(pointTracker3, oldPoints3);
        end

    end
    
    if numPts4 < 10
        % Detection mode.
        bbox4 = mouthDetector.step(videoFrameGray,bbox(1, :));

        if ~isempty(bbox4)
            % Find corner points inside the detected region.
            points4 = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox4(1, :));

            % Re-initialize the point tracker.
            xyPoints4 = points4.Location;

            numPts4 = size(xyPoints4,1);
            release(pointTracker4);
            initialize(pointTracker4, xyPoints4, videoFrameGray);

            % Save a copy of the points.
            oldPoints4 = xyPoints4;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints4 = bbox2points(bbox4(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon4 = reshape(bboxPoints4', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon4, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints4, '+', 'Color', 'yellow');
        end

    else
        % Tracking mode.
        [xyPoints4, isFound4] = step(pointTracker4, videoFrameGray);
        visiblePoints4 = xyPoints4(isFound4, :);
        oldInliers4 = oldPoints4(isFound4, :);

        numPts4 = size(visiblePoints4, 1);

        if numPts4 >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform4, oldInliers4, visiblePoints4] = estimateGeometricTransform(...
                oldInliers4, visiblePoints4, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints4 = transformPointsForward(xform4, bboxPoints4);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon4 = reshape(bboxPoints4', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon4, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints4, '+', 'Color', 'yellow');

            % Reset the points.
            oldPoints4 = visiblePoints4;
            setPoints(pointTracker4, oldPoints4);
        end

    end
    
    if numPts5 < 10
        % Detection mode.
        bbox5 = noseDetector.step(videoFrameGray,bbox(1, :));

        if ~isempty(bbox5)
            % Find corner points inside the detected region.
            points5 = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox5(1, :));

            % Re-initialize the point tracker.
            xyPoints5 = points5.Location;

            numPts5 = size(xyPoints5,1);
            release(pointTracker5);
            initialize(pointTracker5, xyPoints5, videoFrameGray);

            % Save a copy of the points.
            oldPoints5 = xyPoints5;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints5 = bbox2points(bbox5(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon5 = reshape(bboxPoints5', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon5, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints5, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints5, isFound5] = step(pointTracker5, videoFrameGray);
        visiblePoints5 = xyPoints5(isFound5, :);
        oldInliers5 = oldPoints5(isFound5, :);

        numPts5 = size(visiblePoints5, 1);

        if numPts5 >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform5, oldInliers5, visiblePoints5] = estimateGeometricTransform(...
                oldInliers5, visiblePoints5, 'similarity', 'MaxDistance', 3);

            % Apply the transformation to the bounding box.
            bboxPoints5 = transformPointsForward(xform5, bboxPoints5);
            

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon5 = reshape(bboxPoints5', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon5, 'LineWidth', 3);

            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints5, '+', 'Color', 'blue');

            % Reset the points.
            oldPoints5 = visiblePoints5;
            setPoints(pointTracker5, oldPoints5);
        end

    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);