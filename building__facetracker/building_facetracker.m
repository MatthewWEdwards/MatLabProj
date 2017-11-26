close all
clear

try
    % Create the webcam object.
    cam = webcam();
    
    % Capture one frame to get its size.
    videoFrame = snapshot(cam);
    frameSize = size(videoFrame);
    
    % Create the video player object.
    videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
    
    runLoop = true;
    frameCount = 0;
    
    % Create the face tracker object.
    faceDetector = vision.CascadeObjectDetector();
    eyeDetector = vision.CascadeObjectDetector('EyePairSmall','UseROI',true);
    noseDetector = vision.CascadeObjectDetector('Nose','UseROI',true);
    faceTracker = OurFaceTracker(faceDetector, eyeDetector, noseDetector, frameSize);
    
    while runLoop && frameCount < 400
        frameCount = frameCount + 1;
        % Get the next frame.
        videoFrame = snapshot(cam);
        videoFrameGray = rgb2gray(videoFrame);
        
        % track face
        faceTracker = faceTracker.Step(videoFrameGray);
        
        % add annotated face data to video frame
        videoFrame_ann = addAnnotatedFaceDataToVideo(faceTracker, videoFrame);
        
        % Display the annotated video frame using the video player object.
        step(videoPlayer, videoFrame_ann);
        
        % Check whether the video player window has been closed.
        runLoop = isOpen(videoPlayer);
    end
catch EX
    clean();
    rethrow(EX);
end
clean();

function [] = clean()
    if(exist('videoPlayer', 'var'))
        release(videoPlayer);
    end    
    clear cam;
end

function [videoFrame] = addAnnotatedFaceDataToVideo(faceTracker, videoFrame)
    % ADD FACE DATA
    % Display a bounding box around the face being tracked.
    videoFrame = insertShape(videoFrame, 'Polygon', ...
        faceTracker.FaceDetector.BboxPolygon, 'LineWidth', 3);
    
    % Display tracked points.
    videoFrame = insertMarker(videoFrame, ...
        faceTracker.FaceDetector.OldPts, '+', 'Color', 'white');
    
    % ADD EYES DATA
    videoFrame = insertShape(videoFrame, 'Polygon', ...
        faceTracker.EyesDetector.BboxPolygon, 'LineWidth', 3);
    videoFrame = insertMarker(videoFrame, ...
        faceTracker.EyesDetector.OldPts, '+', 'Color', 'blue');
    
    % ADD NOSE DATA
    videoFrame = insertShape(videoFrame, 'Polygon', ...
        faceTracker.NoseDetector.BboxPolygon, 'LineWidth', 3);
    videoFrame = insertMarker(videoFrame, ...
        faceTracker.NoseDetector.OldPts, '+', 'Color', 'white');
end