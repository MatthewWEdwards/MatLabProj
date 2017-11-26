close all
clear

showAnnotatedVid = true;
showMaskVid = false;

try
    % Create the webcam object.
    cam = webcam();
    
    % Capture one frame to get its size.
    videoFrame = snapshot(cam);
    frameSize = size(videoFrame);
    
    % Create specified video players.
    if(showAnnotatedVid)
        videoPlayerAnnotated = vision.VideoPlayer('Position', ...
            [100 100 [frameSize(2), frameSize(1)]+30]);
    end
    if(showMaskVid)
        load('mask_im.mat');
        videoPlayerMask = vision.VideoPlayer('Position', ...
            [100 100 [frameSize(2), frameSize(1)]+30]);
    end
    
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
        
        % Display the annotated video frame using the video player object.
        if(showAnnotatedVid)
            videoFrame_ann = faceTracker.annotateData(videoFrame);
            step(videoPlayerAnnotated, videoFrame_ann);
        end
        
        % generate mask
        if(showMaskVid)
            [az, el] = faceTracker.estimateFaceOrientation();
            
        end

        % Check that any video players we specified are still open.
        runLoop = (showAnnotatedVid && isOpen(videoPlayerAnnotated)) || ...
                  (showMaskVid && isOpen(videoPlayerMask));
    end
catch EX
    clean();
    rethrow(EX);
end
clean();

function [] = clean()
    if(exist('videoPlayerAnnotated', 'var'))
        release(videoPlayerAnnotated);
    end 
    if(exist('videoPlayerMask', 'var'))
        release(videoPlayerMask);
    end    
    clear cam;
end