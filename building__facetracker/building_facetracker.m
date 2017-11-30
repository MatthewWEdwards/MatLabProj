close all
clear all

try
    % Create the webcam object.
    cam = webcam();
    
    % Capture one frame to get its size.
    videoFrame = snapshot(cam);
    frameSize = size(videoFrame);
    
    runLoop = true;
    frameCount = 0;
    moustacheMode = true;
    
    % Create the face tracker object.
    faceDetector = vision.CascadeObjectDetector('FrontalFaceLBP');
    eyeDetector = vision.CascadeObjectDetector('EyePairSmall','UseROI',true);
    noseDetector = vision.CascadeObjectDetector('Nose','UseROI',true);
    faceTracker = OurFaceTracker(faceDetector, eyeDetector, noseDetector, frameSize);
    
    % initialize mask functionality
    load('mask_im.mat');
    maskRealizer = OurMaskRealizer();
    
    [showL, showR, vidFig] = SideBySideVideo('FaceMask JaviMaCoop', 'Face Data', 'FaceMask Video', ...
        'Toggle Stache Mode', @toggleMoustache);
    
    while runLoop && frameCount < 400
        frameCount = frameCount + 1;
        % Get the next frame.
        videoFrame = snapshot(cam);
        videoFrameGray = rgb2gray(videoFrame);
        
        % track face
        faceTracker = faceTracker.Step(videoFrameGray);
        
        % Display the annotated video frame using the video player object.
        if(faceTracker.hasFace())
            videoFrame_ann = faceTracker.annotateData(videoFrame);
            if(moustacheMode)
                videoFrame_mask = maskRealizer.AddMoustache(videoFrame, faceTracker);
            else
                videoFrame_mask = maskRealizer.AddMask(videoFrame, faceTracker);
            end
        else
            videoFrame_ann = insertText(videoFrame,[550, 380],'No face detected.',...
                'BoxColor', 'Red', 'TextColor', 'White', 'FontSize', 18);
            videoFrame_ann = faceTracker.annotateData(videoFrame);
            videoFrame_mask = insertText(videoFrame,[550, 380],'No face detected.',...
                'BoxColor', 'Red', 'TextColor', 'White', 'FontSize', 18);
        end
        showL(videoFrame_ann);
        showR(videoFrame_mask);
        
        % Check that video-playing figure is still open
        runLoop = numel(ismember(findall(0,'type','figure'),vidFig)) > 0;
    end
catch EX
    clean();
    rethrow(EX);
end
clean();

function toggleMoustache(~,~)
      evalin('base', 'moustacheMode = ~moustacheMode');  
end

function clean()
    % just some hacky MATLAB workspace stuff so that I can
    % clean up without reusing code. Evaluates code in 'base' workspace.
    try
        if(evalin('base', 'exist(''videoPlayerAnnotated'', ''var'')'))
            evalin('base', 'release(videoPlayerAnnotated)');
        end 
    end
    try
        if(evalin('base', 'exist(''videoPlayerMask'', ''var'')'))
            evalin('base', 'release(videoPlayerMask)');
        end 
    end
    try
        evalin('base', 'clear cam');
    end
end