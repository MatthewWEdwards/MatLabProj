classdef OurFaceTracker
    %OURFACETRACKER
    
    properties
        FaceDetector
        EyesDetector
        NoseDetector
        FrameSize
        DetectOrTrackThreshold
    end
    
    methods
        function obj = OurFaceTracker(faceCascDetector, eyesCascDetector, noseCascDetector, frameSize)
            obj.FrameSize = frameSize;
            obj.FaceDetector = OurObjDetector(faceCascDetector, frameSize, 10);
            obj.EyesDetector = OurObjDetector(eyesCascDetector, frameSize, 10);
            obj.NoseDetector = OurObjDetector(noseCascDetector, frameSize, 10);
            
            obj.DetectOrTrackThreshold = 10;
        end
        
        function obj = Step(obj, videoFrameGray)
            if(obj.FaceDetector.NumPts < obj.DetectOrTrackThreshold)
               obj.FaceDetector = obj.FaceDetector.DetectRaw(videoFrameGray); 
            else
                obj.FaceDetector = obj.FaceDetector.Track(videoFrameGray);
            end
            
            if(obj.EyesDetector.NumPts < obj.DetectOrTrackThreshold)
               obj.EyesDetector = obj.EyesDetector.DetectInBbox(...
                   videoFrameGray, obj.FaceDetector.Bbox); 
            else
                obj.EyesDetector = obj.EyesDetector.Track(videoFrameGray);
            end
            
            if(obj.NoseDetector.NumPts < obj.DetectOrTrackThreshold)
                obj.NoseDetector = obj.NoseDetector.DetectInBbox(...
                    videoFrameGray, obj.FaceDetector.Bbox);
            else
                obj.NoseDetector = obj.NoseDetector.Track(videoFrameGray);
            end
        end
        
        function videoFrame = annotateData(obj, videoFrame)
            try
            videoFrame = obj.annotateDetectorData(videoFrame, obj.FaceDetector, 'white', 'Face');
            end
            try
            videoFrame = obj.annotateDetectorData(videoFrame, obj.EyesDetector, 'blue', 'Eyes');
            end
            try
            videoFrame = obj.annotateDetectorData(videoFrame, obj.NoseDetector, 'white', 'Nose');
            end
        end
        
        function videoFrame = annotateDetectorData(~, videoFrame, detector, ptClr, label)
            %videoFrame = insertObjectAnnotation(videoFrame, 'rectangle', detector.Bbox, label);
            videoFrame = insertShape(videoFrame, 'Polygon', ...
                detector.BboxPolygon, 'LineWidth', 3);
            videoFrame = insertMarker(videoFrame, ...
                detector.OldPts, '+', 'Color', ptClr);
        end
        
        function [azimuth, elevation] = EstimateFaceOrientation(obj)
            bboxPolygonEyes = obj.EyesDetector.BboxPolygon;
            bboxPolygonNose = obj.NoseDetector.BboxPolygon;
            if(size(bboxPolygonEyes,2) == 8 && size(bboxPolygonNose,2) == 8)
                xEyesAvg = (bboxPolygonEyes(1) + bboxPolygonEyes(3) + bboxPolygonEyes(5) + bboxPolygonEyes(7)) / 4;
                yEyesAvg = (bboxPolygonEyes(2) + bboxPolygonEyes(4) + bboxPolygonEyes(6) + bboxPolygonEyes(8)) / 4;
                
                xNoseAvg = (bboxPolygonNose(1) + bboxPolygonNose(3) + bboxPolygonNose(5) + bboxPolygonNose(7)) / 4;
                yNoseAvg = (bboxPolygonNose(2) + bboxPolygonNose(4) + bboxPolygonNose(6) + bboxPolygonNose(8)) / 4;
                azimuth = atan((xNoseAvg - xEyesAvg)/(yNoseAvg - yEyesAvg));
                elevation = atan((yNoseAvg - yEyesAvg)/(10));
            end
        end
        
        function bool = hasFace(obj)
            bool = obj.FaceDetector.hasObj() && ...
                   obj.EyesDetector.hasObj() && ...
                   obj.NoseDetector.hasObj();
        end
        
        function handlebarMountLoc = HandlebarLoc(obj)
            noseCenter = obj.NoseDetector.BboxCenter();
            eyesCenter = obj.EyesDetector.BboxCenter();
            eyeNoseVec = noseCenter - eyesCenter; 
            mag = sqrt(sum(eyeNoseVec.^2));
            eyeNoseVecNorm = eyeNoseVec/mag;
            % locate philtrum down eye-nose line from noseCenter
            handlebarMountLoc = noseCenter + 24*eyeNoseVecNorm;
        end
        
        function obj = Reset(obj)
           obj.FaceDetector = obj.FaceDetector.Reset();
           obj.EyesDetector = obj.EyesDetector.Reset();
           obj.NoseDetector = obj.NoseDetector.Reset();
        end
    end
    
end

