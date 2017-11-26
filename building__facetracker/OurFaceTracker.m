classdef OurFaceTracker
    %OURFACETRACKER
    
    properties
        FaceDetector
        EyesDetector
        NoseDetector
        DetectOrTrackThreshold
    end
    
    methods
        function obj = OurFaceTracker(faceCascDetector, eyesCascDetector, noseCascDetector, frameSize)
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
            videoFrame = obj.annotateDetectorData(videoFrame, obj.FaceDetector, 'white');
            videoFrame = obj.annotateDetectorData(videoFrame, obj.EyesDetector, 'blue');
            videoFrame = obj.annotateDetectorData(videoFrame, obj.NoseDetector, 'white');
        end
        
        function videoFrame = annotateDetectorData(~, videoFrame, detector, ptClr)
            videoFrame = insertShape(videoFrame, 'Polygon', ...
                detector.BboxPolygon, 'LineWidth', 3);
            videoFrame = insertMarker(videoFrame, ...
                detector.OldPts, '+', 'Color', ptClr);
        end
        
        function [azimuth, elevation] = estimateFaceOrientation(obj)
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
    end
    
end

