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
        
        
    end
    
end

