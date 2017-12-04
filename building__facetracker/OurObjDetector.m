classdef OurObjDetector
    %OUROBJDETECTOR
    
    properties
        Detector
        PointTracker
        NumPts
        OldPts
        Bbox
        BboxPts
        BboxPolygon
        FrameSize
        TrackThreshold
    end
    
    methods
        function obj = OurObjDetector(cascadeObjDetector, frameSize, trackThreshold)
            obj.Detector = cascadeObjDetector;
            obj.PointTracker = vision.PointTracker('MaxBidirectionalError', 2);
            obj.NumPts = 0;
            obj.FrameSize = frameSize;
            obj.TrackThreshold = trackThreshold;
        end
        
        function obj = Detect(obj, videoFrameGray)           
           if ~isempty(obj.Bbox)
               % Find corner points inside the detected region.
               points = detectMinEigenFeatures(videoFrameGray, 'ROI', obj.Bbox(1, :));
               
               % Re-initialize the point tracker.
               xyPoints = points.Location;
               obj.NumPts = size(xyPoints,1);
               release(obj.PointTracker);
               initialize(obj.PointTracker, xyPoints, videoFrameGray);
               
               % Save a copy of old pts for use in tracking mode
               obj.OldPts = xyPoints;
           end
        end
        
        function obj = DetectRaw(obj, videoFrameGray)
            obj = obj.UpdateBboxData(obj.Detector.step(videoFrameGray));
            obj = obj.Detect(videoFrameGray);
        end
        
        function obj = DetectInBbox(obj, videoFrameGray, bbox)
            if(size(bbox) ~= 0)
                obj = obj.UpdateBboxData(obj.Detector.step(videoFrameGray,bbox(1, :)));
            else
                obj = obj.UpdateBboxData(obj.Detector.step(videoFrameGray, ...
                    [30, 30, obj.FrameSize(2)-30, obj.FrameSize(1)-30]));
            end
            obj = obj.Detect(videoFrameGray);
        end
        
        function obj = Track(obj, videoFrameGray)
            [xyPoints, isFound] = step(obj.PointTracker, videoFrameGray);
            visiblePoints = xyPoints(isFound, :);
            oldInliers = obj.OldPts(isFound, :);
            
            obj.NumPts = size(visiblePoints, 1);
            
            if obj.NumPts >= obj.TrackThreshold
                % Estimate the geometric transformation between the old points
                % and the new points.
                [xform, ~, visiblePoints] = estimateGeometricTransform(...
                    oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);
                
                % Apply the transformation to the bounding box.
                bboxPts = transformPointsForward(xform, obj.BboxPts);
                obj = obj.UpdateBboxPtData(bboxPts);
                
                % Reset the points.
                obj.OldPts = visiblePoints;
                setPoints(obj.PointTracker, obj.OldPts);
            end
        end
        
        function obj = UpdateBboxData(obj, bbox)
            obj.Bbox = bbox;
            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            if(size(bbox) ~= 0)
                bboxPts = bbox2points(obj.Bbox(1, :));
                obj = obj.UpdateBboxPtData(bboxPts);  
            end                        
        end
        
        function obj = UpdateBboxPtData(obj, bboxPts)
            obj.BboxPts = bboxPts;
            
            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            obj.BboxPolygon = reshape(obj.BboxPts', 1, []);
        end
        
        function bool = hasObj(obj)
            bool = obj.NumPts >= obj.TrackThreshold;
        end
        
        function center = BboxCenter(obj)
            [geom,~,~] = polygeom(obj.BboxPts(:,1), obj.BboxPts(:,2));
            center = fliplr(geom(2:3)); % x coord is row (2nd dim), etc
        end
        
        function obj = Reset(obj)
            obj.Bbox = [];
            obj.BboxPts = [];
            obj.NumPts = 0;
        end
    end
    
end

