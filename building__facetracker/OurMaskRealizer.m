classdef OurMaskRealizer
    %OURMASKREALIZER 
    
    properties
        FaceTracker
        MaskMapAz
    end
    
    methods
        function obj = OurMaskRealizer(faceTracker)
            obj.FaceTracker = faceTracker;
            
            load('MaskMap_Azimuth.mat');
            obj.MaskMapAz = MaskMap;
        end
        
        function obj = UpdateMask(obj)
            
        end
        
        function maskIm = maskImFromOrient(az)
            % Retrieve image based on orientation of the face
            sampled_azimuth = int8((180.0*az/pi)/1)*1;
            if sampled_azimuth > 50 
               sampled_azimuth =  50;
            end
            if sampled_azimuth < -50
               sampled_azimuth = -50;
            end
            mask = obj.MaskMapAz(sampled_azimuth);
        end
end

