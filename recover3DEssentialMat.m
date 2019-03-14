function [R,t,point3Ds] = recover3DEssentialMat(point1,point2)
[E,~] = cv.findEssentialMat(point1,point2, 'CameraMatrix',camera_matrix,'Method','Ransac','Threshold',0.5,'Confidence',0.999);
[R, t, ~,~,point3Ds] = cv.recoverPose(E, point1,point2,'CameraMatrix',camera_matrix);
end

