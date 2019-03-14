function [R,t,point3Ds] = recover3DHomographyMat(point1,point2)
[ dat_norm_img1,T1 ] = normalise2dpts(point1);
[ dat_norm_img2,T2 ] = normalise2dpts(point2);
[H, ~] = cv.findHomography(dat_norm_img1(1:2,:)',dat_norm_img2(1:2,:)','Method','Ransac','RansacReprojThreshold',0.1,'MaxIters',100);
Hg = T2 \ H * T1;
[motions,nmotions] = cv.decomposeHomographyMat(Hg,camera_matrix);

end
function [ret,inliers] = check_motion(R,t,camera_matrix,point1,point2)
points4D1 = cv.triangulatePoints(eye(3,4), [R,t], point1, point2);
num = size(point1,1);
inliers = ones(1,size(point1,1));
for i = 1:size(points4D1,1)
    point3d = points4D1(:,i) / points4D1(4,i);
    if(point3d(3) < 0)
        
    end
end
points4D2 = [R,t;zeros(1,3),1] * 
end