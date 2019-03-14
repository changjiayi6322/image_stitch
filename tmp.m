cd vlfeat-0.9.14/toolbox;
feval('vl_setup');
cd ../..;
%读取图像并矫正
scale = 1;
src_img1 = imresize(cv.imread('stitch/4.png'),scale);
src_img2 = imresize(cv.imread('stitch/5.png'),scale);
[camera_matrix,dist_coff] = read_camera_param('TUM1.yaml');
undistort_img1 = cv.undistort(src_img1,camera_matrix,dist_coff);
undistort_img2 = cv.undistort(src_img2,camera_matrix,dist_coff);
img1 = undistort_img1(20:end-40,20:end-40,:);
img2 = undistort_img2(20:end-40,20:end-40,:);
[height,width,~] = size(img1);
%特征点检测和匹配
[ kp1,ds1 ] = vl_sift(single(rgb2gray(img1)),'PeakThresh', 0,'edgethresh',500);
[ kp2,ds2 ] = vl_sift(single(rgb2gray(img2)),'PeakThresh', 0,'edgethresh',500); 
matches = vl_ubcmatch(ds1,ds2);
%得到的匹配特征点
data_orig = [ kp1(1:2,matches(1,:)) ; ones(1,size(matches,2)) ; kp2(1:2,matches(2,:)) ; ones(1,size(matches,2)) ];
% Show results of RANSAC.
% figure;
% imshow([img1 img2]);
% hold on;
% plot(kp1(1,:),kp1(2,:),'ro','LineWidth',2);
% plot(kp2(1,:)+size(img1,2),kp2(2,:),'ro','LineWidth',2);
% for i=1:length(inliers)
%   plot(data_orig(1,inliers(i)),data_orig(2,inliers(i)),'go','LineWidth',2);
%   plot(data_orig(4,inliers(i))+size(img1,2),data_orig(5,inliers(i)),'go','LineWidth',2);
%   plot([data_orig(1,inliers(i)) data_orig(4,inliers(i))+size(img1,2)],[data_orig(2,inliers(i)) data_orig(5,inliers(i))],'g-');
% end
% title('Ransac''s results');
% point1 = data_orig(1:3,:);
% point2 = data_orig(4:6,:);
% [ dat_norm_img1,T1 ] = normalise2dpts(point1);
% [ dat_norm_img2,T2 ] = normalise2dpts(point2);
% [H, mask] = cv.findHomography(dat_norm_img1(1:2,:)',dat_norm_img2(1:2,:)','Method','Ransac','RansacReprojThreshold',0.1,'MaxIters',100);
% Hg = T2 \ H * T1;
% [motions,nmotions] = cv.decomposeHomographyMat(Hg,camera_matrix);
% inliers = find(mask == 1);
% num_inliers = length(inliers);
% src_points = cell(1,num_inliers);
% dst_points = cell(1,num_inliers);
% for i = 1:num_inliers
%    src_points{i} = point1(1:2,i)';
%    dst_points{i} = point2(1:2,i)';
% end
% points3Ds = cell(1,nmotions);
% for i = 1:nmotions
%     points4D1 = cv.triangulatePoints(camera_matrix * eye(3,4), camera_matrix * [motions.R{i},motions.t{i}], src_points, dst_points);  
%     points3Ds{i} = zeros(3,num_inliers);
%     for j = 1:num_inliers
%         points3Ds{i}(:,j) = points4D1(1:3,j) / points4D1(4,j);
%     end
% end


