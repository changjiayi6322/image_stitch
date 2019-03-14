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
%img1 = undistort_img1(20:end-40,20:end-40,:);
%img2 = undistort_img2(20:end-40,20:end-40,:);
img1 = undistort_img1;
img2 = undistort_img2;
[height,width,~] = size(img1);
%特征点检测和匹配
[ kp1,ds1 ] = vl_sift(single(rgb2gray(img1)),'PeakThresh', 0,'edgethresh',500);
[ kp2,ds2 ] = vl_sift(single(rgb2gray(img2)),'PeakThresh', 0,'edgethresh',500); 
matches = vl_ubcmatch(ds1,ds2);
%得到的匹配特征点
data_orig = [ kp1(1:2,matches(1,:)) ; ones(1,size(matches,2)) ; kp2(1:2,matches(2,:)) ; ones(1,size(matches,2)) ];
point1 = data_orig(1:2,:);
point2 = data_orig(4:5,:);
[E,mask] = cv.findEssentialMat(point1',point2', 'CameraMatrix',camera_matrix,'Method','Ransac','Threshold',1.0,'Confidence',0.999);
inliers = find(mask ~= 0);
point1 = point1(:,inliers);
point2 = point2(:,inliers);
[R, t, good,mask,point4Ds] = cv.recoverPose(E, point1',point2','CameraMatrix',camera_matrix);
point3Ds = zeros(3,good);
good_inds = find(mask ~= 0);
for j = 1:good
    point3Ds(:,j) = point4Ds(1:3,good_inds(j)) / point4Ds(4,good_inds(j));
end
point1 = point1(:,good_inds);
point2 = point2(:,good_inds);
[Rnew,tnew,point3dnew] = optimizePosePoint(R,t,camera_matrix,point1,point2,point3Ds);

mat = [point3dnew',ones(good,1)];
[U,S,V] = svd(mat);
plane = V(:,4);
[Rplane,tplane] = R_t_from_plane(plane);
point3DRts = Rplane * point3dnew + tplane;
% outlier_ind = find(abs(point3DRts(3,:)) > 1);
% point1_outlier = point1(:,outlier_ind);
% point2_outlier = point2(:,outlier_ind);
% figure;
% imshow([img1 img2]);
% hold on;
% plot(point1_outlier(1,:),point1_outlier(2,:),'ro','LineWidth',2);
% plot(point2_outlier(1,:)+size(img1,2),point2_outlier(2,:),'ro','LineWidth',2);
% for i=1:length(outlier_ind)
%    plot([point1_outlier(1,i) point2_outlier(1,i)+size(img1,2)],[point1_outlier(2,i) point2_outlier(2,i)],'g-');
% end
% title('Ransac''s results');
% dist2line = (plane(1:3)' * point3dnew + plane(4)) / norm(plane(1:3));
Tplane = [Rplane,tplane;0,0,0,1];

invT1 = inv(Tplane);
H1 = camera_matrix * invT1(1:3,[1,2,4]);
invT2 = [Rnew,tnew;0,0,0,1] / Tplane;
H2 = camera_matrix * invT2(1:3,[1,2,4]);
px1=[0;0;1];
px2=[0;height-1;1];
px3=[width-1;0;1];
px4=[width-1;height-1;1];
warp_homo1 = H1 \ [px1,px2,px3,px4];
warp_homo2 = H2 \ [px1,px2,px3,px4];
warp_homo = [warp_homo1,warp_homo2];
warppx = warp_homo(1:2,:) ./ warp_homo(3,:);
minp = min(warppx,[],2);
maxp = max(warppx,[],2);
xx = point3DRts(1,:);
yy = point3DRts(2,:);
zz = point3DRts(3,:);

% pixel = 1500;
% scale = pixel / (maxp(1) - minp(1));
% img_warp1 = uint8(zeros(pixel,pixel,3));
% for i = 0:(pixel-1)
%     for j = 0:(pixel-1)
%         point_plane = [i / scale + minp(1);j / scale + minp(2);1];
%         warp_point1 = H1 * point_plane;
%         %warp_point2 = H2 * point_plane;
%         pixel1 = int32(warp_point1(1:2) / warp_point1(3));
%         %pixel2 = warp_point2(1:2) / warp_point2(3);
%         if pixel1(1) > 0 && pixel1(1) < width - 1 && pixel1(2) > 0 && pixel1(2) < height - 1
%             img_warp1(j+1,i+1,:) = img1(pixel1(2)+1,pixel1(1)+1,:);
%         end
%     end
% end
% img_warp2 = uint8(zeros(pixel,pixel,3));
% for i = 0:(pixel-1)
%     for j = 0:(pixel-1)
%         point_plane = [i / scale + minp(1);j / scale + minp(2);1];
%         warp_point1 = H2 * point_plane;
%         %warp_point2 = H2 * point_plane;
%         pixel1 = int32(warp_point1(1:2) / warp_point1(3));
%         %pixel2 = warp_point2(1:2) / warp_point2(3);
%         if pixel1(1) > 0 && pixel1(1) < width - 1 && pixel1(2) > 0 && pixel1(2) < height - 1
%             img_warp2(j+1,i+1,:) = img2(pixel1(2)+1,pixel1(1)+1,:);
%         end
%     end
% end
% 
% img_warp = (img_warp1 + img_warp2) / 2;    





