function [Rnew,tnew,point3dnew] = optimizePosePoint(R,t,camera_matrix,point1,point2,point3d)
point_num = size(point3d,2);
iter_num = 50000;
count = 1;
errors = zeros(1,iter_num);
lamda = zeros(1,iter_num);
lamda(count) = 1000000;
I = eye(6+3*point_num);
errors(count) = inf;
while errors(count) > 0.0001 && count <= (iter_num - 1)   
    [H,b,errors(count+1)] = calError(R,t,camera_matrix,point1,point2,point3d);
    fprintf('count = %d,current error is %f\n',count,errors(count+1));
	if (errors(count+1) >= errors(count))
        break;
    else
        Rnew = R;tnew = t;point3dnew = point3d;
        lamda(count+1) = lamda(count) / 1.5;  
        param = (H + lamda(count) * I) \ b;
        [R,t,point3d]=update_param(Rnew,tnew,point3dnew,param);
    end
    count = count + 1;
end
end
function [R_new,t_new,point3d_new]=update_param(R,t,point3d,param)
point_num = size(point3d,2);
matrix_new = se3Exp(param((3*point_num+1):(3*point_num+6))) * [R,t;0,0,0,1];
R_new = matrix_new(1:3,1:3);
R_new = (R_new * R_new')^(-0.5)*R_new;
t_new = matrix_new(1:3,4);
point3d_new = point3d + reshape(param(1:(3*point_num)),[3,point_num]);
end
function [H,b,error] = calError(R,t,camera_matrix,point1,point2,point3d)
point_num = size(point1,2);
point3d2=R*point3d + t;
reporj_point2 = camera_matrix*point3d2;
error2=reporj_point2(1:2,:) ./ reporj_point2(3,:)-point2;
reporj_point1 = camera_matrix*point3d;
error1 = reporj_point1(1:2,:) ./ reporj_point1(3,:) - point1;
H=zeros(6+3*point_num,6+3*point_num);
b=zeros(6+3*point_num,2);
error=0;
for i = 1:point_num
    err1_diff_point=-[camera_matrix(1,1)/point3d(3,i),0,-camera_matrix(1,1)*point3d(1,i)/(point3d(3,i)^2)
                   0,camera_matrix(2,2)/point3d(3,i),-camera_matrix(2,2)*point3d(2,i)/(point3d(3,i)^2)];
    err2_diff_point2=-[camera_matrix(1,1)/point3d2(3,i),0,-camera_matrix(1,1)*point3d2(1,i)/(point3d2(3,i)^2)
                   0,camera_matrix(2,2)/point3d2(3,i),-camera_matrix(2,2)*point3d2(2,i)/(point3d2(3,i)^2)];
    err2_diff_point = err2_diff_point2*R;
    err2_diff_pose=err2_diff_point2*[eye(3),-my_hat(point3d2)];   
    H((3*i-2):(3*i),(3*i-2):(3*i))=H((3*i-2):(3*i),(3*i-2):(3*i))+err2_diff_point'*err2_diff_point+err1_diff_point'*err1_diff_point;
    H((3*i-2):(3*i),(3*point_num+1):(3*point_num+6))=H((3*i-2):(3*i),(3*point_num+1):(3*point_num+6))+err2_diff_point'*err2_diff_pose;
    H((3*point_num+1):(3*point_num+6),(3*i-2):(3*i))=H((3*point_num+1):(3*point_num+6),(3*i-2):(3*i))+err2_diff_pose'*err2_diff_point;
    H((3*point_num+1):(3*point_num+6),(3*point_num+1):(3*point_num+6))=H((3*point_num+1):(3*point_num+6),(3*point_num+1):(3*point_num+6))+err2_diff_pose'*err2_diff_pose;
    b((3*i-2):(3*i),:)=b((3*i-2):(3*i),:)+err1_diff_point'*error1(:,i)+err2_diff_point'*error2(:,i);
    b((3*point_num+1):(3*point_num+6),:)=b((3*point_num+1):(3*point_num+6),:)+err2_diff_pose'*error2(:,i);
    error = error+error1(:,i)'*error1(:,i)+error2(:,i)'*error2(:,i);
end
end

