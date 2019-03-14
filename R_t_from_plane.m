function [R,t] = R_t_from_plane(plane)
n = plane(1:3) / norm(plane(1:3));
d = plane(4) / norm(plane(1:3));
if (n(3) < 0)
    n = -n;d = -d;
end
ref = [0;0;1];
if(norm(ref - n) < 0.001)
    R = eye(3);
else
    axis1 = cross(n,ref);
    axis1 = axis1 / norm(axis1);
    axis2 = cross(n,axis1);
    axis2 = axis2 / norm(axis2);
    R = [axis1';axis2';n'];
end
t = [0;0;d];
end

