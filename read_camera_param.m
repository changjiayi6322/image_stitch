function [camera_matrix,dist_coff] = read_camera_param(filename)
s = cv.FileStorage(filename);
camera_matrix = [getfield(s,'Camera.fx'),0,getfield(s,'Camera.cx');0,getfield(s,'Camera.fy'),getfield(s,'Camera.cy');0,0,1];
dist_coff = [getfield(s,'Camera.k1'),getfield(s,'Camera.k2'),getfield(s,'Camera.p1'),getfield(s,'Camera.p2'),getfield(s,'Camera.k3')];
end

