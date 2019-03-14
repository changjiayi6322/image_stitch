scatter3(xx,yy,zz);
hold on
[X3,Y3,Z3]=griddata(xx,yy,zz,linspace(min(xx),max(xx))',linspace(min(yy),max(yy)),'v4');
%pcolor(X3,Y3,Z3);
%shading interp
%figure,contourf(X3,Y3,Z3)
figure,surf(X3,Y3,Z3)
hold on
scatter3(xx,yy,zz)

tri=delaunay(xx,yy);
figure,trimesh(tri,xx,yy)