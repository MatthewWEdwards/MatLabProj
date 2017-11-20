close all
clear

error_mes = python('get_obj_from_image.py');
[V,C,F] = read_vertices_and_faces_from_obj_file('downloads/matt.obj');
trisurf(F,V(:,1),V(:,2),V(:,3),'FaceVertexCData', C, 'edgecolor','none');
 


figure;
az = 0; el = -40;
A = viewmtx(az, el);
vdim4 = [V ones(size(V,1), 1)];
vdim4 = vdim4';
viewV = A*vdim4;
viewV = viewV';
p = patch('Faces', F, 'Vertices', viewV(:,1:2),'FaceVertexCData', C,...
    'FaceColor', 'flat', 'edgecolor','none');
