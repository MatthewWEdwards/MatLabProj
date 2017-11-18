close all
clear

error_mes = python('get_obj_from_image.py');
[V,C,F] = read_vertices_and_faces_from_obj_file('downloads/a14b922f8c771675dcd5fda8746c8e1d.obj');
trisurf(F,V(:,1),V(:,2),V(:,3),'FaceVertexCData', C);
light('Position',[-1.0,-1.0,100.0],'Style','infinite');
lighting phong;
