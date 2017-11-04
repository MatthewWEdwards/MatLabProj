error_mes = python('matt.jpg');
read_vertices_and_faces_from_obj_file('downloads/a14b922f8c771675dcd5fda8746c8e1d.obj');
trisurf(F,V(:,1),V(:,2),V(:,3),'FaceColor',[0.26,0.33,1.0 ]);
light('Position',[-1.0,-1.0,100.0],'Style','infinite');
lighting phong;