function [ image ] = obj_to_im( path_to_obj, height, width, az, el)
%Takes an object file and attempts to produce an image
%   path_to_obj: File path to object file
%   height: Output Image height
%   width: Output Image Width
%   az: azimuth of the face object
%   el: elevation of the face object

    % Get polygon-face information
    [V,C,F] = read_vertices_and_faces_from_obj_file(path_to_obj);
    A = viewmtx(az, el);
    vdim4 = [V ones(size(V,1), 1)];
    vdim4 = vdim4';
    viewV = A*vdim4;
    V = viewV';
    sz_verticies = size(V);
    sz_faces = size(F);
    
    % Set up mesh-to-image mapping variables
    max_x = max(max(V(:,1)));
    max_y = max(max(V(:,2)));
    min_x = min(min(V(:,1)));
    min_y = min(min(V(:,2)));
    delta_x = (max_x-min_x) / (height-1);
    delta_y = (max_y-min_y) / (width-1);
    
    % Assign each vertex an pixel in the image
    vector_indicies = zeros(sz_verticies(1), 2);
    for vec = 1:sz_verticies(1)
        vector_indicies(vec, 1) = uint16((V(vec, 1)-min_x)/delta_x);
        vector_indicies(vec, 2) = uint16((V(vec, 2)-min_y)/delta_y);
    end
    vector_indicies = vector_indicies+1;

    % Assign each pixel values based on the corresponding faces
    image = zeros(height, width, 3);
    for face = 1:sz_faces(1)
        
       % Get region of interest for this face
       x_vec = double([vector_indicies(F(face, 1), 1); vector_indicies(F(face, 2),1); vector_indicies(F(face, 3), 1)]);
       y_vec = double([vector_indicies(F(face, 1), 2); vector_indicies(F(face, 2),2); vector_indicies(F(face, 3), 2)]);
       [min_x, argmin] = min(x_vec);
       x_vec(argmin) = min_x-1;
       [max_x, argmax] = max(x_vec);
       x_vec(argmax) = max_x+1;
       [min_y, argmin] = min(y_vec);
       y_vec(argmin) = min_y-1;
       [max_y, argmax] = max(y_vec);
       y_vec(argmax) = max_y+1;       
       
       % Replace ROI with the average of the current image content place
       % the content of the face
       mask = poly2mask(x_vec, y_vec, height, width);
       color = (C(F(face,1), :) + C(F(face, 2), :) + C(F(face, 3), :))/3;
       masked_image = image .* mask;
       masked_image(:,:,1) = (masked_image(:,:,1) + (mask * color(1)))/2;
       masked_image(:,:,2) = (masked_image(:,:,2) + (mask * color(2)))/2;
       masked_image(:,:,3) = (masked_image(:,:,3) + (mask * color(3)))/2;     
       image = (image .* ~mask) + masked_image; 
    end
end



