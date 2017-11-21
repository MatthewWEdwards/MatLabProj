function [ image ] = obj_to_im( path_to_obj, height, width )
%Takes an object file and attempts to produce an image
%   path_to_obj: File path to object file
%   height: Output Image height
%   width: Output Image Width

    [V,C,~] = read_vertices_and_faces_from_obj_file(path_to_obj);
    sz_verticies = size(V);
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

    % Assign each pixel values based on the corresponding verticies
    image = zeros(height, width, 3);
    for vec = 1:sz_verticies(1)
        image_x = vector_indicies(vec, 1);
        image_y = vector_indicies(vec, 2);
        colors = reshape(C(vec, :), 1, 1, 3);
        if image(image_x, image_y) == 0
            image(image_x, image_y, :) = colors;
        else
            image(image_x, image_y, :) = (image(image_x, image_y, :) + colors)/2;
        end
    end
end



