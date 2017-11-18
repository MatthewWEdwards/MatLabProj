function [ patch_data ] = patch_angle( V, C, F, az, el)
%PATCH_ANGLE calls patch with rotated V
A = viewmtx(az, el);
vdim4 = [V ones(size(V,1), 1)];
vdim4 = vdim4';
viewV = A*vdim4;
viewV = viewV';
patch_data = patch('Faces', F, 'Vertices', viewV(:,1:2),...
    'FaceVertexCData', C, 'FaceColor', 'flat', 'edgecolor','none');
end

