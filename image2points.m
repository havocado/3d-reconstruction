% input: grayscale image in double, 
%       sampling stepSize (size 1: all pixels in image, sparse samples), 
%       quaternion (array of 4 elements), 
%       camera position (array of 3 elements)
% output: resulting points in world coord,
%       projection matrix (for debugging),
%       camera space Coords (for debugging)
function [result, projMat, cameraSpaceCoords] = image2points(im, stepSize, quat, cameraPos)
    % Intrinsic parameters
    near = 0.01; far = 1000.0; 
    [h, w] = size(im);
    hfov = 90*(2*pi/360);
    s_x = 2 * near * tan(hfov/2);
    s_y = s_x * h/w;
    r = s_x/2;
    t = s_y/2;
    
    % Downsampling using stepSize parameter
    count = 0;
    x_n = []; y_n = []; z_n = [];
    for i=1:stepSize:w
        for j=1:stepSize:h
            if (im(j, i) < 0.2 || im(j, i) > 0.8), continue; end
            count = count + 1;
            x_n(count) = (i - w/2)/w;
            y_n(count) = ((-1)*j + h/2)/h;
            z_n(count) = im(j, i);
        end
    end
    % NDC coordinates
    NDC = [x_n; y_n; z_n];

    % A and B components used in Projection Matrix
    A_comp = (far)/(far-near);
    B_comp = near*far/(far-near)*(-1);

    % NDC mapped to frustum - near plane
    w_n_prime = z_n + A_comp;
    w_n_prime = 1./w_n_prime;
    w_n_prime = w_n_prime * B_comp;
    % 2. Plane coordinates
    planeProj = [x_n.*w_n_prime; y_n.*w_n_prime; z_n.*w_n_prime; w_n_prime];

    % Inverse projection
    projMat = [(near/r), 0,0,0; ...
        0, (near/t), 0,0; ...
        0,0, A_comp, B_comp; ...
        0,0,1,0];
    
    cameraSpaceCoords = projMat\planeProj;
    
    % Revert rotation
    % building rotation matrix
    quatVector = quatnormalize(quat);
    imquat = quaternion(quatVector);
    rotationMat = rotmat(imquat, 'point'); % 3x3
    rotationMat = [rotationMat, zeros(3,1); zeros(1,3), 1]; % 4x4
    
    % get coord before rotation
    beforeRotCoords = rotationMat\cameraSpaceCoords;

    % Revert translation
    moveMat = eye(4);
    moveMat(1, 4) = cameraPos(1);
    moveMat(2, 4) = cameraPos(2);
    moveMat(3, 4) = cameraPos(3);

    % get coord before translation
    beforeMove = moveMat\beforeRotCoords;
    
    % Convert final result to cartesian coord 
    result = hom2cart(beforeMove')';
end