close all;
clear;

% For plotting specific scene
use_target_scene = true;
target_scene = 25;

% Read extrinsics file
extrinsics = csvread('parsedOutput.csv');

% Loop over images
all_points = [];
for i=1:50
    % Read file
    filename = ['habitat-img/test.depth.' num2str(i,'%05d') '.png'];
    im = im2double(imread(filename));
   
    % Modifying camera pos and quaternion to work with our code
    modifiedCameraPos = [extrinsics(i,3); extrinsics(i,2); extrinsics(i,1)]';
    modifiedQuaternion = [extrinsics(i,4); (-1)*extrinsics(i,7); extrinsics(i,6); (-1)*extrinsics(i,5)]';
    
    % call image2points
    [points, projMat, cameraSpaceCoords] = ...
        image2points(im, 20, modifiedQuaternion, modifiedCameraPos);

    % Combine results into 1 big 3*n array
    all_points = [all_points points];
    
    % Saving target image
    if (use_target_scene && i == target_scene)
        target_im = im;
        target_points = points;
        target_projMat = projMat;
        target_cameraSpaceCoords = cameraSpaceCoords;
    end
end

% Plot all samples
figure(1)
hold on;
scatter3(all_points(1,:), all_points(3,:), all_points(2,:), '.b');
if (use_target_scene)
    scatter3(target_points(1,:), target_points(3,:), target_points(2,:), '.m');
end
axis equal;
title('All sample points, world coord')
xlabel('x'); ylabel('z'); zlabel('y');
if (use_target_scene)
    legend('all points', ['scene ' num2str(target_scene)]);
else
    legend('all points');
end

% Plot target image
if (use_target_scene)
    figure(2)
    imshow(target_im);
    title(['image ' num2str(target_scene)]);
    
    figure(3)
    scatter3(target_points(1,:), target_points(3,:), target_points(2,:), '.m');
    axis equal;
    title(['scene ' num2str(target_scene) ', world coord']);
    xlabel('x'); ylabel('z'); zlabel('y');

    figure(4)
    cameraCart = hom2cart(target_cameraSpaceCoords')';
    scatter3(cameraCart(1,:), cameraCart(3,:), cameraCart(2,:), '.b');
    axis equal;
    title(['scene ' num2str(target_scene) ', camera coord']);
    xlabel('x'); ylabel('z'); zlabel('y');
end
