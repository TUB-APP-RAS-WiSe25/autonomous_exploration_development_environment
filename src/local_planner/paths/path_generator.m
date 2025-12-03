clc;
clear all;
close all;

%% generate path
%{.
dis = 1.0; %%tune --> distance between x waypoints (increase; smoother decrease;  more reactive)
angle = 27; %%overall spread of the initial "V-shape"
deltaAngle = angle / 3; %%step size (smaller --> more paths generated)
scale = 0.65; %%reduces spread for deeper paths

pathStartAll = zeros(4, 0);
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

%% Ackermann vehicle limits
wheelbase = 2.6;             % [m] TODO: measure
deltaMax = 30 * pi/180;      % [rad] maximum steering angle TODO: measure
kappaMax = tan(deltaMax) / wheelbase;   % maximum curvature

theta0 = 0;                   % vehicle initial heading along +X ?
maxYawChange = pi/2;          % max allowed heading change relative to current heading (±90°)
ds = 0.05;                    %step size: distance the vehicle moves forward along the path at each step.

function kappa = computeCurvature(x, y)
    dx  = gradient(x);
    dy  = gradient(y);
    ddx = gradient(dx);
    ddy = gradient(dy);
    kappa = abs(dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^(3/2));
end

fprintf('\nGenerating paths\n');

for shift1 = -angle : deltaAngle : angle
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0];
    
    pathStartR = 0 : 0.01 : dis;
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);
    
    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));
    
    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
    pathStartAll = [pathStartAll, pathStart];
    
    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          2 * dis, shift2, 0;
                          3 * dis - 0.001, shift3, 0;
                          3 * dis, shift3, 0];

                pathR = 0 : 0: ds : waypts(end, 1);
                % %pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                % %%prevent turning in place (alway moves forward now)
                % % spline lateral offset instead of heading
                % latOffset = spline(waypts(:,1), waypts(:,2), pathR);


                % % pathX = pathR .* cos(pathShift * pi / 180);
                % % pathY = pathR .* sin(pathShift * pi / 180);

                % %theta = pathShift * pi/180; 
                % % compute heading from geometry
                % dLat = gradient(latOffset, pathR);
                % theta = atan(dLat);     % small-angle Ackermann approximation

                % pathX = zeros(size(theta));
                % %pathY = zeros(size(theta));
                % pathY = latOffset;   % actual lateral offset


                % pathZ = zeros(size(pathX));
                % ds = 0.01; % step size = pathR step

                % for k = 2:length(theta)
                %     dx = pathR(k) - pathR(k-1);
                %     pathX(k) = pathX(k-1) + dx * cos(theta(k));
                %     pathY(k) = pathY(k-1) + dx * sin(theta(k));
                % end

                % %apply curvature check
                % kappa = computeCurvature(pathX, pathY);
                % if max(kappa) > kappaMax
                %     continue; %too sharp for ackemann steering --> skip the path
                % end
                 % Frenet lateral offset spline
            latOffset = spline(waypts(:,1), waypts(:,2), min(pathR, waypts(end,1)));
            dLat = gradient(latOffset, pathR);
            ddLat = gradient(dLat, pathR);
            
            % enforce curvature limits
            scaleFactor = 1.0;
            for iter=1:12
                kappaTest = abs(scaleFactor*ddLat ./ (1+(scaleFactor*dLat).^2).^(3/2));
                if max(kappaTest)<=kappaMax*1.0001
                    break;
                end
                scaleFactor = scaleFactor * 0.75;
            end
            latOffset = latOffset * scaleFactor;
            dLat = dLat * scaleFactor;
            ddLat = ddLat * scaleFactor;
            
            theta = atan(dLat); theta = unwrap(theta);
            kappa = ddLat ./ (1+dLat.^2).^(3/2);
            
         
            % prevent backwards motion
            pathX = zeros(size(theta));
            pathY = zeros(size(theta));
            pathY(1) = latOffset(1);
            for k=2:length(theta)
                dx = ds * cos(theta(k-1));
                if dx <= 0
                    dx = eps; % small positive step to prevent backward motion
                end
                pathX(k) = pathX(k-1) + dx;
                pathY(k) = pathY(k-1) + ds * sin(theta(k-1));
            end
            
            % U-turn (if final x < 0) %%now just drives backwards??
            if pathX(end) < 0
                extendFactorMax = 4; % max extension
                for ef = 2:extendFactorMax
                    newMaxS = ef * waypts(end,1);
                    pathR_new = 0:ds:newMaxS;
                    latOffset_new = spline(waypts(:,1), waypts(:,2), min(pathR_new, waypts(end,1)));
                    dLat_new = gradient(latOffset_new, pathR_new);
                    ddLat_new = gradient(dLat_new, pathR_new);
                    theta_new = atan(dLat_new); theta_new = unwrap(theta_new);

                    % forward-only integration
                    pathX_new = zeros(size(theta_new));
                    pathY_new = zeros(size(theta_new));
                    pathY_new(1) = latOffset_new(1);
                    for k2=2:length(theta_new)
                        dx = ds * cos(theta_new(k2-1));
                        if dx <= 0, dx = eps; end
                        pathX_new(k2) = pathX_new(k2-1) + dx;
                        pathY_new(k2) = pathY_new(k2-1) + ds * sin(theta_new(k2-1));
                    end

                    % check heading & curvature
                    pathAngle = atan2(pathY_new(end), pathX_new(end));
                    angleDiff = mod(pathAngle - theta0 + pi, 2*pi) - pi;
                    kappa_new = ddLat_new ./ (1 + dLat_new.^2).^(3/2);

                    if max(abs(kappa_new)) <= kappaMax*1.0001 && pathX_new(end) >= 0 && abs(angleDiff) <= maxYawChange
                        pathX = pathX_new; pathY = pathY_new; theta = theta_new; kappa = kappa_new;
                        break;
                    end
                end
            end

            % Final curvature check
            if max(abs(kappa)) > kappaMax
                    continue; % skip infeasible paths
            end

                
            % ---------------------------
            % Save path
            % ---------------------------
            pathZ = zeros(size(pathX));
            path = [pathX; pathY; pathZ; ones(size(pathX))*pathID; ones(size(pathX))*groupID];
            pathAll = [pathAll, path];
            pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];
            pathID = pathID + 1;
            
            plot3(pathX, pathY, pathZ);
        end
    end
    
    groupID = groupID + 1
end

pathID

fileID = fopen('startPaths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathStartAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d\n', pathStartAll);
fclose(fileID);

fileID = fopen('paths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathAll);
fclose(fileID);

fileID = fopen('pathList.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathList, 2));
fprintf(fileID, 'property float end_x\n');
fprintf(fileID, 'property float end_y\n');
fprintf(fileID, 'property float end_z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathList);
fclose(fileID);

pause(1.0);
%}

%% find correspondence
%{.
voxelSize = 0.02;
searchRadius = 0.45;
offsetX = 3.2;
offsetY = 4.5;
voxelNumX = 161;
voxelNumY = 451;

fprintf('\nPreparing voxels\n');

indPoint = 1;
voxelPointNum = voxelNumX * voxelNumY;
voxelPoints = zeros(voxelPointNum, 2);
for indX = 0 : voxelNumX - 1
    x = offsetX - voxelSize * indX;
    scaleY = x / offsetX + searchRadius / offsetY * (offsetX - x) / offsetX;
    for indY = 0 : voxelNumY - 1
        y = scaleY * (offsetY - voxelSize * indY);

        voxelPoints(indPoint, 1) = x;
        voxelPoints(indPoint, 2) = y;
        
        indPoint  = indPoint + 1;
    end
end

plot3(voxelPoints(:, 1), voxelPoints(:, 2), zeros(voxelPointNum, 1), 'k.');
pause(1.0);

fprintf('\nCollision checking\n');

[ind, dis] = rangesearch(pathAll(1 : 2, :)', voxelPoints, searchRadius);

fprintf('\nSaving correspondences\n');

fileID = fopen('correspondences.txt', 'w');

for i = 1 : voxelPointNum
    fprintf(fileID, '%d ', i - 1);
    
    indVoxel = sort(ind{i});
    indVoxelNum = size(indVoxel, 2);
    
    pathIndRec = -1;
    for j = 1 : indVoxelNum
        pathInd = pathAll(4, indVoxel(j));
        if pathInd == pathIndRec
            continue;
        end

        fprintf(fileID, '%d ', pathInd);
        pathIndRec = pathInd;
    end
    fprintf(fileID, '-1\n');
    
    if mod(i, 1000) == 0
        i
    end
end

fclose(fileID);

fprintf('\nProcessing complete\n');
%}
