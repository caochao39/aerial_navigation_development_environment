clc;
clear all;
close all;

%% generate path
%{.
dis = 2;
horiAngle = 27.0;
deltaHoriAngle = horiAngle / 2;
vertAngle = 9.0;
deltaVertAngle = vertAngle / 1;
scale = 0.65;

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

fprintf('\nGenerating paths\n');

for shift11 = -horiAngle : deltaHoriAngle : horiAngle
    for shift12 = -2 * vertAngle : deltaVertAngle : 2 * vertAngle
        wayptsStart = [0, 0, shift12;
                       dis, shift11, shift12];

        pathStartR = 0 : 0.1 : dis;
        pathStartShiftHori = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);
        pathStartShiftVert = spline(wayptsStart(:, 1), wayptsStart(:, 3), pathStartR);

        pathStartX = pathStartR .* cos(pathStartShiftHori * pi / 180);
        pathStartY = pathStartR .* sin(pathStartShiftHori * pi / 180);
        pathStartZ = pathStartR .* sin(pathStartShiftVert * pi / 180);

        pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
        pathStartAll = [pathStartAll, pathStart];

        for shift21 = -horiAngle * scale + shift11 : deltaHoriAngle * scale : horiAngle * scale + shift11
            for shift22 = -vertAngle * scale + shift12 : deltaVertAngle * scale : vertAngle * scale + shift12
                for shift31 = -horiAngle * scale^2 + shift21 : deltaHoriAngle * scale^2 : horiAngle * scale^2 + shift21
                    for shift32 = -vertAngle * scale^2 + shift22 : deltaVertAngle * scale^2 : vertAngle * scale^2 + shift22
                        if abs(shift32) > 2 * vertAngle 
                            continue;
                        end
                        
                        waypts = [pathStartR', pathStartShiftHori', pathStartShiftVert';
                                  2 * dis, shift21, shift22;
                                  3 * dis - 0.001, shift31, shift32;
                                  3 * dis, shift31, shift32];

                        pathR = 0 : 0.1 : waypts(end, 1);
                        pathShiftHori = spline(waypts(:, 1), waypts(:, 2), pathR);
                        pathShiftVert = spline(waypts(:, 1), waypts(:, 3), pathR);

                        pathX = pathR .* cos(pathShiftHori * pi / 180);
                        pathY = pathR .* sin(pathShiftHori * pi / 180);
                        pathZ = pathR .* sin(pathShiftVert * pi / 180);

                        path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                        pathAll = [pathAll, path];
                        pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];

                        pathID = pathID + 1;

                        plot3(pathX, pathY, pathZ);
                    end
                end
            end
        end
        
        groupID = groupID + 1
    end
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
voxelSize = 0.2;
searchRadiusHori = 1.2;
searchRadiusVert = 0.8;
sensorOffset = 0;
offsetX = 6.4;
offsetY = 9.0;
offsetZ = 3.3;
voxelNumX = 33;
voxelNumY = 91;
voxelNumZ = 34;
batchNum = 20;

searchScale = searchRadiusHori / searchRadiusVert;
pathAll(3, :) = searchScale * pathAll(3, :);

fprintf('\nPreparing voxels\n');

indPoint = 1;
voxelPointNum = voxelNumX * voxelNumY * voxelNumZ;
voxelPoints = zeros(voxelPointNum, 3);
for indX = 0 : voxelNumX - 1
    x = offsetX - voxelSize * indX
    scaleY = x / offsetX + searchRadiusHori / offsetY * (offsetX - x) / offsetX;
    scaleZ = x / offsetX + searchRadiusVert / offsetZ * (offsetX - x) / offsetX;
    for indY = 0 : voxelNumY - 1
        y = scaleY * (offsetY - voxelSize * indY);
        for indZ = 0 : voxelNumZ - 1
            z = searchScale * (scaleZ * (offsetZ - voxelSize * indZ) + sensorOffset);

            voxelPoints(indPoint, 1) = x;
            voxelPoints(indPoint, 2) = y;
            voxelPoints(indPoint, 3) = z;
           
            indPoint  = indPoint + 1;
        end
    end
end

plot3(voxelPoints(:, 1), voxelPoints(:, 2), voxelPoints(:, 3) / searchScale - sensorOffset, 'k.');
pause(1.0);

fileID = fopen('correspondences.txt', 'w');

for batchCount = 0 : batchNum - 1
    fprintf('\nBatch %d collision checking\n', batchCount + 1);

    startID = floor(voxelPointNum * batchCount / batchNum) + 1;
    endID = floor(voxelPointNum * (batchCount + 1) / batchNum);
    
    ind = rangesearch(pathAll(1 : 3, :)', voxelPoints(startID : endID, :), searchRadiusHori);

    fprintf('\nBatch %d saving correspondences\n', batchCount + 1);

    for i = 1 : endID - startID + 1
        fwrite(fileID, i + startID - 2, 'int32');

        indVoxel = sort(ind{i});
        indVoxelNum = size(indVoxel, 2);

        pathIndRec = -1;
        for j = 1 : indVoxelNum
            pathInd = pathAll(4, indVoxel(j));
            if pathInd == pathIndRec
                continue;
            end

            fwrite(fileID, pathInd, 'int16');
            pathIndRec = pathInd;
        end
        fwrite(fileID, -1, 'int16');
    end
    
    clear ind;
end
fclose(fileID);

fprintf('\nProcessing complete\n');
%}
