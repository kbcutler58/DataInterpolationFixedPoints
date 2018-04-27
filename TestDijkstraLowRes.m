
%% Clear all variables
clear
clc

%% Add dependent functions  to path
currentFolder = pwd;
addpath(currentFolder);
addpath(strcat(currentFolder,'\toolbox_subset'));
addpath(strcat(currentFolder,'\dijkstra'));

%% Load Mesh
addpath(strcat(currentFolder,'\Meshes'));
[vertices,faces] = read_vertices_and_faces_from_obj_file('calfMesh_highres.obj');

%% Load Optical Data Locations
addpath(strcat(currentFolder,'\DataPoints\LowRes'));

% Translate and Offset Mesh
meshScale = 80;
vertices2 = vertices*meshScale;
oldCenter(1) = .5*(max(vertices2(:,1))+min(vertices2(:,1)));
oldCenter(2) = .5*(max(vertices2(:,2))+min(vertices2(:,2)));
oldCenter(3) = .5*(max(vertices2(:,3))+min(vertices2(:,3)));
verticesFinal(:,1) = vertices2(:,1) - oldCenter(1);
verticesFinal(:,2) = vertices2(:,2) - oldCenter(2);
verticesFinal(:,3) = vertices2(:,3) - oldCenter(3);

% Load in Fixed Data points
% Low Res Data Locations
calibPoints = importdata('fixedPoints_meshTest.txt');

% Average Points Locations for Fixed Points
for i = 1:length(calibPoints)/4
    range = (1+4*(i-1):4*i);
    calibPointsFinal(i,1) = mean(calibPoints(range,1));
    calibPointsFinal(i,2) = mean(calibPoints(range,2));
    calibPointsFinal(i,3) = mean(calibPoints(range,3));
end

fixedDataLocations = calibPointsFinal;
% Reorder fixed locations
tempMat = zeros(length(fixedDataLocations),3);
for i = 1:8
    tempMat(i,:) = fixedDataLocations((6*i)-5,:);
    tempMat(8+i,:) = fixedDataLocations((6*i)-4,:);
    tempMat(16+i,:) = fixedDataLocations((6*i)-3,:);
    tempMat(24+i,:) = fixedDataLocations((6*i)-2,:);
    tempMat(32+i,:) = fixedDataLocations((6*i)-1,:);
    tempMat(40+i,:) = fixedDataLocations((6*i)-0,:);

end

fixedDataLocations = tempMat;

clear calibPointsFinal calibPoints;

%% Load in Calibration Points
addpath(strcat(currentFolder,'\CalibrationPoints'));
calibPoints = importdata('fixedPoints_mesh1.txt');

% Average Points Locations for Fixed Points
for i = 1:length(calibPoints)/4
    range = (1+4*(i-1):4*i);
    calibPointsFinal(i,1) = mean(calibPoints(range,1));
    calibPointsFinal(i,2) = mean(calibPoints(range,2));
    calibPointsFinal(i,3) = mean(calibPoints(range,3));
end

% Display Fixed Points and 3D Mesh
close all
hold on
trimesh(faces,verticesFinal(:,1),verticesFinal(:,2),verticesFinal(:,3))
scatter3(fixedDataLocations(:,1),fixedDataLocations(:,2),fixedDataLocations(:,3),'filled','k');
scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3),'filled','r');
axis equal
%% Load Test Path Points if available
addpath(strcat(currentFolder,'\TestPathPoints'));
load('Head2Head.mat');

%% Load Optical Data

% Change to Optical Data Path
addpath(strcat(currentFolder,'\OpticalData\LowSampled'));

% Load in measurement times
fid2=fopen('ptwarm_160628_calf2__TIME.asc');
opticalTimes=textscan(fid2,'%s %s');
fclose('all');

% Sort measurements by time
[a,b]=sort(opticalTimes{2}(2:end));

% Load in optical chromophore measurements
fid = fopen('ptwarm_160628_calf2__SUM.asc');
opticalStringData = textscan(fid, '%s');
fclose('all');
% Scan each line looking for keywords
for i=1:size(opticalStringData{1,1})
      ind1(i)=strcmp(opticalStringData{1,1}(i),'wavelength(nm)');
      ind2(i)=strcmp(opticalStringData{1,1}(i),'FDPM');
      ind3(i)=strcmp(opticalStringData{1,1}(i),'HbO2');
end
startIndex = find(ind1==1)+1;
endIndex = find(ind2==1,1);
% Find number of points based on keywords
numPoints = endIndex-startIndex;
chromIndex = find(ind3==1);
chromIndex = chromIndex(2); % Switch to 1 for FDPM only
count=1;

% Change order of loaded data to match timing
for j = 1:numPoints
for i = b(j)
    chromMat(count,1) = str2num(char(opticalStringData{1,1}(chromIndex+i)));
    chromMat(count,2) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints+i+1)));
    chromMat(count,3) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*2+i+2)));
    chromMat(count,4) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*3+i+3)));
    chromMat(count,5) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*4+i+4)));
    chromMat(count,6) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*5+i+5)));
    chromMat(count,7) = str2num(char(opticalStringData{1,1}(chromIndex+numPoints*6+i+6)));
    count=count+1;
end
end

%% Find Mesh Locations for Interpolation
interpRange = zeros(length(verticesFinal),1);
for i = 1:length(fixedDataLocations)
    distance(:,1) = (verticesFinal(:,1)-fixedDataLocations(i,1)).^2;
    distance(:,2) = (verticesFinal(:,2)-fixedDataLocations(i,2)).^2;
    distance(:,3) = (verticesFinal(:,3)-fixedDataLocations(i,3)).^2;
    distance = sqrt(distance(:,1)+distance(:,2)+distance(:,3));          
    [temp1,temp2] = find(distance < 1.6);
    interpRange(temp1) = 1  ;  
end
interpRange = logical(interpRange);
scatter3(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3))

[adjacencyMatrix] = triangulation2adjacency(faces,verticesFinal);

%% Find Closest Locations on Mesh to Selected Start / End Points
differenceVertex1 = (verticesFinal(:,1) - Test_Points_cursor_info(1,1).Position(:,1));
differenceVertex2 = (verticesFinal(:,2) - Test_Points_cursor_info(1,1).Position(:,2));
differenceVertex3 = (verticesFinal(:,3) - Test_Points_cursor_info(1,1).Position(:,3));
[~,startIndex] = min(abs(differenceVertex1)+abs(differenceVertex2)+abs(differenceVertex3));

differenceVertex1 = (verticesFinal(:,1) - Test_Points_cursor_info(1,2).Position(:,1));
differenceVertex2 = (verticesFinal(:,2) - Test_Points_cursor_info(1,2).Position(:,2));
differenceVertex3 = (verticesFinal(:,3) - Test_Points_cursor_info(1,2).Position(:,3));
[~,endIndex] = min(abs(differenceVertex1)+abs(differenceVertex2)+abs(differenceVertex3));


%% Calculate path using dijkstra's algorithm
[costs,path1] = dijkstra(adjacencyMatrix,verticesFinal,startIndex,endIndex);

%gplot(adjacencyMatrix,verticesFinal,'k.:'); hold on;
%plot(verticesFinal(path1,1),verticesFinal(path,2),'ro-','LineWidth',2); hold off

close all
% Interpolate Data

% chromophoreSelect = 3; % 1 Oxyhemo 2 Deoxyhemo 3 Water 4 Lipid
for chromophoreSelect = 1:4
V = chromMat(1:length(fixedDataLocations),chromophoreSelect);
VF = scatteredInterpolant(fixedDataLocations(:,1),fixedDataLocations(:,2),fixedDataLocations(:,3),V,'linear');
VFx = VF(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3));

scale =.5;
close all


figure('units','normalized','outerposition',[0 0 .5 1])
hold on
h = trisurf(faces,verticesFinal(:,1),verticesFinal(:,2),verticesFinal(:,3));
set(h,'FaceColor',[255/255,224/255,189/255])
% set(h,'EdgeAlpha',0.05);
scatter3(fixedDataLocations(:,1),fixedDataLocations(:,2),fixedDataLocations(:,3),'filled','k');
scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3),'filled','b');
% h = trisurf(faces,verticesFinal(:,1),verticesFinal(:,2),verticesFinal(:,3));
VFx2 = VF(verticesFinal(path1(:),1),verticesFinal(path1(:),2),verticesFinal(path1(:),3));
scatter3(verticesFinal(path1(:),1),verticesFinal(path1(:),2),verticesFinal(path1(:),3),100*scale,VFx2,'s','filled');
% scatter3(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3),100*scale,VFx,'s','filled')
%scatter3(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3),100*scale,VFx,'s','filled')
colormap('hot');
% colormap('bone');
axis equal
colorbar

switch chromophoreSelect
    case 2
        title('HighResDeOxy')
        caxis([7.847, 22.271]) % deoxy
    case 1
        title('HighResOxy')
        caxis([10 95]) % oxy
    case 3
         title('HighResWater')
        caxis([25 100]) % water
%         caxis([0 50])
    case 4
        title('HighResLipid')
        caxis([32 75]) % lipid
    case 5
        caxis([0 120])
    case 7
        caxis([0 1])
end

axis off
 campos([6.0868 -185.2579 -229.5211])
campos([ -101.3027 -185.0767 -164.1013])
camroll(24)
zoom(1.5)

%% Path Plots
figure

plot(verticesFinal(path1(:)),VFx2,'*');
switch chromophoreSelect
    case 2
        title('LowResDeOxy'),xlabel('Position in cm'),ylabel('DeOxy Concentration');
        saveas(gcf,'LowResDeOxyPlot.png')
        ylim([7.847, 22.271]) % deoxy
    case 1
        title('LowResOxy'),xlabel('Position in cm'),ylabel('Oxy Concentration');
        saveas(gcf,'LowResOxyPlot.png')
        ylim([10 95]) % oxy
    case 3
        title('LowResWater'),xlabel('Position in cm'),ylabel('Water Concentration');
        saveas(gcf,'LowResWaterPlot.png')
        ylim([25 100]) % water
%         caxis([0 50])
    case 4
        title('LowResLipid'),xlabel('Position in cm'),ylabel('Lipid Concentration');
        saveas(gcf,'LowResLipidPlot.png')
        ylim([32 75]) % lipid
    case 5
        axis([0 120])
    case 7
        axis([0 1])
end

end