% Calf Data Visualization (Preselected points)
clear
clc

%% Load in Mesh

% Change to mesh folder
cd('C:\Users\BLI\Desktop\BLI_ProbePathRender\JMonkeyEngine\JMEApplication_LiveProbeTracking\input');

% Read in OBJ model
[vertices,faces] = read_vertices_and_faces_from_obj_file('meshDefinition_mesh1.obj');

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
calibPoints = importdata('fixedPoints_meshTest.txt');
% calibPoints = importdata('fixedPoints_meshTest2.txt');

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
    tempMat(40+i,:) = fixedDataLocations((6*i)-1,:);

end

fixedDataLocations = tempMat;
scatter3(tempMat(:,1),tempMat(:,2),tempMat(:,3));

% for i = 1:8
%     tempMat(i,:) = fixedDataLocations((6*i)-5,:);
% end


clear calibPointsFinal calibPoints;

% Load in Calibration Points
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

%% Load Optical Data

% Change to Optical Data Path
cd('C:\Users\BLI\Desktop\Downloads\LowResCalfplots')

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

%%  Select Region for Interpolation
%{
xmin = min(calibPointsFinal(:,1));
xmax = max(calibPointsFinal(:,1));
ymin = min(calibPointsFinal(:,2));
ymax = max(calibPointsFinal(:,2));
zmin = min(calibPointsFinal(:,3));
zmax = max(calibPointsFinal(:,3));

xmin = min(finalDataLocations(:,1));
xmax = max(finalDataLocations(:,1));
ymin = min(finalDataLocations(:,2));
ymax = max(finalDataLocations(:,2));
zmin = min(finalDataLocations(:,3));
zmax = max(finalDataLocations(:,3));

factor = 0;
condition1 = verticesFinal(:,1) > xmin-factor;
condition2 = verticesFinal(:,1) < xmax+factor;
condition3 = verticesFinal(:,2) > ymin-factor;
condition4 = verticesFinal(:,2) < ymax+factor;
condition5 = verticesFinal(:,3) > zmin-factor;
condition6 = verticesFinal(:,3) < zmax+factor;
interpRange = condition1&condition2&condition3&condition4&condition5&condition6;
%}

interpRange = zeros(length(verticesFinal),1);
for i = 1:length(fixedDataLocations)
    distance(:,1) = (verticesFinal(:,1)-fixedDataLocations(i,1)).^2;
    distance(:,2) = (verticesFinal(:,2)-fixedDataLocations(i,2)).^2;
    distance(:,3) = (verticesFinal(:,3)-fixedDataLocations(i,3)).^2;
    distance = sqrt(distance(:,1)+distance(:,2)+distance(:,3));          
    [temp1,temp2] = find(distance < 1.5);
    interpRange(temp1) = 1  ;  
end
interpRange = logical(interpRange);
scatter3(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3))

% close all
% hold on
% trimesh(faces,verticesFinal(:,1),verticesFinal(:,2),verticesFinal(:,3))
% scatter3(fixedDataLocations(:,1),fixedDataLocations(:,2),fixedDataLocations(:,3),'filled','k');
% scatter3(calibPointsFinal(:,1),calibPointsFinal(:,2),calibPointsFinal(:,3),'filled','r');
% scatter3(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3),'filled','b')
% axis equal

%%
close all
% Interpolate Data
chromophoreSelect = 4; % 1 Oxyhemo 2 Deoxyhemo 3 Water 4 Lipid
V = chromMat(1:length(fixedDataLocations),chromophoreSelect);
VF = scatteredInterpolant(fixedDataLocations(:,1),fixedDataLocations(:,2),fixedDataLocations(:,3),V,'linear');
VFx = VF(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3));

scale =3;

figure('units','normalized','outerposition',[0 0 .5 1])
hold on
h = trisurf(faces,verticesFinal(:,1),verticesFinal(:,2),verticesFinal(:,3));
set(h,'FaceColor',[255/255,224/255,189/255])
set(h,'EdgeAlpha',.2);
scatter3(verticesFinal(interpRange,1),verticesFinal(interpRange,2),verticesFinal(interpRange,3),100*scale,VFx,'s','filled')
% scatter3(fixedDataLocations(:,1),fixedDataLocations(:,2),fixedDataLocations(:,3),100*scale,V,'filled')
scatter3(calibPointsFinal(:,1)+.2,calibPointsFinal(:,2),calibPointsFinal(:,3),200*scale,'filled','k')

colormap('jet');
axis equal
colorbar

switch chromophoreSelect
    case 2
        caxis([7.847, 22.271]) % deoxy
    case 1
        caxis([10 95]) % oxy
    case 3
        caxis([25 100]) % water
    case 4
        caxis([32 75]) % lipid
end

axis off

 campos([6.0868 -185.2579 -229.5211])
 zoom(1.5)
